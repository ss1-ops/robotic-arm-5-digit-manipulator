#!/usr/bin/env python3
"""
Moveo Arm Joint Controller - Desktop GUI for Manual Testing
Connects to armpi via SSH and provides real-time joint control
"""

import sys
import json
import threading
import time
import shlex
import re
from dataclasses import dataclass
from typing import Optional

import paramiko
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QLineEdit, QPushButton, QGroupBox, QGridLayout,
    QMessageBox, QInputDialog, QPlainTextEdit
)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QObject, QThread
from PyQt5.QtGui import QFont


@dataclass
class JointConfig:
    """Configuration for each joint"""
    name: str
    min_angle: float
    max_angle: float
    default: float


JOINT_CONFIGS = [
    JointConfig("Joint_1", -3.14, 3.14, 0.0),      # Waist: ±180°
    JointConfig("Joint_2", -1.57, 1.57, 0.0),      # Shoulder: ±90°
    JointConfig("Joint_3", -1.57, 1.57, 0.0),      # Elbow: ±90°
    JointConfig("Joint_4", -3.14, 3.14, 0.0),      # Wrist Roll: ±180°
    JointConfig("Joint_5", -1.57, 1.57, 0.0),      # Wrist Pitch: ±90°
]


class SSHWorker(QObject):
    """Handles SSH communication in a separate thread"""
    
    status_changed = pyqtSignal(str)
    joints_updated = pyqtSignal(list)
    error_occurred = pyqtSignal(str)
    log_line = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.client: Optional[paramiko.SSHClient] = None
        self.connected = False
        self.host = "armpi.local"
        self.username = "armpi"
        self.password: Optional[str] = None
        # Paramiko channels are sensitive to parallel exec_command calls.
        self._ssh_lock = threading.Lock()
        self.log_client: Optional[paramiko.SSHClient] = None
        self.log_stdout = None
        self.log_stderr = None
        self.log_running = False
        self.log_thread: Optional[threading.Thread] = None
        
    def connect_ssh(self, password: Optional[str] = None):
        """Establish SSH connection to armpi. Tries agent/key auth first; falls back to password."""
        try:
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.client.connect(
                self.host,
                username=self.username,
                password=password,
                look_for_keys=True,
                allow_agent=True,
                timeout=5,
            )
            self.password = password
            self.connected = True
            self.status_changed.emit(f"✓ Connected to {self.host}")
            return True
        except Exception as e:
            self.connected = False
            self.error_occurred.emit(f"SSH Connection Failed: {str(e)}")
            self.status_changed.emit(f"✗ Connection failed: {str(e)}")
            return False
    
    def disconnect_ssh(self):
        """Close SSH connection"""
        self.stop_log_stream()
        if self.client:
            self.client.close()
            self.connected = False
            self.status_changed.emit("Disconnected from armpi")

    def _build_ros_shell_command(self, command: str) -> str:
        """Wrap a command in a ROS-ready non-login bash shell.

        Using 'bash -c' (not '-lc') avoids sourcing /etc/profile and
        ~/.bash_profile whose motd/banner output would pollute stdout and
        corrupt any parsed command output.
        """
        ros_preamble = (
            "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; "
            "for d in /opt/ros/jazzy /opt/ros/humble /opt/ros/iron /opt/ros/foxy; do "
            "if [ -f \"$d/setup.bash\" ]; then source \"$d/setup.bash\" >/dev/null 2>&1; break; fi; "
            "done; "
            "if [ -f \"$HOME/moveo_ws/install/local_setup.bash\" ]; then source \"$HOME/moveo_ws/install/local_setup.bash\" >/dev/null 2>&1; "
            "elif [ -f \"$HOME/moveo_ws/install/setup.bash\" ]; then source \"$HOME/moveo_ws/install/setup.bash\" >/dev/null 2>&1; fi; "
            "command -v ros2 >/dev/null 2>&1 || { echo '[env] ros2 not found after sourcing' >&2; exit 127; }; "
        )
        return f"bash -c {shlex.quote(ros_preamble + command)}"
    
    def run_ros_command(self, command: str) -> str:
        """Execute a ROS command on armpi via SSH"""
        if not self.connected or not self.client:
            self.log_line.emit("[SSH] ERROR: not connected")
            raise RuntimeError("Not connected to SSH")
        full_command = self._build_ros_shell_command(command)
        self.log_line.emit(f"[SSH] exec: {command[:120]}")

        with self._ssh_lock:
            last_error = None
            for attempt in range(2):
                stdin = stdout = stderr = None
                try:
                    stdin, stdout, stderr = self.client.exec_command(full_command)
                    output = stdout.read().decode('utf-8')
                    error = stderr.read().decode('utf-8')
                    exit_code = stdout.channel.recv_exit_status()
                    self.log_line.emit(f"[SSH] exit={exit_code} stdout={output.strip()!r} stderr={error.strip()!r}")
                    if exit_code != 0:
                        raise RuntimeError(
                            f"Remote command failed (exit {exit_code})\n"
                            f"cmd: {command}\nstderr: {error.strip()}\nstdout: {output.strip()}"
                        )
                    return output
                except Exception as e:
                    last_error = e
                    self.log_line.emit(f"[SSH] attempt {attempt+1} exception: {e}")
                    if "ChannelException" not in str(e):
                        raise
                    time.sleep(0.1)
                finally:
                    for s in (stdin, stdout, stderr):
                        try:
                            if s is not None: s.close()
                        except Exception:
                            pass

            raise RuntimeError(str(last_error))
    
    def publish_joint_state(self, joint_angles: list):
        """Publish joint angles directly to /joint_commands (JointState, BEST_EFFORT)."""
        self.log_line.emit(f"[pub] enter publish_joint_state: connected={self.connected} angles={joint_angles}")
        if not self.connected:
            self.log_line.emit("[pub] SKIP: not connected")
            return

        try:
            angles_str = "[" + ", ".join(f"{a:.4f}" for a in joint_angles) + "]"
            msg_yaml = (
                f"{{name: ['j1','j2','j3','j4','j5'], "
                f"position: {angles_str}, velocity: [], effort: []}}"
            )
            self.log_line.emit(f"[pub] msg_yaml: {msg_yaml}")

            ros_command = (
                f"timeout 5 ros2 topic pub --once --qos-reliability best_effort "
                f"/joint_commands sensor_msgs/msg/JointState "
                f"{shlex.quote(msg_yaml)}"
                # capture stderr so it shows in our log instead of being suppressed
            )
            self.log_line.emit(f"[pub] ros_command: {ros_command}")

            result = self.run_ros_command(ros_command)
            self.log_line.emit(f"[pub] done, result: {result.strip()!r}")

        except Exception as e:
            self.log_line.emit(f"[pub] EXCEPTION: {e}")
            self.error_occurred.emit(f"Failed to send command: {str(e)}")
    
    def get_current_joints(self) -> Optional[list]:
        """Read current joint positions from /joint_states"""
        if not self.connected:
            return None
        
        try:
            # grep filters to the one line that is the position array so that
            # any ROS init messages or banner text never reaches the parser.
            command = (
                'ros2 topic echo --once /joint_states --field position 2>/dev/null'
                ' | grep -m1 -E \'\\[[-0-9., eE]+\\]\''
            )
            output = self.run_ros_command(command)
            if output:
                # Output is now guaranteed to be a single line like:
                #   [0.0, 0.0, 0.0, 0.0, 0.0]
                # or array('d', [0.0, 0.0, 0.0, 0.0, 0.0])
                bracket = re.search(r'\[([^\]]+)\]', output)
                if bracket:
                    numbers = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", bracket.group(1))
                    if len(numbers) >= 5:
                        angles = [float(x) for x in numbers[:5]]
                        self.joints_updated.emit(angles)
                        return angles
        except Exception as e:
            pass  # Silently fail for polling
        return None

    def start_log_stream(self):
        """Start a dedicated SSH stream that tails live ROS2 topic output."""
        if not self.connected or self.log_running:
            return

        try:
            self.log_client = paramiko.SSHClient()
            self.log_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.log_client.connect(
                self.host,
                username=self.username,
                password=self.password,
                look_for_keys=True,
                allow_agent=True,
                timeout=5,
            )

            # [cmd] = positions landing on /joint_commands (what ESP32 receives)
            # [fb]  = /joint_states feedback (stepper dead-reckoning via broadcaster)
            log_command = (
                "echo '[log] ROS2 stream connected' && "
                "(ros2 topic echo /joint_commands sensor_msgs/msg/JointState --field position "
                "2>/dev/null | grep -v '^---$' | sed 's/^/[cmd] /') & "
                "(ros2 topic echo /joint_states sensor_msgs/msg/JointState --field position "
                "2>/dev/null | grep -v '^---$' | sed 's/^/[fb] /') & "
                "wait"
            )

            _, self.log_stdout, self.log_stderr = self.log_client.exec_command(
                self._build_ros_shell_command(log_command),
                get_pty=True
            )
            self.log_running = True
            self.log_thread = threading.Thread(target=self._log_reader_loop, daemon=True)
            self.log_thread.start()
        except Exception as e:
            self.log_running = False
            self.log_line.emit(f"[log] Failed to start ROS2 stream: {str(e)}")

    def _log_reader_loop(self):
        """Read lines from the dedicated ROS2 log stream."""
        try:
            while self.log_running and self.log_stdout:
                line = self.log_stdout.readline()
                if not line:
                    break
                self.log_line.emit(line.rstrip())
        except Exception as e:
            if self.log_running:
                self.log_line.emit(f"[log] stream error: {str(e)}")
        finally:
            self.log_running = False

    def stop_log_stream(self):
        """Stop the dedicated ROS2 log stream."""
        self.log_running = False

        for stream in (self.log_stdout, self.log_stderr):
            try:
                if stream is not None:
                    stream.close()
            except Exception:
                pass

        self.log_stdout = None
        self.log_stderr = None

        if self.log_client:
            try:
                self.log_client.close()
            except Exception:
                pass
            self.log_client = None
    
    def estop(self):
        """Emergency stop - freeze all joints"""
        try:
            # Publish zero velocities or maintain current position
            # For now, just publish current zeros to stop movement
            self.publish_joint_state([0.0] * 5)
            self.status_changed.emit("E-STOP: Joints frozen")
        except Exception as e:
            self.error_occurred.emit(f"E-STOP failed: {str(e)}")


class JointController(QMainWindow):
    """Main GUI window for joint control"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Moveo Arm - Joint Controller")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize SSH worker
        self.ssh_worker = SSHWorker()
        self.ssh_thread = QThread()
        self.ssh_worker.moveToThread(self.ssh_thread)
        self.ssh_thread.start()
        
        # Connect signals
        self.ssh_worker.status_changed.connect(self.update_status)
        self.ssh_worker.error_occurred.connect(self.show_error)
        self.ssh_worker.joints_updated.connect(self.update_sliders_from_ros)
        self.ssh_worker.log_line.connect(self.append_log_line)
        
        self.commanded_angles = [0.0] * 5
        self.latest_joint_angles = [0.0] * 5
        self.estop_active = False
        self._send_state_lock = threading.Lock()
        self._send_in_flight = False
        self._pending_angles: Optional[list] = None
        self._console_counter = 0  # throttle high-rate console lines
        
        self.init_ui()
        
        # Polling timer for joint state updates
        self.poll_timer = None
        
    def init_ui(self):
        """Initialize UI components"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout()
        
        # Connection panel
        connection_layout = QHBoxLayout()
        self.connect_btn = QPushButton("🔗 Connect SSH")
        self.connect_btn.clicked.connect(self.connect_ssh)
        self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 8px;")
        
        self.disconnect_btn = QPushButton("🔌 Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_ssh)
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 8px;")
        
        connection_layout.addWidget(self.connect_btn)
        connection_layout.addWidget(self.disconnect_btn)
        connection_layout.addStretch()
        
        main_layout.addLayout(connection_layout)
        
        # Joint control panel
        joints_group = QGroupBox("Joint Control")
        joints_layout = QGridLayout()
        
        self.sliders = {}
        self.text_boxes = {}
        self.update_buttons = {}
        
        for idx, config in enumerate(JOINT_CONFIGS):
            row = idx * 2
            
            # Label
            label = QLabel(config.name)
            label.setFont(QFont("Arial", 10, QFont.Bold))
            joints_layout.addWidget(label, row, 0)
            
            # Slider
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(config.min_angle * 100))
            slider.setMaximum(int(config.max_angle * 100))
            slider.setValue(int(config.default * 100))
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(50)
            slider.valueChanged.connect(lambda val, j=idx: self.on_slider_change(j, val))
            slider.setEnabled(False)
            self.sliders[idx] = slider
            joints_layout.addWidget(slider, row, 1)
            
            # Text box with value in radians
            text_box = QLineEdit()
            text_box.setText(f"{config.default:.3f}")
            text_box.setMaximumWidth(100)
            text_box.setEnabled(False)
            self.text_boxes[idx] = text_box
            joints_layout.addWidget(text_box, row, 2)
            
            # Unit label
            unit_label = QLabel("rad")
            joints_layout.addWidget(unit_label, row, 3)
            
            # Update button for text entry
            update_btn = QPushButton(f"Set {config.name}")
            update_btn.clicked.connect(lambda checked=False, j=idx: self.on_text_update(j))
            update_btn.setEnabled(False)
            update_btn.setMaximumWidth(120)
            self.update_buttons[idx] = update_btn
            joints_layout.addWidget(update_btn, row, 4)
            
            # Current position label
            pos_label = QLabel("0.000")
            joints_layout.addWidget(pos_label, row + 1, 1, 1, 2)
            self.position_labels = getattr(self, 'position_labels', {})
            self.position_labels[idx] = pos_label
        
        joints_group.setLayout(joints_layout)
        main_layout.addWidget(joints_group)
        
        # E-STOP and control buttons
        control_layout = QHBoxLayout()
        
        self.estop_btn = QPushButton("🛑 E-STOP (FREEZE ALL)")
        self.estop_btn.setStyleSheet("background-color: #ff0000; color: white; font-weight: bold; font-size: 14px; padding: 12px;")
        self.estop_btn.clicked.connect(self.emergency_stop)
        self.estop_btn.setEnabled(False)
        control_layout.addWidget(self.estop_btn)
        
        refresh_btn = QPushButton("🔄 Refresh Joint States")
        refresh_btn.clicked.connect(self.refresh_joint_states)
        refresh_btn.setEnabled(False)
        self.refresh_btn = refresh_btn
        control_layout.addWidget(refresh_btn)
        
        control_layout.addStretch()
        main_layout.addLayout(control_layout)

        # Live ROS console panel
        console_group = QGroupBox("ROS2 Live Console")
        console_layout = QVBoxLayout()

        self.ros_console = QPlainTextEdit()
        self.ros_console.setReadOnly(True)
        self.ros_console.setMaximumBlockCount(1200)
        self.ros_console.setPlaceholderText("ROS2 messages will stream here after SSH connect...")
        console_layout.addWidget(self.ros_console)

        console_btn_layout = QHBoxLayout()
        clear_console_btn = QPushButton("Clear Console")
        clear_console_btn.clicked.connect(self.ros_console.clear)
        console_btn_layout.addWidget(clear_console_btn)
        console_btn_layout.addStretch()
        console_layout.addLayout(console_btn_layout)

        console_group.setLayout(console_layout)
        main_layout.addWidget(console_group)
        
        central_widget.setLayout(main_layout)
        
        # Status bar
        self.statusBar().showMessage("Disconnected")
        
    def connect_ssh(self):
        """Connect to armpi - tries key/agent auth first, prompts for password only if needed."""
        self.connect_btn.setEnabled(False)
        self.statusBar().showMessage("Connecting...")
        self.statusBar().repaint()

        def do_connect():
            # First attempt: key/agent auth (no password needed)
            success = self.ssh_worker.connect_ssh(password=None)
            if not success:
                # Key auth failed - ask for password on the main thread via signal
                # Fall back by prompting on the GUI thread.
                import queue
                pw_queue: queue.Queue = queue.Queue()

                def ask_password():
                    password, accepted = QInputDialog.getText(
                        self,
                        "SSH Password",
                        f"Password for {self.ssh_worker.username}@{self.ssh_worker.host}:",
                        QLineEdit.Password,
                    )
                    pw_queue.put(password if accepted else None)

                from PyQt5.QtCore import QMetaObject, Qt as _Qt
                QMetaObject.invokeMethod(self, "_run_password_dialog",
                                         _Qt.BlockingQueuedConnection)
                # Use instance attribute set by _run_password_dialog
                pw = getattr(self, '_pending_password', None)
                if pw is None:
                    self.connect_btn.setEnabled(True)
                    return
                success = self.ssh_worker.connect_ssh(password=pw)

            if success:
                self.disconnect_btn.setEnabled(True)
                
                # Enable controls
                for slider in self.sliders.values():
                    slider.setEnabled(True)
                for text_box in self.text_boxes.values():
                    text_box.setEnabled(True)
                for btn in self.update_buttons.values():
                    btn.setEnabled(True)
                self.estop_btn.setEnabled(True)
                self.refresh_btn.setEnabled(True)
                
                # Start polling joint states
                self.start_polling()
                self.ssh_worker.start_log_stream()
            else:
                self.connect_btn.setEnabled(True)
        
        thread = threading.Thread(target=do_connect, daemon=True)
        thread.start()
    
    @pyqtSlot()
    def _run_password_dialog(self):
        """Show password dialog on the main thread; result stored in self._pending_password."""
        password, accepted = QInputDialog.getText(
            self,
            "SSH Password",
            f"Password for {self.ssh_worker.username}@{self.ssh_worker.host}:",
            QLineEdit.Password,
        )
        self._pending_password = password if accepted else None

    def disconnect_ssh(self):
        """Disconnect from armpi"""
        self.ssh_worker.disconnect_ssh()
        self.disconnect_btn.setEnabled(False)
        self.connect_btn.setEnabled(True)
        
        # Disable controls
        for slider in self.sliders.values():
            slider.setEnabled(False)
        for text_box in self.text_boxes.values():
            text_box.setEnabled(False)
        for btn in self.update_buttons.values():
            btn.setEnabled(False)
        self.estop_btn.setEnabled(False)
        self.refresh_btn.setEnabled(False)
        
        self.stop_polling()
        self.ssh_worker.stop_log_stream()
    
    def on_slider_change(self, joint_idx: int, slider_value: int):
        """Handle slider movement - auto-send to ROS"""
        angle_rad = slider_value / 100.0
        print(f"[SLIDER] j{joint_idx+1} slider_value={slider_value} angle_rad={angle_rad:.4f} prev={self.commanded_angles[joint_idx]:.4f}")

        # Dead-band: ignore sub-0.01 rad jitter to avoid flooding the controller.
        if abs(angle_rad - self.commanded_angles[joint_idx]) < 0.01:
            print(f"[SLIDER] j{joint_idx+1} dead-band skip (delta={abs(angle_rad - self.commanded_angles[joint_idx]):.4f})")
            return

        self.commanded_angles[joint_idx] = angle_rad
        print(f"[SLIDER] j{joint_idx+1} commanded_angles now={self.commanded_angles}")

        # Update text box
        self.text_boxes[joint_idx].setText(f"{angle_rad:.3f}")

        # Auto-send to ROS
        self.send_joint_angles()
    
    def on_text_update(self, joint_idx: int):
        """Handle manual text entry"""
        raw = self.text_boxes[joint_idx].text()
        print(f"[TEXT] j{joint_idx+1} raw input: {raw!r}")
        try:
            angle_rad = float(raw)
            config = JOINT_CONFIGS[joint_idx]
            angle_rad = max(config.min_angle, min(config.max_angle, angle_rad))
            print(f"[TEXT] j{joint_idx+1} parsed+clamped: {angle_rad:.4f}")

            self.commanded_angles[joint_idx] = angle_rad
            print(f"[TEXT] j{joint_idx+1} commanded_angles now={self.commanded_angles}")

            self.sliders[joint_idx].blockSignals(True)
            self.sliders[joint_idx].setValue(int(angle_rad * 100))
            self.sliders[joint_idx].blockSignals(False)
            self.text_boxes[joint_idx].setText(f"{angle_rad:.3f}")

            self.send_joint_angles()
        except ValueError:
            print(f"[TEXT] j{joint_idx+1} ValueError on input {raw!r}")
            QMessageBox.warning(self, "Invalid Input", "Please enter a valid number in radians")
    
    def send_joint_angles(self):
        """Send current joint angles to ROS"""
        print(f"[SEND] send_joint_angles called: connected={self.ssh_worker.connected} estop={self.estop_active} angles={self.commanded_angles}")
        if not self.ssh_worker.connected or self.estop_active:
            print(f"[SEND] SKIP: connected={self.ssh_worker.connected} estop={self.estop_active}")
            return

        angles = list(self.commanded_angles)
        with self._send_state_lock:
            if self._send_in_flight:
                self._pending_angles = angles
                print(f"[SEND] in-flight, coalescing to pending={angles}")
                return
            self._send_in_flight = True
        print(f"[SEND] dispatching worker with angles={angles}")
        thread = threading.Thread(target=self._send_joint_angles_worker, args=(angles,), daemon=True)
        thread.start()

    def _send_joint_angles_worker(self, initial_angles: list):
        """Serialize outgoing joint commands to avoid SSH channel exhaustion."""
        angles_to_send = initial_angles

        while True:
            try:
                self.ssh_worker.publish_joint_state(angles_to_send)
            except Exception as e:
                self.ssh_worker.error_occurred.emit(f"Failed to send joints: {str(e)}")

            with self._send_state_lock:
                if self._pending_angles is not None:
                    angles_to_send = self._pending_angles
                    self._pending_angles = None
                    continue
                self._send_in_flight = False
                break
    
    def emergency_stop(self):
        """Emergency stop - freeze all joints"""
        self.estop_active = True
        self.commanded_angles = list(self.latest_joint_angles)
        self.ssh_worker.publish_joint_state(self.latest_joint_angles)
        self.ssh_worker.status_changed.emit("E-STOP: joints held at current position")
        
        # Disable sliders during E-STOP
        for slider in self.sliders.values():
            slider.setEnabled(False)
        for text_box in self.text_boxes.values():
            text_box.setEnabled(False)
        for btn in self.update_buttons.values():
            btn.setEnabled(False)
        
        self.statusBar().showMessage("🛑 E-STOP ACTIVE - Reset to continue")
        
        # Show dialog to reset
        reply = QMessageBox.warning(
            self,
            "E-STOP Activated",
            "All joints frozen. Click 'Reset' to resume control.",
            QMessageBox.Ok
        )
        
        if reply == QMessageBox.Ok:
            self.reset_from_estop()
    
    def reset_from_estop(self):
        """Reset after E-STOP"""
        self.estop_active = False
        
        # Re-enable controls
        for slider in self.sliders.values():
            slider.setEnabled(True)
        for text_box in self.text_boxes.values():
            text_box.setEnabled(True)
        for btn in self.update_buttons.values():
            btn.setEnabled(True)
        
        self.statusBar().showMessage("E-STOP reset - Ready for control")
    
    def refresh_joint_states(self):
        """Manually refresh joint states from ROS"""
        def do_refresh():
            self.ssh_worker.get_current_joints()
        
        thread = threading.Thread(target=do_refresh)
        thread.daemon = True
        thread.start()
    
    def start_polling(self):
        """Start polling joint states"""
        def poll():
            while self.ssh_worker.connected:
                try:
                    self.ssh_worker.get_current_joints()
                except:
                    pass
                time.sleep(1.0)
        
        self.poll_thread = threading.Thread(target=poll)
        self.poll_thread.daemon = True
        self.poll_thread.start()
    
    def stop_polling(self):
        """Stop polling joint states"""
        pass  # Daemon thread will stop when SSH disconnects
    
    def update_sliders_from_ros(self, angles: list):
        """Update UI sliders from ROS joint state"""
        for idx, angle in enumerate(angles[:5]):
            self.latest_joint_angles[idx] = angle
            # Update position labels
            if idx in self.position_labels:
                self.position_labels[idx].setText(f"Current: {angle:.3f} rad")
    
    def update_status(self, message: str):
        """Update status bar"""
        self.statusBar().showMessage(message)

    def append_log_line(self, line: str):
        """Append one line to the ROS2 live console.

        High-rate feedback lines ([cmd]/[fb]) are throttled to every 5th
        message (~4 Hz display from a 20 Hz stream) to keep the console
        readable without losing [tx] / [log] / error lines.
        """
        if line.startswith(("[cmd]", "[fb]")):
            self._console_counter += 1
            if self._console_counter % 5 != 0:
                return
        else:
            self._console_counter = 0  # always show non-feedback lines immediately
        self.ros_console.appendPlainText(line)
    
    def show_error(self, message: str):
        """Show error dialog"""
        QMessageBox.critical(self, "Error", message)
    
    def closeEvent(self, event):
        """Clean up on window close"""
        self.ssh_worker.stop_log_stream()
        if self.ssh_worker.connected:
            self.ssh_worker.disconnect_ssh()
        self.ssh_thread.quit()
        self.ssh_thread.wait()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = JointController()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
