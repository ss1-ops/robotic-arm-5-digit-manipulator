#!/usr/bin/env python3
"""
Moveo Arm Joint Controller - Desktop GUI for Manual Testing
Connects to armpi via SSH and provides real-time joint control
"""

import sys
import json
import threading
import time
from dataclasses import dataclass
from typing import Optional

import paramiko
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QLineEdit, QPushButton, QGroupBox, QGridLayout,
    QMessageBox, QInputDialog
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QThread
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
    
    def __init__(self):
        super().__init__()
        self.client: Optional[paramiko.SSHClient] = None
        self.connected = False
        self.host = "armpi.local"
        self.username = "armpi"
        self.password: Optional[str] = None
        
    def connect_ssh(self, password: str):
        """Establish SSH connection to armpi"""
        try:
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.client.connect(self.host, username=self.username, password=password, timeout=5)
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
        if self.client:
            self.client.close()
            self.connected = False
            self.status_changed.emit("Disconnected from armpi")
    
    def run_ros_command(self, command: str) -> str:
        """Execute a ROS command on armpi via SSH"""
        if not self.connected or not self.client:
            raise RuntimeError("Not connected to SSH")
        
        # Source ROS environment and run command
        full_command = (
            "source /opt/ros/jazzy/setup.bash && "
            "source ~/moveo_ws/install/local_setup.bash && "
            f"{command}"
        )
        
        stdin, stdout, stderr = self.client.exec_command(full_command)
        output = stdout.read().decode('utf-8')
        error = stderr.read().decode('utf-8')
        
        if error and "Warning" not in error:
            raise RuntimeError(error)
        
        return output
    
    def publish_joint_state(self, joint_angles: list):
        """Publish joint angles to /manual_joint_command topic"""
        if not self.connected:
            return
        
        try:
            # Convert to JSON for the manual controller
            angles_json = json.dumps(joint_angles)
            # Escape quotes for shell
            escaped_json = angles_json.replace('"', '\\"')
            
            # Publish to manual_joint_command topic (the helper node listens to this)
            ros_command = (
                f"ros2 topic pub --once /manual_joint_command std_msgs/msg/String \"data: '{escaped_json}'\" 2>/dev/null"
            )
            
            self.run_ros_command(ros_command)
            
        except Exception as e:
            self.error_occurred.emit(f"Failed to publish joints: {str(e)}")
    
    def get_current_joints(self) -> Optional[list]:
        """Read current joint positions from /joint_states"""
        if not self.connected:
            return None
        
        try:
            command = (
                'ros2 topic echo --once /joint_states | grep -A 10 "position:" | '
                'head -1 | tr -d "[]" | tr "," "\\n" | xargs'
            )
            output = self.run_ros_command(command)
            if output:
                angles = [float(x) for x in output.strip().split()]
                self.joints_updated.emit(angles)
                return angles
        except Exception as e:
            pass  # Silently fail for polling
        return None
    
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
        
        self.commanded_angles = [0.0] * 5
        self.latest_joint_angles = [0.0] * 5
        self.estop_active = False
        
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
        
        central_widget.setLayout(main_layout)
        
        # Status bar
        self.statusBar().showMessage("Disconnected")
        
    def connect_ssh(self):
        """Connect to armpi"""
        password, accepted = QInputDialog.getText(
            self,
            "SSH Password",
            f"Password for {self.ssh_worker.username}@{self.ssh_worker.host}:",
            QLineEdit.Password,
        )
        if not accepted:
            return

        self.connect_btn.setEnabled(False)
        self.statusBar().showMessage("Connecting...")
        self.statusBar().repaint()
        
        def do_connect():
            success = self.ssh_worker.connect_ssh(password)
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
            else:
                self.connect_btn.setEnabled(True)
        
        thread = threading.Thread(target=do_connect, daemon=True)
        thread.start()
    
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
    
    def on_slider_change(self, joint_idx: int, slider_value: int):
        """Handle slider movement - auto-send to ROS"""
        angle_rad = slider_value / 100.0
        self.commanded_angles[joint_idx] = angle_rad
        
        # Update text box
        self.text_boxes[joint_idx].setText(f"{angle_rad:.3f}")
        
        # Auto-send to ROS
        self.send_joint_angles()
    
    def on_text_update(self, joint_idx: int):
        """Handle manual text entry"""
        try:
            angle_rad = float(self.text_boxes[joint_idx].text())
            config = JOINT_CONFIGS[joint_idx]
            
            # Clamp to valid range
            angle_rad = max(config.min_angle, min(config.max_angle, angle_rad))
            
            self.commanded_angles[joint_idx] = angle_rad
            
            # Update slider
            self.sliders[joint_idx].blockSignals(True)
            self.sliders[joint_idx].setValue(int(angle_rad * 100))
            self.sliders[joint_idx].blockSignals(False)
            
            # Update text box
            self.text_boxes[joint_idx].setText(f"{angle_rad:.3f}")
            
            # Send to ROS
            self.send_joint_angles()
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter a valid number in radians")
    
    def send_joint_angles(self):
        """Send current joint angles to ROS"""
        if not self.ssh_worker.connected or self.estop_active:
            return
        
        def do_send():
            try:
                self.ssh_worker.publish_joint_state(self.commanded_angles)
            except Exception as e:
                self.ssh_worker.error_occurred.emit(f"Failed to send joints: {str(e)}")
        
        thread = threading.Thread(target=do_send)
        thread.daemon = True
        thread.start()
    
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
                time.sleep(0.5)
        
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
    
    def show_error(self, message: str):
        """Show error dialog"""
        QMessageBox.critical(self, "Error", message)
    
    def closeEvent(self, event):
        """Clean up on window close"""
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
