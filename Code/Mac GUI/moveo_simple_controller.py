#!/usr/bin/env python3
"""
Moveo Arm - Simple Joint Controller
Connects via SSH, resets all ROS2/agent state to clean slate,
then sends joint commands directly with:
  ros2 topic pub --once --qos-reliability best_effort /joint_commands ...
"""

import sys
import shlex
import threading
import time
from dataclasses import dataclass

import paramiko
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QLineEdit, QPushButton, QGroupBox, QGridLayout,
    QPlainTextEdit, QMessageBox,
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject


# ── Joint definitions ──────────────────────────────────────────────────────────
@dataclass
class Joint:
    label: str
    min_rad: float
    max_rad: float

JOINTS = [
    Joint("J1  Waist",       -3.14,  3.14),
    Joint("J2  Shoulder",    -1.57,  1.57),
    Joint("J3  Elbow",       -1.57,  1.57),
    Joint("J4  Wrist Roll",  -3.14,  3.14),
    Joint("J5  Wrist Pitch", -1.57,  1.57),
]

SLIDER_SCALE = 100   # slider integer units per radian


# ── SSH / ROS worker ───────────────────────────────────────────────────────────
class Worker(QObject):
    log      = pyqtSignal(str)
    status   = pyqtSignal(str)  # "connected" | "disconnected" | error text

    HOST = "armpi.local"
    USER = "armpi"

    # Bash snippet that runs on the Pi on every connection:
    #   1. kill any old micro_ros_agent processes
    #   2. clear stale FastDDS SHM files
    #   3. start a fresh micro_ros_agent in the background
    STARTUP_SCRIPT = (
        "fuser -k 8888/udp 2>/dev/null; "
        "sleep 1; "
        "rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null; "
        "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; "
        "source ~/microros_ws/install/setup.bash; "
        "nohup ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4 "
        "  > /tmp/mra.log 2>&1 &disown; "
        "echo '[startup] micro_ros_agent started'; "
        "sleep 5; "                          # give ESP32 time to (re)connect
        "grep -m1 'session established' /tmp/mra.log "
        "  && echo '[startup] ESP32 connected' "
        "  || echo '[startup] WARNING: ESP32 not yet connected (check serial)'"
    )

    # Preamble prepended to every ros2 command
    ROS_PREAMBLE = (
        "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; "
        "source /opt/ros/jazzy/setup.bash; "
    )

    def __init__(self):
        super().__init__()
        self._client: paramiko.SSHClient | None = None
        self._lock = threading.Lock()
        self.connected = False

    # ── connection ──────────────────────────────────────────────────────────
    def connect(self, password: str | None = None):
        try:
            self.log.emit("[SSH] connecting…")
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            c.connect(self.HOST, username=self.USER,
                      password=password,
                      look_for_keys=True, allow_agent=True, timeout=8)
            self._client = c
            self.connected = True
            self.log.emit("[SSH] connected — running startup script…")
            self.status.emit("connected")

            # Run startup script (blocks until agent is up)
            out, err, code = self._exec(self.STARTUP_SCRIPT, timeout=20)
            for line in (out + err).splitlines():
                self.log.emit(f"  {line}")
            if code != 0:
                self.log.emit(f"[startup] exit={code}")

        except Exception as e:
            self.log.emit(f"[SSH] connect failed: {e}")
            self.status.emit(f"error: {e}")
            self.connected = False

    def disconnect(self):
        self.connected = False
        try:
            if self._client:
                self._client.close()
        except Exception:
            pass
        self._client = None
        self.status.emit("disconnected")
        self.log.emit("[SSH] disconnected")

    # ── sending a command ────────────────────────────────────────────────────
    def send_angles(self, angles: list[float]):
        if not self.connected:
            self.log.emit("[tx] SKIP — not connected")
            return

        pos_str = "[" + ", ".join(f"{a:.4f}" for a in angles) + "]"
        msg = (
            "{name: ['j1','j2','j3','j4','j5'], "
            f"position: {pos_str}, velocity: [], effort: []}}"
        )
        # Build the full one-liner used in the terminal test
        cmd = (
            self.ROS_PREAMBLE +
            "timeout 6 ros2 topic pub --once --qos-reliability best_effort "
            f"/joint_commands sensor_msgs/msg/JointState {shlex.quote(msg)}"
        )

        self.log.emit(f"[tx] angles={pos_str}")
        self.log.emit(f"[cmd] {cmd[:120]}…")

        out, err, code = self._exec(cmd, timeout=8)
        self.log.emit(f"[rx] exit={code} stdout={out.strip()!r} stderr={err.strip()!r}")

    # ── internal exec ────────────────────────────────────────────────────────
    def _exec(self, cmd: str, timeout: int = 10):
        full = f"bash -c {shlex.quote(cmd)}"
        with self._lock:
            stdin = stdout = stderr = None
            try:
                stdin, stdout, stderr = self._client.exec_command(full, timeout=timeout)
                out  = stdout.read().decode("utf-8", errors="replace")
                err  = stderr.read().decode("utf-8", errors="replace")
                code = stdout.channel.recv_exit_status()
                return out, err, code
            except Exception as e:
                return "", str(e), -1
            finally:
                for s in (stdin, stdout, stderr):
                    try:
                        if s: s.close()
                    except Exception:
                        pass


# ── Main window ───────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Moveo Arm — Simple Controller")
        self.resize(820, 680)

        self._worker = Worker()
        self._worker.log.connect(self._append_log)
        self._worker.status.connect(self._on_status)

        self._angles = [0.0] * 5
        self._send_lock  = threading.Lock()
        self._in_flight  = False
        self._pending: list[float] | None = None
        self._estop = False

        self._build_ui()

    # ── UI construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        vlay = QVBoxLayout(root)
        vlay.setSpacing(6)

        # Connection bar
        conn_box = QGroupBox("Connection")
        h = QHBoxLayout(conn_box)
        self._btn_connect    = QPushButton("Connect SSH")
        self._btn_disconnect = QPushButton("Disconnect")
        self._btn_disconnect.setEnabled(False)
        self._lbl_status     = QLabel("disconnected")
        self._btn_connect.clicked.connect(self._do_connect)
        self._btn_disconnect.clicked.connect(self._do_disconnect)
        h.addWidget(self._btn_connect)
        h.addWidget(self._btn_disconnect)
        h.addWidget(self._lbl_status, 1)
        vlay.addWidget(conn_box)

        # Joint sliders
        joints_box = QGroupBox("Joint Control")
        grid = QGridLayout(joints_box)
        self._sliders:   list[QSlider]   = []
        self._textboxes: list[QLineEdit] = []
        self._set_btns:  list[QPushButton] = []

        for i, j in enumerate(JOINTS):
            lbl = QLabel(j.label)
            lbl.setFixedWidth(130)

            sl = QSlider(Qt.Horizontal)
            sl.setMinimum(int(j.min_rad * SLIDER_SCALE))
            sl.setMaximum(int(j.max_rad * SLIDER_SCALE))
            sl.setValue(0)
            sl.valueChanged.connect(lambda v, idx=i: self._slider_moved(idx, v))

            tb = QLineEdit("0.000")
            tb.setFixedWidth(80)
            tb.returnPressed.connect(lambda idx=i: self._text_entered(idx))

            unit = QLabel("rad")

            btn = QPushButton(f"Set J{i+1}")
            btn.setFixedWidth(75)
            btn.clicked.connect(lambda _, idx=i: self._text_entered(idx))

            grid.addWidget(lbl,  i, 0)
            grid.addWidget(sl,   i, 1)
            grid.addWidget(tb,   i, 2)
            grid.addWidget(unit, i, 3)
            grid.addWidget(btn,  i, 4)

            self._sliders.append(sl)
            self._textboxes.append(tb)
            self._set_btns.append(btn)

        vlay.addWidget(joints_box)

        # Action buttons
        action_row = QHBoxLayout()
        self._btn_send_all = QPushButton("Send All Joints")
        self._btn_send_all.setStyleSheet("font-weight:bold; padding:6px;")
        self._btn_send_all.clicked.connect(self._send_now)

        self._btn_home = QPushButton("Go Home (0,0,0,0,0)")
        self._btn_home.clicked.connect(self._go_home)

        self._btn_estop = QPushButton("⛔  E-STOP")
        self._btn_estop.setStyleSheet("background:#c0392b; color:white; font-weight:bold; padding:6px;")
        self._btn_estop.clicked.connect(self._estop_toggle)

        action_row.addWidget(self._btn_send_all)
        action_row.addWidget(self._btn_home)
        action_row.addStretch()
        action_row.addWidget(self._btn_estop)
        vlay.addLayout(action_row)

        # Console
        console_box = QGroupBox("Console")
        cv = QVBoxLayout(console_box)
        self._console = QPlainTextEdit()
        self._console.setReadOnly(True)
        self._console.setMaximumBlockCount(500)
        self._console.setStyleSheet("background:#1e1e1e; color:#d4d4d4; font-family:Menlo,monospace; font-size:12px;")
        self._console.setFixedHeight(220)
        cv.addWidget(self._console)
        btn_clear = QPushButton("Clear")
        btn_clear.setFixedWidth(70)
        btn_clear.clicked.connect(self._console.clear)
        cv.addWidget(btn_clear, alignment=Qt.AlignRight)
        vlay.addWidget(console_box)

        self._set_controls_enabled(False)

    # ── slot helpers ──────────────────────────────────────────────────────────
    def _append_log(self, text: str):
        self._console.appendPlainText(text)

    def _on_status(self, s: str):
        self._lbl_status.setText(s)
        connected = (s == "connected")
        self._btn_connect.setEnabled(not connected)
        self._btn_disconnect.setEnabled(connected)
        self._set_controls_enabled(connected and not self._estop)
        if connected:
            self._lbl_status.setStyleSheet("color:#2ecc71; font-weight:bold;")
        else:
            self._lbl_status.setStyleSheet("color:#e74c3c;")

    def _set_controls_enabled(self, enabled: bool):
        for w in (self._sliders + self._textboxes + self._set_btns +
                  [self._btn_send_all, self._btn_home]):
            w.setEnabled(enabled)

    # ── connection actions ────────────────────────────────────────────────────
    def _do_connect(self):
        self._btn_connect.setEnabled(False)
        self._lbl_status.setText("connecting…")
        t = threading.Thread(target=self._worker.connect, daemon=True)
        t.start()

    def _do_disconnect(self):
        self._worker.disconnect()

    # ── slider / text input ───────────────────────────────────────────────────
    def _slider_moved(self, idx: int, value: int):
        angle = value / SLIDER_SCALE
        self._angles[idx] = angle
        self._textboxes[idx].blockSignals(True)
        self._textboxes[idx].setText(f"{angle:.3f}")
        self._textboxes[idx].blockSignals(False)
        # auto-send on drag (coalesced)
        self._dispatch(list(self._angles))

    def _text_entered(self, idx: int):
        try:
            angle = float(self._textboxes[idx].text())
        except ValueError:
            QMessageBox.warning(self, "Bad input", "Enter a number in radians.")
            return
        j = JOINTS[idx]
        angle = max(j.min_rad, min(j.max_rad, angle))
        self._angles[idx] = angle
        self._textboxes[idx].setText(f"{angle:.3f}")
        self._sliders[idx].blockSignals(True)
        self._sliders[idx].setValue(int(angle * SLIDER_SCALE))
        self._sliders[idx].blockSignals(False)
        self._dispatch(list(self._angles))

    def _send_now(self):
        self._dispatch(list(self._angles), force=True)

    def _go_home(self):
        self._angles = [0.0] * 5
        for i in range(5):
            self._sliders[i].blockSignals(True)
            self._sliders[i].setValue(0)
            self._sliders[i].blockSignals(False)
            self._textboxes[i].setText("0.000")
        self._dispatch([0.0] * 5, force=True)

    # ── send pipeline (coalescing) ────────────────────────────────────────────
    def _dispatch(self, angles: list[float], force: bool = False):
        if self._estop or not self._worker.connected:
            return
        with self._send_lock:
            if self._in_flight:
                self._pending = angles
                return
            self._in_flight = True
        threading.Thread(target=self._send_worker, args=(angles,), daemon=True).start()

    def _send_worker(self, angles: list[float]):
        try:
            self._worker.send_angles(angles)
        finally:
            with self._send_lock:
                pending = self._pending
                self._pending = None
                if pending is not None:
                    self._in_flight = True
                else:
                    self._in_flight = False
            if pending is not None:
                threading.Thread(target=self._send_worker, args=(pending,), daemon=True).start()

    # ── E-STOP ────────────────────────────────────────────────────────────────
    def _estop_toggle(self):
        self._estop = not self._estop
        if self._estop:
            self._btn_estop.setText("✅  Resume")
            self._btn_estop.setStyleSheet("background:#e67e22; color:white; font-weight:bold; padding:6px;")
            self._set_controls_enabled(False)
            self._append_log("[ESTOP] all motion frozen")
        else:
            self._btn_estop.setText("⛔  E-STOP")
            self._btn_estop.setStyleSheet("background:#c0392b; color:white; font-weight:bold; padding:6px;")
            self._set_controls_enabled(self._worker.connected)
            self._append_log("[ESTOP] resumed")


# ── entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
