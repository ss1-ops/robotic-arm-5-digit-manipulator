#!/usr/bin/env python3
"""
Moveo Arm - Simple Joint Controller  (socket edition)

On Connect SSH:
  1. Kills old agent, clears SHM, starts fresh micro_ros_agent
  2. Starts moveo_publisher.py on the Pi — a persistent ROS2 node + TCP server
     on port 9000 that publishes to /joint_commands with no per-call startup
  3. Opens an SSH direct-tcpip channel to that port

Sending a command:
  - Writes one JSON line {"position": [...]} to the channel
  - Round-trip ~50ms instead of 3-5s per ros2 topic pub --once call
"""

import json
import math
import shlex
import sys
import threading
import time
import urllib.request
from dataclasses import dataclass

import paramiko
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QLineEdit, QPushButton, QGroupBox, QGridLayout,
    QPlainTextEdit, QMessageBox, QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap, QColor


# ── Joint definitions ──────────────────────────────────────────────────────────
@dataclass
class Joint:
    label: str
    min_rad: float
    max_rad: float

JOINTS = [
    Joint("J1  Waist",       -2.00,  2.40),
    Joint("J2  Shoulder",    -1.95,  1.95),
    Joint("J3  Elbow",       -2.20,  2.20),
    Joint("J4  Wrist Roll",  -3.14,  3.14),
    Joint("J5  Wrist Pitch", -1.75,  1.75),
]

SLIDER_SCALE   = 100   # slider integer units per radian
PUBLISHER_PORT = 9000  # TCP port on the Pi where moveo_publisher.py listens
PI_HOST        = "192.168.1.142"  # Pi IP (mDNS 'armpi.local' resolution is flaky; use IP)
CAMERA_URL     = f"http://{PI_HOST}:8080/"  # MJPEG stream from the Pi camera


# ── Worker ─────────────────────────────────────────────────────────────────────
class Worker(QObject):
    log           = pyqtSignal(str)
    status        = pyqtSignal(str)   # "connected" | "disconnected" | error text
    angles_update = pyqtSignal(list)  # emitted after a cartesian IK response with solved angles

    HOST = PI_HOST
    USER = "armpi"

    # Individual startup commands — run sequentially in connect() via _ssh_step()
    _CMD_KILL = (
        # Use process NAME match (no -f) for micro_ros_agent so pkill doesn't
        # accidentally match and kill the bash shell running this very command.
        # For moveo_publisher (python3 process), match the .py filename which
        # won't appear in our bash command's argv.
        "pkill -9 micro_ros_agent 2>/dev/null; "
        "pkill -9 -f 'moveo_publisher\\.py' 2>/dev/null; "
        "sleep 1; "
        "rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null; "
        'echo "[startup] old processes cleared"'
    )
    _CMD_AGENT = (
        # USB CDC serial transport — firmware uses set_microros_transports()
        # which binds to Serial. Pi side runs the agent against /dev/ttyACM0.
        # If /dev/ttyACM0 is missing the ESP32 either isn't plugged in or
        # hasn't enumerated yet — wait briefly for it.
        "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; "
        "source ~/microros_ws/install/setup.bash; "
        "for i in 1 2 3 4 5; do [ -e /dev/ttyACM0 ] && break; sleep 1; done; "
        "if [ ! -e /dev/ttyACM0 ]; then "
        '  echo "[startup] ERROR: /dev/ttyACM0 not present (ESP32 unplugged?)"; '
        "  exit 1; "
        "fi; "
        "setsid ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v4 "
        "  > /tmp/mra.log 2>&1 </dev/null & "
        "sleep 3; "
        "pgrep -c micro_ros_agent > /dev/null "
        '  && echo "[startup] micro_ros_agent running (serial:/dev/ttyACM0)" '
        '  || { echo "[startup] ERROR: agent not found"; cat /tmp/mra.log; }'
    )
    _CMD_ESP32 = (
        "sleep 4; "
        "grep -m1 \"session established\" /tmp/mra.log "
        '  && echo "[startup] ESP32 connected" '
        '  || echo "[startup] WARNING: ESP32 not yet connected"; '
        "tail -3 /tmp/mra.log"
    )
    _CMD_PUBLISHER = (
        "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; "
        "source /opt/ros/jazzy/setup.bash; "
        "setsid python3 ~/ros_nodes/moveo_publisher.py "
        "  > /tmp/moveo_publisher.log 2>&1 </dev/null & "
        # Poll up to 15 s (1 s intervals) for port 9000 to appear
        "for i in $(seq 1 15); do "
        "  ss -tlnp 2>/dev/null | grep -q :9000 && break; "
        "  sleep 1; "
        "done; "
        "ss -tlnp 2>/dev/null | grep -q :9000 "
        '  && echo "[startup] moveo_publisher ready on :9000" '
        '  || { echo "[startup] ERROR: publisher failed"; cat /tmp/moveo_publisher.log; }'
    )
    _CMD_VISION = (
        # Start the stereo camera node + browser MJPEG stream (detached).
        # Camera streams raw (as-mounted) frames; the stream display-rotates for viewing.
        "source /opt/ros/jazzy/setup.bash; "
        "bash ~/vision/start_vision.sh > /tmp/vision_start.log 2>&1; "
        "sleep 1; "
        "pgrep -f stereo_camera_node > /dev/null "
        '  && echo "[startup] vision up — view: http://armpi.local:8080/" '
        '  || { echo "[startup] WARNING: vision not running"; cat /tmp/vision_start.log; }'
    )

    def __init__(self):
        super().__init__()
        self._ssh: paramiko.SSHClient | None = None
        self._ssh_lock = threading.Lock()
        self._chan = None          # paramiko direct-tcpip channel
        self._chan_lock = threading.Lock()
        self._transport: paramiko.Transport | None = None
        self.connected = False
        self._speed_scale = 1.0   # last sent speed (0.0–1.0)

    # ── connect ───────────────────────────────────────────────────────────────
    def connect(self, password=None):
        try:
            self.log.emit("[SSH] connecting…")
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            c.connect(self.HOST, username=self.USER, password=password,
                      look_for_keys=True, allow_agent=True, timeout=8)
            self._ssh = c
            self._transport = c.get_transport()
            self.log.emit("[SSH] connected — running startup script…")

            self._ssh_step("kill",      self._CMD_KILL,      timeout=8)
            self._ssh_step("agent",     self._CMD_AGENT,     timeout=20)
            self._ssh_step("esp32",     self._CMD_ESP32,     timeout=12)
            self._ssh_step("publisher", self._CMD_PUBLISHER, timeout=20)
            self._ssh_step("vision",    self._CMD_VISION,    timeout=20)

            self._open_channel()
            self.connected = True
            self.status.emit("connected")

        except Exception as e:
            self.log.emit(f"[SSH] connect failed: {e}")
            self.status.emit(f"error: {e}")
            self.connected = False

    def _ssh_step(self, label: str, cmd: str, timeout: int = 15):
        """Run one SSH command and stream its output to the log."""
        out, err, code = self._exec(cmd, timeout=timeout)
        combined = (out + err).strip()
        if combined:
            for line in combined.splitlines():
                self.log.emit(f"  {line}")
        elif code != 0:
            self.log.emit(f"  [startup/{label}] exited {code} (no output)")

    def _open_channel(self):
        try:
            if self._chan:
                try: self._chan.close()
                except Exception: pass
            chan = self._transport.open_channel(
                "direct-tcpip",
                ("127.0.0.1", PUBLISHER_PORT),
                ("127.0.0.1", 0),
            )
            self._chan = chan
            self.log.emit(f"[tunnel] channel open to Pi:{PUBLISHER_PORT}")
        except Exception as e:
            self._chan = None
            self.log.emit(f"[tunnel] ERROR: {e}")

    def disconnect(self):
        self.connected = False
        with self._chan_lock:
            if self._chan:
                try: self._chan.close()
                except Exception: pass
                self._chan = None
        if self._ssh:
            try: self._ssh.close()
            except Exception: pass
            self._ssh = None
        self.status.emit("disconnected")
        self.log.emit("[SSH] disconnected")

    # ── send angles ───────────────────────────────────────────────────────────
    def send_angles(self, angles: list, speed: float | None = None):
        if not self.connected:
            self.log.emit("[tx] SKIP — not connected")
            return

        data: dict = {"position": [round(a, 4) for a in angles]}
        if speed is not None:
            s = max(0.01, min(1.0, speed))
            data["speed"] = round(s, 3)
            self._speed_scale = s
        payload = json.dumps(data) + "\n"
        pos_str = "[" + ", ".join(f"{a:.3f}" for a in angles) + "]"
        spd_str = f" speed={data['speed']:.0%}" if "speed" in data else ""
        self.log.emit(f"[tx] {pos_str}{spd_str}")
        self._send_payload(payload)

    def _send_payload(self, payload: str) -> dict | None:
        """Send a raw JSON payload and wait for ack. Returns parsed response dict or None."""
        with self._chan_lock:
            if self._chan is None or self._chan.closed:
                self.log.emit("[tx] channel closed — reopening…")
                self._open_channel()
                if self._chan is None:
                    self.log.emit("[tx] ERROR: could not reopen channel")
                    return None
            try:
                self._chan.sendall(payload.encode())
                self._chan.settimeout(2.0)
                ack_buf = b""
                while b"\n" not in ack_buf:
                    chunk = self._chan.recv(256)
                    if not chunk:
                        break
                    ack_buf += chunk
                resp = json.loads(ack_buf.strip())
                if resp.get("ok"):
                    if "angles" in resp:
                        degs = ", ".join(f"{math.degrees(a):.1f}°" for a in resp["angles"])
                        fk_mm = resp.get("fk_err_mm")
                        suffix = f" (err {fk_mm:.1f}mm)" if fk_mm is not None else ""
                        self.log.emit(f"[IK] joints: {degs}{suffix}")
                    else:
                        self.log.emit("[ack] ok")
                else:
                    self.log.emit(f"[ack] error: {resp.get('error')}")
                return resp
            except Exception as e:
                self.log.emit(f"[tx] ERROR: {e} — will reopen channel on next send")
                try: self._chan.close()
                except Exception: pass
                self._chan = None
                return None

    def send_speed(self, scale: float):
        """Send a speed-only update (no position change)."""
        if not self.connected:
            return
        s = max(0.01, min(1.0, scale))
        self._speed_scale = s
        payload = json.dumps({"speed": round(s, 3)}) + "\n"
        self.log.emit(f"[speed] {s:.0%}")
        self._send_payload(payload)

    def send_cartesian(self, xyz: list):
        """Send a Cartesian XYZ target (metres); Pi-side ikpy solves joint angles."""
        if not self.connected:
            self.log.emit("[IK] SKIP — not connected")
            return
        payload = json.dumps({"cartesian": [round(v, 4) for v in xyz]}) + "\n"
        self.log.emit(f"[IK] target x={xyz[0]:.3f} y={xyz[1]:.3f} z={xyz[2]:.3f} m")
        resp = self._send_payload(payload)
        # Sync GUI sliders with the IK solution returned by the Pi so the next
        # IK call gets the correct current-joint seed (avoids elbow flips).
        if resp and resp.get("ok") and "angles" in resp:
            self.angles_update.emit(resp["angles"])

    def send_home(self):
        """Tell ESP32 that its current physical position is home (zero all counters)."""
        if not self.connected:
            self.log.emit("[home] SKIP — not connected")
            return
        payload = json.dumps({"home": True}) + "\n"
        self.log.emit("[home] sending set-home command")
        self._send_payload(payload)

    # ── SSH exec helper ───────────────────────────────────────────────────────
    def _exec(self, cmd: str, timeout: int = 10):
        full = f"bash -c {shlex.quote(cmd)}"
        with self._ssh_lock:
            stdin = stdout = stderr = None
            try:
                stdin, stdout, stderr = self._ssh.exec_command(full, timeout=timeout)
                out  = stdout.read().decode("utf-8", errors="replace")
                err  = stderr.read().decode("utf-8", errors="replace")
                code = stdout.channel.recv_exit_status()
                return out, err, code
            except Exception as e:
                # Include exception type so socket.timeout (empty str) is visible
                msg = f"{type(e).__name__}: {e}" if str(e) else type(e).__name__
                return "", msg, -1
            finally:
                for s in (stdin, stdout, stderr):
                    try:
                        if s: s.close()
                    except Exception: pass

    def stream_approach_log(self):
        """Tail ~/approach.log over SSH and mirror goto_object's progress into the
        GUI log until it prints the finish sentinel (or a safety timeout)."""
        path = f"/home/{self.USER}/approach.log"
        try:
            with self._ssh_lock:
                ch = self._ssh.get_transport().open_session()
                ch.exec_command(f"tail -n +1 -F {path} 2>/dev/null")
            ch.settimeout(2.0)
            buf = ""; t0 = time.time(); done = False
            while not done and time.time() - t0 < 180:
                try:
                    data = ch.recv(4096).decode("utf-8", "replace")
                except Exception:
                    continue   # recv timeout — re-check the overall deadline
                if not data:
                    break
                buf += data
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.rstrip("\r")
                    if not line.strip():
                        continue
                    i = line.find("[goto_object]: ")        # strip the ROS log prefix
                    self.log.emit("[approach] " + (line[i + 15:] if i >= 0 else line))
                    if "goto_object finished" in line:
                        done = True; break
            try: ch.close()
            except Exception: pass
        except Exception as e:
            self.log.emit(f"[approach] (log stream error: {type(e).__name__})")


# ── Camera worker ───────────────────────────────────────────────────────────────
class CameraWorker(QObject):
    """Reads an MJPEG stream over HTTP and emits decoded frames as QImage.

    Runs in its own daemon thread; reconnects automatically on error. JPEG
    frames are located by their SOI/EOI markers so it works with mjpg-streamer
    style multipart streams regardless of the boundary string.
    """
    frame  = pyqtSignal(QImage)
    status = pyqtSignal(str)   # human-readable connection state

    def __init__(self, url: str):
        super().__init__()
        self.url = url
        self._running = False

    def start(self):
        if self._running:
            return
        self._running = True
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._running = False

    def _run(self):
        while self._running:
            try:
                self.status.emit("connecting…")
                stream = urllib.request.urlopen(self.url, timeout=5)
                self.status.emit("")
                buf = b""
                while self._running:
                    chunk = stream.read(4096)
                    if not chunk:
                        break               # stream ended — reconnect
                    buf += chunk
                    start = buf.find(b"\xff\xd8")          # JPEG SOI
                    end   = buf.find(b"\xff\xd9", start + 2) if start != -1 else -1
                    if start != -1 and end != -1:
                        jpg = buf[start:end + 2]
                        buf = buf[end + 2:]
                        img = QImage.fromData(jpg, "JPG")
                        if not img.isNull():
                            self.frame.emit(img)
                    # Guard against runaway growth if markers never align
                    if len(buf) > 2_000_000:
                        buf = buf[-1_000_000:]
            except Exception as e:
                if self._running:
                    self.status.emit(f"offline — {e}")
                    time.sleep(2)


# ── Clickable camera label ──────────────────────────────────────────────────────
class ClickableLabel(QLabel):
    """QLabel that emits the click position (in pixmap pixels) on mouse press."""
    clicked = pyqtSignal(int, int)

    def mousePressEvent(self, e):
        self.clicked.emit(int(e.x()), int(e.y()))


# ── Main window ───────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Moveo Arm — Simple Controller")
        self.resize(1140, 700)

        self._worker = Worker()
        self._worker.log.connect(self._append_log)
        self._worker.status.connect(self._on_status)
        self._worker.angles_update.connect(self._on_ik_angles)

        self._angles    = [0.0] * 5
        self._speed_pct = 100       # 1–100 %
        self._send_lock = threading.Lock()
        self._in_flight = False
        self._pending   = None
        self._estop     = False

        self._build_ui()

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
        self._sliders   = []
        self._textboxes = []
        self._set_btns  = []

        for i, j in enumerate(JOINTS):
            lbl = QLabel(j.label)
            lbl.setFixedWidth(130)

            sl = QSlider(Qt.Horizontal)
            sl.setMinimum(int(j.min_rad * SLIDER_SCALE))
            sl.setMaximum(int(j.max_rad * SLIDER_SCALE))
            sl.setValue(0)
            sl.valueChanged.connect(lambda v, idx=i: self._slider_preview(idx, v))
            sl.sliderReleased.connect(lambda idx=i: self._slider_released(idx))

            tb = QLineEdit("0.000")
            tb.setFixedWidth(80)
            tb.returnPressed.connect(lambda idx=i: self._text_entered(idx))

            unit = QLabel("rad")
            btn  = QPushButton(f"Set J{i+1}")
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

        # Speed control
        speed_box = QGroupBox("Speed")
        sh = QHBoxLayout(speed_box)
        sh.addWidget(QLabel("Slow"))
        self._speed_slider = QSlider(Qt.Horizontal)
        self._speed_slider.setMinimum(1)
        self._speed_slider.setMaximum(100)
        self._speed_slider.setValue(100)
        self._speed_slider.setTickInterval(10)
        self._speed_slider.setTickPosition(QSlider.TicksBelow)
        self._speed_slider.valueChanged.connect(self._speed_preview)
        self._speed_slider.sliderReleased.connect(self._speed_released)
        sh.addWidget(self._speed_slider, 1)
        sh.addWidget(QLabel("Fast"))
        self._lbl_speed = QLabel("100%")
        self._lbl_speed.setFixedWidth(45)
        self._lbl_speed.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        sh.addWidget(self._lbl_speed)
        vlay.addWidget(speed_box)

        # Cartesian IK control
        cart_box = QGroupBox("Cartesian Control (IK)")
        cg = QGridLayout(cart_box)
        self._cart_fields = {}
        defaults = {"X": "0.250", "Y": "0.000", "Z": "0.300"}
        for col, (axis, default) in enumerate(defaults.items()):
            cg.addWidget(QLabel(f"{axis} (m)"), 0, col * 2)
            tb = QLineEdit(default)
            tb.setFixedWidth(80)
            tb.returnPressed.connect(self._send_cartesian_clicked)
            cg.addWidget(tb, 0, col * 2 + 1)
            self._cart_fields[axis] = tb
        self._btn_ik = QPushButton("Solve IK → Move")
        self._btn_ik.setStyleSheet("font-weight:bold; padding:4px 10px;")
        self._btn_ik.clicked.connect(self._send_cartesian_clicked)
        cg.addWidget(self._btn_ik, 0, 6)
        vlay.addWidget(cart_box)

        # Action buttons
        action_row = QHBoxLayout()
        self._btn_send_all = QPushButton("Send All Joints")
        self._btn_send_all.setStyleSheet("font-weight:bold; padding:6px;")
        self._btn_send_all.clicked.connect(self._send_now)

        self._btn_home = QPushButton("Go Home (0,0,0,0,0)")
        self._btn_home.clicked.connect(self._go_home)

        self._btn_set_home = QPushButton("📍 Set as Home")
        self._btn_set_home.setToolTip(
            "Physically position the arm at its home pose, then click here.\n"
            "Tells the ESP32 'you are now at zero' — resets internal counters."
        )
        self._btn_set_home.clicked.connect(self._set_as_home)

        self._btn_stop_approach = QPushButton("✋ Stop Approach")
        self._btn_stop_approach.setToolTip("Stop the click-to-go visual approach")
        self._btn_stop_approach.clicked.connect(self._stop_approach)

        self._btn_estop = QPushButton("⛔  E-STOP")
        self._btn_estop.setStyleSheet(
            "background:#c0392b; color:white; font-weight:bold; padding:6px;"
        )
        self._btn_estop.clicked.connect(self._estop_toggle)

        action_row.addWidget(self._btn_send_all)
        action_row.addWidget(self._btn_home)
        action_row.addWidget(self._btn_set_home)
        action_row.addStretch()
        action_row.addWidget(self._btn_stop_approach)
        action_row.addWidget(self._btn_estop)
        vlay.addLayout(action_row)

        # Bottom row: console (left half) + camera viewer (right half)
        bottom_row = QHBoxLayout()

        # Console
        console_box = QGroupBox("Console")
        cv = QVBoxLayout(console_box)
        self._console = QPlainTextEdit()
        self._console.setReadOnly(True)
        self._console.setMaximumBlockCount(500)
        self._console.setStyleSheet(
            "background:#1e1e1e; color:#d4d4d4; font-family:Menlo,monospace; font-size:12px;"
        )
        self._console.setMinimumHeight(220)
        cv.addWidget(self._console)
        btn_clear = QPushButton("Clear")
        btn_clear.setFixedWidth(70)
        btn_clear.clicked.connect(self._console.clear)
        cv.addWidget(btn_clear, alignment=Qt.AlignRight)
        bottom_row.addWidget(console_box, 1)

        # Camera viewer — width hugs the stream (no side letterboxing)
        cam_box = QGroupBox("Camera")
        cam_box.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Preferred)
        camv = QVBoxLayout(cam_box)
        self._cam_label = ClickableLabel("camera:\nclick an object to go to it")
        self._cam_label.clicked.connect(self._on_camera_click)
        self._cam_label.setCursor(Qt.CrossCursor)
        self._cam_label.setAlignment(Qt.AlignCenter)
        self._cam_label.setMinimumSize(160, 120)
        self._cam_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Ignored)
        self._cam_label.setStyleSheet("background:#000; color:#888;")
        camv.addWidget(self._cam_label)
        bottom_row.addWidget(cam_box, 0)

        vlay.addLayout(bottom_row)

        self._set_controls_enabled(False)

        # Start the camera stream (independent of the SSH/ROS connection)
        self._last_frame: QImage | None = None
        self._camera = CameraWorker(CAMERA_URL)
        self._camera.frame.connect(self._on_camera_frame)
        self._camera.status.connect(self._on_camera_status)
        self._camera.start()

    def _on_camera_frame(self, img: QImage):
        self._last_frame = img
        self._update_camera_pixmap()

    def _on_camera_status(self, msg: str):
        if msg:
            self._cam_label.setText(f"camera: {msg}")

    def _update_camera_pixmap(self):
        if self._last_frame is None or self._last_frame.isNull():
            return
        # Lock the panel to a 4:3 box (width = height * 4/3, driven by the
        # console height). Fill it by expanding + centre-cropping so there are
        # no black side bars and no distortion.
        h = max(self._cam_label.height(), 120)
        w = round(h * 4 / 3)
        self._cam_label.setFixedWidth(w)
        pm = QPixmap.fromImage(self._last_frame).scaled(
            w, h, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation
        )
        if pm.width() != w or pm.height() != h:
            x = (pm.width()  - w) // 2
            y = (pm.height() - h) // 2
            pm = pm.copy(x, y, w, h)
        self._cam_label.setPixmap(pm)

    # ── Click-to-go visual approach ─────────────────────────────────────────────
    def _on_camera_click(self, x, y):
        if not self._worker.connected:
            self._append_log("[approach] connect SSH first"); return
        if self._estop:
            self._append_log("[approach] release E-STOP first"); return
        lo, hi = self._sample_hsv(x, y)
        if lo is None:
            self._append_log("[approach] click directly on a COLORED object"); return
        self._append_log(f"[approach] going to object  HSV {lo}-{hi}, stop ~22cm  (✋ to abort)")
        angles = [self._sliders[i].value() / SLIDER_SCALE for i in range(len(JOINTS))]
        cmd = (f"bash ~/vision/run_approach_object.sh "
               f"{lo[0]} {lo[1]} {lo[2]} {hi[0]} {hi[1]} {hi[2]} 0.22")

        def _go():
            # Move to the viewing pose FIRST so it's the publisher's last command;
            # the publisher re-publishes it at 2 Hz, so the approach node picks it
            # up as q0 as soon as it subscribes. Do NOT send again after launch —
            # that would fight the Jacobian probe once the servo starts moving.
            self._worker.send_angles(angles)
            time.sleep(0.5)
            self._worker._exec(cmd, timeout=15)
            self._worker.stream_approach_log()   # mirror goto_object's steps into the GUI log
        threading.Thread(target=_go, daemon=True).start()

    def _sample_hsv(self, x, y):
        """Average a small patch at the click and return OpenCV HSV lo/hi range."""
        pm = self._cam_label.pixmap()
        if pm is None or pm.isNull():
            return None, None
        img = pm.toImage()
        w, h = img.width(), img.height()
        if not (0 <= x < w and 0 <= y < h):
            return None, None
        r = g = b = n = 0
        for dx in range(-3, 4):
            for dy in range(-3, 4):
                px, py = x + dx, y + dy
                if 0 <= px < w and 0 <= py < h:
                    c = img.pixelColor(px, py)
                    r += c.red(); g += c.green(); b += c.blue(); n += 1
        col = QColor(r // n, g // n, b // n)
        hh, ss, vv, _ = col.getHsv()            # h 0-359 (-1 if gray), s/v 0-255
        if hh < 0 or ss < 40:
            return None, None                   # achromatic — not color-trackable
        hcv = hh // 2                           # OpenCV hue 0-179
        dh, ds, dv = 12, 90, 90
        lo = [max(0, hcv - dh), max(0, ss - ds), max(0, vv - dv)]
        hi = [min(179, hcv + dh), min(255, ss + ds), min(255, vv + dv)]
        return lo, hi

    def _stop_approach(self):
        if self._worker.connected:
            threading.Thread(target=lambda: self._worker._exec(
                "pkill -9 -f 'approach_object.py|goto_object.py|approach_servo.py|stereo_depth_node.py' 2>/dev/null", timeout=8),
                daemon=True).start()
        self._append_log("[approach] stop sent")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_camera_pixmap()

    def closeEvent(self, event):
        try:
            self._camera.stop()
        except Exception:
            pass
        super().closeEvent(event)

    def _on_ik_angles(self, angles: list):
        """Sync sliders and internal state after a successful IK command."""
        for i, a in enumerate(angles[:5]):
            self._angles[i] = a
            self._sliders[i].blockSignals(True)
            self._sliders[i].setValue(int(a * SLIDER_SCALE))
            self._sliders[i].blockSignals(False)
            self._textboxes[i].blockSignals(True)
            self._textboxes[i].setText(f"{a:.3f}")
            self._textboxes[i].blockSignals(False)

    def _append_log(self, text: str):
        self._console.appendPlainText(text)

    def _on_status(self, s: str):
        self._lbl_status.setText(s)
        connected = (s == "connected")
        self._btn_connect.setEnabled(not connected)
        self._btn_disconnect.setEnabled(connected)
        self._set_controls_enabled(connected and not self._estop)
        self._lbl_status.setStyleSheet(
            "color:#2ecc71; font-weight:bold;" if connected else "color:#e74c3c;"
        )

    def _set_controls_enabled(self, enabled: bool):
        for w in self._sliders + self._textboxes + self._set_btns + [
            self._btn_send_all, self._btn_home, self._btn_set_home,
            self._speed_slider, self._btn_ik
        ] + list(self._cart_fields.values()):
            w.setEnabled(enabled)

    def _do_connect(self):
        self._btn_connect.setEnabled(False)
        self._lbl_status.setText("connecting…")
        threading.Thread(target=self._worker.connect, daemon=True).start()

    def _do_disconnect(self):
        self._worker.disconnect()

    def _send_cartesian_clicked(self):
        try:
            xyz = [
                float(self._cart_fields["X"].text()),
                float(self._cart_fields["Y"].text()),
                float(self._cart_fields["Z"].text()),
            ]
        except ValueError:
            QMessageBox.warning(self, "Bad input", "X, Y, Z must be numbers in metres.")
            return
        if self._estop or not self._worker.connected:
            return
        threading.Thread(target=self._worker.send_cartesian, args=(xyz,), daemon=True).start()

    def _speed_preview(self, value: int):
        """Update label while dragging — no network send."""
        self._speed_pct = value
        self._lbl_speed.setText(f"{value}%")

    def _speed_released(self):
        """Send speed only after slider is released."""
        value = self._speed_slider.value()
        self._speed_pct = value
        self._lbl_speed.setText(f"{value}%")
        if self._worker.connected and not self._estop:
            threading.Thread(
                target=self._worker.send_speed,
                args=(value / 100.0,),
                daemon=True,
            ).start()

    def _slider_preview(self, idx: int, value: int):
        """Update textbox while dragging — no network send."""
        angle = value / SLIDER_SCALE
        self._angles[idx] = angle
        self._textboxes[idx].blockSignals(True)
        self._textboxes[idx].setText(f"{angle:.3f}")
        self._textboxes[idx].blockSignals(False)

    def _slider_released(self, idx: int):
        """Dispatch command only after slider is released."""
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

    def _set_as_home(self):
        """Declare current physical position as home — zeros ESP32 counters without moving."""
        from PyQt5.QtWidgets import QMessageBox
        reply = QMessageBox.question(
            self,
            "Set as Home",
            "This tells the ESP32 that the arm's CURRENT PHYSICAL POSITION is home (all joints = 0).\n\n"
            "Make sure the arm is already at its desired home pose before confirming.",
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if reply != QMessageBox.Ok:
            return
        # Reset GUI to zero to stay in sync
        self._angles = [0.0] * 5
        for i in range(5):
            self._sliders[i].blockSignals(True)
            self._sliders[i].setValue(0)
            self._sliders[i].blockSignals(False)
            self._textboxes[i].setText("0.000")
        threading.Thread(target=self._worker.send_home, daemon=True).start()

    def _dispatch(self, angles: list, force: bool = False):
        if self._estop or not self._worker.connected:
            return
        with self._send_lock:
            if self._in_flight:
                self._pending = angles
                return
            self._in_flight = True
        threading.Thread(target=self._send_worker, args=(angles,), daemon=True).start()

    def _send_worker(self, angles: list):
        try:
            self._worker.send_angles(angles)
        finally:
            with self._send_lock:
                pending = self._pending
                self._pending = None
                self._in_flight = pending is not None
            if pending is not None:
                threading.Thread(target=self._send_worker, args=(pending,), daemon=True).start()

    def _estop_toggle(self):
        self._estop = not self._estop
        if self._estop:
            self._btn_estop.setText("✅  Resume")
            self._btn_estop.setStyleSheet(
                "background:#e67e22; color:white; font-weight:bold; padding:6px;"
            )
            self._set_controls_enabled(False)
            self._append_log("[ESTOP] all motion frozen")
        else:
            self._btn_estop.setText("⛔  E-STOP")
            self._btn_estop.setStyleSheet(
                "background:#c0392b; color:white; font-weight:bold; padding:6px;"
            )
            self._set_controls_enabled(self._worker.connected)
            self._append_log("[ESTOP] resumed")


# ── entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
