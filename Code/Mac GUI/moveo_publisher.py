#!/usr/bin/env python3
"""
moveo_publisher.py  —  Persistent joint-command publisher for the Moveo arm.

Runs on the Pi as a combined ROS2 node + TCP socket server.
The Mac GUI connects once and sends angle arrays as JSON lines:
  {"position": [j1, j2, j3, j4, j5]}

The node publishes each received message to /joint_commands immediately,
with no DDS startup delay (the publisher is already warm and matched).

Usage:
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  source /opt/ros/jazzy/setup.bash
  python3 ~/ros_nodes/moveo_publisher.py
"""

import json
import socket
import sys
import threading
import time

try:
    import numpy as np
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
    _IKPY_AVAILABLE = True
except ImportError:
    _IKPY_AVAILABLE = False
    print("[IK] ikpy not installed — cartesian commands disabled. Run: pip3 install ikpy", flush=True)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5"]
TCP_HOST    = "0.0.0.0"
TCP_PORT    = 9000

# ── Moveo kinematic chain ──────────────────────────────────────────────────────
# Link lengths in METRES — measure from your physical arm and update these.
# Current values are approximate for the BCN3D Moveo / AR2 geometry.
_L_BASE  = 0.230   # base plate to shoulder pivot (vertical)
_L_UPPER = 0.228   # shoulder to elbow pivot (upper arm)
_L_FORE  = 0.235   # elbow to wrist pitch pivot (forearm)
_L_EE    = 0.040   # wrist pitch pivot to end-effector tip

if _IKPY_AVAILABLE:
    MOVEO_CHAIN = Chain(
        name="moveo",
        active_links_mask=[False, True, True, True, True, True, False],
        links=[
            OriginLink(),
            # J1: waist — rotates about Z (yaw, swings arm left/right in X-Y plane)
            URDFLink("j1", origin_translation=[0, 0, _L_BASE],  origin_orientation=[0,0,0], rotation=[0,0,1], bounds=(-2.00, 2.40)),
            # J2: shoulder — rotates about X (pitch, swings upper arm in Y-Z plane)
            #   At home (j2=0) upper arm points straight up along +Z
            URDFLink("j2", origin_translation=[0, 0, 0],        origin_orientation=[0,0,0], rotation=[1,0,0], bounds=(-1.95, 1.95)),
            # J3: elbow — upper arm length along Z; rotates about X
            URDFLink("j3", origin_translation=[0, 0, _L_UPPER], origin_orientation=[0,0,0], rotation=[1,0,0], bounds=(-2.20, 2.20)),
            # J4: wrist roll — forearm length along Z; rotates about Z (roll)
            URDFLink("j4", origin_translation=[0, 0, _L_FORE],  origin_orientation=[0,0,0], rotation=[0,0,1], bounds=(-3.14, 3.14)),
            # J5: wrist pitch — rotates about X
            URDFLink("j5", origin_translation=[0, 0, 0],        origin_orientation=[0,0,0], rotation=[1,0,0], bounds=(-1.75, 1.75)),
            # End effector (passive) — extends upward along Z
            URDFLink("ee", origin_translation=[0, 0, _L_EE],    origin_orientation=[0,0,0], rotation=[0,0,0], bounds=(0, 0)),
        ],
    )
else:
    MOVEO_CHAIN = None

# Max reachable distance from the shoulder pivot (0, 0, _L_BASE)
_MAX_REACH = _L_UPPER + _L_FORE + _L_EE   # 0.464 m


def solve_ik(xyz: list) -> tuple:
    """Return ([j1..j5], fk_err_mm) for Cartesian target xyz (metres).

    Uses analytical 2R warm starts + multi-restart to avoid local minima.
    Raises ValueError if the target is outside the arm's workspace.
    """
    import math as _math
    if not _IKPY_AVAILABLE or MOVEO_CHAIN is None:
        raise RuntimeError("ikpy is not installed on this Pi")

    x, y, z = xyz[0], xyz[1], xyz[2]

    # Reachability check — Euclidean distance from shoulder pivot to target
    dist = _math.sqrt(x*x + y*y + (z - _L_BASE)**2)
    if dist > _MAX_REACH:
        raise ValueError(
            f"target ({x:.3f},{y:.3f},{z:.3f}) is {dist:.3f}m from shoulder — "
            f"exceeds max reach {_MAX_REACH:.3f}m"
        )

    # J1: waist azimuth toward target (prevents 180° flip local minima)
    j1_guess = _math.atan2(y, x) if (abs(x) > 1e-4 or abs(y) > 1e-4) else 0.0

    # 2R analytical pre-solve for J2, J3 warm starts (J5=0, wrist aligned).
    # Geometry: vertical chain — at home both links point along +Z.
    # Positive j2/j3 (right-hand rule about +X) swings the arm toward +Y.
    # Angles measured from +Z (vertical), so:
    #   j2 = atan2(r_h, dz) - atan2(L2*sin_j3, L1 + L2*cos_j3)
    #   j3 = signed elbow bend angle
    r_h    = _math.sqrt(x*x + y*y)   # horizontal distance to target
    dz_sh  = z - _L_BASE              # height above shoulder pivot
    L1, L2 = _L_UPPER, _L_FORE + _L_EE
    cos_j3 = max(-1.0, min(1.0, (r_h**2 + dz_sh**2 - L1**2 - L2**2) / (2*L1*L2)))
    sin_j3_abs = _math.sqrt(max(0.0, 1.0 - cos_j3**2))
    gamma  = _math.atan2(r_h, dz_sh)  # angle of target from vertical (+Z)

    warm_starts = []
    for sign in [1, -1]:   # elbow-forward and elbow-backward
        sj3 = sign * sin_j3_abs
        j3  = _math.atan2(sj3, cos_j3)
        j2  = gamma - _math.atan2(L2 * sj3, L1 + L2 * cos_j3)
        j2  = max(-1.94, min(1.94, j2))
        j3  = max(-2.19, min(2.19, j3))
        warm_starts.append([0.0, j1_guess, j2, j3, 0.0, 0.0, 0.0])
    warm_starts.append([0.0, j1_guess, 0.0, 0.0, 0.0, 0.0, 0.0])  # fallback

    best_result, best_err, best_fk = None, float('inf'), (x, y, z)
    for warm in warm_starts:
        try:
            result = MOVEO_CHAIN.inverse_kinematics(target_position=xyz, initial_position=warm)
        except Exception:
            continue
        fk  = MOVEO_CHAIN.forward_kinematics(result)
        ex, ey, ez = fk[0, 3], fk[1, 3], fk[2, 3]
        err = _math.sqrt((ex-x)**2 + (ey-y)**2 + (ez-z)**2)
        if err < best_err:
            best_err, best_result, best_fk = err, result, (ex, ey, ez)

    # For targets on the Z axis (x≈0, y≈0), J1 is degenerate — force to 0
    if abs(x) < 0.01 and abs(y) < 0.01:
        best_result = list(best_result)
        best_result[1] = 0.0

    fk_err_mm = best_err * 1000
    print(
        f"[IK] target=({x:.3f},{y:.3f},{z:.3f}) "
        f"fk=({best_fk[0]:.3f},{best_fk[1]:.3f},{best_fk[2]:.3f}) err={fk_err_mm:.1f}mm",
        flush=True,
    )
    if fk_err_mm > 20:
        print(f"[IK] WARNING: large FK error ({fk_err_mm:.1f}mm) — arm geometry may need calibration", flush=True)

    return list(best_result[1:6]), fk_err_mm   # drop OriginLink [0] and ee [6]


class JointPublisherNode(Node):
    def __init__(self):
        super().__init__("moveo_publisher")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(JointState, "/joint_commands", qos)
        self._speed_pub = self.create_publisher(Float32, "/speed_scale", qos)
        self._home_pub  = self.create_publisher(Float32, "/home_cmd", qos)
        self.get_logger().info("moveo_publisher ready, advertising /joint_commands + /speed_scale + /home_cmd")

    def publish(self, angles: list):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = JOINT_NAMES
        msg.position = [float(a) for a in angles]
        self._pub.publish(msg)
        self.get_logger().info(f"published {[f'{a:.3f}' for a in angles]}")

    def publish_home(self):
        msg = Float32()
        msg.data = 1.0
        self._home_pub.publish(msg)
        self.get_logger().info("published /home_cmd")

    def publish_speed(self, scale: float):
        scale = max(0.0, min(1.0, scale))
        msg = Float32()
        msg.data = scale
        self._speed_pub.publish(msg)
        self.get_logger().info(f"speed_scale → {scale:.2f}")


def serve(node: JointPublisherNode):
    """TCP server — accepts multiple simultaneous connections, each in its own thread.
    A lock ensures only one client publishes joint commands at a time."""
    _publish_lock = threading.Lock()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Retry bind in case a previous process is still releasing the port
    for attempt in range(10):
        try:
            srv.bind((TCP_HOST, TCP_PORT))
            break
        except OSError as e:
            if attempt < 9:
                print(f"[socket] bind failed ({e}), retrying in 1s...", flush=True)
                time.sleep(1)
            else:
                raise
    srv.listen(5)
    print(f"[socket] listening on {TCP_HOST}:{TCP_PORT}", flush=True)

    def handle_client(conn, addr):
        print(f"[socket] client connected: {addr}", flush=True)
        buf = ""
        try:
            while True:
                chunk = conn.recv(4096).decode("utf-8", errors="replace")
                if not chunk:
                    break
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        data = json.loads(line)
                        # Optional speed_scale update — may accompany other commands
                        if "speed" in data:
                            with _publish_lock:
                                node.publish_speed(float(data["speed"]))
                        # Home command: {"home": true}  — zero ESP32 position tracking
                        if "home" in data and data["home"]:
                            with _publish_lock:
                                node.publish_home()
                            conn.sendall(b'{"ok":true}\n')
                        # Cartesian IK command: {"cartesian": [x, y, z]}  (metres)
                        elif "cartesian" in data:
                            angles, fk_err_mm = solve_ik(data["cartesian"])
                            with _publish_lock:
                                node.publish(angles)
                            ack = json.dumps({"ok": True, "angles": [round(a, 4) for a in angles], "fk_err_mm": round(fk_err_mm, 1)}) + "\n"
                            conn.sendall(ack.encode())
                        # Direct joint-angle command
                        elif "position" in data:
                            angles = data["position"]
                            if len(angles) != 5:
                                raise ValueError(f"expected 5 positions, got {len(angles)}")
                            with _publish_lock:
                                node.publish(angles)
                            conn.sendall(b'{"ok":true}\n')
                        else:
                            conn.sendall(b'{"ok":true}\n')
                    except Exception as e:
                        msg = json.dumps({"ok": False, "error": str(e)}) + "\n"
                        conn.sendall(msg.encode())
                        print(f"[socket] error: {e}", flush=True)
        except Exception as e:
            print(f"[socket] connection error: {e}", flush=True)
        finally:
            conn.close()
            print(f"[socket] client disconnected: {addr}", flush=True)

    while True:
        conn, addr = srv.accept()
        t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
        t.start()


def main():
    rclpy.init()
    node = JointPublisherNode()

    # Spin ROS2 in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # TCP server runs in main thread
    try:
        serve(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
