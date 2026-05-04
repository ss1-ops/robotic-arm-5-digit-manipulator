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
_L_BASE  = 0.173   # base plate to shoulder pivot (vertical)
_L_UPPER = 0.228   # shoulder to elbow pivot (upper arm)
_L_FORE  = 0.136   # elbow to wrist pitch pivot (forearm)
_L_EE    = 0.100   # wrist pitch pivot to end-effector tip

if _IKPY_AVAILABLE:
    MOVEO_CHAIN = Chain(
        name="moveo",
        active_links_mask=[False, True, True, True, True, True, False],
        links=[
            OriginLink(),
            # J1: waist — rotates about Z
            URDFLink("j1", origin_translation=[0, 0, _L_BASE],  origin_orientation=[0,0,0], rotation=[0,0,1], bounds=(-3.14, 3.14)),
            # J2: shoulder — rotates about Y
            URDFLink("j2", origin_translation=[0, 0, 0],        origin_orientation=[0,0,0], rotation=[0,1,0], bounds=(-1.57, 1.57)),
            # J3: elbow — upper arm extends along X
            URDFLink("j3", origin_translation=[_L_UPPER, 0, 0], origin_orientation=[0,0,0], rotation=[0,1,0], bounds=(-1.57, 1.57)),
            # J4: wrist roll — forearm extends along X
            URDFLink("j4", origin_translation=[_L_FORE, 0, 0],  origin_orientation=[0,0,0], rotation=[1,0,0], bounds=(-3.14, 3.14)),
            # J5: wrist pitch
            URDFLink("j5", origin_translation=[0, 0, 0],        origin_orientation=[0,0,0], rotation=[0,1,0], bounds=(-1.57, 1.57)),
            # End effector (passive — not optimised)
            URDFLink("ee", origin_translation=[_L_EE, 0, 0],    origin_orientation=[0,0,0], rotation=[0,0,0], bounds=(0, 0)),
        ],
    )
    _ik_warm = [0.0] * 7   # warm-start for IK solver (updated after each solve)
else:
    MOVEO_CHAIN = None
    _ik_warm   = None


def solve_ik(xyz: list) -> list:
    """Return [j1..j5] joint angles for Cartesian target xyz (metres)."""
    global _ik_warm
    if not _IKPY_AVAILABLE or MOVEO_CHAIN is None:
        raise RuntimeError("ikpy is not installed on this Pi")
    result    = MOVEO_CHAIN.inverse_kinematics(target_position=xyz, initial_position=_ik_warm)
    _ik_warm  = list(result)
    return list(result[1:6])   # drop OriginLink [0] and ee [6]


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
        self.get_logger().info("moveo_publisher ready, advertising /joint_commands + /speed_scale")

    def publish(self, angles: list):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = JOINT_NAMES
        msg.position = [float(a) for a in angles]
        self._pub.publish(msg)
        self.get_logger().info(f"published {[f'{a:.3f}' for a in angles]}")

    def publish_speed(self, scale: float):
        scale = max(0.0, min(1.0, scale))
        msg = Float32()
        msg.data = scale
        self._speed_pub.publish(msg)
        self.get_logger().info(f"speed_scale → {scale:.2f}")


def serve(node: JointPublisherNode):
    """TCP server — one persistent connection at a time, newline-delimited JSON."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((TCP_HOST, TCP_PORT))
    srv.listen(1)
    print(f"[socket] listening on {TCP_HOST}:{TCP_PORT}", flush=True)

    while True:
        conn, addr = srv.accept()
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
                        # Optional speed_scale update (0.0–1.0)
                        if "speed" in data:
                            node.publish_speed(float(data["speed"]))
                        # Cartesian IK command: {"cartesian": [x, y, z]}  (metres)
                        if "cartesian" in data:
                            angles = solve_ik(data["cartesian"])
                            node.publish(angles)
                        # Direct joint-angle command
                        elif "position" in data:
                            angles = data["position"]
                            if len(angles) != 5:
                                raise ValueError(f"expected 5 positions, got {len(angles)}")
                            node.publish(angles)
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
