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

# ── Moveo kinematic chain (SINGLE SOURCE OF TRUTH) ─────────────────────────────
# Link lengths + Chain definition live in software/mac-gui/kinematics.py .
#
# On the Pi we prefer to import from a sibling kinematics.py (deployed together)
# so there is truly only one source of truth. If the import fails (standalone
# deployment or old install), we fall back to the embedded definitions below.
#
# When you change lengths, edit ONLY kinematics.py, then re-deploy both files
# to the Pi (kinematics.py + this file) into the same directory (~ /ros_nodes).
try:
    from kinematics import (
        MOVEO_CHAIN,
        L_BASE as _L_BASE,
        L_WAIST as _L_WAIST,
        L_UPPER as _L_UPPER,
        L_FORE as _L_FORE,
        L_WRIST as _L_WRIST,
        L_EE as _L_EE,
        MAX_REACH as _MAX_REACH,
        forward_kinematics,          # returns user frame (+X forward, +Y left)
        chain_to_user,
        user_to_chain,
        forward_kinematics_matrix,   # 4x4 matrix (translation in internal chain frame)
    )
    print("[IK] using kinematics.py (single source of truth) from same directory")
except Exception as _e:
    print(f"[IK] kinematics.py import failed ({_e}) — using embedded definitions")
    # define fallbacks if needed
    def forward_kinematics(joints):
        # fallback raw internal (should not happen if kinematics.py is present)
        full = [0.0] + list(joints) + [0.0]
        fk = MOVEO_CHAIN.forward_kinematics(full)
        cx, cy, cz = fk[0,3], fk[1,3], fk[2,3]
        return -cy, cx, cz
    def chain_to_user(p): cx,cy,cz = p; return -cy, cx, cz
    def user_to_chain(p): x,y,z = p; return y, -x, z
    def forward_kinematics_matrix(joints):
        full = [0.0] + list(joints) + [0.0]
        return MOVEO_CHAIN.forward_kinematics(full)
    # Fallback (must stay in sync with kinematics.py)
    _L_BASE  = 0.20   # base plate to J1 (waist) rotation axis
    _L_WAIST = 0.140  # riser J1→J2 (shoulder)
    _L_UPPER = 0.22   # shoulder (J2) → elbow (J3)
    _L_FORE  = 0.11   # elbow (J3) → wrist-roll (J4)
    _L_WRIST = 0.11   # wrist-roll (J4) → wrist-pitch (J5)
    _L_EE    = 0.06   # wrist-pitch (J5) → EE tip

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
                URDFLink("j2", origin_translation=[0, 0, _L_WAIST], origin_orientation=[0,0,0], rotation=[1,0,0], bounds=(-1.95, 1.95)),
                # J3: elbow — upper arm length along Z; rotates about X
                URDFLink("j3", origin_translation=[0, 0, _L_UPPER], origin_orientation=[0,0,0], rotation=[1,0,0], bounds=(-2.20, 2.20)),
                # J4: wrist roll — forearm length along Z; rotates about Z (roll)
                URDFLink("j4", origin_translation=[0, 0, _L_FORE],  origin_orientation=[0,0,0], rotation=[0,0,1], bounds=(-3.14, 3.14)),
                # J5: wrist pitch — rotates about X
                URDFLink("j5", origin_translation=[0, 0, _L_WRIST], origin_orientation=[0,0,0], rotation=[1,0,0], bounds=(-1.60, 1.60)),
                # End effector (passive) — extends upward along Z
                URDFLink("ee", origin_translation=[0, 0, _L_EE],    origin_orientation=[0,0,0], rotation=[0,0,0], bounds=(0, 0)),
            ],
        )
    else:
        MOVEO_CHAIN = None

    _MAX_REACH = _L_UPPER + _L_FORE + _L_WRIST + _L_EE
    MAX_REACH = _MAX_REACH  # for any code that expects the public name

# Back-compat: ensure _MAX_REACH exists even in import-success path
if "_MAX_REACH" not in dir():
    _MAX_REACH = MAX_REACH if "MAX_REACH" in dir() else (_L_UPPER + _L_FORE + _L_WRIST + _L_EE)


def solve_ik(xyz: list, current_joints: list = None) -> tuple:
    """Return ([j1..j5], fk_err_mm) for Cartesian target xyz (metres).

    FRAME: callers use the robot/REP-103 convention +X=forward, +Y=left, +Z=up.
    (See kinematics.py: user_to_chain / chain_to_user / forward_kinematics for the
    canonical conversion. The internal ikpy chain is rotated 90° for solver
    convenience; all higher-level code should stay in the user frame.)

    When current_joints ([j1..j5] radians) is provided, biases the solution
    toward the current configuration so the arm never flips elbow-up/down
    unexpectedly during continuous motion.
    Raises ValueError if the target is outside the arm's workspace.
    """
    import math as _math
    if not _IKPY_AVAILABLE or MOVEO_CHAIN is None:
        raise RuntimeError("ikpy is not installed on this Pi")

    # User frame → internal chain frame (encapsulated in user_to_chain).
    # Reassign xyz itself so the ikpy call (target_position=xyz) also uses it.
    xyz = list(user_to_chain(xyz))
    x, y, z = xyz[0], xyz[1], xyz[2]

    # Reachability check — Euclidean distance from shoulder pivot to target
    dist = _math.sqrt(x*x + y*y + (z - _L_BASE)**2)
    if dist > _MAX_REACH:
        raise ValueError(
            f"target ({x:.3f},{y:.3f},{z:.3f}) is {dist:.3f}m from shoulder — "
            f"exceeds max reach {_MAX_REACH:.3f}m"
        )

    # J1: waist azimuth toward target. J2 rotates about +Y, so at j1=0 the arm's
    # reach direction is along +X (azimuth 0°). No offset needed: j1_aim is just
    # the azimuth of the target in the model frame.
    if abs(x) > 1e-4 or abs(y) > 1e-4:
        j1_aim = _math.atan2(y, x)
        # wrap to (-pi, pi] then clamp to the reachable waist range
        j1_aim = (j1_aim + _math.pi) % (2 * _math.pi) - _math.pi
        j1_guess = max(-2.00, min(2.40, j1_aim))
    else:
        j1_guess = 0.0

    # 2R analytical pre-solve for J2, J3 warm starts (J5=0, wrist aligned).
    # Geometry: vertical chain — at home both links point along +Z.
    # Positive j2/j3 (right-hand rule about +X) swings the arm toward +Y.
    # Angles measured from +Z (vertical), so:
    #   j2 = atan2(r_h, dz) - atan2(L2*sin_j3, L1 + L2*cos_j3)
    #   j3 = signed elbow bend angle
    r_h    = _math.sqrt(x*x + y*y)   # horizontal distance to target
    # Height above the *shoulder pitch* (J2) axis. The 2R (j2/j3) starts at the J2 pivot,
    # which is L_WAIST above the J1 origin used for _L_BASE in the chain.
    dz_sh  = z - (_L_BASE + _L_WAIST)
    # Effective L2 for 2R analytical warm-start: FORE + WRIST + EE (when wrist straight).
    L1, L2 = _L_UPPER, _L_FORE + _L_WRIST + _L_EE
    cos_j3 = max(-1.0, min(1.0, (r_h**2 + dz_sh**2 - L1**2 - L2**2) / (2*L1*L2)))
    sin_j3_abs = _math.sqrt(max(0.0, 1.0 - cos_j3**2))
    gamma  = _math.atan2(r_h, dz_sh)  # angle of target from vertical (+Z)

    warm_starts = []
    # Seed with current joint state first — this is the continuity bias.
    # ikpy will find the nearest valid solution in the same configuration
    # (elbow-up/down) as the arm is currently in.
    if current_joints is not None and len(current_joints) == 5:
        warm_starts.append([0.0] + list(current_joints) + [0.0])
    for sign in [1, -1]:   # elbow-forward and elbow-backward
        sj3 = sign * sin_j3_abs
        j3  = _math.atan2(sj3, cos_j3)
        j2  = gamma - _math.atan2(L2 * sj3, L1 + L2 * cos_j3)
        j2  = max(-1.94, min(1.94, j2))
        j3  = max(-2.19, min(2.19, j3))
        warm_starts.append([0.0, j1_guess, j2, j3, 0.0, 0.0, 0.0])
    warm_starts.append([0.0, j1_guess, 0.0, 0.0, 0.0, 0.0, 0.0])  # fallback

    # Collect all candidates, filtering out solutions with large FK error.
    # Then choose by joint-space continuity when current_joints is known,
    # otherwise by FK accuracy alone.
    _FK_THRESHOLD_M = 0.050  # 50 mm — reject clearly diverged solutions
    # "naturalness" penalty: a contorted reach-behind pose uses a ~180° wrist
    # roll (j4) and big bends; a clean front pose keeps j4 near 0. Penalising it
    # breaks ties between two FK-equal solutions toward the relaxed front one.
    def _naturalness(res):
        j1_, j2_, j3_, j4_, j5_ = res[1], res[2], res[3], res[4], res[5]
        return abs(j4_) + 0.25 * (abs(j2_) + abs(j3_) + abs(j5_))
    candidates = []          # (fk_err_m, j_dist, naturalness, result, fk_xyz)
    best_result, best_err, best_fk = None, float('inf'), (x, y, z)  # global fallback
    for warm in warm_starts:
        try:
            result = MOVEO_CHAIN.inverse_kinematics(target_position=xyz, initial_position=warm)
        except Exception:
            continue
        fk  = MOVEO_CHAIN.forward_kinematics(result)
        ex, ey, ez = fk[0, 3], fk[1, 3], fk[2, 3]
        fk_err = _math.sqrt((ex-x)**2 + (ey-y)**2 + (ez-z)**2)
        if fk_err < best_err:                      # track best-ever for fallback
            best_err, best_result, best_fk = fk_err, result, (ex, ey, ez)
        if fk_err > _FK_THRESHOLD_M:
            continue
        j_dist = 0.0
        if current_joints is not None:
            j_dist = _math.sqrt(sum((result[k+1] - current_joints[k])**2 for k in range(5)))
        candidates.append((fk_err, j_dist, _naturalness(result), result, (ex, ey, ez)))

    if candidates:
        # When current_joints known, prefer joint-space continuity (avoid
        # elbow-up/down flips); otherwise pick the most natural accurate pose.
        # In both cases bucket FK error to mm so naturalness can break ties
        # between two solutions that both reach the target.
        if current_joints is not None:
            candidates.sort(key=lambda c: (round(c[0] * 1000), c[1], c[2]))
        else:
            candidates.sort(key=lambda c: (round(c[0] * 1000), c[2]))
        best_err, _, _, best_result, best_fk = candidates[0]

    if best_result is None:
        raise ValueError(
            f"IK found no solution for target ({x:.3f},{y:.3f},{z:.3f}) — "
            f"likely unreachable or singular")

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


# Joint bounds (radians) — mirror MOVEO_CHAIN link bounds above
_JOINT_BOUNDS = [(-2.00, 2.40), (-1.95, 1.95), (-2.20, 2.20), (-3.14, 3.14), (-1.60, 1.60)]
# Table geometry: 14 cm tall, 13 cm radius.
# Two-zone floor rule:
#   r <= TABLE_RADIUS  →  EE must stay above the table top (TABLE_HEIGHT)
#   r >  TABLE_RADIUS  →  EE may descend to Z_FLOOR_CLEAR (below table surface)
_TABLE_HEIGHT  = 0.02   # metres — minimum Z over table footprint
_TABLE_RADIUS  = 0.14   # metres — radial clearance boundary (table r=0.13 + 1cm margin)
_Z_FLOOR_CLEAR = -0.10  # metres — floor when outside table footprint


def check_collision(joints: list) -> tuple:
    """Return (ok: bool, reason: str). Checks joint limits and Moveo-specific
    self-collision heuristics. reason is empty string when ok=True."""
    import math as _math

    # 1. Hard joint limit check
    for i, (lo, hi) in enumerate(_JOINT_BOUNDS):
        j = joints[i]
        if j < lo - 1e-3 or j > hi + 1e-3:
            return False, f"J{i+1}={j:.3f} outside limits [{lo:.2f}, {hi:.2f}]"

    # 2. Z-floor: two-zone table collision.
    #   Inside table footprint (r <= _TABLE_RADIUS): floor is table top height.
    #   Outside table footprint (r >  _TABLE_RADIUS): floor drops to _Z_FLOOR_CLEAR.
    if _IKPY_AVAILABLE and MOVEO_CHAIN is not None:
        import math as _math
        full = [0.0] + list(joints) + [0.0]
        fk = MOVEO_CHAIN.forward_kinematics(full)
        ee_x, ee_y, ee_z = fk[0, 3], fk[1, 3], fk[2, 3]
        ee_r = _math.sqrt(ee_x**2 + ee_y**2)
        if ee_r <= _TABLE_RADIUS:
            z_floor = _TABLE_HEIGHT
            if ee_z < z_floor:
                return False, f"end-effector Z={ee_z:.3f}m above table footprint (r={ee_r:.3f}m ≤ {_TABLE_RADIUS}m), min Z={z_floor}m"
        else:
            z_floor = _Z_FLOOR_CLEAR
            if ee_z < z_floor:
                return False, f"end-effector Z={ee_z:.3f}m below floor (r={ee_r:.3f}m > {_TABLE_RADIUS}m), min Z={z_floor}m"

    # 3. Elbow fold-back: J2+J3 < -2.8 rad risks forearm hitting base column
    j2, j3 = joints[1], joints[2]
    if j2 + j3 < -2.8:
        return False, f"elbow fold-back risk: J2+J3={j2+j3:.2f} (limit -2.8 rad)"

    # 4. Wrist-to-forearm: extreme J5 combined with extreme J3
    j5 = joints[4]
    if abs(j5) > 1.4 and abs(j3) > 1.8:
        return False, f"wrist/forearm collision risk: |J5|={abs(j5):.2f} with |J3|={abs(j3):.2f}"

    return True, ""


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
        # Last successfully commanded angles — used to seed IK for continuity.
        # Protected by _joints_lock for thread safety.
        self._last_joints = [0.0] * 5
        self._joints_lock = threading.Lock()
        # Re-publish the last command at a low rate. /joint_commands is VOLATILE
        # (not latched), so a subscriber that comes up AFTER a command (e.g. the
        # approach servo, launched on a GUI click) would otherwise wait forever
        # for the next send. Re-publishing lets it pick up the current pose as
        # soon as it's subscribed. During a servo it just echoes the latest jog
        # command (no conflict). Gated until the first real command so we don't
        # drive the arm to home on startup.
        self._has_command = False
        self.create_timer(0.5, self._republish)

    def _republish(self):
        if not self._has_command:
            return
        with self._joints_lock:
            angles = list(self._last_joints)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = angles
        self._pub.publish(msg)

    def publish(self, angles: list):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = JOINT_NAMES
        msg.position = [float(a) for a in angles]
        with self._joints_lock:
            self._last_joints = list(msg.position)
        self._has_command = True
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
                            xyz_req = list(data["cartesian"])
                            if xyz_req[2] < 0.02:
                                print(f"[socket] Z floor clamp: {xyz_req[2]:.3f} → 0.020 m", flush=True)
                                xyz_req[2] = 0.02
                            with node._joints_lock:
                                curr = list(node._last_joints)
                            angles, fk_err_mm = solve_ik(xyz_req, curr)
                            ok, reason = check_collision(angles)
                            if not ok:
                                raise ValueError(f"collision/limit: {reason}")
                            with _publish_lock:
                                node.publish(angles)
                            # Compute achieved position in *user frame* using the exact angles being sent.
                            # This lets the caller see what the solver's model says the EE will be at
                            # (should closely match the requested target when err is low).
                            full = [0.0] + list(angles) + [0.0]
                            fk_int = MOVEO_CHAIN.forward_kinematics(full)
                            ix, iy, iz = fk_int[0, 3], fk_int[1, 3], fk_int[2, 3]
                            achieved = list(chain_to_user((ix, iy, iz)))  # internal -> user frame
                            ack = json.dumps({
                                "ok": True,
                                "angles": [round(a, 6) for a in angles],   # higher precision to reduce FK recompute error
                                "fk_err_mm": round(fk_err_mm, 1),
                                "achieved": [round(v, 4) for v in achieved]
                            }) + "\n"
                            conn.sendall(ack.encode())
                        # Trajectory command: {"trajectory": [[x,y,z], ...], "dt": 0.2}
                        # Solves IK for each waypoint sequentially — each solution seeds
                        # the next — then publishes at dt-second intervals for smooth
                        # continuous motion without stopping between waypoints.
                        elif "trajectory" in data:
                            waypoints = data["trajectory"]
                            dt = float(data.get("dt", 0.2))
                            dt = max(0.05, min(5.0, dt))  # clamp 50 ms – 5 s
                            if not isinstance(waypoints, list) or len(waypoints) == 0:
                                raise ValueError("trajectory must be a non-empty list of [x,y,z] points")
                            with node._joints_lock:
                                curr = list(node._last_joints)
                            results = []
                            for i, wp in enumerate(waypoints):
                                angles, fk_err = solve_ik(wp, curr)
                                ok, reason = check_collision(angles)
                                if not ok:
                                    raise ValueError(f"waypoint {i}: {reason}")
                                with _publish_lock:
                                    node.publish(angles)
                                curr = angles   # seed next waypoint from this solution
                                results.append({"angles": [round(a, 4) for a in angles],
                                                "fk_err_mm": round(fk_err, 1)})
                                if i < len(waypoints) - 1:
                                    time.sleep(dt)
                            ack = json.dumps({"ok": True, "waypoints": len(waypoints),
                                              "results": results}) + "\n"
                            conn.sendall(ack.encode())
                        # Direct joint-angle command
                        elif "position" in data:
                            angles = data["position"]
                            if len(angles) != 5:
                                raise ValueError(f"expected 5 positions, got {len(angles)}")
                            ok, reason = check_collision(angles)
                            if not ok:
                                raise ValueError(f"collision/limit: {reason}")
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
