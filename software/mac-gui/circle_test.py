#!/usr/bin/env python3
"""
circle_test.py — Command the arm to trace a circle in the X-Z plane at y = -0.3.

Circle centre: (0, -0.3, z_centre)
Circle radius: defined below.

Connects to moveo_publisher TCP socket on armpi.local:9000.

Two modes:
  default        — sends one {"cartesian": [...]} per waypoint, waits for ack,
                   then sleeps --delay seconds before the next (arm stops between points).
  --trajectory   — sends all waypoints as a single {"trajectory": [...], "dt": <delay>}
                   batch; the Pi solves IK and publishes at dt-second intervals without
                   the arm ever fully stopping (smooth continuous motion).

Usage:
    python3 circle_test.py [--radius 0.08] [--points 36] [--delay 1.5] [--speed 0.4]
    python3 circle_test.py --trajectory [--delay 0.4] [--points 36]
"""

import argparse
import json
import math
import socket
import time

PI_HOST   = "armpi.local"
PI_PORT   = 9000

def recv_line(sock):
    buf = ""
    while "\n" not in buf:
        chunk = sock.recv(4096).decode("utf-8", errors="replace")
        if not chunk:
            raise ConnectionError("socket closed before ack")
        buf += chunk
    return json.loads(buf.split("\n")[0])

def send_cartesian(sock, x, y, z):
    cmd = json.dumps({"cartesian": [round(x, 4), round(y, 4), round(z, 4)]}) + "\n"
    sock.sendall(cmd.encode())
    return recv_line(sock)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--radius",     type=float, default=0.08,  help="Circle radius in metres (default 0.08)")
    parser.add_argument("--points",     type=int,   default=36,    help="Number of waypoints (default 36)")
    parser.add_argument("--delay",      type=float, default=1.5,   help="Seconds between waypoints (default 1.5; use ~0.3-0.5 with --trajectory)")
    parser.add_argument("--speed",      type=float, default=0.4,   help="Speed scale 0.0-1.0 sent to arm (default 0.4)")
    parser.add_argument("--cx",         type=float, default=0.0,   help="Circle centre X in metres (default 0.0)")
    parser.add_argument("--cz",         type=float, default=0.45,  help="Circle centre Z in metres (default 0.45)")
    parser.add_argument("--trajectory", action="store_true",        help="Send all waypoints as one trajectory batch for smooth continuous motion")
    args = parser.parse_args()

    Y     = -0.3
    cx    = args.cx
    cz    = args.cz
    r     = args.radius
    n     = args.points
    delay = args.delay

    # Build waypoint list (n+1 to close the loop back to start)
    waypoints = []
    for i in range(n + 1):
        theta = 2 * math.pi * i / n
        waypoints.append([
            round(cx + r * math.cos(theta), 4),
            round(Y, 4),
            round(cz + r * math.sin(theta), 4),
        ])

    mode = "trajectory" if args.trajectory else "cartesian"
    print(f"Circle: centre=({cx:.3f}, {Y:.3f}, {cz:.3f})  radius={r:.3f}m  "
          f"{n} points  delay={delay}s  speed={args.speed}  mode={mode}")

    # Trajectory mode uses a longer socket timeout (whole trip can take n*dt seconds)
    total_timeout = max(30, (n + 2) * delay + 10)

    with socket.create_connection((PI_HOST, PI_PORT), timeout=10) as sock:
        # Set speed first and consume its ack (server sends {"ok":true} for speed-only packets)
        sock.sendall((json.dumps({"speed": args.speed}) + "\n").encode())
        recv_line(sock)

        if args.trajectory:
            # ── Trajectory mode: one batch, Pi handles timing ──────────────
            print(f"Sending {len(waypoints)}-waypoint trajectory (dt={delay}s) …")
            cmd = json.dumps({"trajectory": waypoints, "dt": delay}) + "\n"
            sock.sendall(cmd.encode())
            sock.settimeout(total_timeout)
            ack = recv_line(sock)
            if ack.get("ok"):
                print(f"Trajectory complete — {ack.get('waypoints')} waypoints executed")
                for i, res in enumerate(ack.get("results", [])):
                    degs = [round(math.degrees(a), 1) for a in res["angles"]]
                    print(f"  wp{i+1:3d}: err={res['fk_err_mm']}mm  joints={degs}")
            else:
                print(f"FAILED: {ack.get('error')}")

        else:
            # ── Cartesian mode: one waypoint at a time ─────────────────────
            for i, (x, y, z) in enumerate(waypoints):
                theta_deg = round(360.0 * i / n, 1)
                print(f"  [{i+1:3d}/{n+1}] theta={theta_deg:6.1f}°  target=({x:.3f}, {y:.3f}, {z:.3f})",
                      end="  ", flush=True)
                try:
                    ack = send_cartesian(sock, x, y, z)
                    if ack.get("ok"):
                        degs = [round(math.degrees(a), 1) for a in ack["angles"]]
                        print(f"ok  err={ack.get('fk_err_mm','?')}mm  joints={degs}")
                    else:
                        print(f"FAILED: {ack.get('error')}")
                except Exception as e:
                    print(f"ERROR: {e}")
                time.sleep(delay)

    print("Circle complete.")

if __name__ == "__main__":
    main()
