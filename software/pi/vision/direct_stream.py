#!/usr/bin/env python3
"""
direct_stream.py — serve the stereo camera over HTTP with no ROS.

Opens the ELP dual-lens USB camera directly via V4L2, splits each
side-by-side frame into left and right halves, and serves:
  /         — HTML viewer page
  /stream   — multipart MJPEG (browser live view, scaled + lower quality)
  /left     — latest left-half JPEG (high quality, for Mac vision offload)
  /right    — latest right-half JPEG (high quality)

Run:
    python3 direct_stream.py [--device 0] [--port 8080]
"""

import argparse
import http.server
import socketserver
import threading
import time

import cv2

_LATEST: dict[str, bytes | None] = {"preview": None, "left": None, "right": None}
_LOCK = threading.Lock()


class _Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(b"""<html><body>
<h2>Pi Stereo Stream</h2>
<img src="/stream" style="max-width:100%;"><br>
<a href="/left">/left</a> &nbsp; <a href="/right">/right</a>
</body></html>""")
            return

        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with _LOCK:
                        j = _LATEST["preview"]
                    if j is not None:
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + j + b"\r\n"
                        )
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        if self.path in ("/left", "/right"):
            key = self.path[1:]
            with _LOCK:
                j = _LATEST.get(key)
            if j is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(j)))
            self.end_headers()
            self.wfile.write(j)
            return

        self.send_response(404)
        self.end_headers()


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def _open_camera(device: int, width: int, height: int, fps: int):
    for attempt in range(6):
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if cap.isOpened():
            break
        cap.release()
        print(f"camera open attempt {attempt + 1}/6 failed; retrying...", flush=True)
        time.sleep(1.0)
    else:
        raise RuntimeError(
            f"cannot open /dev/video{device} after retries — "
            "check `lsusb | grep 32e4` and `ls /dev/video*`"
        )
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"camera open: requested {width}x{height}, got {aw}x{ah}", flush=True)
    if aw < width:
        print(
            f"WARNING: got width {aw} (< {width}); camera may be in single-lens fallback mode",
            flush=True,
        )
    return cap


def _capture_loop(cap, preview_scale: float, preview_quality: int, proc_quality: int):
    frames = 0
    t0 = time.time()
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.02)
            continue
        half = frame.shape[1] // 2
        left  = frame[:, :half]
        right = frame[:, half : half * 2]

        if preview_scale != 1.0:
            preview = cv2.resize(
                left, None, fx=preview_scale, fy=preview_scale,
                interpolation=cv2.INTER_AREA,
            )
        else:
            preview = left

        _, buf_p = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, preview_quality])
        _, buf_l = cv2.imencode(".jpg", left,    [cv2.IMWRITE_JPEG_QUALITY, proc_quality])
        _, buf_r = cv2.imencode(".jpg", right,   [cv2.IMWRITE_JPEG_QUALITY, proc_quality])

        with _LOCK:
            _LATEST["preview"] = buf_p.tobytes()
            _LATEST["left"]    = buf_l.tobytes()
            _LATEST["right"]   = buf_r.tobytes()

        frames += 1
        if time.time() - t0 >= 5.0:
            print(f"streaming {frames / (time.time() - t0):.1f} fps", flush=True)
            frames = 0
            t0 = time.time()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device",          type=int,   default=0)
    ap.add_argument("--width",           type=int,   default=2560)
    ap.add_argument("--height",          type=int,   default=960)
    ap.add_argument("--fps",             type=int,   default=30)
    ap.add_argument("--port",            type=int,   default=8080)
    ap.add_argument("--preview-scale",   type=float, default=0.5)
    ap.add_argument("--preview-quality", type=int,   default=70)
    ap.add_argument("--proc-quality",    type=int,   default=85)
    args = ap.parse_args()

    cap = _open_camera(args.device, args.width, args.height, args.fps)

    threading.Thread(
        target=_capture_loop,
        args=(cap, args.preview_scale, args.preview_quality, args.proc_quality),
        daemon=True,
    ).start()

    srv = _Server(("0.0.0.0", args.port), _Handler)
    print(f"streaming on http://<pi>:{args.port}/  (no ROS)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()


if __name__ == "__main__":
    main()
