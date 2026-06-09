#!/usr/bin/env python3
"""
mjpeg_stream.py — serve the stereo left view as an MJPEG stream for a browser.

Subscribes to /stereo/left/image_raw, optionally rectifies (from stereo_calib.yaml)
and overlays live checkerboard detection, and serves it over HTTP so you can watch
the camera (and whether the board is detected) from any browser while jogging the
arm — no ROS needed on the viewing machine.

Open:  http://armpi.local:8080/

Runs alongside stereo_camera_node + hand_eye_calibrate (multiple subscribers OK).

Run:
    python3 mjpeg_stream.py --ros-args -p calib:=/home/armpi/vision/stereo_calib.yaml
"""

import http.server
import socketserver
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

_LATEST = {"jpeg": None, "left": None, "right": None}
_LOCK = threading.Lock()


class _Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/":
            # simple HTML page with stream + links for processing
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            html = b"""<html><body>
<h2>Pi Stereo Stream</h2>
<img src="/stream" style="max-width:100%;"><br>
<a href="/left">/left</a> (high-Q JPEG for Mac processing)<br>
<a href="/right">/right</a> (high-Q JPEG for Mac processing)<br>
<p>Use these from your compute for offloaded vision.</p>
</body></html>"""
            self.wfile.write(html)
            return

        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with _LOCK:
                        j = _LATEST["jpeg"]
                    if j is not None:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + j + b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        if self.path in ("/left", "/right"):
            key = "left" if self.path == "/left" else "right"
            with _LOCK:
                j = _LATEST.get(key)
            if j is None:
                self.send_response(503); self.end_headers(); return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(j)))
            self.end_headers()
            self.wfile.write(j)
            return

        self.send_response(404); self.end_headers(); return


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True   # rebind immediately on restart (avoid TIME_WAIT "address in use")
    daemon_threads = True


def start_server(port):
    srv = _Server(("0.0.0.0", port), _Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


class StreamNode(Node):
    def __init__(self):
        super().__init__("mjpeg_stream")
        p = self.declare_parameter
        self.port = p("port", 8080).value
        self.scale = p("scale", 0.5).value
        self.cols = p("inner_cols", 8).value
        self.rows = p("inner_rows", 6).value
        self.display_rotate = p("display_rotate_180", False).value  # cosmetic only (upside-down mount)
        calib = p("calib", "").value
        self.proc_quality = p("proc_quality", 85).value  # high quality for Mac offload grabs

        self.maps = None
        if calib:
            fs = cv2.FileStorage(calib, cv2.FILE_STORAGE_READ)
            if fs.isOpened():
                g = lambda n: fs.getNode(n).mat()
                K1, D1, R1, P1 = g("K1"), g("D1"), g("R1"), g("P1")
                w = int(fs.getNode("image_width").real()); h = int(fs.getNode("image_height").real())
                fs.release()
                self.maps = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_16SC2)

        self.create_subscription(Image, "/stereo/left/image_raw", self._cb_left, qos_profile_sensor_data)
        self.create_subscription(Image, "/stereo/right/image_raw", self._cb_right, qos_profile_sensor_data)
        start_server(self.port)
        self.get_logger().info(f"MJPEG stream on http://<pi>:{self.port}/  "
                               f"(rectified={'yes' if self.maps else 'no'})  "
                               f"Also serving /left and /right for offload processing at Q={self.proc_quality}")

    def _process_for_preview(self, img):
        if self.maps is not None:
            img = cv2.remap(img, *self.maps, cv2.INTER_LINEAR)
        if self.scale != 1.0:
            img = cv2.resize(img, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        if self.display_rotate:
            img = cv2.rotate(img, cv2.ROTATE_180)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(
            gray, (self.cols, self.rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        if ok:
            cv2.drawChessboardCorners(img, (self.cols, self.rows), corners, ok)
        banner = "BOARD OK" if ok else "no board"
        color = (0, 220, 0) if ok else (0, 0, 255)
        cv2.putText(img, banner, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        return img

    def _cb_left(self, msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        preview = self._process_for_preview(img.copy())
        oktag, buf = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if oktag:
            with _LOCK:
                _LATEST["jpeg"] = buf.tobytes()

        # High quality for Mac offload processing (raw or lightly processed)
        proc_img = img
        if self.maps is not None:
            proc_img = cv2.remap(proc_img, *self.maps, cv2.INTER_LINEAR)
        oktag, buf = cv2.imencode(".jpg", proc_img, [cv2.IMWRITE_JPEG_QUALITY, self.proc_quality])
        if oktag:
            with _LOCK:
                _LATEST["left"] = buf.tobytes()

    def _cb_right(self, msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        proc_img = img
        if self.maps is not None:
            proc_img = cv2.remap(proc_img, *self.maps, cv2.INTER_LINEAR)
        oktag, buf = cv2.imencode(".jpg", proc_img, [cv2.IMWRITE_JPEG_QUALITY, self.proc_quality])
        if oktag:
            with _LOCK:
                _LATEST["right"] = buf.tobytes()


def main():
    rclpy.init()
    node = StreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
