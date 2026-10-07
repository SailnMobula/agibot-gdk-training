"""Shows images in the browser. The robot has no screen, so the exercises serve MJPEG over HTTP.

    viewer.serve({"head": lambda: grab(camera, agibot_gdk.CameraType.kHeadColor)})

Each stream is a function returning a BGR image (h, w, 3) uint8 or a depth image (h, w) uint16
in mm, or None when there is no frame yet. Open http://<robot>:8000.
"""

from __future__ import annotations

import os
import threading
import time
from collections.abc import Callable, Mapping
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

Stream = Callable[[], np.ndarray | None]
DEPTH_RANGE_MM = 3000.0


def jpeg(image: np.ndarray) -> bytes:
    if image.dtype == np.uint16:
        scaled = np.clip(image.astype(np.float32) / DEPTH_RANGE_MM * 255, 0, 255).astype(np.uint8)
        image = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    return cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()


def mark(image: np.ndarray, pixel: tuple[float, float], label: str) -> None:
    """Draws a labelled circle. Pixels outside the image are skipped."""
    u, v = int(round(pixel[0])), int(round(pixel[1]))
    if 0 <= u < image.shape[1] and 0 <= v < image.shape[0]:
        cv2.circle(image, (u, v), 8, (0, 255, 0), 2)
        cv2.putText(image, label, (u + 12, v + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


def serve(streams: Mapping[str, Stream], port: int = 8000, hz: float = 10.0) -> None:
    """Starts the server and blocks until Ctrl+C."""
    if start(streams, port, hz) is None:
        return
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


def start(streams: Mapping[str, Stream], port: int = 8000, hz: float = 10.0) -> ThreadingHTTPServer | None:
    """Starts the server in the background and returns. Use it when the terminal shows a table."""
    if os.environ.get("SHOW_ONCE"):
        for name, stream in streams.items():
            image = stream()
            print(f"{name}: {'no frame' if image is None else f'{image.shape} {image.dtype}'}")
        return None

    page = "".join(f"<h3>{name}</h3><img src='/{name}' style='max-width:100%'>" for name in streams)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            name = self.path.strip("/")
            if name not in streams:
                body = f"<html><body style='font-family:sans-serif'>{page}</body></html>".encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    image = streams[name]()
                    if image is not None:
                        data = jpeg(image)
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n")
                    time.sleep(1.0 / hz)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args: object) -> None:
            pass

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"viewer on port {port}, open http://<robot>:{port} or http://localhost:{port}")
    return server
