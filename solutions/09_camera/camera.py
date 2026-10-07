"""Exercise 9, camera. Shows the head and wrist camera images in the browser.

    python3 exercises/09_camera/camera.py

Then open http://localhost:8000.
"""

from __future__ import annotations

import sys
import time
from dataclasses import asdict, dataclass

import agibot_gdk
import cv2
import numpy as np

from gdk_training import show, viewer

CAMERAS = {
    "head": agibot_gdk.CameraType.kHeadColor,
    "head_depth": agibot_gdk.CameraType.kHeadDepth,
    "hand_left": agibot_gdk.CameraType.kHandLeftColor,
    "hand_right": agibot_gdk.CameraType.kHandRightColor,
}
TIMEOUT_MS = 100.0


@dataclass(frozen=True, kw_only=True)
class CameraInfo:
    width: int  # px
    height: int  # px
    fps: float
    fx: float  # focal length in px
    fy: float
    cx: float  # principal point in px
    cy: float


def info(camera: agibot_gdk.Camera, camera_type: agibot_gdk.CameraType) -> CameraInfo:
    """Reads the image size, the frame rate and the intrinsics of one camera."""
    # TODO: Call get_image_shape, get_image_fps and get_camera_intrinsic. The attribute
    # intrinsic of the last result is the list [fx, fy, cx, cy].
    width, height = camera.get_image_shape(camera_type)
    fx, fy, cx, cy = camera.get_camera_intrinsic(camera_type).intrinsic
    return CameraInfo(width=width, height=height, fps=camera.get_image_fps(camera_type),
                      fx=fx, fy=fy, cx=cx, cy=cy)


def grab(camera: agibot_gdk.Camera, camera_type: agibot_gdk.CameraType) -> np.ndarray | None:
    """Fetches the latest image of one camera and decodes it.

    Returns:
        A colour image with shape (height, width, 3), dtype uint8 and channel order BGR.
        A depth image (colour format RS2_FORMAT_Z16) with shape (height, width), dtype uint16
        and unit mm. None when the camera has no frame yet.
    """
    # TODO: Call get_latest_image with TIMEOUT_MS. Return None for a missing image or
    # empty data. Decode JPEG and PNG data with cv2.imdecode. Reshape uncompressed data to the
    # image size and convert RGB to BGR.
    image = camera.get_latest_image(camera_type, TIMEOUT_MS)
    if image is None or len(image.data) == 0:
        return None
    if image.encoding != agibot_gdk.Encoding.UNCOMPRESSED:
        return cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR)
    if image.color_format == agibot_gdk.ColorFormat.RS2_FORMAT_Z16:
        return np.frombuffer(image.data, np.uint16).reshape(image.height, image.width)
    pixels = np.frombuffer(image.data, np.uint8).reshape(image.height, image.width, 3)
    if image.color_format == agibot_gdk.ColorFormat.RGB:
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    return pixels


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    camera = agibot_gdk.Camera(list(CAMERAS.values()))
    time.sleep(3.0)
    try:
        print(show.table([{"camera": name, **asdict(info(camera, camera_type))} for name, camera_type in CAMERAS.items()]))
        viewer.serve({name: (lambda camera_type=camera_type: grab(camera, camera_type))
                      for name, camera_type in CAMERAS.items()})
    finally:
        camera.close_camera()
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
