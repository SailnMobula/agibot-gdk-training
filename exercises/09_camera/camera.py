"""Exercise 9, camera. Shows the head and wrist camera images in the browser.

    python3 exercises/09_camera/camera.py

Then open http://<robot>:8000.
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
    raise NotImplementedError


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
    raise NotImplementedError


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
