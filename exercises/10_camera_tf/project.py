"""Exercise 10, camera and TF. Draws both end effectors into the head camera image.

    python3 exercises/10_camera_tf/project.py

Then open http://localhost:8000.
"""

from __future__ import annotations

import sys
import time

import agibot_gdk
import cv2
import numpy as np

from gdk_training import geometry, show, viewer
from gdk_training.jog import Jog

HEAD = agibot_gdk.CameraType.kHeadColor
END_EFFECTORS = {"left": "arm_l_end_link", "right": "arm_r_end_link"}


def camera_from_base(tf: agibot_gdk.TF) -> geometry.Matrix:
    """Computes the transform from base_link into the optical frame of the head camera.

    Returns:
        A 4x4 matrix T with p_camera = T @ p_base.
    """
    # TODO: Combine two transforms. The TF tree gives the transform between base_link and
    # head_link3. get_tf_from_sensor(kHeadRGBDToHeadLink3) gives the transform between head_link3
    # and the camera. Use the lookup direction you found in exercise 8 and geometry.inverse.
    raise NotImplementedError


def project(point: np.ndarray, intrinsic: list[float]) -> tuple[float, float] | None:
    """Projects a point onto the image with the pinhole model.

    Args:
        point: [x, y, z] or [x, y, z, 1] in the optical frame. z points forward, x right, y down.
        intrinsic: [fx, fy, cx, cy] in px.

    Returns:
        The pixel (u, v), or None when the point is behind the camera.
    """
    # TODO: Return None for z <= 0. Otherwise u = cx + fx * x / z and v = cy + fy * y / z.
    raise NotImplementedError


def end_effector_points(robot: agibot_gdk.Robot) -> dict[str, np.ndarray]:
    """Returns the end-effector origins in base_link as homogeneous points [x, y, z, 1]."""
    status = robot.get_motion_control_status()
    poses = dict(zip(status.frame_names, status.frame_poses))
    return {side: geometry.from_pose(poses[frame])[:, 3] for side, frame in END_EFFECTORS.items()}


def pixels(robot: agibot_gdk.Robot, tf: agibot_gdk.TF, intrinsic: list[float]) -> list[dict[str, object]]:
    """Returns the position in the camera frame and the pixel of both end effectors as table rows."""
    transform = camera_from_base(tf)
    rows = []
    for side, point in end_effector_points(robot).items():
        in_camera = transform @ point
        pixel = project(in_camera, intrinsic)
        rows.append({"end effector": side, "in camera": list(in_camera[:3]), "u": pixel[0] if pixel else "-",
                     "v": pixel[1] if pixel else "-"})
    return rows


def annotated(robot: agibot_gdk.Robot, tf: agibot_gdk.TF, camera: agibot_gdk.Camera,
              intrinsic: list[float]) -> np.ndarray | None:
    """Returns the head image with a circle at the projected position of each end effector."""
    image = camera.get_latest_image(HEAD, 100.0)
    if image is None or len(image.data) == 0:
        return None
    if image.encoding != agibot_gdk.Encoding.UNCOMPRESSED:
        picture = cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR)
    else:
        picture = np.frombuffer(image.data, np.uint8).reshape(image.height, image.width, 3).copy()
    transform = camera_from_base(tf)
    for side, point in end_effector_points(robot).items():
        pixel = project(transform @ point, intrinsic)
        if pixel is not None:
            viewer.mark(picture, pixel, side)
    return picture


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot, tf, camera = agibot_gdk.Robot(), agibot_gdk.TF(), agibot_gdk.Camera([HEAD])
    time.sleep(3.0)
    try:
        intrinsic = list(camera.get_camera_intrinsic(HEAD).intrinsic)
        viewer.start({"head": lambda: annotated(robot, tf, camera, intrinsic)})
        jog = Jog(robot)
        show.live(lambda: pixels(robot, tf, intrinsic), keys=jog.keys, help=jog.help)
    finally:
        camera.close_camera()
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
