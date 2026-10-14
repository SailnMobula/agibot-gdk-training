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
TIMEOUT_MS = 100.0


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

    pose_by_frame: dict[str, agibot_gdk.Pose] = {}
    for frame_name, pose in zip(status.frame_names, status.frame_poses):
        pose_by_frame[frame_name] = pose

    points: dict[str, np.ndarray] = {}
    for side, frame_name in END_EFFECTORS.items():
        pose_matrix = geometry.from_pose(pose_by_frame[frame_name])
        points[side] = pose_matrix[:, 3]  # the last column is the origin [x, y, z, 1]
    return points


def decode_color(image: object) -> np.ndarray:
    """Turns an image of the GDK into a numpy array (height, width, 3).

    image is the result of get_latest_image. Its class has no name in the GDK binding, hence object.
    """
    raw_bytes = np.frombuffer(image.data, np.uint8)
    if image.encoding != agibot_gdk.Encoding.UNCOMPRESSED:
        return cv2.imdecode(raw_bytes, cv2.IMREAD_COLOR)
    # copy, because the buffer of the GDK is read-only and the circles are drawn into the array
    pixels = raw_bytes.reshape(image.height, image.width, 3).copy()
    return pixels


class HeadView:
    """What the table and the browser show: both end effectors in the head camera."""

    def __init__(self, robot: agibot_gdk.Robot, tf: agibot_gdk.TF, camera: agibot_gdk.Camera,
                 intrinsic: list[float]) -> None:
        self.robot = robot
        self.tf = tf
        self.camera = camera
        self.intrinsic = intrinsic

    def table(self) -> list[dict[str, object]]:
        """The position in the camera frame and the pixel of both end effectors."""
        transform = camera_from_base(self.tf)
        rows: list[dict[str, object]] = []
        for side, point_in_base in end_effector_points(self.robot).items():
            point_in_camera = transform @ point_in_base
            pixel = project(point_in_camera, self.intrinsic)
            u: float | str = "-"
            v: float | str = "-"
            if pixel is not None:  # None when the point is behind the camera
                u, v = pixel
            row = {
                "end effector": side,
                "in camera": list(point_in_camera[:3]),
                "u": u,
                "v": v,
            }
            rows.append(row)
        return rows

    def image(self) -> np.ndarray | None:
        """The head image with a circle at the projected position of each end effector."""
        image = self.camera.get_latest_image(HEAD, TIMEOUT_MS)
        if image is None or len(image.data) == 0:
            return None
        picture = decode_color(image)

        transform = camera_from_base(self.tf)
        for side, point_in_base in end_effector_points(self.robot).items():
            point_in_camera = transform @ point_in_base
            pixel = project(point_in_camera, self.intrinsic)
            if pixel is not None:
                viewer.mark(picture, pixel, side)
        return picture


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    robot = agibot_gdk.Robot()
    tf = agibot_gdk.TF()
    camera = agibot_gdk.Camera([HEAD])
    time.sleep(3.0)
    try:
        intrinsic = list(camera.get_camera_intrinsic(HEAD).intrinsic)
        head_view = HeadView(robot, tf, camera, intrinsic)
        streams = {"head": head_view.image}
        viewer.start(streams)

        jog = Jog(robot)
        show.live(head_view.table, keys=jog.keys, help=jog.help)
    finally:
        camera.close_camera()
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
