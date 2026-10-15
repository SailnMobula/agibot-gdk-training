"""Exercise 11, lidar. Reads both lidar clouds, moves them into base_link and shows them in 3D.

    python3 exercises/11_lidar/lidar.py

Then open http://localhost:8000. The key b switches your height_band on and off.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass

import agibot_gdk
import numpy as np

from gdk_training import cloud3d, geometry, show

LIDARS = {"front": agibot_gdk.LidarType.kLidarFront, "back": agibot_gdk.LidarType.kLidarBack}
EXTRINSICS = {"front": agibot_gdk.SensorExtrinsicType.kChassisFrontLidarToBaseLink,
              "back": agibot_gdk.SensorExtrinsicType.kChassisBackLidarToBaseLink}
# The datatype codes of a PointField and the numpy type each one stands for, as in ROS PointCloud2.
DATATYPES = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}
TIMEOUT_MS = 200.0
LATENCY_WINDOW_S = 5.0
BAND = (0.1, 1.8)  # m, heights that block the robot


@dataclass(frozen=True, kw_only=True)
class LidarInfo:
    lidar: str  # "front" or "back"
    fps: float
    avg_latency_ms: float
    p99_latency_ms: float
    points: int  # in the latest cloud
    fields: str  # e.g. "x <f4@0 y <f4@4"


def info(lidar: agibot_gdk.Lidar, name: str) -> LidarInfo:
    """Reads rate, latency, number of points and the field layout of one lidar."""
    # TODO: Call get_lidar_fps, get_lidar_latency with LATENCY_WINDOW_S and get_latest_pointcloud
    # with TIMEOUT_MS. Write each field as name, numpy type from DATATYPES and offset.
    raise NotImplementedError


def decode(cloud: agibot_gdk.PointCloud) -> np.ndarray:
    """Returns the points of a cloud as an (N, 3) float array x, y, z in m, in the sensor frame."""
    # TODO: Build a numpy dtype with the names x, y, z, their types and offsets from cloud.fields
    # and itemsize point_step. Read data with np.frombuffer and stack the three columns.
    raise NotImplementedError


def reflectivity(cloud: agibot_gdk.PointCloud) -> np.ndarray:
    """Stretch: returns the reflectivity of every point, (N,) from 0 to 255."""
    # TODO: Like decode, for the field reflectivity.
    raise NotImplementedError


def sensor_to_base(tf: agibot_gdk.TF, name: str) -> geometry.Matrix:
    """Returns the 4x4 matrix that maps a point in the frame of the lidar into base_link."""
    # TODO: Call get_tf_from_sensor with EXTRINSICS[name] and convert it with geometry.from_transform.
    # Find out whether it maps sensor to base_link or the other way round.
    raise NotImplementedError


def to_base(points: np.ndarray, transform: geometry.Matrix) -> np.ndarray:
    """Applies a 4x4 matrix to (N, 3) points."""
    # TODO: The upper left 3x3 block of transform is the rotation, the right column the translation:
    # rotation = transform[:3, :3] and translation = transform[:3, 3]. One point p moves as
    # rotation @ p + translation. points holds one point per row, so for all points at once:
    # points @ rotation.T + translation.
    raise NotImplementedError


def height_band(points: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    """Keeps the points between z_min and z_max."""
    # TODO: A boolean mask on the z column.
    raise NotImplementedError


class LidarDisplay:
    """What the 3D view and the table show, and the key b that switches the height band."""

    def __init__(self, lidar: agibot_gdk.Lidar, tf: agibot_gdk.TF) -> None:
        self.lidar = lidar
        self.tf = tf
        self.band_on = False

    def points(self, name: str) -> np.ndarray:
        """The cloud of one lidar in base_link, with the reflectivity as fourth column when it is written."""
        cloud = self.lidar.get_latest_pointcloud(LIDARS[name], TIMEOUT_MS)
        points_in_sensor = decode(cloud)
        points = to_base(points_in_sensor, sensor_to_base(self.tf, name))
        try:
            points = np.column_stack([points, reflectivity(cloud)])
        except NotImplementedError:
            pass  # the stretch is not written yet, the view colours by height then
        if self.band_on:
            z_min, z_max = BAND
            points = height_band(points, z_min, z_max)  # the reflectivity column comes along
        return points

    def front_points(self) -> np.ndarray:
        return self.points("front")

    def back_points(self) -> np.ndarray:
        return self.points("back")

    def table(self) -> list[LidarInfo]:
        rows: list[LidarInfo] = []
        for name in LIDARS:
            rows.append(info(self.lidar, name))
        return rows

    def toggle_band(self) -> str:
        self.band_on = not self.band_on
        if self.band_on:
            switched = "on"
        else:
            switched = "off"
        return f"height band {BAND[0]} to {BAND[1]} m {switched}"


def robot_at_origin() -> np.ndarray:
    """The pose of the robot in the 3D view. The view is base_link, so the robot is at its origin."""
    return np.eye(4)


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    lidar = agibot_gdk.Lidar()
    tf = agibot_gdk.TF()
    time.sleep(2.0)
    display = LidarDisplay(lidar, tf)
    try:
        layers = {"front": display.front_points, "back": display.back_points}
        cloud3d.start(layers, robot=robot_at_origin, title="Lidar in base_link")

        keys = {"b": display.toggle_band}
        show.live(display.table, hz=1.0, keys=keys, help="b height band")
    finally:
        lidar.close_lidar()
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
