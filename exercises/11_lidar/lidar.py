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


def decode(cloud: object) -> np.ndarray:
    """Returns the points of a cloud as an (N, 3) float array x, y, z in m, in the sensor frame."""
    # TODO: Build a numpy dtype with the names x, y, z, their types and offsets from cloud.fields
    # and itemsize point_step. Read data with np.frombuffer and stack the three columns.
    raise NotImplementedError


def reflectivity(cloud: object) -> np.ndarray:
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
    # TODO: Rotate with transform[:3, :3] and add transform[:3, 3], for all points at once.
    raise NotImplementedError


def height_band(points: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    """Keeps the points between z_min and z_max."""
    # TODO: A boolean mask on the z column.
    raise NotImplementedError


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    lidar, tf = agibot_gdk.Lidar(), agibot_gdk.TF()
    time.sleep(2.0)
    band = {"on": False}

    def layer(name: str) -> np.ndarray:
        cloud = lidar.get_latest_pointcloud(LIDARS[name], TIMEOUT_MS)
        points = to_base(decode(cloud), sensor_to_base(tf, name))
        try:
            points = np.column_stack([points, reflectivity(cloud)])
        except NotImplementedError:
            pass
        if band["on"]:
            points = height_band(points, *BAND)  # the reflectivity column comes along
        return points

    def toggle() -> str:
        band["on"] = not band["on"]
        return f"height band {BAND[0]} to {BAND[1]} m {'on' if band['on'] else 'off'}"

    try:
        cloud3d.start({name: (lambda name=name: layer(name)) for name in LIDARS}, robot=lambda: np.eye(4),
                      title="Lidar in base_link")
        show.live(lambda: [info(lidar, name) for name in LIDARS], hz=1.0, keys={"b": toggle},
                  help="b height band")
    finally:
        lidar.close_lidar()
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
