"""Lidar point clouds as numpy arrays. The complete version of exercise 11.

    lidar, tf = agibot_gdk.Lidar(), agibot_gdk.TF()
    points = scan_in_base(lidar, tf)           # (N, 3) of both lidars in base_link
    scan = without_robot(height_band(points, 0.1, 1.8), 0.5)  # a 2D scan without floor, ceiling and robot

A point cloud carries raw bytes. fields says where x, y and z sit within the point_step bytes of
one point. The extrinsic of a lidar maps a point in the sensor frame into base_link.
"""

from __future__ import annotations

import agibot_gdk
import numpy as np

from gdk_training import geometry

LIDARS = {"front": agibot_gdk.LidarType.kLidarFront, "back": agibot_gdk.LidarType.kLidarBack}
EXTRINSICS = {"front": "kChassisFrontLidarToBaseLink", "back": "kChassisBackLidarToBaseLink"}
DATATYPES = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}
TIMEOUT_MS = 200.0
BAND = (0.1, 1.8)    # m, heights that block the robot
MIN_RANGE = 0.5      # m, closer points are the robot itself


def field_array(cloud: object, name: str) -> np.ndarray:
    """Returns one field of every point, e.g. "reflectivity"."""
    field = next(f for f in cloud.fields if f.name == name)
    layout = np.dtype({"names": [name], "formats": [DATATYPES[field.datatype]], "offsets": [field.offset],
                       "itemsize": cloud.point_step})
    count = cloud.width * cloud.height
    return np.frombuffer(np.asarray(cloud.data, np.uint8), layout, count=count)[name]


def decode(cloud: object) -> np.ndarray:
    """Returns the points of a cloud as an (N, 3) float array x, y, z."""
    if cloud is None or cloud.width * cloud.height == 0:
        return np.zeros((0, 3))
    return np.stack([field_array(cloud, axis).astype(float) for axis in "xyz"], axis=1)


def sensor_to_base(tf: agibot_gdk.TF, name: str) -> geometry.Matrix:
    """Returns T_base_sensor of the lidar "front" or "back"."""
    return geometry.from_transform(tf.get_tf_from_sensor(getattr(agibot_gdk.SensorExtrinsicType, EXTRINSICS[name])))


def to_base(points: np.ndarray, transform: geometry.Matrix) -> np.ndarray:
    """Applies a 4x4 matrix to (N, 3) points."""
    return points @ transform[:3, :3].T + transform[:3, 3]


def height_band(points: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    """Keeps the points between two heights."""
    return points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)]


def without_robot(points: np.ndarray, min_range: float) -> np.ndarray:
    """Drops the points closer than min_range to the z axis, the lidar's hits on the robot itself."""
    return points[np.hypot(points[:, 0], points[:, 1]) >= min_range]


def scan_in_base(lidar: agibot_gdk.Lidar, tf: agibot_gdk.TF, names: tuple[str, ...] = ("front", "back")) -> np.ndarray:
    """Returns the latest points of the lidars in base_link, one (N, 3) array."""
    parts = [to_base(decode(lidar.get_latest_pointcloud(LIDARS[name], TIMEOUT_MS)), sensor_to_base(tf, name))
             for name in names]
    return np.concatenate(parts) if parts else np.zeros((0, 3))
