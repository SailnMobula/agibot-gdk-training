"""Exercise 15, map in 3D. Shows the point cloud of the map and builds an own occupancy grid from it.

    python3 exercises/15_map3d/map3d.py [--map ID]

The cloud is at http://localhost:8001, the robot's grid and your grid at http://localhost:8000.
l switches between the cloud as stored and in the map frame with your to_map. + and - move the
height band by 0.1 m.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable

import agibot_gdk
import cv2
import numpy as np

from gdk_training import cloud3d, lidar, maps, show, viewer

BAND_Z_MIN = 0.1  # m, the lowest height that blocks the robot
BAND_Z_MAX = 1.8  # m, the highest height that blocks the robot
BAND_STEP = 0.1  # m, how far + and - move the band
SCALE = 2  # screen pixels per cell

# The values of the cells in grid_map.data.
CELL_UNKNOWN = -1
CELL_FREE = 0
CELL_OCCUPIED = 100

# The two ways to show the cloud. l switches between them.
AS_STORED = "as stored"
IN_MAP_FRAME = "in the map frame"

# Colours in BGR, the channel order of OpenCV.
BOTH_COLOR = (0, 0, 0)
ONLY_OWN_COLOR = (0, 0, 230)
ONLY_ROBOT_COLOR = (230, 120, 0)
FREE_COLOR = (255, 255, 255)
UNKNOWN_COLOR = (128, 128, 128)


def map_cloud(info: agibot_gdk.MapInfo) -> np.ndarray:
    """Returns the point cloud of the map, (N, 3), in the frame the robot stores it in."""
    # TODO: info.nonground_cloud is a point cloud like those of exercise 11. lidar.decode reads it.
    points = lidar.decode(info.nonground_cloud)
    return points


def to_map(points: np.ndarray, tf: agibot_gdk.TF) -> np.ndarray:
    """Moves the cloud into the map frame.

    The robot stores the cloud in the frame of the front lidar at the moment mapping started. The
    map frame is base_link at that moment.
    """
    # TODO: lidar.sensor_to_base gives the extrinsic of the front lidar, lidar.to_base applies it.
    front_lidar_to_base = lidar.sensor_to_base(tf, "front")
    points_on_map = lidar.to_base(points, front_lidar_to_base)
    return points_on_map


def to_grid(points: np.ndarray, map_geometry: maps.MapGeometry, z_min: float, z_max: float) -> np.ndarray:
    """Returns a bool array (height, width), row 0 at the origin like the robot's grid.

    A cell is True when at least one point between z_min and z_max falls into it.
    """
    # TODO: Keep the points of the band. Column and row are the distance from the origin divided by
    # the resolution, rounded down. Drop points outside the grid.
    heights = points[:, 2]
    is_in_band = (heights >= z_min) & (heights <= z_max)
    points_in_band = points[is_in_band]

    x_from_origin = points_in_band[:, 0] - map_geometry.origin_x
    y_from_origin = points_in_band[:, 1] - map_geometry.origin_y
    columns = np.floor(x_from_origin / map_geometry.resolution).astype(int)
    rows = np.floor(y_from_origin / map_geometry.resolution).astype(int)

    is_row_inside = (rows >= 0) & (rows < map_geometry.height)
    is_column_inside = (columns >= 0) & (columns < map_geometry.width)
    is_inside = is_row_inside & is_column_inside

    grid = np.zeros((map_geometry.height, map_geometry.width), bool)
    grid[rows[is_inside], columns[is_inside]] = True
    return grid


def agreement(own: np.ndarray, info: agibot_gdk.MapInfo) -> dict[str, float]:
    """Compares your grid with the robot's.

    Returns:
        "own cells": the number of your occupied cells, "in occupied", "in free" and "in unknown":
        the share of them that the robot marks occupied, free and unknown, "robot found": the
        share of the robot's occupied cells that are also yours.
    """
    robot_cells = maps.cells(info)
    robot_occupied = robot_cells == CELL_OCCUPIED
    robot_free = robot_cells == CELL_FREE
    robot_unknown = robot_cells == CELL_UNKNOWN
    own_count = int(own.sum())

    def share_of_own(robot_mask: np.ndarray) -> float:
        """The share of your occupied cells that lie in robot_mask, 0 when you have none."""
        if own_count == 0:
            return 0.0
        both_count = int((own & robot_mask).sum())
        return both_count / own_count

    robot_occupied_count = int(robot_occupied.sum())
    found_count = int((own & robot_occupied).sum())
    if robot_occupied_count == 0:
        robot_found = 0.0
    else:
        robot_found = found_count / robot_occupied_count

    numbers = {
        "own cells": own_count,
        "in occupied": share_of_own(robot_occupied),
        "in free": share_of_own(robot_free),
        "in unknown": share_of_own(robot_unknown),
        "robot found": robot_found,
    }
    return numbers


def overlay(info: agibot_gdk.MapInfo, own: np.ndarray) -> np.ndarray:
    """Both grids in one image: both black, only yours red, only the robot's blue."""
    robot = maps.cells(info) == CELL_OCCUPIED
    only_own = own & ~robot
    only_robot = robot & ~own
    both = robot & own

    # Row 0 of the grids is at the origin, but row 0 of an image is at the top: flip the masks.
    image = maps.grid_image(info)
    image[np.flipud(only_own)] = ONLY_OWN_COLOR
    image[np.flipud(only_robot)] = ONLY_ROBOT_COLOR
    image[np.flipud(both)] = BOTH_COLOR

    view, _, _ = maps.crop(image, info)
    view_height, view_width = view.shape[:2]
    enlarged = cv2.resize(view, (view_width * SCALE, view_height * SCALE), interpolation=cv2.INTER_NEAREST)
    legend = [
        ("occupied in both", BOTH_COLOR),
        ("only in your grid", ONLY_OWN_COLOR),
        ("only in the robot's", ONLY_ROBOT_COLOR),
        ("free", FREE_COLOR),
        ("unknown", UNKNOWN_COLOR),
    ]
    return maps.with_legend(enlarged, legend)


class GridComparison:
    """What the keys do, what the table shows and the two images.

    Every on_... method belongs to one key. Its return value appears below the table.
    """

    def __init__(self, info: agibot_gdk.MapInfo, tf: agibot_gdk.TF) -> None:
        self.info = info
        self.tf = tf
        self.map_geometry = maps.geometry_of(info)
        self.stored_cloud = map_cloud(info)
        self.cloud_on_map: np.ndarray | None = None  # computed with your to_map when l is pressed
        self.mode = AS_STORED
        self.z_min = BAND_Z_MIN
        self.z_max = BAND_Z_MAX
        self.floor = cloud3d.Floor(maps.grid_image(info), self.map_geometry.origin_x, self.map_geometry.origin_y,
                                   self.map_geometry.resolution)
        self.compare()

    def cloud(self) -> np.ndarray:
        """The cloud in the current mode."""
        if self.mode == AS_STORED:
            return self.stored_cloud
        if self.cloud_on_map is None:
            self.cloud_on_map = to_map(self.stored_cloud, self.tf)
        return self.cloud_on_map

    def compare(self) -> None:
        """Builds your grid from the cloud and compares it with the robot's."""
        self.own_grid = to_grid(self.cloud(), self.map_geometry, self.z_min, self.z_max)
        self.grids_image = overlay(self.info, self.own_grid)
        self.numbers = agreement(self.own_grid, self.info)

    def tilt(self) -> str:
        """The angle between gravity and -z. gravity is measured in the frame of the stored cloud."""
        gravity = np.array([self.info.gravity.x, self.info.gravity.y, self.info.gravity.z])
        if self.mode == IN_MAP_FRAME:
            rotation = lidar.sensor_to_base(self.tf, "front")[:3, :3]
            gravity = rotation @ gravity
        cos_angle = -gravity[2] / np.linalg.norm(gravity)
        angle = np.degrees(np.arccos(cos_angle))
        return f"{angle:.1f} deg"

    def on_switch_mode(self) -> str:
        if self.mode == AS_STORED:
            self.mode = IN_MAP_FRAME
        else:
            self.mode = AS_STORED
        self.compare()
        return f"cloud {self.mode}"

    def on_band_up(self) -> str:
        return self.move_band(BAND_STEP)

    def on_band_down(self) -> str:
        return self.move_band(-BAND_STEP)

    def move_band(self, step: float) -> str:
        self.z_min = round(self.z_min + step, 2)  # round, so that 0.1 + 0.1 shows as 0.2
        self.z_max = round(self.z_max + step, 2)
        self.compare()
        return f"band {self.z_min:.1f} to {self.z_max:.1f} m"

    def table(self) -> dict[str, object]:
        """Read by show.live twice per second."""
        lowest_z = float(np.percentile(self.cloud()[:, 2], 1))  # 1 % of the points are lower
        settings = {
            "map": self.info.id,
            "points": len(self.stored_cloud),
            "cloud": self.mode,
            "tilt of gravity": self.tilt(),
            "lowest z": lowest_z,
            "band": [self.z_min, self.z_max],
        }
        table = {"settings": settings, "agreement": self.numbers}
        return table

    def get_floor(self) -> cloud3d.Floor:
        return self.floor

    def get_grids_image(self) -> np.ndarray:
        return self.grids_image


def main() -> None:
    parser = argparse.ArgumentParser(description="Shows the point cloud of a map and an own grid built from it.")
    parser.add_argument("--map", type=int, help="map id, default: the current map")
    arguments = parser.parse_args()
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    map_api = agibot_gdk.Map()
    tf = agibot_gdk.TF()
    time.sleep(2.0)
    try:
        if arguments.map is not None:
            map_id = arguments.map
        else:
            map_id = map_api.get_curr_map().id
        info = map_api.get_map(map_id)
        comparison = GridComparison(info, tf)

        clouds = {"map cloud": comparison.cloud}
        cloud3d.start(clouds, floor=comparison.get_floor, port=8001, title=f"Map {info.id}", colors=("height",))
        images = {"grids": comparison.get_grids_image}
        viewer.start(images, hz=2.0)

        keys: dict[str, Callable[[], object]] = {
            "l": comparison.on_switch_mode,
            "+": comparison.on_band_up,
            "-": comparison.on_band_down,
        }
        show.live(comparison.table, hz=2.0, keys=keys, help="l as stored / map frame   + - move the band")
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
