"""Maps as images and arrays. The complete version of exercise 14 and the map cloud of exercise 15.

    map_api = agibot_gdk.Map()
    info = map_api.get_map(map_api.get_curr_map().id)
    geo = geometry_of(info)
    image = grid_image(info)
    u, v = world_to_pixel(geo, x, y)

The grid has one value per cell, row by row, starting at the origin: -1 unknown, 0 free,
100 occupied. Image row 0 is at the top, so the image is the grid upside down.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import agibot_gdk
import cv2
import numpy as np

from gdk_training import geometry, lidar

UNKNOWN, FREE, OCCUPIED = -1, 0, 100
COLORS = {UNKNOWN: (128, 128, 128), FREE: (255, 255, 255), OCCUPIED: (0, 0, 0)}


@dataclass(frozen=True, kw_only=True)
class MapGeometry:
    width: int  # cells
    height: int  # cells
    resolution: float  # m per cell
    origin_x: float  # m, map frame position of the corner of cell (0, 0)
    origin_y: float


@dataclass(frozen=True, kw_only=True)
class GuidePoint:
    id: int
    type: int
    x: float  # m, map frame
    y: float
    yaw: float  # rad


def current_map(map_api: agibot_gdk.Map) -> object:
    """Returns the MapInfo of the current map."""
    return map_api.get_map(map_api.get_curr_map().id)


def geometry_of(info: object) -> MapGeometry:
    grid = info.grid_map
    return MapGeometry(width=grid.width, height=grid.height, resolution=grid.resolution,
                       origin_x=grid.origin.position.x, origin_y=grid.origin.position.y)


def cells(info: object) -> np.ndarray:
    """Returns the grid as an int array (height, width), row 0 at the origin."""
    grid = info.grid_map
    return np.asarray(grid.data, np.int16).reshape(grid.height, grid.width)


def cell_of(info: object, x: float, y: float) -> tuple[int, int] | None:
    """Returns row and column in cells(info) of the cell that contains x, y, or None outside the map."""
    geo = geometry_of(info)
    row = math.floor((y - geo.origin_y) / geo.resolution)
    column = math.floor((x - geo.origin_x) / geo.resolution)
    return (row, column) if 0 <= row < geo.height and 0 <= column < geo.width else None


def cells_around(info: object, x: float, y: float, radius: float) -> np.ndarray:
    """Returns the square of grid values around x, y that reaches radius in m to each side.

    The square is cut off at the border of the map. x, y has to be on the map.
    """
    row, column = cell_of(info, x, y)
    reach = math.ceil(radius / geometry_of(info).resolution)
    return cells(info)[max(row - reach, 0):row + reach + 1, max(column - reach, 0):column + reach + 1]


def world_to_pixel(geo: MapGeometry, x: float, y: float) -> tuple[float, float]:
    """Converts a point of the map frame in m to image coordinates (u right, v down)."""
    return (x - geo.origin_x) / geo.resolution, geo.height - (y - geo.origin_y) / geo.resolution


def pixel_to_world(geo: MapGeometry, u: float, v: float) -> tuple[float, float]:
    return geo.origin_x + u * geo.resolution, geo.origin_y + (geo.height - v) * geo.resolution


def grid_image(info: object) -> np.ndarray:
    """Returns a BGR image of the grid: unknown grey, free white, occupied black."""
    grid = cells(info)
    image = np.full((*grid.shape, 3), COLORS[UNKNOWN], np.uint8)
    image[grid == FREE] = COLORS[FREE]
    image[grid == OCCUPIED] = COLORS[OCCUPIED]
    return np.ascontiguousarray(image[::-1])


def guide_points(info: object) -> list[GuidePoint]:
    return [GuidePoint(id=point.id, type=point.type, x=point.pt.position.x, y=point.pt.position.y,
                       yaw=geometry.rpy(geometry.from_pose(point.pt))[2]) for point in info.guide_pts]


def robot_pose(slam: agibot_gdk.Slam) -> tuple[float, float, float]:
    """Returns x, y in m and yaw in rad of the robot on the current map."""
    pose = slam.get_curr_pose()
    return pose.position.x, pose.position.y, geometry.rpy(geometry.from_pose(pose))[2]


def pixel(geo: MapGeometry, x: float, y: float) -> tuple[int, int]:
    u, v = world_to_pixel(geo, x, y)
    return int(math.floor(u)), int(math.floor(v))


def draw_guide_points(image: np.ndarray, geo: MapGeometry, points: list[GuidePoint]) -> None:
    for point in points:
        u, v = pixel(geo, point.x, point.y)
        tip = pixel(geo, point.x + 0.4 * math.cos(point.yaw), point.y + 0.4 * math.sin(point.yaw))
        cv2.circle(image, (u, v), 4, (255, 120, 0), -1)
        cv2.line(image, (u, v), tip, (255, 120, 0), 1)
        cv2.putText(image, str(point.id), (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 120, 0), 1)


def draw_robot(image: np.ndarray, geo: MapGeometry, x: float, y: float, yaw: float,
               color: tuple[int, int, int] = (0, 160, 0)) -> None:
    u, v = pixel(geo, x, y)
    tip = pixel(geo, x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw))
    cv2.circle(image, (u, v), max(2, int(0.3 / geo.resolution)), color, 1)
    cv2.arrowedLine(image, (u, v), tip, color, 2, tipLength=0.3)


def draw_points(image: np.ndarray, geo: MapGeometry, points: np.ndarray, color: tuple[int, int, int]) -> None:
    """Draws points of the map frame, (N, 2) or (N, 3), as single pixels."""
    if len(points) == 0:
        return
    u = np.floor((points[:, 0] - geo.origin_x) / geo.resolution).astype(int)
    v = np.floor(geo.height - (points[:, 1] - geo.origin_y) / geo.resolution).astype(int)
    keep = (u >= 0) & (u < image.shape[1]) & (v >= 0) & (v < image.shape[0])
    image[v[keep], u[keep]] = color


def crop(image: np.ndarray, info: object, margin: int = 20) -> tuple[np.ndarray, int, int]:
    """Cuts the image to the known part of the map. Returns the image and the offset u0, v0."""
    known = np.argwhere(cells(info)[::-1] != UNKNOWN)
    if len(known) == 0:
        return image, 0, 0
    (v0, u0), (v1, u1) = known.min(0), known.max(0)
    v0, u0 = max(0, v0 - margin), max(0, u0 - margin)
    return image[v0:v1 + margin, u0:u1 + margin], u0, v0


class MapView:
    """Draws the known part of a map, enlarged, with the robot, points and a goal on top.

        view = MapView(info)
        image = view.draw(robot=(x, y, yaw), points=scan_on_map, goal=(x, y, yaw))
    """

    def __init__(self, info: object, scale: int = 2) -> None:
        self.geo = geometry_of(info)
        self.guides = guide_points(info)
        image = grid_image(info)
        draw_guide_points(image, self.geo, self.guides)
        self.base, self.u0, self.v0 = crop(image, info)
        self.scale = scale

    def draw(self, robot: tuple[float, float, float] | None = None, points: np.ndarray | None = None,
             goal: tuple[float, float, float] | None = None, goal_color: tuple[int, int, int] = (0, 0, 255)) -> np.ndarray:
        geo = self.geo
        image = np.zeros((geo.height, geo.width, 3), np.uint8)
        h, w = self.base.shape[:2]
        image[self.v0:self.v0 + h, self.u0:self.u0 + w] = self.base
        if points is not None:
            draw_points(image, geo, points, (0, 0, 230))
        if goal is not None:
            draw_robot(image, geo, *goal, color=goal_color)
        if robot is not None:
            draw_robot(image, geo, *robot)
        view = image[self.v0:self.v0 + h, self.u0:self.u0 + w]
        return cv2.resize(view, (w * self.scale, h * self.scale), interpolation=cv2.INTER_NEAREST)

    def to_world(self, u: float, v: float) -> tuple[float, float] | None:
        """Turns a pixel of the drawn image, e.g. a click, into x, y on the map. None beside the map."""
        h, w = self.base.shape[:2]
        if not (0 <= u < w * self.scale and 0 <= v < h * self.scale):
            return None
        return pixel_to_world(self.geo, self.u0 + u / self.scale, self.v0 + v / self.scale)


def with_legend(image: np.ndarray, rows: list[tuple[str, tuple[int, int, int]]], width: int = 170) -> np.ndarray:
    """Adds a white strip on the right with one coloured square and a text per row."""
    strip = np.full((max(image.shape[0], 30 + 22 * len(rows)), width, 3), 255, np.uint8)
    for i, (text, color) in enumerate(rows):
        y = 20 + 22 * i
        cv2.rectangle(strip, (10, y - 6), (22, y + 6), color, -1)
        cv2.rectangle(strip, (10, y - 6), (22, y + 6), (160, 160, 160), 1)
        cv2.putText(strip, text, (34, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    canvas = np.full((strip.shape[0], image.shape[1] + width, 3), 255, np.uint8)
    canvas[:image.shape[0], :image.shape[1]] = image
    canvas[:, image.shape[1]:] = strip
    return canvas


def map_cloud(info: object) -> np.ndarray:
    """Returns the point cloud of the map as (N, 3) as stored: in the front lidar frame at the start of mapping."""
    return lidar.decode(info.nonground_cloud)


def cloud_on_map(info: object, tf: agibot_gdk.TF) -> np.ndarray:
    """Returns the point cloud of the map in the map frame, z up, the floor near z = 0.

    The robot stores the cloud in the frame of the front lidar when mapping started. The map frame is
    base_link at that moment, so the extrinsic of the front lidar maps the cloud onto the map.
    """
    return lidar.to_base(map_cloud(info), lidar.sensor_to_base(tf, "front"))


def to_grid(points: np.ndarray, geo: MapGeometry, z_min: float, z_max: float) -> np.ndarray:
    """Returns a bool grid (height, width), row 0 at the origin: cells hit by a point in the band."""
    band = points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)]
    column = np.floor((band[:, 0] - geo.origin_x) / geo.resolution).astype(int)
    row = np.floor((band[:, 1] - geo.origin_y) / geo.resolution).astype(int)
    keep = (row >= 0) & (row < geo.height) & (column >= 0) & (column < geo.width)
    grid = np.zeros((geo.height, geo.width), bool)
    grid[row[keep], column[keep]] = True
    return grid
