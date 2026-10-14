"""Exercise 14, map in 2D. Draws the occupancy grid, the guide points and the robot.

    python3 exercises/14_map2d/map2d.py [--map ID]

Then open http://localhost:8000. r reloads the map, for example after you added a guide point
on the HMI. Without --map the script shows the current map. The drive keys work as in exercise 12.
Click on the map: the table shows the point in the map frame, computed with your pixel_to_world.
l shows the live lidar scan at the pose of the robot.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass

import agibot_gdk
import cv2
import numpy as np

from gdk_training import geometry, lidar, show, viewer
from gdk_training.drive import Drive

SCALE = 2  # screen pixels per cell
AUTO_RELOAD_S = 2.0
SCAN_Z_MIN = 0.1  # m, the lowest height of the scan
SCAN_Z_MAX = 1.8  # m, the highest height of the scan
SCAN_MIN_RANGE = 0.5  # m, closer points are the robot itself
CROP_MARGIN = 20  # pixels around the known part of the map

# The values of the cells in grid_map.data.
CELL_UNKNOWN = -1
CELL_FREE = 0
CELL_OCCUPIED = 100

# Colours in BGR, the channel order of OpenCV.
UNKNOWN_COLOR = (128, 128, 128)
FREE_COLOR = (255, 255, 255)
OCCUPIED_COLOR = (0, 0, 0)
ORIGIN_COLOR = (255, 0, 255)
GUIDE_COLOR = (255, 120, 0)
ROBOT_COLOR = (0, 160, 0)
CLICK_COLOR = (0, 0, 230)
SCAN_COLOR = (0, 150, 255)


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


def geometry_of(info: agibot_gdk.MapInfo) -> MapGeometry:
    """Reads size, resolution and origin of the grid."""
    # TODO: info.grid_map has width, height, resolution and origin, a Pose.
    grid_map = info.grid_map
    origin = grid_map.origin.position
    map_geometry = MapGeometry(
        width=grid_map.width,
        height=grid_map.height,
        resolution=grid_map.resolution,
        origin_x=origin.x,
        origin_y=origin.y,
    )
    return map_geometry


def world_to_pixel(map_geometry: MapGeometry, x: float, y: float) -> tuple[float, float]:
    """Converts a point of the map frame in m to image coordinates u to the right, v down.

    Pixel (u, v) covers u..u+1 and v..v+1. Image row 0 is the top row.
    """
    # TODO: Subtract the origin and divide by the resolution. Count v from the top of the image.
    u = (x - map_geometry.origin_x) / map_geometry.resolution
    rows_from_bottom = (y - map_geometry.origin_y) / map_geometry.resolution
    v = map_geometry.height - rows_from_bottom
    return u, v


def pixel_to_world(map_geometry: MapGeometry, u: float, v: float) -> tuple[float, float]:
    """The inverse of world_to_pixel."""
    # TODO: Solve world_to_pixel for x and y.
    x = map_geometry.origin_x + u * map_geometry.resolution
    rows_from_bottom = map_geometry.height - v
    y = map_geometry.origin_y + rows_from_bottom * map_geometry.resolution
    return x, y


def grid_image(info: agibot_gdk.MapInfo) -> np.ndarray:
    """Returns a BGR image (height, width, 3) uint8: unknown grey, free white, occupied black."""
    # TODO: Reshape grid_map.data to (height, width). Its row 0 is at the origin, at the bottom
    # of the image.
    grid_map = info.grid_map
    cells = np.asarray(grid_map.data, np.int16).reshape(grid_map.height, grid_map.width)

    image = np.full((grid_map.height, grid_map.width, 3), UNKNOWN_COLOR, np.uint8)
    image[cells == CELL_FREE] = FREE_COLOR
    image[cells == CELL_OCCUPIED] = OCCUPIED_COLOR

    # Row 0 of the grid is at the origin, but row 0 of an image is at the top.
    image_upside_down = np.flipud(image)
    return np.ascontiguousarray(image_upside_down)


def guide_points(info: agibot_gdk.MapInfo) -> list[GuidePoint]:
    """Returns the guide points of the map."""
    # TODO: info.guide_pts is a list with id, type and pt, a Pose. geometry.from_pose and
    # geometry.rpy give the yaw.
    points = []
    for guide in info.guide_pts:
        roll, pitch, yaw = geometry.rpy(geometry.from_pose(guide.pt))
        point = GuidePoint(id=guide.id, type=guide.type, x=guide.pt.position.x, y=guide.pt.position.y, yaw=yaw)
        points.append(point)
    return points


def robot_pose(slam: agibot_gdk.Slam) -> tuple[float, float, float]:
    """Returns x and y in m and yaw in rad of the robot on the current map."""
    # TODO: get_curr_pose returns a Pose.
    pose = slam.get_curr_pose()
    roll, pitch, yaw = geometry.rpy(geometry.from_pose(pose))
    return pose.position.x, pose.position.y, yaw


def changed(old: agibot_gdk.MapInfo, new: agibot_gdk.MapInfo) -> bool:
    """Stretch: tells whether two MapInfo of the same map differ in the grid or the guide points."""
    # TODO: counter and timestamp_ns stay 0 on the robot. Compare the content.
    old_cells = list(old.grid_map.data)
    new_cells = list(new.grid_map.data)
    if old_cells != new_cells:
        return True

    old_guides = guide_points(old)
    new_guides = guide_points(new)
    return old_guides != new_guides


def with_legend(image: np.ndarray) -> np.ndarray:
    """Adds a white strip on the right that explains the colours and marks."""
    strip_height = max(image.shape[0], 150)
    strip_width = 150
    strip = np.full((strip_height, strip_width, 3), 255, np.uint8)

    legend = [
        ("free", FREE_COLOR),
        ("occupied", OCCUPIED_COLOR),
        ("unknown", UNKNOWN_COLOR),
        ("map origin", ORIGIN_COLOR),
        ("guide point", GUIDE_COLOR),
        ("robot", ROBOT_COLOR),
        ("click", CLICK_COLOR),
        ("lidar scan", SCAN_COLOR),
    ]
    y = 20
    for text, color in legend:
        if text == "map origin":
            cv2.drawMarker(strip, (16, y), color, cv2.MARKER_CROSS, 12, 1)
        elif text == "guide point":
            cv2.circle(strip, (16, y), 4, color, -1)
        elif text == "click":
            cv2.drawMarker(strip, (16, y), color, cv2.MARKER_TILTED_CROSS, 10, 2)
        elif text == "robot":
            cv2.arrowedLine(strip, (8, y), (26, y), color, 2, tipLength=0.4)
        else:
            cv2.rectangle(strip, (10, y - 6), (22, y + 6), color, -1)  # filled square
            cv2.rectangle(strip, (10, y - 6), (22, y + 6), (160, 160, 160), 1)  # grey border
        cv2.putText(strip, text, (34, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
        y += 22

    # The map on the left, the strip on the right.
    image_height, image_width = image.shape[:2]
    canvas = np.full((strip_height, image_width + strip_width, 3), 255, np.uint8)
    canvas[:image_height, :image_width] = image
    canvas[:, image_width:] = strip
    return canvas


class MapDisplay:
    """The map image with guide points, origin, robot and lidar scan, drawn with your functions."""

    def __init__(self, map_api: agibot_gdk.Map, slam: agibot_gdk.Slam, map_id: int | None,
                 sensors: agibot_gdk.Lidar, tf: agibot_gdk.TF) -> None:
        self.map_api = map_api
        self.slam = slam
        self.map_id = map_id  # None means the current map
        self.sensors = sensors
        self.tf = tf
        self.scan = False  # whether the lidar scan is drawn
        self.checked = time.monotonic()  # when the map was last compared with the robot's
        self.auto = True  # False once changed raises NotImplementedError
        self.crop = (0, 0)  # grid pixel (u, v) of the top left corner of the shown image
        self.click: tuple[float, float] | None = None  # x, y in m of the last click
        self.click_error = ""
        self.load()

    def load(self) -> str:
        if self.map_id is not None:
            map_id = self.map_id
        else:
            map_id = self.map_api.get_curr_map().id
        self.info = self.map_api.get_map(map_id)
        self.map_geometry = geometry_of(self.info)
        self.guides = guide_points(self.info)
        self.base = grid_image(self.info)
        return f"map {map_id} loaded, {len(self.guides)} guide points"

    def reload_if_changed(self) -> None:
        if not self.auto:
            return
        if time.monotonic() - self.checked < AUTO_RELOAD_S:
            return
        self.checked = time.monotonic()
        try:
            latest = self.map_api.get_map(self.info.id)
            if changed(self.info, latest):
                self.load()
        except NotImplementedError:
            self.auto = False

    def pixel(self, x: float, y: float) -> tuple[int, int]:
        """The whole pixel that contains x, y."""
        u, v = world_to_pixel(self.map_geometry, x, y)
        return int(math.floor(u)), int(math.floor(v))

    def on_click(self, u: float, v: float) -> None:
        """Turns a click on the shown image into a point on the map."""
        # The shown image is cropped and enlarged. Undo both to get the pixel of the grid.
        crop_u, crop_v = self.crop
        grid_u = crop_u + u / SCALE
        grid_v = crop_v + v / SCALE
        is_on_grid = 0 <= grid_u < self.map_geometry.width and 0 <= grid_v < self.map_geometry.height
        if not is_on_grid:
            return  # a click on the legend
        try:
            self.click = pixel_to_world(self.map_geometry, grid_u, grid_v)
            self.click_error = ""
        except NotImplementedError:
            self.click = None
            self.click_error = "write pixel_to_world first"

    def toggle_scan(self) -> str:
        self.scan = not self.scan
        if self.scan:
            return "lidar scan on"
        return "lidar scan off"

    def draw_scan(self, image: np.ndarray, robot_x: float, robot_y: float, robot_yaw: float) -> None:
        """Draws the scan of both lidars at the pose of the robot: base_link to map with x, y and yaw."""
        points = lidar.scan_in_base(self.sensors, self.tf)
        points = lidar.height_band(points, SCAN_Z_MIN, SCAN_Z_MAX)
        points = lidar.without_robot(points, SCAN_MIN_RANGE)
        every_second_point = points[::2]  # enough for the picture, and twice as fast

        # Rotate each point by the yaw of the robot and move it to the position of the robot.
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        image_height, image_width = image.shape[:2]
        for point in every_second_point:
            x_in_base = point[0]
            y_in_base = point[1]
            x_on_map = robot_x + cos_yaw * x_in_base - sin_yaw * y_in_base
            y_on_map = robot_y + sin_yaw * x_in_base + cos_yaw * y_in_base
            u, v = self.pixel(x_on_map, y_on_map)
            if 0 <= u < image_width and 0 <= v < image_height:
                image[v, u] = SCAN_COLOR

    def draw_guide_points(self, image: np.ndarray) -> None:
        """Draws each guide point as a dot, a line in its direction and its id."""
        for point in self.guides:
            u, v = self.pixel(point.x, point.y)
            heading_x = point.x + 0.4 * math.cos(point.yaw)  # 0.4 m in the direction of the point
            heading_y = point.y + 0.4 * math.sin(point.yaw)
            cv2.circle(image, (u, v), 4, GUIDE_COLOR, -1)
            cv2.line(image, (u, v), self.pixel(heading_x, heading_y), GUIDE_COLOR, 1)
            cv2.putText(image, str(point.id), (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, GUIDE_COLOR, 1)

    def draw_robot(self, image: np.ndarray, robot_x: float, robot_y: float, robot_yaw: float) -> None:
        """Draws the robot as an arrow of 0.6 m in its direction."""
        heading_x = robot_x + 0.6 * math.cos(robot_yaw)
        heading_y = robot_y + 0.6 * math.sin(robot_yaw)
        start = self.pixel(robot_x, robot_y)
        end = self.pixel(heading_x, heading_y)
        cv2.arrowedLine(image, start, end, ROBOT_COLOR, 2, tipLength=0.3)

    def cropped(self, image: np.ndarray) -> np.ndarray:
        """Cuts the image to the part that is not unknown grey, with CROP_MARGIN around it."""
        is_known = np.any(image != UNKNOWN_COLOR, axis=2)
        known_pixels = np.argwhere(is_known)  # one row (v, u) per known pixel
        first_row, first_column = known_pixels.min(axis=0)
        last_row, last_column = known_pixels.max(axis=0)

        top = max(int(first_row) - CROP_MARGIN, 0)
        left = max(int(first_column) - CROP_MARGIN, 0)
        bottom = int(last_row) + CROP_MARGIN
        right = int(last_column) + CROP_MARGIN
        self.crop = (left, top)  # on_click needs it to find the pixel of a click
        return image[top:bottom, left:right]

    def image(self) -> np.ndarray:
        self.reload_if_changed()
        image = self.base.copy()

        origin = self.pixel(0.0, 0.0)
        cv2.drawMarker(image, origin, ORIGIN_COLOR, cv2.MARKER_CROSS, 12, 1)
        self.draw_guide_points(image)
        if self.click is not None:
            click_x, click_y = self.click
            cv2.drawMarker(image, self.pixel(click_x, click_y), CLICK_COLOR, cv2.MARKER_TILTED_CROSS, 10, 2)
        robot_x, robot_y, robot_yaw = robot_pose(self.slam)
        if self.scan:
            self.draw_scan(image, robot_x, robot_y, robot_yaw)
        self.draw_robot(image, robot_x, robot_y, robot_yaw)

        view = self.cropped(image)
        view_height, view_width = view.shape[:2]
        enlarged_size = (view_width * SCALE, view_height * SCALE)
        enlarged = cv2.resize(view, enlarged_size, interpolation=cv2.INTER_NEAREST)
        return with_legend(enlarged)

    def clicked(self) -> dict[str, object]:
        if self.click is None:
            return {"x": "-", "y": "-", "cell": self.click_error or "click on the map"}

        x, y = self.click
        u, v = self.pixel(x, y)
        # Keep the pixel inside the image, a click on the edge can land just outside.
        row = min(max(v, 0), self.map_geometry.height - 1)
        column = min(max(u, 0), self.map_geometry.width - 1)
        color = tuple(self.base[row, column].tolist())
        if color == FREE_COLOR:
            cell = "free"
        elif color == OCCUPIED_COLOR:
            cell = "occupied"
        else:
            cell = "unknown"
        return {"x": x, "y": y, "cell": cell}

    def table(self) -> dict[str, object]:
        map_section = {
            "id": self.info.id,
            "cells": f"{self.map_geometry.width} x {self.map_geometry.height}",
            "resolution": self.map_geometry.resolution,
            "origin": [self.map_geometry.origin_x, self.map_geometry.origin_y],
            "guide points": len(self.guides),
            "auto reload": self.auto,
        }
        x, y, yaw = robot_pose(self.slam)
        robot_section = {"x": x, "y": y, "yaw deg": math.degrees(yaw)}
        table = {
            "map": map_section,
            "robot": robot_section,
            "click": self.clicked(),
            "guide points": self.guides,
        }
        return table


def main() -> None:
    parser = argparse.ArgumentParser(description="Shows a map with guide points and the robot.")
    parser.add_argument("--map", type=int, help="map id, default: the current map")
    arguments = parser.parse_args()
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    map_api = agibot_gdk.Map()
    slam = agibot_gdk.Slam()
    pnc = agibot_gdk.Pnc()
    sensors = agibot_gdk.Lidar()
    tf = agibot_gdk.TF()
    time.sleep(2.0)
    drive = Drive(pnc)  # the drive keys of exercise 12
    try:
        display = MapDisplay(map_api, slam, arguments.map, sensors, tf)

        images = {"map": display.image}
        clicks = {"map": display.on_click}
        viewer.start(images, hz=2.0, clicks=clicks)

        keys: dict[str, Callable[[], object]] = {
            "r": display.load,
            "l": display.toggle_scan,
            **drive.keys,
        }

        def help_line() -> str:
            return f"r reload   l lidar scan   {drive.help()}"

        show.live(display.table, hz=2.0, keys=keys, help=help_line)
    finally:
        drive.release()
        sensors.close_lidar()
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
