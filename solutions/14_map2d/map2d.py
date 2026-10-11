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
from dataclasses import dataclass

import agibot_gdk
import cv2
import numpy as np

from gdk_training import geometry, lidar, show, viewer
from gdk_training.drive import Drive

SCALE = 2  # screen pixels per cell
AUTO_RELOAD_S = 2.0
SCAN_BAND = (0.1, 1.8)  # m, the heights of the scan
SCAN_MIN_RANGE = 0.5  # m, closer points are the robot itself


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


def geometry_of(info: object) -> MapGeometry:
    """Reads size, resolution and origin of the grid."""
    # TODO: info.grid_map has width, height, resolution and origin, a Pose.
    grid = info.grid_map
    return MapGeometry(width=grid.width, height=grid.height, resolution=grid.resolution,
                       origin_x=grid.origin.position.x, origin_y=grid.origin.position.y)


def world_to_pixel(geo: MapGeometry, x: float, y: float) -> tuple[float, float]:
    """Converts a point of the map frame in m to image coordinates u to the right, v down.

    Pixel (u, v) covers u..u+1 and v..v+1. Image row 0 is the top row.
    """
    # TODO: Subtract the origin and divide by the resolution. Count v from the top of the image.
    return (x - geo.origin_x) / geo.resolution, geo.height - (y - geo.origin_y) / geo.resolution


def pixel_to_world(geo: MapGeometry, u: float, v: float) -> tuple[float, float]:
    """The inverse of world_to_pixel."""
    # TODO: Solve world_to_pixel for x and y.
    return geo.origin_x + u * geo.resolution, geo.origin_y + (geo.height - v) * geo.resolution


def grid_image(info: object) -> np.ndarray:
    """Returns a BGR image (height, width, 3) uint8: unknown grey, free white, occupied black."""
    # TODO: Reshape grid_map.data to (height, width). Its row 0 is at the origin, at the bottom
    # of the image.
    grid = np.asarray(info.grid_map.data, np.int16).reshape(info.grid_map.height, info.grid_map.width)
    image = np.full((*grid.shape, 3), 128, np.uint8)
    image[grid == 0] = 255
    image[grid == 100] = 0
    return np.ascontiguousarray(image[::-1])


def guide_points(info: object) -> list[GuidePoint]:
    """Returns the guide points of the map."""
    # TODO: info.guide_pts is a list with id, type and pt, a Pose. geometry.from_pose and
    # geometry.rpy give the yaw.
    return [GuidePoint(id=point.id, type=point.type, x=point.pt.position.x, y=point.pt.position.y,
                       yaw=geometry.rpy(geometry.from_pose(point.pt))[2]) for point in info.guide_pts]


def robot_pose(slam: agibot_gdk.Slam) -> tuple[float, float, float]:
    """Returns x and y in m and yaw in rad of the robot on the current map."""
    # TODO: get_curr_pose returns a Pose.
    pose = slam.get_curr_pose()
    return pose.position.x, pose.position.y, geometry.rpy(geometry.from_pose(pose))[2]


def changed(old: object, new: object) -> bool:
    """Stretch: tells whether two MapInfo of the same map differ in the grid or the guide points."""
    # TODO: counter and timestamp_ns stay 0 on the robot. Compare the content.
    def guides(info: object) -> list[tuple]:
        return [(p.id, p.type, round(p.x, 3), round(p.y, 3), round(p.yaw, 3)) for p in guide_points(info)]
    return list(old.grid_map.data) != list(new.grid_map.data) or guides(old) != guides(new)


ORIGIN_COLOR, GUIDE_COLOR, ROBOT_COLOR, CLICK_COLOR = (255, 0, 255), (255, 120, 0), (0, 160, 0), (0, 0, 230)  # BGR
SCAN_COLOR = (0, 150, 255)


def with_legend(image: np.ndarray) -> np.ndarray:
    """Adds a white strip on the right that explains the colours and marks."""
    strip = np.full((max(image.shape[0], 150), 150, 3), 255, np.uint8)
    rows = [("free", (255, 255, 255)), ("occupied", (0, 0, 0)), ("unknown", (128, 128, 128)),
            ("map origin", ORIGIN_COLOR), ("guide point", GUIDE_COLOR), ("robot", ROBOT_COLOR), ("click", CLICK_COLOR),
            ("lidar scan", SCAN_COLOR)]
    for i, (text, color) in enumerate(rows):
        y = 20 + 22 * i
        if text == "map origin":
            cv2.drawMarker(strip, (16, y), color, cv2.MARKER_CROSS, 12, 1)
        elif text == "guide point":
            cv2.circle(strip, (16, y), 4, color, -1)
        elif text == "click":
            cv2.drawMarker(strip, (16, y), color, cv2.MARKER_TILTED_CROSS, 10, 2)
        elif text == "robot":
            cv2.arrowedLine(strip, (8, y), (26, y), color, 2, tipLength=0.4)
        else:
            cv2.rectangle(strip, (10, y - 6), (22, y + 6), color, -1)
            cv2.rectangle(strip, (10, y - 6), (22, y + 6), (160, 160, 160), 1)
        cv2.putText(strip, text, (34, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    canvas = np.full((strip.shape[0], image.shape[1] + strip.shape[1], 3), 255, np.uint8)
    canvas[:image.shape[0], :image.shape[1]] = image
    canvas[:, image.shape[1]:] = strip
    return canvas


class MapDisplay:
    """The map image with guide points, origin, robot and lidar scan, drawn with your functions."""

    def __init__(self, map_api: agibot_gdk.Map, slam: agibot_gdk.Slam, map_id: int | None,
                 sensors: agibot_gdk.Lidar, tf: agibot_gdk.TF) -> None:
        self.map_api, self.slam, self.map_id, self.sensors, self.tf = map_api, slam, map_id, sensors, tf
        self.scan = False
        self.checked = time.monotonic()
        self.auto = True
        self.crop = (0, 0)  # grid pixel of the top left corner of the shown image
        self.click: tuple[float, float] | None = None
        self.click_error = ""
        self.load()

    def load(self) -> str:
        map_id = self.map_id if self.map_id is not None else self.map_api.get_curr_map().id
        self.info = self.map_api.get_map(map_id)
        self.geo = geometry_of(self.info)
        self.guides = guide_points(self.info)
        self.base = grid_image(self.info)
        return f"map {map_id} loaded, {len(self.guides)} guide points"

    def reload_if_changed(self) -> None:
        if not self.auto or time.monotonic() - self.checked < AUTO_RELOAD_S:
            return
        self.checked = time.monotonic()
        try:
            if changed(self.info, self.map_api.get_map(self.info.id)):
                self.load()
        except NotImplementedError:
            self.auto = False

    def pixel(self, x: float, y: float) -> tuple[int, int]:
        u, v = world_to_pixel(self.geo, x, y)
        return int(math.floor(u)), int(math.floor(v))

    def on_click(self, u: float, v: float) -> None:
        """Turns a click on the shown image into a point on the map."""
        u, v = self.crop[0] + u / SCALE, self.crop[1] + v / SCALE
        if not (0 <= u < self.geo.width and 0 <= v < self.geo.height):
            return  # the legend
        try:
            self.click, self.click_error = pixel_to_world(self.geo, u, v), ""
        except NotImplementedError:
            self.click, self.click_error = None, "write pixel_to_world first"

    def toggle_scan(self) -> str:
        self.scan = not self.scan
        return f"lidar scan {'on' if self.scan else 'off'}"

    def draw_scan(self, image: np.ndarray, x: float, y: float, yaw: float) -> None:
        """Draws the scan of both lidars at the pose of the robot: base_link to map with x, y and yaw."""
        points = lidar.scan_in_base(self.sensors, self.tf)
        points = lidar.without_robot(lidar.height_band(points, *SCAN_BAND), SCAN_MIN_RANGE)[::2]
        c, s = math.cos(yaw), math.sin(yaw)
        for px, py in points[:, :2]:
            u, v = self.pixel(x + c * px - s * py, y + s * px + c * py)
            if 0 <= u < image.shape[1] and 0 <= v < image.shape[0]:
                image[v, u] = SCAN_COLOR

    def image(self) -> np.ndarray:
        self.reload_if_changed()
        image = self.base.copy()
        origin = self.pixel(0.0, 0.0)
        cv2.drawMarker(image, origin, ORIGIN_COLOR, cv2.MARKER_CROSS, 12, 1)
        for point in self.guides:
            u, v = self.pixel(point.x, point.y)
            cv2.circle(image, (u, v), 4, GUIDE_COLOR, -1)
            cv2.line(image, (u, v), self.pixel(point.x + 0.4 * math.cos(point.yaw), point.y + 0.4 * math.sin(point.yaw)),
                     GUIDE_COLOR, 1)
            cv2.putText(image, str(point.id), (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, GUIDE_COLOR, 1)
        if self.click:
            cv2.drawMarker(image, self.pixel(*self.click), CLICK_COLOR, cv2.MARKER_TILTED_CROSS, 10, 2)
        x, y, yaw = robot_pose(self.slam)
        if self.scan:
            self.draw_scan(image, x, y, yaw)
        cv2.arrowedLine(image, self.pixel(x, y), self.pixel(x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw)),
                        ROBOT_COLOR, 2, tipLength=0.3)
        known = np.argwhere(image.min(axis=2) != 128)
        (v0, u0), (v1, u1) = np.maximum(known.min(0) - 20, 0), known.max(0) + 20
        self.crop = (int(u0), int(v0))
        view = image[v0:v1, u0:u1]
        view = cv2.resize(view, (view.shape[1] * SCALE, view.shape[0] * SCALE), interpolation=cv2.INTER_NEAREST)
        return with_legend(view)

    def clicked(self) -> dict:
        if self.click is None:
            return {"x": "-", "y": "-", "cell": self.click_error or "click on the map"}
        x, y = self.click
        u, v = self.pixel(x, y)
        value = self.base[min(max(v, 0), self.geo.height - 1), min(max(u, 0), self.geo.width - 1), 0]
        return {"x": x, "y": y, "cell": {255: "free", 0: "occupied"}.get(int(value), "unknown")}

    def table(self) -> dict:
        x, y, yaw = robot_pose(self.slam)
        return {"map": {"id": self.info.id, "cells": f"{self.geo.width} x {self.geo.height}",
                        "resolution": self.geo.resolution, "origin": [self.geo.origin_x, self.geo.origin_y],
                        "guide points": len(self.guides), "auto reload": self.auto},
                "robot": {"x": x, "y": y, "yaw deg": math.degrees(yaw)},
                "click": self.clicked(),
                "guide points": self.guides}


def main() -> None:
    parser = argparse.ArgumentParser(description="Shows a map with guide points and the robot.")
    parser.add_argument("--map", type=int, help="map id, default: the current map")
    arguments = parser.parse_args()
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    map_api, slam, pnc = agibot_gdk.Map(), agibot_gdk.Slam(), agibot_gdk.Pnc()
    sensors, tf = agibot_gdk.Lidar(), agibot_gdk.TF()
    time.sleep(2.0)
    drive = Drive(pnc)
    try:
        display = MapDisplay(map_api, slam, arguments.map, sensors, tf)
        viewer.start({"map": display.image}, hz=2.0, clicks={"map": display.on_click})
        show.live(display.table, hz=2.0, keys={"r": display.load, "l": display.toggle_scan, **drive.keys},
                  help=lambda: f"r reload   l lidar scan   {drive.help()}")
    finally:
        drive.release()
        sensors.close_lidar()
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
