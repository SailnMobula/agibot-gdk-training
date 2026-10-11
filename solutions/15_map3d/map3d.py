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

import agibot_gdk
import cv2
import numpy as np

from gdk_training import cloud3d, lidar, maps, show, viewer

BAND = (0.1, 1.8)  # m
BAND_STEP = 0.1
OCCUPIED = 100
MODES = ("as stored", "in the map frame")


def map_cloud(info: object) -> np.ndarray:
    """Returns the point cloud of the map, (N, 3), in the frame the robot stores it in."""
    # TODO: info.nonground_cloud is a point cloud like those of exercise 11. lidar.decode reads it.
    return lidar.decode(info.nonground_cloud)


def to_map(points: np.ndarray, tf: agibot_gdk.TF) -> np.ndarray:
    """Moves the cloud into the map frame.

    The robot stores the cloud in the frame of the front lidar at the moment mapping started. The
    map frame is base_link at that moment.
    """
    # TODO: lidar.sensor_to_base gives the extrinsic of the front lidar, lidar.to_base applies it.
    return lidar.to_base(points, lidar.sensor_to_base(tf, "front"))


def to_grid(points: np.ndarray, geo: maps.MapGeometry, z_min: float, z_max: float) -> np.ndarray:
    """Returns a bool array (height, width), row 0 at the origin like the robot's grid.

    A cell is True when at least one point between z_min and z_max falls into it.
    """
    # TODO: Keep the points of the band. Column and row are the distance from the origin divided by
    # the resolution, rounded down. Drop points outside the grid.
    band = points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)]
    column = np.floor((band[:, 0] - geo.origin_x) / geo.resolution).astype(int)
    row = np.floor((band[:, 1] - geo.origin_y) / geo.resolution).astype(int)
    keep = (row >= 0) & (row < geo.height) & (column >= 0) & (column < geo.width)
    grid = np.zeros((geo.height, geo.width), bool)
    grid[row[keep], column[keep]] = True
    return grid


def agreement(own: np.ndarray, info: object) -> dict[str, float]:
    """Compares your grid with the robot's.

    Returns:
        "own cells": the number of your occupied cells, "in occupied", "in free" and "in unknown":
        the share of them that the robot marks occupied, free and unknown, "robot found": the
        share of the robot's occupied cells that are also yours.
    """
    robot = maps.cells(info)
    count = int(own.sum())
    share = lambda mask: float((own & mask).sum()) / count if count else 0.0
    occupied = robot == OCCUPIED
    return {"own cells": count, "in occupied": share(occupied), "in free": share(robot == 0),
            "in unknown": share(robot == -1),
            "robot found": float((own & occupied).sum()) / max(1, int(occupied.sum()))}


def overlay(info: object, own: np.ndarray) -> np.ndarray:
    """Both grids in one image: both black, only yours red, only the robot's blue."""
    robot = maps.cells(info) == OCCUPIED
    image = maps.grid_image(info)
    image[(own & ~robot)[::-1]] = (0, 0, 230)
    image[(robot & ~own)[::-1]] = (230, 120, 0)
    image[(robot & own)[::-1]] = (0, 0, 0)
    view, _, _ = maps.crop(image, info)
    view = cv2.resize(view, (view.shape[1] * 2, view.shape[0] * 2), interpolation=cv2.INTER_NEAREST)
    return maps.with_legend(view, [("occupied in both", (0, 0, 0)), ("only in your grid", (0, 0, 230)),
                                   ("only in the robot's", (230, 120, 0)), ("free", (255, 255, 255)),
                                   ("unknown", (128, 128, 128))])


def main() -> None:
    parser = argparse.ArgumentParser(description="Shows the point cloud of a map and an own grid built from it.")
    parser.add_argument("--map", type=int, help="map id, default: the current map")
    arguments = parser.parse_args()
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    map_api, tf = agibot_gdk.Map(), agibot_gdk.TF()
    time.sleep(2.0)
    try:
        info = map_api.get_map(arguments.map if arguments.map is not None else map_api.get_curr_map().id)
        geo = maps.geometry_of(info)
        raw = map_cloud(info)
        clouds: dict[str, np.ndarray] = {MODES[0]: raw}
        state = {"mode": MODES[0], "band": BAND}
        result: dict = {}

        def cloud() -> np.ndarray:
            if state["mode"] not in clouds:
                clouds[state["mode"]] = to_map(raw, tf)
            return clouds[state["mode"]]

        def tilt() -> str:
            """The angle between gravity and -z. gravity is measured in the frame of the stored cloud."""
            g = np.array([info.gravity.x, info.gravity.y, info.gravity.z])
            if state["mode"] == MODES[1]:
                g = lidar.sensor_to_base(tf, "front")[:3, :3] @ g
            return f"{np.degrees(np.arccos(-g[2] / np.linalg.norm(g))):.1f} deg"

        def compute() -> None:
            own = to_grid(cloud(), geo, *state["band"])
            result.update(own=own, image=overlay(info, own), numbers=agreement(own, info))

        def next_mode() -> str:
            state["mode"] = MODES[(MODES.index(state["mode"]) + 1) % len(MODES)]
            compute()
            return f"cloud {state['mode']}"

        def shift(step: float) -> str:
            low, high = state["band"]
            state["band"] = (round(low + step, 2), round(high + step, 2))
            compute()
            return f"band {state['band'][0]:.1f} to {state['band'][1]:.1f} m"

        compute()
        floor = cloud3d.Floor(maps.grid_image(info), geo.origin_x, geo.origin_y, geo.resolution)
        cloud3d.start({"map cloud": cloud}, floor=lambda: floor, port=8001, title=f"Map {info.id}",
                      colors=("height",))
        viewer.start({"grids": lambda: result["image"]}, hz=2.0)
        show.live(lambda: {"settings": {"map": info.id, "points": len(raw), "cloud": state["mode"],
                                        "tilt of gravity": tilt(), "lowest z": float(np.percentile(cloud()[:, 2], 1)),
                                        "band": list(state["band"])}, "agreement": result["numbers"]},
                  hz=2.0, keys={"l": next_mode, "+": lambda: shift(BAND_STEP), "-": lambda: shift(-BAND_STEP)},
                  help="l as stored / map frame   + - move the band")
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
