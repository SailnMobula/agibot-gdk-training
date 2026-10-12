"""Navigation part of the fake: lidar, SLAM, maps and PNC (planner and chassis).

The world is the map recorded on the robot (data/map_<id>.*): occupancy grid, map point cloud,
gravity and guide points. The fake keeps the true pose of the robot on that map. SLAM reports
it, chassis commands and tasks move it, and the lidars ray-cast against the occupied cells.

Behaviour copied from the robot (GDK 3.3.8, 2026-10-09):
- walls and infeasible_areas of a MapInfo raise TypeError, guide points have no name
- the chassis stops 0.6 s after the last move_chassis
- a task cannot start while another one is active, e.g. the remote control task of the chassis
- normal_navi keeps the task running while it retries, appends the message once per attempt and
  fails after about 6 s with "goal occupied" when the goal lies in or next to an obstacle
- relative_move drives straight

The map id comes from the file name. The fake has no virtual walls or no-go zones: the GDK cannot
hand them out, so the fake cannot copy them faithfully. Try those on the robot.

FAKE_INSTANT=1 finishes moves, retries and map switches at once. FAKE_POSE_ERROR=dx,dy,dyaw_deg
shifts the pose that SLAM reports against the true pose. FAKE_REMOTE_CONTROL=1 starts with a
leftover remote control task, as the robot does after the HMI or a crashed script.
"""

from __future__ import annotations

import heapq
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

import agibot_gdk as _gdk
from gdk_training import geometry

DATA = Path(__file__).with_name("data")
LIDAR_DATA = json.loads((DATA / "lidar.json").read_text())

CHASSIS_TIMEOUT_S = 0.6
TRANSITION_S = 0.3
NAVI_SPEED = 0.3          # m/s
NAVI_TURN = 0.5           # rad/s
RELATIVE_SPEED = 0.25     # m/s
RELATIVE_OVERSHOOT = 1.06
RETRY_S = 1.2
RETRIES = 5
ROBOT_RADIUS = 0.35       # m, obstacles are inflated by this for planning
BODY_RADIUS = 0.25        # m, the robot stops when an occupied cell is closer
PLAN_STEP = 2             # planning grid = map grid downsampled by this
MAP_SWITCH_S = 1.0
CEILING = 3.0             # m
MAX_RANGE = 30.0          # m
AZIMUTH_BINS = 1080
ACTIVE = (1, 2, 3, 4, 5, 6)
TYPE_NAVIGATION, TYPE_REMOTE_CONTROL, TYPE_RELATIVE_MOVE = 1, 3, 4
GOAL_OCCUPIED = "No valid path could be found: goal occupied"
NO_PATH = "No valid path could be found"
NOT_IDLE = "Task is not in IDLE, FAIL, SUCCESS or CANCELED state"
UNREADABLE = ("Unable to convert function return value to a Python type! The signature was\n\t"
              "(self: agibot_gdk.agibot_gdk.MapInfo) -> list[list[agibot::gdk::Point3d]]")

# Self hits: boxes of the robot body in base_link, (min xyz, max xyz).
BODY = [((-0.30, -0.18, 0.0), (0.25, 0.18, 0.28)), ((-0.15, -0.15, 0.28), (0.15, 0.15, 1.6))]


def instant() -> bool:
    return bool(os.environ.get("FAKE_INSTANT"))


def log_error(text: str) -> None:
    """Prints a line like the glog errors of the GDK."""
    print(f"E{time.strftime('%m%d %H:%M:%S')} pnc_impl.cpp] [GDK]{text}", file=sys.stderr)


class LidarType(Enum):
    kLidarUnknown = 0
    kLidarFront = 1
    kLidarBack = 2


class PointField:
    def __init__(self, name: str = "", offset: int = 0, datatype: int = 0, count: int = 1) -> None:
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, count


class PointCloud:
    def __init__(self, data: np.ndarray | None = None, fields: list[dict] | None = None, point_step: int = 0,
                 timestamp_ns: int = 0, is_dense: bool = False) -> None:
        self.data = np.zeros(0, np.uint8) if data is None else data
        self.fields = [PointField(**f) for f in fields or []]
        self.point_step = point_step
        self.width = len(self.data) // point_step if point_step else 0
        self.height = 1 if self.width else 0
        self.row_step = len(self.data)
        self.data_size = len(self.data)
        self.is_bigendian = False
        self.is_dense = is_dense
        self.timestamp_ns = timestamp_ns


class OccupancyGrid:
    def __init__(self) -> None:
        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin = _gdk.Pose()
        self.data: list[int] = []
        self.timestamp_ns = 0


class GuidePtInfo:
    def __init__(self) -> None:
        self.id = 0
        self.pt = _gdk.Pose()
        self.timestamp_ns = 0
        self.type = 0


class MapName:
    def __init__(self, id: int = 0, is_curr_map: bool = False) -> None:
        self.id, self.is_curr_map, self.name = id, is_curr_map, ""


class MapInfo:
    """Like the real MapInfo. walls and infeasible_areas cannot be converted on the robot either."""

    def __init__(self) -> None:
        self.id = 0
        self.name = ""
        self.counter = 0
        self.timestamp_ns = 0
        self.status = 1
        self.gravity = _gdk.Vector3()
        self.grid_map = OccupancyGrid()
        self.guide_pts: list[GuidePtInfo] = []
        self.ground_cloud = PointCloud()
        self.nonground_cloud = PointCloud()

    @property
    def walls(self) -> list:
        raise TypeError(UNREADABLE)

    @property
    def infeasible_areas(self) -> list:
        raise TypeError(UNREADABLE)


class NaviReq:
    def __init__(self) -> None:
        self.target = _gdk.Pose()


class PNCTaskState:
    def __init__(self, id: int = 0, state: int = 0, type: int = 0, message: str = "") -> None:
        self.id, self.state, self.type, self.message = id, state, type, message


@dataclass
class MapData:
    id: int
    grid: np.ndarray            # (height, width) int8, row 0 at the origin
    resolution: float
    origin: tuple[float, float]
    gravity: tuple[float, float, float]
    guide_pts: list[dict]
    cloud: np.ndarray           # raw bytes of the map cloud
    cloud_fields: list[dict]
    cloud_step: int
    _plan: np.ndarray | None = None

    def cell(self, x: float, y: float) -> tuple[int, int]:
        """Returns row, column of the cell that contains the point."""
        return int(math.floor((y - self.origin[1]) / self.resolution)), int(math.floor((x - self.origin[0]) / self.resolution))

    def inside(self, row: int, column: int) -> bool:
        return 0 <= row < self.grid.shape[0] and 0 <= column < self.grid.shape[1]

    def planning(self) -> np.ndarray:
        """Returns the inflated obstacles and unknown cells on the planning grid."""
        if self._plan is None:
            disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * int(ROBOT_RADIUS / self.resolution) + 1,) * 2)
            walls = cv2.dilate((self.grid == 100).astype(np.uint8), disk).astype(bool) | (self.grid == -1)
            h, w = (s // PLAN_STEP * PLAN_STEP for s in self.grid.shape)
            self._plan = walls[:h, :w].reshape(h // PLAN_STEP, PLAN_STEP, w // PLAN_STEP, PLAN_STEP).any(axis=(1, 3))
        return self._plan


def world_file() -> Path:
    """Returns data/map_<id>.json, the one recorded map. Rename both files to give it another id."""
    files = sorted(DATA.glob("map_*.json"))
    if len(files) != 1:
        raise RuntimeError(f"expected one data/map_<id>.json, found {[f.name for f in files]}")
    return files[0]


def load_map(path: Path) -> MapData:
    """Loads data/map_<id>.json and data/map_<id>.npz. The id is taken from the file name."""
    meta = json.loads(path.read_text())
    arrays = np.load(path.with_suffix(".npz"))
    step = meta["cloud_point_step"]
    offsets = {f["name"]: f["offset"] for f in meta["cloud_fields"]}
    layout = np.dtype({"names": ["x", "y", "z", "reflectivity", "tag"],
                       "formats": ["<f4", "<f4", "<f4", "u1", "u1"],
                       "offsets": [offsets[n] for n in ("x", "y", "z", "reflectivity", "tag")], "itemsize": step})
    cloud = np.zeros(len(arrays["cloud_xyz"]), layout)
    for i, axis in enumerate("xyz"):
        cloud[axis] = arrays["cloud_xyz"][:, i]
    cloud["reflectivity"], cloud["tag"] = arrays["cloud_reflectivity"], arrays["cloud_tag"]
    gravity = meta["gravity"]
    return MapData(id=int(path.stem.removeprefix("map_")), grid=arrays["grid"], resolution=meta["resolution"],
                   origin=(meta["origin"]["x"], meta["origin"]["y"]), gravity=(gravity["x"], gravity["y"], gravity["z"]),
                   guide_pts=meta["guide_pts"], cloud=cloud.view(np.uint8).reshape(-1), cloud_fields=meta["cloud_fields"],
                   cloud_step=step)


@dataclass
class Task:
    id: int
    type: int
    state: int
    since: float                                  # time of the last state change
    lines: list[str] = field(default_factory=list)
    path: list[tuple[float, float]] = field(default_factory=list)
    goal_yaw: float | None = None
    speed: float = NAVI_SPEED
    failure: str = ""                             # the planner fails with this message after its retries
    attempts: int = 0
    blocking: np.ndarray | None = None            # cells that stop a relative move

    @property
    def message(self) -> str:
        return "\n".join(self.lines)


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def astar(blocked: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]] | None:
    """8-connected A* on a bool grid. Returns the cells from start to goal or None."""
    rows, columns = blocked.shape
    moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    cost = {start: 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    queue = [(0.0, start)]
    while queue:
        _, current = heapq.heappop(queue)
        if current == goal:
            path = [current]
            while current in parent:
                current = parent[current]
                path.append(current)
            return path[::-1]
        r, c = current
        for dr, dc, step in moves:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < columns) or blocked[nr, nc]:
                continue
            new = cost[current] + step
            if new < cost.get((nr, nc), math.inf):
                cost[(nr, nc)] = new
                parent[(nr, nc)] = current
                heapq.heappush(queue, (new + math.hypot(goal[0] - nr, goal[1] - nc), (nr, nc)))
    return None


class NavWorld:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.world_map = load_map(world_file())   # the physical world, also when another map is current
        self.maps: dict[int, MapData] = {self.world_map.id: self.world_map}
        self.current = self.world_map.id
        self.pending: tuple[int, float] | None = None
        x, y, yaw = json.loads(world_file().read_text())["start_pose"]
        self.x, self.y, self.yaw = x, y, yaw
        self.slam_state = 2
        self.task: Task | None = None
        self.next_id = 1
        self.command = (0.0, 0.0)
        self.command_time = -math.inf
        self.time = time.monotonic()
        disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * int(BODY_RADIUS / self.world_map.resolution) + 1,) * 2)
        self.body_blocked = cv2.dilate((self.world_map.grid == 100).astype(np.uint8), disk).astype(bool)
        self.scans: dict[LidarType, tuple[float, PointCloud]] = {}
        if os.environ.get("FAKE_REMOTE_CONTROL"):
            self.start_task(TYPE_REMOTE_CONTROL, state=2)

    # --- time ---------------------------------------------------------------------------------

    def step(self) -> None:
        """Advances the world to now. Every public call does this first."""
        now = time.monotonic()
        dt, self.time = now - self.time, now
        if self.pending and (instant() or now >= self.pending[1]):
            self.current, self.pending = self.pending[0], None
        task = self.task
        if task is None:
            return
        if task.type == TYPE_REMOTE_CONTROL:
            if task.state == 2 and now - self.command_time < CHASSIS_TIMEOUT_S:
                vx, wz = self.command
                self.yaw = wrap(self.yaw + wz * dt)
                self.try_move(self.x + vx * dt * math.cos(self.yaw), self.y + vx * dt * math.sin(self.yaw))
            if task.state == 6 and (instant() or now - task.since >= TRANSITION_S):
                self.set_state(task, 7)
            return
        self.advance(task, dt, now)

    def advance(self, task: Task, dt: float, now: float) -> None:
        transitions = {1: 2, 3: 4, 5: 2, 6: 7}
        while task.state in transitions and (instant() or now - task.since >= TRANSITION_S):
            self.set_state(task, transitions[task.state])
        if task.state != 2:
            return
        if task.failure:
            running = now - task.since
            attempts = RETRIES if instant() else min(RETRIES, 1 + int(running / RETRY_S))
            while task.attempts < attempts:
                task.attempts += 1
                task.lines.append(task.failure)
            if instant() or running >= RETRY_S * RETRIES + 0.3:
                self.set_state(task, 8)
            return
        budget = math.inf if instant() else dt
        while budget > 0 and task.path:
            tx, ty = task.path[0]
            distance = math.hypot(tx - self.x, ty - self.y)
            if distance > 1e-6 and task.type == TYPE_NAVIGATION:
                self.yaw = math.atan2(ty - self.y, tx - self.x)
            reach = min(distance, task.speed * budget)
            budget -= reach / task.speed
            nx = self.x + (tx - self.x) * (reach / distance if distance > 1e-6 else 1.0)
            ny = self.y + (ty - self.y) * (reach / distance if distance > 1e-6 else 1.0)
            if task.blocking is not None and self.blocked_at(nx, ny, task.blocking):
                task.lines.append("Obstacle in the way, relative move stopped")
                self.set_state(task, 8)
                return
            self.x, self.y = nx, ny
            if reach >= distance - 1e-9:
                task.path.pop(0)
        if task.path:
            return
        if task.goal_yaw is not None:
            turn = wrap(task.goal_yaw - self.yaw)
            limit = math.inf if instant() else NAVI_TURN * max(budget, 0.0)
            self.yaw = wrap(self.yaw + max(-limit, min(limit, turn)))
            if abs(wrap(task.goal_yaw - self.yaw)) > 1e-3:
                return
        self.set_state(task, 9)

    def blocked_at(self, x: float, y: float, blocked: np.ndarray) -> bool:
        row, column = self.world_map.cell(x, y)
        return not self.world_map.inside(row, column) or bool(blocked[row, column])

    def try_move(self, x: float, y: float) -> None:
        """Moves the robot unless the body would touch an occupied cell. Leaving such a cell is allowed."""
        if self.blocked_at(x, y, self.body_blocked) and not self.blocked_at(self.x, self.y, self.body_blocked):
            return
        self.x, self.y = x, y

    # --- tasks --------------------------------------------------------------------------------

    def set_state(self, task: Task, state: int) -> None:
        task.state, task.since = state, time.monotonic()

    def active(self) -> bool:
        return self.task is not None and self.task.state in ACTIVE

    def start_task(self, task_type: int, state: int = 1, **kwargs: object) -> Task:
        self.task = Task(id=self.next_id, type=task_type, state=state, since=time.monotonic(), **kwargs)
        self.next_id += 1
        return self.task

    def find(self, task_id: int, call: str) -> Task:
        if self.task is None or self.task.id != task_id:
            raise RuntimeError(f"{call} failed")
        return self.task

    def plan(self, x: float, y: float) -> tuple[list[tuple[float, float]], str]:
        """Plans on the current map. Returns the path in metres or a failure message."""
        data = self.maps[self.current]
        walls = data.planning()
        step = data.resolution * PLAN_STEP
        to_cell = lambda px, py: (int((py - data.origin[1]) / step), int((px - data.origin[0]) / step))
        start, goal = to_cell(self.x, self.y), to_cell(x, y)
        inside = lambda cell: 0 <= cell[0] < walls.shape[0] and 0 <= cell[1] < walls.shape[1]
        if not inside(goal) or walls[goal]:
            return [], GOAL_OCCUPIED
        if not inside(start):
            return [], NO_PATH
        blocked = walls.copy()
        blocked[max(0, start[0] - 1):start[0] + 2, max(0, start[1] - 1):start[1] + 2] = False  # leave a wall nearby
        cells = astar(blocked, start, goal)
        if cells is None:
            return [], NO_PATH
        points = [(data.origin[0] + (c + 0.5) * step, data.origin[1] + (r + 0.5) * step) for r, c in cells[1:-1:3]]
        return points + [(x, y)], ""

    # --- lidar --------------------------------------------------------------------------------

    def map_from_base(self) -> np.ndarray:
        return geometry.matrix((self.x, self.y, 0.0), (0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2)))

    def scan(self, lidar_type: LidarType) -> PointCloud:
        now = time.monotonic()
        cached = self.scans.get(lidar_type)
        if cached and now - cached[0] < 0.1:
            return cached[1]
        name = "kChassisFrontLidarToBaseLink" if lidar_type == LidarType.kLidarFront else "kChassisBackLidarToBaseLink"
        extrinsic = LIDAR_DATA["extrinsics"][name]
        base_from_sensor = geometry.matrix(tuple(extrinsic["translation"]), tuple(extrinsic["rotation_xyzw"]))
        cloud = make_scan(self.world_map, self.map_from_base(), base_from_sensor, LIDAR_DATA["points"])
        self.scans[lidar_type] = (now, cloud)
        return cloud


def make_scan(world: MapData, map_from_base: np.ndarray, base_from_sensor: np.ndarray, count: int) -> PointCloud:
    """Ray-casts a scan like a Mid-360: random directions, -7 to 52 degrees elevation, in the sensor frame.

    Occupied cells are walls from the floor to the ceiling. The robot body gives self hits.
    """
    rng = np.random.default_rng()
    azimuth = rng.uniform(-math.pi, math.pi, count)
    elevation = np.radians(rng.uniform(-7.0, 52.0, count))
    directions = np.stack([np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth), np.sin(elevation)], 1)
    map_from_sensor = map_from_base @ base_from_sensor
    origin = map_from_sensor[:3, 3]
    rays = directions @ map_from_sensor[:3, :3].T
    horizontal = np.hypot(rays[:, 0], rays[:, 1])

    # Distance to the first occupied cell for each azimuth bin of the map frame.
    bins = np.linspace(-math.pi, math.pi, AZIMUTH_BINS, endpoint=False)
    steps = np.arange(0.0, MAX_RANGE, world.resolution / 2)
    xs = origin[0] + np.outer(np.cos(bins), steps)
    ys = origin[1] + np.outer(np.sin(bins), steps)
    rows = np.floor((ys - world.origin[1]) / world.resolution).astype(int)
    columns = np.floor((xs - world.origin[0]) / world.resolution).astype(int)
    valid = (rows >= 0) & (rows < world.grid.shape[0]) & (columns >= 0) & (columns < world.grid.shape[1])
    hit = np.zeros(rows.shape, bool)
    hit[valid] = world.grid[rows[valid], columns[valid]] == 100
    first = np.where(hit.any(1), hit.argmax(1), len(steps) - 1)
    wall = steps[first]

    ray_bin = ((np.arctan2(rays[:, 1], rays[:, 0]) + math.pi) / (2 * math.pi) * AZIMUTH_BINS).astype(int) % AZIMUTH_BINS
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(horizontal > 1e-6, wall[ray_bin] / horizontal, math.inf)
        t = np.minimum(t, np.where(rays[:, 2] < 0, -origin[2] / rays[:, 2], math.inf))
        t = np.minimum(t, np.where(rays[:, 2] > 0, (CEILING - origin[2]) / rays[:, 2], math.inf))
    t = np.minimum(t, MAX_RANGE)
    reflectivity = np.where(rays[:, 2] * t <= -origin[2] + 0.01, 25, 90).astype(np.uint8)

    # Self hits on the body boxes, computed in base_link.
    base_origin = base_from_sensor[:3, 3]
    base_rays = directions @ base_from_sensor[:3, :3].T
    for low, high in BODY:
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (np.asarray(low) - base_origin) / base_rays
            t2 = (np.asarray(high) - base_origin) / base_rays
        near = np.nanmax(np.minimum(t1, t2), axis=1)
        far = np.nanmin(np.maximum(t1, t2), axis=1)
        body = (near <= far) & (near > 0) & (near < t)
        t = np.where(body, near, t)
        reflectivity = np.where(body, 200, reflectivity).astype(np.uint8)

    t = t + rng.normal(0.0, 0.01, count)
    points = directions * t[:, None]
    fields = LIDAR_DATA["fields"]
    offsets = {f["name"]: f["offset"] for f in fields}
    layout = np.dtype({"names": ["x", "y", "z", "reflectivity", "tag", "timestamp"],
                       "formats": ["<f4", "<f4", "<f4", "u1", "u1", "<f8"],
                       "offsets": [offsets[n] for n in ("x", "y", "z", "reflectivity", "tag", "timestamp")],
                       "itemsize": LIDAR_DATA["point_step"]})
    out = np.zeros(count, layout)
    out["x"], out["y"], out["z"] = points[:, 0], points[:, 1], points[:, 2]
    out["reflectivity"] = reflectivity
    out["timestamp"] = time.time() + np.linspace(0.0, 0.1, count)
    return PointCloud(out.view(np.uint8).reshape(-1), fields, LIDAR_DATA["point_step"], time.time_ns())


world = NavWorld()


def _pose_xy_yaw(x: float, y: float, yaw: float) -> _gdk.Pose:
    pose = _gdk.Pose()
    pose.position = _gdk._vector3(x, y, 0.0)
    pose.orientation = _gdk.Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return pose


def _yaw(pose: object) -> float:
    return geometry.rpy(geometry.from_pose(pose))[2]


class Lidar:
    def __init__(self, lidar_types: list[LidarType] | None = None) -> None:
        self.types = set(lidar_types or [LidarType.kLidarFront, LidarType.kLidarBack])

    def get_latest_pointcloud(self, lidar_type: LidarType, timeout_ms: float = 100.0) -> PointCloud:
        if lidar_type not in self.types:
            raise RuntimeError(f"{lidar_type} was not requested from this Lidar")
        with world.lock:
            world.step()
            return world.scan(lidar_type)

    def get_lidar_fps(self, lidar_type: LidarType) -> float:
        return LIDAR_DATA["fps"][lidar_type.name]

    def get_lidar_latency(self, lidar_type: LidarType, window_s: float = 5.0) -> object:
        return SimpleNamespace(**LIDAR_DATA["latency"][lidar_type.name])

    def close_lidar(self) -> object:
        return _gdk.GDKRes.kSuccess


class Slam:
    def get_slam_state(self) -> int:
        with world.lock:
            world.step()
            return world.slam_state

    def get_curr_pose(self) -> _gdk.Pose:
        with world.lock:
            world.step()
            dx, dy, dyaw = (float(v) for v in (os.environ.get("FAKE_POSE_ERROR") or "0,0,0").split(","))
            return _pose_xy_yaw(world.x + dx, world.y + dy, world.yaw + math.radians(dyaw))

    def start_mapping(self) -> None:
        with world.lock:
            if world.slam_state == 1:
                raise RuntimeError("StartMapping failed")
            world.slam_state = 1

    def stop_mapping(self) -> None:
        """Saves the new map with the highest id plus one, as on the robot. It becomes the current map
        about 1 s later."""
        with world.lock:
            if world.slam_state != 1:
                raise RuntimeError("StopMapping failed")
            world.slam_state = 2
            source = world.world_map
            new_id = max(world.maps) + 1
            world.maps[new_id] = MapData(id=new_id, grid=source.grid.copy(), resolution=source.resolution,
                                         origin=source.origin, gravity=source.gravity, guide_pts=[], cloud=source.cloud,
                                         cloud_fields=source.cloud_fields, cloud_step=source.cloud_step)
            world.pending = (new_id, time.monotonic() + MAP_SWITCH_S)

    def cancel_mapping(self) -> None:
        with world.lock:
            if world.slam_state != 1:
                raise RuntimeError("CancelMapping failed")
            world.slam_state = 0


class Map:
    def get_all_map(self) -> list[MapName]:
        with world.lock:
            world.step()
            return [MapName(map_id, map_id == world.current) for map_id in sorted(world.maps)]

    def get_curr_map(self) -> MapName:
        with world.lock:
            world.step()
            return MapName(world.current, True)

    def get_map(self, map_id: int) -> MapInfo:
        with world.lock:
            world.step()
            if map_id not in world.maps:
                raise RuntimeError(f"GetMap failed: no map {map_id}")
            data = world.maps[map_id]
        info = MapInfo()
        info.id = data.id
        info.gravity = _gdk._vector3(*data.gravity)
        grid = info.grid_map
        grid.height, grid.width = data.grid.shape
        grid.resolution = data.resolution
        grid.origin = _pose_xy_yaw(data.origin[0], data.origin[1], 0.0)
        grid.data = data.grid.ravel().tolist()
        for point in data.guide_pts:
            guide = GuidePtInfo()
            guide.id, guide.type = point["id"], point["type"]
            guide.pt.position = _gdk._vector3(**point["position"])
            guide.pt.orientation = _gdk.Quaternion(**point["orientation"])
            info.guide_pts.append(guide)
        info.nonground_cloud = PointCloud(data.cloud, data.cloud_fields, data.cloud_step, is_dense=True)
        return info

    def switch_map(self, map_id: int) -> None:
        with world.lock:
            if map_id not in world.maps:
                raise RuntimeError(f"SwitchMap failed: no map {map_id}")
            world.current, world.pending = map_id, None

    def remove_map(self, map_id: int) -> None:
        with world.lock:
            if map_id not in world.maps or map_id == world.current:
                raise RuntimeError(f"RemoveMap failed: map {map_id}")
            del world.maps[map_id]


class Pnc:
    def request_chassis_control(self, control_mode: int) -> None:
        with world.lock:
            world.step()
            if world.active():
                log_error(f"Pnc remote control failed,reason:{NOT_IDLE}")
                raise RuntimeError("RequestChassisControl failed")
            world.start_task(TYPE_REMOTE_CONTROL, state=2)
            world.command, world.command_time = (0.0, 0.0), -math.inf

    def move_chassis(self, twist: object) -> None:
        """Ackermann mode: linear.x and angular.z move the robot, linear.y is ignored."""
        with world.lock:
            world.step()
            task = world.task
            if task is None or task.type != TYPE_REMOTE_CONTROL or task.state != 2:
                # Not verified on the robot. The fake refuses, so that a missing request shows up.
                raise RuntimeError("MoveChassis failed: request_chassis_control first")
            world.command = (float(twist.linear.x), float(twist.angular.z))
            world.command_time = time.monotonic()

    def normal_navi(self, request: NaviReq) -> None:
        with world.lock:
            world.step()
            if world.active():
                log_error(f"Pnc normal navi failed,reason:{NOT_IDLE}")
                raise RuntimeError("NormalNavi failed")
            x, y = request.target.position.x, request.target.position.y
            path, failure = world.plan(x, y)
            world.start_task(TYPE_NAVIGATION, path=path, goal_yaw=_yaw(request.target), failure=failure)
            world.step()

    def relative_move(self, request: NaviReq) -> None:
        """Moves in a straight line to a pose given in base_link."""
        with world.lock:
            world.step()
            if world.active():
                log_error(f"Pnc relative move failed,reason:{NOT_IDLE}")
                raise RuntimeError("RelativeMove failed")
            dx, dy = request.target.position.x * RELATIVE_OVERSHOOT, request.target.position.y * RELATIVE_OVERSHOOT
            c, s = math.cos(world.yaw), math.sin(world.yaw)
            target = (world.x + c * dx - s * dy, world.y + s * dx + c * dy)
            world.start_task(TYPE_RELATIVE_MOVE, path=[target], goal_yaw=wrap(world.yaw + _yaw(request.target)),
                             speed=RELATIVE_SPEED, blocking=world.body_blocked)
            world.step()

    def get_task_state(self) -> PNCTaskState:
        with world.lock:
            world.step()
            task = world.task
            if task is None:
                return PNCTaskState()
            return PNCTaskState(task.id, task.state, task.type, task.message)

    def pause_task(self, task_id: int) -> None:
        with world.lock:
            world.step()
            task = world.find(task_id, "PauseTask")
            if task.state != 2 or task.type == TYPE_REMOTE_CONTROL:
                raise RuntimeError("PauseTask failed")
            world.set_state(task, 3)
            world.step()

    def resume_task(self, task_id: int) -> None:
        with world.lock:
            world.step()
            task = world.find(task_id, "ResumeTask")
            if task.state != 4:
                raise RuntimeError("ResumeTask failed")
            world.set_state(task, 5)
            world.step()

    def cancel_task(self, task_id: int) -> None:
        with world.lock:
            world.step()
            task = world.find(task_id, "CancelTask")
            if task.state in ACTIVE:
                world.set_state(task, 6)
                task.path = []
                world.step()
