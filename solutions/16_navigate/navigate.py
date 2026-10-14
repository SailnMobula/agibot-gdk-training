"""Exercise 16, navigate. Sets goals on the map, checks them and sends them to the robot.

    python3 exercises/16_navigate/navigate.py

Open http://localhost:8000. A goal is set first and checked at once; a separate key sends it.

    click on the map   goal there, facing away from the robot
    1 to 9             goal at that guide point
    x                  type a goal: x y yaw (m, m, deg)
    g                  go: normal_navi, returns at once
    w                  go and wait: navigate_to, the result appears in the table
    p, r, c            pause, resume, cancel the current task
    m                  relative move: dx dy dyaw in base_link (m, m, deg), at most 1.5 m
"""

from __future__ import annotations

import functools
import math
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

import agibot_gdk
import numpy as np

from gdk_training import maps, show, viewer

CLEARANCE = 0.4  # m, a goal closer to an occupied cell is rejected
MAX_RELATIVE_MOVE = 1.5  # m
TIMEOUT_S = 120.0
POLL_S = 0.2

# The values of the cells in grid_map.data.
CELL_FREE = 0
CELL_OCCUPIED = 100

TASK_STATES = {0: "idle", 1: "starting", 2: "running", 3: "pausing", 4: "paused", 5: "resuming", 6: "canceling",
               7: "canceled", 8: "fail", 9: "success"}
TASK_TYPES = {0: "idle", 1: "navigation", 2: "high precision", 3: "remote control", 4: "relative move",
              5: "path navigation"}
FINISHED_STATES = ("canceled", "fail", "success")  # a task in one of these states has ended

# Colours in BGR, the channel order of OpenCV.
GOAL_OK_COLOR = (0, 160, 255)
GOAL_BAD_COLOR = (0, 0, 255)
LEGEND = [
    ("free", (255, 255, 255)),
    ("occupied", (0, 0, 0)),
    ("unknown", (128, 128, 128)),
    ("guide point", (255, 120, 0)),
    ("robot", (0, 160, 0)),
    ("goal, check ok", GOAL_OK_COLOR),
    ("goal with problems", GOAL_BAD_COLOR),
]
NO_GOAL_HINT = "click on the map, press 1 to 9 or x"


@dataclass(frozen=True, kw_only=True)
class Task:
    id: int
    state: str  # a value of TASK_STATES
    type: str  # a value of TASK_TYPES
    message: str  # the last line of the robot's message
    attempts: int  # the number of lines; the robot adds one per attempt


def navi_req(x: float, y: float, yaw: float) -> agibot_gdk.NaviReq:
    """Builds a NaviReq with the target pose x, y in m and yaw in rad."""
    # TODO: agibot_gdk.NaviReq() has target, a Pose. The yaw goes into the quaternion as
    # z = sin(yaw / 2), w = cos(yaw / 2).
    request = agibot_gdk.NaviReq()
    request.target.position.x = x
    request.target.position.y = y
    request.target.position.z = 0.0
    # A rotation about the z axis only: x and y of the quaternion stay 0.
    request.target.orientation.x = 0.0
    request.target.orientation.y = 0.0
    request.target.orientation.z = math.sin(yaw / 2)
    request.target.orientation.w = math.cos(yaw / 2)
    return request


def task(pnc: agibot_gdk.Pnc) -> Task:
    """Reads the current task."""
    # TODO: get_task_state has id, state, type and message. The robot appends the same line to the
    # message on every attempt. Return the last line and the number of lines.
    task_state = pnc.get_task_state()

    lines = []
    for line in task_state.message.splitlines():
        if line.strip():
            lines.append(line)
    if lines:
        last_line = lines[-1]
    else:
        last_line = ""

    current = Task(
        id=task_state.id,
        state=TASK_STATES.get(task_state.state, str(task_state.state)),
        type=TASK_TYPES.get(task_state.type, str(task_state.type)),
        message=last_line,
        attempts=len(lines),
    )
    return current


def go(pnc: agibot_gdk.Pnc, x: float, y: float, yaw: float) -> None:
    """Starts a navigation to a pose on the map. Returns at once."""
    # TODO: normal_navi with navi_req.
    request = navi_req(x, y, yaw)
    pnc.normal_navi(request)


def relative_move(pnc: agibot_gdk.Pnc, dx: float, dy: float, dyaw: float) -> None:
    """Starts a relative move by dx, dy in m and dyaw in rad in base_link. Returns at once."""
    # TODO: pnc.relative_move with navi_req.
    request = navi_req(dx, dy, dyaw)
    pnc.relative_move(request)


def pause(pnc: agibot_gdk.Pnc) -> None:
    """Pauses the current task."""
    # TODO: pause_task with the id of the current task.
    current = pnc.get_task_state()
    pnc.pause_task(current.id)


def resume(pnc: agibot_gdk.Pnc) -> None:
    """Resumes the current task."""
    # TODO: resume_task with the id of the current task.
    current = pnc.get_task_state()
    pnc.resume_task(current.id)


def cancel(pnc: agibot_gdk.Pnc) -> None:
    """Cancels the current task."""
    # TODO: cancel_task with the id of the current task.
    current = pnc.get_task_state()
    pnc.cancel_task(current.id)


def on_map(info: agibot_gdk.MapInfo, x: float, y: float) -> bool:
    """Tells whether x, y lies inside the grid of the map."""
    # TODO: maps.cell_of returns row and column of the cell, or None outside the map.
    cell = maps.cell_of(info, x, y)
    return cell is not None


def is_free(info: agibot_gdk.MapInfo, x: float, y: float) -> bool:
    """Tells whether the cell of x, y is free. x, y has to be on the map."""
    # TODO: maps.cells(info) is the grid as an array, indexed with the row and column from
    # maps.cell_of. 0 is free, -1 unknown, 100 occupied.
    cells = maps.cells(info)
    row, column = maps.cell_of(info, x, y)
    value = int(cells[row, column])
    return value == CELL_FREE


def has_clearance(info: agibot_gdk.MapInfo, x: float, y: float, clearance: float) -> bool:
    """Tells whether no occupied cell lies within clearance in m of x, y. x, y has to be on the map."""
    # TODO: maps.cells_around gives the grid values in a square around x, y. Is any of them 100?
    cells_nearby = maps.cells_around(info, x, y, clearance)
    is_occupied = cells_nearby == CELL_OCCUPIED
    any_occupied = bool(is_occupied.any())
    return not any_occupied


def check_goal(info: agibot_gdk.MapInfo, x: float, y: float, clearance: float) -> list[str]:
    """Returns the problems of a goal on the map, an empty list when there are none.

    It can only check what the GDK hands out: the grid. No-go zones and virtual walls are not
    readable, see exercise 14.
    """
    # TODO: Use on_map, is_free and has_clearance. Off the map, report "outside the map" and stop,
    # the other two need a cell. Otherwise collect "not a free cell" and "closer than ... m to an
    # obstacle".
    if not on_map(info, x, y):
        return ["outside the map"]

    problems = []
    if not is_free(info, x, y):
        problems.append("not a free cell")
    if not has_clearance(info, x, y, clearance):
        problems.append(f"closer than {clearance} m to an obstacle")
    return problems


def navigate_to(pnc: agibot_gdk.Pnc, x: float, y: float, yaw: float, timeout: float) -> Task:
    """Sends a goal and waits until the task has ended. Cancels it after timeout s.

    Returns:
        The task as it ended: canceled, fail or success.
    """
    # TODO: Remember the id of the task before go. Poll task every POLL_S until a task with a new id
    # has ended. Call cancel when the timeout has passed.
    id_before = task(pnc).id
    go(pnc, x, y, yaw)
    deadline = time.monotonic() + timeout
    cancel_sent = False
    while True:
        current = task(pnc)
        # The task before go may also have ended. Only the new task counts.
        is_new_task = current.id != id_before
        if is_new_task and current.state in FINISHED_STATES:
            return current

        timed_out = time.monotonic() > deadline
        if timed_out and not cancel_sent:
            cancel(pnc)
            cancel_sent = True  # cancel once, then wait for the task to end as canceled
        time.sleep(POLL_S)


def status(current: Task) -> str:
    """One line about a task, e.g. "task 7 navigation fail (5 attempts) goal occupied"."""
    if current.attempts > 1:
        attempts = f" ({current.attempts} attempts)"
    else:
        attempts = ""
    line = f"task {current.id} {current.type} {current.state}{attempts} {current.message}"
    return line.rstrip()


class Controller:
    """The keys, the click and the table. A goal is set and checked first, a key sends it."""

    def __init__(self, pnc: agibot_gdk.Pnc, slam: agibot_gdk.Slam, map_api: agibot_gdk.Map) -> None:
        self.pnc = pnc
        self.slam = slam
        self.info = maps.current_map(map_api)
        self.view = maps.MapView(self.info)
        self.goal: tuple[float, float, float] | None = None  # x, y in m and yaw in rad
        self.waited = "-"  # the result of the last go and wait
        self.waiter: threading.Thread | None = None  # the thread that runs navigate_to

    def set_goal(self, x: float, y: float, yaw: float) -> str:
        self.goal = (x, y, yaw)
        return f"goal {x:+.2f} {y:+.2f} {math.degrees(yaw):+.0f} deg, see the table"

    def on_click(self, u: float, v: float) -> None:
        point = self.view.to_world(u, v)
        if point is None:
            return  # a click on the legend
        x, y = point
        robot_x, robot_y, _ = maps.robot_pose(self.slam)
        # The goal faces along the line from the robot to the click.
        yaw_away_from_robot = math.atan2(y - robot_y, x - robot_x)
        self.set_goal(x, y, yaw_away_from_robot)

    def guide(self, number: int) -> str:
        for point in self.view.guides:
            if point.id == number:
                return self.set_goal(point.x, point.y, point.yaw)
        ids = [point.id for point in self.view.guides]
        raise ValueError(f"no guide point {number}, the map has {ids}")

    def typed_goal(self, text: str) -> str:
        words = text.split()
        if len(words) != 3:
            raise ValueError("type three numbers: x y yaw")
        x = float(words[0])
        y = float(words[1])
        yaw = math.radians(float(words[2]))
        return self.set_goal(x, y, yaw)

    def relative(self, text: str) -> str:
        words = text.split()
        if len(words) != 3:
            raise ValueError("type three numbers: dx dy dyaw")
        dx = float(words[0])
        dy = float(words[1])
        dyaw = math.radians(float(words[2]))
        if math.hypot(dx, dy) > MAX_RELATIVE_MOVE:
            raise ValueError(f"a relative move is at most {MAX_RELATIVE_MOVE} m")
        relative_move(self.pnc, dx, dy, dyaw)
        return "relative move sent"

    def send(self) -> str:
        if self.goal is None:
            raise ValueError(f"no goal: {NO_GOAL_HINT}")
        x, y, yaw = self.goal
        go(self.pnc, x, y, yaw)
        return "goal sent, watch the task"

    def send_and_wait(self) -> str:
        """Runs navigate_to on its own thread, so that p, r and c still work meanwhile."""
        if self.goal is None:
            raise ValueError(f"no goal: {NO_GOAL_HINT}")
        if self.waiter is not None and self.waiter.is_alive():
            raise ValueError("navigate_to is still waiting")
        self.waited = "waiting"
        self.waiter = threading.Thread(target=self.wait_for_goal, args=(self.goal,), daemon=True)
        self.waiter.start()
        return "navigate_to started"

    def wait_for_goal(self, goal: tuple[float, float, float]) -> None:
        """Runs on the waiter thread. Writes the result of navigate_to into self.waited."""
        x, y, yaw = goal
        try:
            ended = navigate_to(self.pnc, x, y, yaw, TIMEOUT_S)
            self.waited = status(ended)
        except (RuntimeError, NotImplementedError) as error:
            self.waited = f"{type(error).__name__}: {error}"

    def on_pause(self) -> str:
        pause(self.pnc)
        return "pause sent"

    def on_resume(self) -> str:
        resume(self.pnc)
        return "resume sent"

    def on_cancel(self) -> str:
        cancel(self.pnc)
        return "cancel sent"

    def checks(self) -> tuple[dict[str, object], bool]:
        """The goal with the answers of your checks, and whether check_goal found no problem."""
        if self.goal is None:
            return {"goal": NO_GOAL_HINT}, True

        x, y, yaw = self.goal
        row: dict[str, object] = {"x": x, "y": y, "yaw deg": math.degrees(yaw)}
        try:
            is_on_map = on_map(self.info, x, y)
            row["on map"] = is_on_map
            if is_on_map:  # is_free and has_clearance need a cell of the map
                row["free"] = is_free(self.info, x, y)
                row["clearance"] = has_clearance(self.info, x, y, CLEARANCE)
            problems = check_goal(self.info, x, y, CLEARANCE)
        except NotImplementedError:
            row["check_goal"] = "not written yet"
            return row, True

        if problems:
            row["check_goal"] = ", ".join(problems)
        else:
            row["check_goal"] = "ok"
        is_ok = len(problems) == 0
        return row, is_ok

    def image(self) -> np.ndarray:
        _, is_ok = self.checks()
        if is_ok:
            goal_color = GOAL_OK_COLOR
        else:
            goal_color = GOAL_BAD_COLOR
        robot = maps.robot_pose(self.slam)
        view = self.view.draw(robot=robot, goal=self.goal, goal_color=goal_color)
        return maps.with_legend(view, LEGEND)

    def table(self) -> dict[str, object]:
        x, y, yaw = maps.robot_pose(self.slam)
        robot_section = {"x": x, "y": y, "yaw deg": math.degrees(yaw)}
        goal_section, _ = self.checks()
        current = task(self.pnc)  # the table needs your task
        task_section = {
            "id": current.id,
            "state": current.state,
            "type": current.type,
            "message": current.message or "-",
            "attempts": current.attempts,
            "go and wait": self.waited,
        }
        table = {"robot": robot_section, "goal": goal_section, "task": task_section}
        return table

    def keys(self) -> dict[str, Callable[[], object] | show.Line]:
        keys: dict[str, Callable[[], object] | show.Line] = {}
        for number in range(1, 10):
            # functools.partial(self.guide, 3) is a function without arguments that calls self.guide(3).
            keys[str(number)] = functools.partial(self.guide, number)
        keys["x"] = show.Line("goal x y yaw (m, m, deg):", self.typed_goal)
        keys["g"] = self.send
        keys["w"] = self.send_and_wait
        keys["p"] = self.on_pause
        keys["r"] = self.on_resume
        keys["c"] = self.on_cancel
        keys["m"] = show.Line(f"relative move dx dy dyaw (m, m, deg), at most {MAX_RELATIVE_MOVE} m:", self.relative)
        return keys


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    pnc = agibot_gdk.Pnc()
    slam = agibot_gdk.Slam()
    map_api = agibot_gdk.Map()
    time.sleep(2.0)
    try:
        ui = Controller(pnc, slam, map_api)
        images = {"map": ui.image}
        clicks = {"map": ui.on_click}
        viewer.start(images, hz=2.0, clicks=clicks)
        help_line = "click / 1-9 / x goal   g go   w go and wait   p pause  r resume  c cancel   m relative move"
        show.live(ui.table, hz=2.0, keys=ui.keys(), help=help_line)
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
