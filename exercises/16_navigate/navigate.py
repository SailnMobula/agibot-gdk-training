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

import math
import sys
import threading
import time
from dataclasses import dataclass

import agibot_gdk
import numpy as np

from gdk_training import maps, show, viewer

CLEARANCE = 0.4  # m, a goal closer to an occupied cell is rejected
MAX_RELATIVE_MOVE = 1.5  # m
TIMEOUT_S = 120.0
POLL_S = 0.2
TASK_STATES = {0: "idle", 1: "starting", 2: "running", 3: "pausing", 4: "paused", 5: "resuming", 6: "canceling",
               7: "canceled", 8: "fail", 9: "success"}
TASK_TYPES = {0: "idle", 1: "navigation", 2: "high precision", 3: "remote control", 4: "relative move",
              5: "path navigation"}
ENDED = ("idle", "canceled", "fail", "success")
GOAL_OK, GOAL_BAD = (0, 160, 255), (0, 0, 255)  # BGR
LEGEND = [("free", (255, 255, 255)), ("occupied", (0, 0, 0)), ("unknown", (128, 128, 128)),
          ("guide point", (255, 120, 0)), ("robot", (0, 160, 0)), ("goal, check ok", GOAL_OK),
          ("goal with problems", GOAL_BAD)]


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
    raise NotImplementedError


def task(pnc: agibot_gdk.Pnc) -> Task:
    """Reads the current task."""
    # TODO: get_task_state has id, state, type and message. The robot appends the same line to the
    # message on every attempt. Return the last line and the number of lines.
    raise NotImplementedError


def go(pnc: agibot_gdk.Pnc, x: float, y: float, yaw: float) -> None:
    """Starts a navigation to a pose on the map. Returns at once."""
    # TODO: normal_navi with navi_req.
    raise NotImplementedError


def relative_move(pnc: agibot_gdk.Pnc, dx: float, dy: float, dyaw: float) -> None:
    """Starts a relative move by dx, dy in m and dyaw in rad in base_link. Returns at once."""
    # TODO: pnc.relative_move with navi_req.
    raise NotImplementedError


def pause(pnc: agibot_gdk.Pnc) -> None:
    """Pauses the current task."""
    # TODO: pause_task with the id of the current task.
    raise NotImplementedError


def resume(pnc: agibot_gdk.Pnc) -> None:
    """Resumes the current task."""
    # TODO: resume_task with the id of the current task.
    raise NotImplementedError


def cancel(pnc: agibot_gdk.Pnc) -> None:
    """Cancels the current task."""
    # TODO: cancel_task with the id of the current task.
    raise NotImplementedError


def on_map(info: object, x: float, y: float) -> bool:
    """Tells whether x, y lies inside the grid of the map."""
    # TODO: maps.cell_of returns row and column of the cell, or None outside the map.
    raise NotImplementedError


def is_free(info: object, x: float, y: float) -> bool:
    """Tells whether the cell of x, y is free. x, y has to be on the map."""
    # TODO: maps.cells(info) is the grid as an array, indexed with the row and column from
    # maps.cell_of. 0 is free, -1 unknown, 100 occupied.
    raise NotImplementedError


def has_clearance(info: object, x: float, y: float, clearance: float) -> bool:
    """Tells whether no occupied cell lies within clearance in m of x, y. x, y has to be on the map."""
    # TODO: maps.cells_around gives the grid values in a square around x, y. Is any of them 100?
    raise NotImplementedError


def check_goal(info: object, x: float, y: float, clearance: float) -> list[str]:
    """Returns the problems of a goal on the map, an empty list when there are none.

    It can only check what the GDK hands out: the grid. No-go zones and virtual walls are not
    readable, see exercise 14.
    """
    # TODO: Use on_map, is_free and has_clearance. Off the map, report "outside the map" and stop,
    # the other two need a cell. Otherwise collect "not a free cell" and "closer than ... m to an
    # obstacle".
    raise NotImplementedError


def navigate_to(pnc: agibot_gdk.Pnc, x: float, y: float, yaw: float, timeout: float) -> Task:
    """Sends a goal and waits until the task has ended. Cancels it after timeout s.

    Returns:
        The task as it ended: canceled, fail or success.
    """
    # TODO: Remember the id of the task before go. Poll task every POLL_S until a task with a new id
    # has ended. Call cancel when the timeout has passed.
    raise NotImplementedError


def status(current: Task) -> str:
    attempts = f" ({current.attempts} attempts)" if current.attempts > 1 else ""
    return f"task {current.id} {current.type} {current.state}{attempts} {current.message}".rstrip()


class Controller:
    """The keys, the click and the table. A goal is set and checked first, a key sends it."""

    def __init__(self, pnc: agibot_gdk.Pnc, slam: agibot_gdk.Slam, map_api: agibot_gdk.Map) -> None:
        self.pnc, self.slam = pnc, slam
        self.info = maps.current_map(map_api)
        self.view = maps.MapView(self.info)
        self.goal: tuple[float, float, float] | None = None
        self.waited = "-"  # the result of the last go and wait
        self.waiter: threading.Thread | None = None

    def set_goal(self, x: float, y: float, yaw: float) -> str:
        self.goal = (x, y, yaw)
        return f"goal {x:+.2f} {y:+.2f} {math.degrees(yaw):+.0f} deg, see the table"

    def on_click(self, u: float, v: float) -> None:
        point = self.view.to_world(u, v)
        if point is not None:  # None on the legend
            x, y = point
            rx, ry, _ = maps.robot_pose(self.slam)
            self.set_goal(x, y, math.atan2(y - ry, x - rx))

    def guide(self, number: int) -> str:
        point = next((p for p in self.view.guides if p.id == number), None)
        if point is None:
            raise ValueError(f"no guide point {number}, the map has {[p.id for p in self.view.guides]}")
        return self.set_goal(point.x, point.y, point.yaw)

    def typed_goal(self, text: str) -> str:
        x, y, yaw = (float(value) for value in text.split())
        return self.set_goal(x, y, math.radians(yaw))

    def relative(self, text: str) -> str:
        dx, dy, dyaw = (float(value) for value in text.split())
        if math.hypot(dx, dy) > MAX_RELATIVE_MOVE:
            raise ValueError(f"a relative move is at most {MAX_RELATIVE_MOVE} m")
        relative_move(self.pnc, dx, dy, math.radians(dyaw))
        return "relative move sent"

    def send(self) -> str:
        if self.goal is None:
            raise ValueError("no goal: click on the map, press 1 to 9 or x")
        go(self.pnc, *self.goal)
        return "goal sent, watch the task"

    def send_and_wait(self) -> str:
        """Runs navigate_to on its own thread, so that p, r and c still work meanwhile."""
        if self.goal is None:
            raise ValueError("no goal: click on the map, press 1 to 9 or x")
        if self.waiter and self.waiter.is_alive():
            raise ValueError("navigate_to is still waiting")

        def run(goal: tuple[float, float, float]) -> None:
            try:
                self.waited = status(navigate_to(self.pnc, *goal, TIMEOUT_S))
            except (RuntimeError, NotImplementedError) as error:
                self.waited = f"{type(error).__name__}: {error}"

        self.waited = "waiting"
        self.waiter = threading.Thread(target=run, args=(self.goal,), daemon=True)
        self.waiter.start()
        return "navigate_to started"

    def checks(self) -> tuple[dict, bool]:
        """The goal with the answers of your checks, and whether check_goal found no problem."""
        if self.goal is None:
            return {"goal": "click on the map, press 1 to 9 or x"}, True
        x, y, yaw = self.goal
        row: dict = {"x": x, "y": y, "yaw deg": math.degrees(yaw)}
        try:
            row["on map"] = on_map(self.info, x, y)
            if row["on map"]:
                row["free"] = is_free(self.info, x, y)
                row["clearance"] = has_clearance(self.info, x, y, CLEARANCE)
            problems = check_goal(self.info, x, y, CLEARANCE)
        except NotImplementedError:
            row["check_goal"] = "not written yet"
            return row, True
        row["check_goal"] = ", ".join(problems) or "ok"
        return row, not problems

    def image(self) -> np.ndarray:
        _, ok = self.checks()
        view = self.view.draw(robot=maps.robot_pose(self.slam), goal=self.goal, goal_color=GOAL_OK if ok else GOAL_BAD)
        return maps.with_legend(view, LEGEND)

    def table(self) -> dict:
        x, y, yaw = maps.robot_pose(self.slam)
        current = task(self.pnc)  # the table needs your task
        return {"robot": {"x": x, "y": y, "yaw deg": math.degrees(yaw)}, "goal": self.checks()[0],
                "task": {"id": current.id, "state": current.state, "type": current.type,
                         "message": current.message or "-", "attempts": current.attempts,
                         "go and wait": self.waited}}

    def keys(self) -> dict:
        keys = {str(number): (lambda number=number: self.guide(number)) for number in range(1, 10)}
        return {**keys, "x": show.Line("goal x y yaw (m, m, deg):", self.typed_goal),
                "g": self.send, "w": self.send_and_wait,
                "p": lambda: pause(self.pnc) or "pause sent", "r": lambda: resume(self.pnc) or "resume sent",
                "c": lambda: cancel(self.pnc) or "cancel sent",
                "m": show.Line(f"relative move dx dy dyaw (m, m, deg), at most {MAX_RELATIVE_MOVE} m:", self.relative)}


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    pnc, slam, map_api = agibot_gdk.Pnc(), agibot_gdk.Slam(), agibot_gdk.Map()
    time.sleep(2.0)
    try:
        ui = Controller(pnc, slam, map_api)
        viewer.start({"map": ui.image}, hz=2.0, clicks={"map": ui.on_click})
        show.live(ui.table, hz=2.0, keys=ui.keys(),
                  help="click / 1-9 / x goal   g go   w go and wait   p pause  r resume  c cancel   m relative move")
    finally:
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
