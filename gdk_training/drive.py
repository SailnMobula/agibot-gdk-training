"""Drive keys for the live displays. The complete version of exercise 12.

    drive = Drive(pnc)
    show.live(read, keys=drive.keys, help=drive.help)
    drive.release()

The first drive key takes control of the chassis. While a key is held, the terminal repeats it
and a sender thread sends the command at HZ. When no key arrives for HOLD_S, the thread sends
STOP. The robot itself stops 0.6 s after the last command.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

import agibot_gdk

from gdk_training import show

HZ = 10.0
HOLD_S = 0.6           # the terminal repeats a held key after about 0.5 s
LINEAR = 0.2           # m/s per key
ANGULAR = 0.4          # rad/s per key
MAX_LINEAR = 0.3
MAX_ANGULAR = 0.5
CANCELLABLE = (1, 2, 4, 5)   # task states starting, running, paused and resuming: cancel_task ends these
ACTIVE = (1, 2, 3, 4, 5, 6)  # task states starting to canceling: the planner takes nothing new


@dataclass(frozen=True, kw_only=True)
class Command:
    vx: float  # m/s forward
    wz: float  # rad/s counter-clockwise


STOP = Command(vx=0.0, wz=0.0)


def take_control(pnc: agibot_gdk.Pnc) -> None:
    """Ends a task that is still in flight, whatever its type, and requests the chassis."""
    task = pnc.get_task_state()
    if task.state in CANCELLABLE:
        pnc.cancel_task(task.id)
    deadline = time.monotonic() + 3.0
    while pnc.get_task_state().state in ACTIVE and time.monotonic() < deadline:
        time.sleep(0.1)
    pnc.request_chassis_control(0)


def twist(command: Command) -> agibot_gdk.Twist:
    result = agibot_gdk.Twist()
    result.linear.x = max(-MAX_LINEAR, min(MAX_LINEAR, command.vx))  # Vector3 takes no constructor arguments
    result.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, command.wz))
    return result


def deadman(command: Command, age_s: float, hold_s: float) -> Command:
    return command if age_s <= hold_s else STOP


def release(pnc: agibot_gdk.Pnc) -> None:
    """Stops the chassis and ends the task that holds it."""
    task = pnc.get_task_state()
    if task.state in CANCELLABLE:
        try:
            if task.state == 2:
                pnc.move_chassis(twist(STOP))
        finally:
            pnc.cancel_task(task.id)


class Drive:
    def __init__(self, pnc: agibot_gdk.Pnc) -> None:
        self.pnc = pnc
        self.command = STOP
        self.stamp = 0.0
        self.thread: threading.Thread | None = None
        self.running = False
        self.error = ""
        self.keys: dict[str, Callable[[], object]] = {
            "w": lambda: self.set(Command(vx=LINEAR, wz=0.0)), "s": lambda: self.set(Command(vx=-LINEAR, wz=0.0)),
            "a": lambda: self.set(Command(vx=0.0, wz=ANGULAR)), "d": lambda: self.set(Command(vx=0.0, wz=-ANGULAR)),
            " ": lambda: self.set(STOP),
        }

    def help(self) -> str:
        state = "driving" if self.running else "chassis free"
        return f"w s forward/back   a d turn   space stop   ({state}{', ' + self.error if self.error else ''})"

    def set(self, command: Command) -> str:
        if not self.running:
            take_control(self.pnc)
            self.running = True
            self.thread = threading.Thread(target=self.loop, daemon=True)
            self.thread.start()
        self.command, self.stamp = command, time.monotonic()
        return f"vx {command.vx:+.2f} m/s  wz {command.wz:+.2f} rad/s"

    def loop(self) -> None:
        rate = show.Rate(HZ)
        while self.running:
            try:
                self.pnc.move_chassis(twist(deadman(self.command, time.monotonic() - self.stamp, HOLD_S)))
                self.error = ""
            except RuntimeError as error:
                self.error = str(error)
            rate.sleep()

    def release(self) -> None:
        """Stops the sender thread and gives the chassis back. Call it when the script ends."""
        if not self.running:
            return
        self.running = False
        if self.thread:
            self.thread.join()
        release(self.pnc)
