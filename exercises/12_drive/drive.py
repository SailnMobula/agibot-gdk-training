"""Exercise 12, drive. Drives the chassis with keys and a dead-man switch.

    python3 exercises/12_drive/drive.py

w and s drive forward and back, a and d turn, space stops. Hold the key: the terminal repeats it,
and the robot stops when no key has arrived for HOLD_S.
Someone stands at the emergency stop.
"""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass

import agibot_gdk

from gdk_training import show

HZ = 10.0
HOLD_S = 0.6  # s, the terminal repeats a held key after about 0.5 s
LINEAR = 0.2  # m/s per key
ANGULAR = 0.4  # rad/s per key
MAX_LINEAR = 0.3
MAX_ANGULAR = 0.5
REMOTE_CONTROL = 3  # task type
ACTIVE = (1, 2, 3, 4, 5, 6)  # task states starting to canceling
TASK_STATES = {0: "idle", 1: "starting", 2: "running", 3: "pausing", 4: "paused", 5: "resuming", 6: "canceling",
               7: "canceled", 8: "fail", 9: "success"}
TASK_TYPES = {0: "idle", 1: "navigation", 2: "high precision", 3: "remote control", 4: "relative move"}


@dataclass(frozen=True, kw_only=True)
class Command:
    vx: float  # m/s forward
    wz: float  # rad/s counter-clockwise


STOP = Command(vx=0.0, wz=0.0)


def take_control(pnc: agibot_gdk.Pnc) -> None:
    """Requests the chassis. Cancels a remote control task that is still active first."""
    # TODO: Read get_task_state. When its type is REMOTE_CONTROL and its state is in ACTIVE, call
    # cancel_task with its id and wait until the state is no longer active. Then call
    # request_chassis_control(0).
    raise NotImplementedError


def twist(command: Command) -> agibot_gdk.Twist:
    """Builds the Twist for a command."""
    # TODO: agibot_gdk.Twist() has linear and angular, both Vector3.
    raise NotImplementedError


def send(pnc: agibot_gdk.Pnc, command: Command) -> None:
    """Sends one command to the chassis."""
    # TODO: move_chassis with the Twist.
    raise NotImplementedError


def deadman(command: Command, age_s: float, hold_s: float) -> Command:
    """Returns the command while the last key is younger than hold_s, STOP afterwards."""
    # TODO: One comparison.
    raise NotImplementedError


def release(pnc: agibot_gdk.Pnc) -> None:
    """Stops the chassis and ends the remote control task."""
    # TODO: Send STOP. Then cancel the remote control task when it is still active.
    raise NotImplementedError


def limit(command: Command) -> Command:
    return Command(vx=max(-MAX_LINEAR, min(MAX_LINEAR, command.vx)), wz=max(-MAX_ANGULAR, min(MAX_ANGULAR, command.wz)))


class Keys:
    """The last key, the sender thread and what it sent."""

    def __init__(self, pnc: agibot_gdk.Pnc) -> None:
        self.pnc = pnc
        self.command, self.stamp, self.sent, self.error = STOP, 0.0, STOP, ""
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)

    def press(self, command: Command) -> str:
        self.command, self.stamp = command, time.monotonic()
        return f"vx {command.vx:+.2f}  wz {command.wz:+.2f}"

    def loop(self) -> None:
        rate = show.Rate(HZ)
        while self.running:
            command = limit(deadman(self.command, time.monotonic() - self.stamp, HOLD_S))
            try:
                send(self.pnc, command)
                self.sent, self.error = command, ""
            except RuntimeError as error:
                self.error = str(error)
            rate.sleep()


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    pnc = agibot_gdk.Pnc()
    time.sleep(2.0)
    keys = Keys(pnc)

    def read() -> dict:
        task = pnc.get_task_state()
        age = f"{time.monotonic() - keys.stamp:.1f}" if keys.stamp else "no key yet"
        return {"key": {"vx": keys.command.vx, "wz": keys.command.wz, "age s": age},
                "sent": {"vx": keys.sent.vx, "wz": keys.sent.wz, "error": keys.error or "-"},
                "task": {"id": task.id, "state": TASK_STATES.get(task.state, task.state),
                         "type": TASK_TYPES.get(task.type, task.type), "message": task.message or "-"}}

    bindings = {"w": Command(vx=LINEAR, wz=0.0), "s": Command(vx=-LINEAR, wz=0.0),
                "a": Command(vx=0.0, wz=ANGULAR), "d": Command(vx=0.0, wz=-ANGULAR), " ": STOP}
    try:
        take_control(pnc)
        keys.thread.start()
        show.live(read, hz=5.0, keys={key: (lambda c=c: keys.press(c)) for key, c in bindings.items()},
                  help="w s forward/back   a d turn   space stop")
    finally:
        keys.running = False
        if keys.thread.is_alive():
            keys.thread.join()
        release(pnc)
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
