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
TAKE_CONTROL_TIMEOUT_S = 3.0  # how long take_control waits for a canceled task to end
POLL_S = 0.1

# The task states of get_task_state.
TASK_IDLE = 0
TASK_STARTING = 1
TASK_RUNNING = 2
TASK_PAUSING = 3
TASK_PAUSED = 4
TASK_RESUMING = 5
TASK_CANCELING = 6
TASK_CANCELED = 7
TASK_FAIL = 8
TASK_SUCCESS = 9

# cancel_task ends a task in one of these states.
CANCELLABLE = (TASK_STARTING, TASK_RUNNING, TASK_PAUSED, TASK_RESUMING)
# While a task is in one of these states, the planner takes nothing new.
ACTIVE = (TASK_STARTING, TASK_RUNNING, TASK_PAUSING, TASK_PAUSED, TASK_RESUMING, TASK_CANCELING)

TASK_STATES = {
    TASK_IDLE: "idle",
    TASK_STARTING: "starting",
    TASK_RUNNING: "running",
    TASK_PAUSING: "pausing",
    TASK_PAUSED: "paused",
    TASK_RESUMING: "resuming",
    TASK_CANCELING: "canceling",
    TASK_CANCELED: "canceled",
    TASK_FAIL: "fail",
    TASK_SUCCESS: "success",
}
TASK_TYPES = {0: "idle", 1: "navigation", 2: "high precision", 3: "remote control", 4: "relative move"}


@dataclass(frozen=True, kw_only=True)
class Command:
    vx: float  # m/s forward
    wz: float  # rad/s counter-clockwise


STOP = Command(vx=0.0, wz=0.0)


def take_control(pnc: agibot_gdk.Pnc) -> None:
    """Requests the chassis. Ends a task that is still in flight first, whatever its type."""
    # TODO: Read get_task_state. When its state is in CANCELLABLE, call cancel_task with its id.
    # Wait until the state is no longer in ACTIVE, 3 s at most. Then call
    # request_chassis_control(0).
    task = pnc.get_task_state()
    if task.state in CANCELLABLE:
        pnc.cancel_task(task.id)

    # The cancel takes a moment. request_chassis_control fails while a task is still active.
    deadline = time.monotonic() + TAKE_CONTROL_TIMEOUT_S
    while time.monotonic() < deadline:
        task = pnc.get_task_state()
        if task.state not in ACTIVE:
            break
        time.sleep(POLL_S)

    pnc.request_chassis_control(0)


def twist(command: Command) -> agibot_gdk.Twist:
    """Builds the Twist for a command."""
    # TODO: agibot_gdk.Twist() has linear and angular, both Vector3. Set linear.x and angular.z.
    # Vector3 takes no constructor arguments.
    chassis_twist = agibot_gdk.Twist()
    chassis_twist.linear.x = command.vx
    chassis_twist.angular.z = command.wz
    return chassis_twist


def send(pnc: agibot_gdk.Pnc, command: Command) -> None:
    """Sends one command to the chassis."""
    # TODO: move_chassis with the Twist.
    chassis_twist = twist(command)
    pnc.move_chassis(chassis_twist)


def deadman(command: Command, age_s: float, hold_s: float) -> Command:
    """Returns the command while the last key is younger than hold_s, STOP afterwards."""
    # TODO: One comparison.
    if age_s <= hold_s:
        return command
    return STOP


def release(pnc: agibot_gdk.Pnc) -> None:
    """Stops the chassis and ends the task that holds it."""
    # TODO: Read get_task_state. When its state is in CANCELLABLE, send STOP if it is TASK_RUNNING
    # and call cancel_task with its id, also when the STOP failed.
    task = pnc.get_task_state()
    if task.state not in CANCELLABLE:
        return
    try:
        if task.state == TASK_RUNNING:
            send(pnc, STOP)
    finally:
        # finally: the task is canceled also when send raised
        pnc.cancel_task(task.id)


def clamp(value: float, low: float, high: float) -> float:
    """Returns value, but at least low and at most high."""
    return max(low, min(high, value))


def limit(command: Command) -> Command:
    """Keeps a command within MAX_LINEAR and MAX_ANGULAR."""
    vx = clamp(command.vx, -MAX_LINEAR, MAX_LINEAR)
    wz = clamp(command.wz, -MAX_ANGULAR, MAX_ANGULAR)
    return Command(vx=vx, wz=wz)


class DriveSession:
    """The last key, the sender thread and what it sent.

    The sender thread sends a command every 1 / HZ s. The keys only change self.command.
    """

    def __init__(self, pnc: agibot_gdk.Pnc) -> None:
        self.pnc = pnc
        # One Pnc request at a time: a second one while the first is in flight breaks the first
        # ("Broken promise"). The sender thread and the table both ask.
        self.lock = threading.Lock()
        self.command = STOP  # the command of the last key
        self.pressed_at = 0.0  # time.monotonic() of the last key, 0 before the first key
        self.sent = STOP  # the command the sender thread sent last
        self.error = ""  # the error of the last send, "" when it worked
        self.running = True
        self.thread = threading.Thread(target=self.send_loop, daemon=True)

    def press(self, command: Command) -> str:
        self.command = command
        self.pressed_at = time.monotonic()
        return f"vx {command.vx:+.2f}  wz {command.wz:+.2f}"

    def forward(self) -> str:
        return self.press(Command(vx=LINEAR, wz=0.0))

    def back(self) -> str:
        return self.press(Command(vx=-LINEAR, wz=0.0))

    def turn_left(self) -> str:
        return self.press(Command(vx=0.0, wz=ANGULAR))

    def turn_right(self) -> str:
        return self.press(Command(vx=0.0, wz=-ANGULAR))

    def stop(self) -> str:
        return self.press(STOP)

    def send_loop(self) -> None:
        rate = show.Rate(HZ)
        while self.running:
            key_age_s = time.monotonic() - self.pressed_at
            command = deadman(self.command, key_age_s, HOLD_S)
            command = limit(command)
            try:
                with self.lock:
                    send(self.pnc, command)
                self.sent = command
                self.error = ""
            except RuntimeError as error:
                self.error = str(error)
            rate.sleep()

    def table(self) -> dict[str, dict[str, object]]:
        """Read by show.live five times per second."""
        with self.lock:
            task = self.pnc.get_task_state()

        if self.pressed_at:
            key_age = f"{time.monotonic() - self.pressed_at:.1f}"
        else:
            key_age = "no key yet"

        key_row = {"vx": self.command.vx, "wz": self.command.wz, "age s": key_age}
        sent_row = {"vx": self.sent.vx, "wz": self.sent.wz, "error": self.error or "-"}
        task_row = {
            "id": task.id,
            "state": TASK_STATES.get(task.state, task.state),
            "type": TASK_TYPES.get(task.type, task.type),
            "message": task.message or "-",
        }
        return {"key": key_row, "sent": sent_row, "task": task_row}


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    pnc = agibot_gdk.Pnc()
    session = DriveSession(pnc)

    keys = {
        "w": session.forward,
        "s": session.back,
        "a": session.turn_left,
        "d": session.turn_right,
        " ": session.stop,
    }
    try:
        time.sleep(2.0)
        take_control(pnc)
        session.thread.start()
        show.live(session.table, hz=5.0, keys=keys, help="w s forward/back   a d turn   space stop")
    finally:
        session.running = False
        if session.thread.is_alive():
            session.thread.join()
        try:
            release(pnc)
        finally:  # the GDK is released also when the chassis could not be given back
            agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
