"""Terminal output for the exercises. It draws a table that refreshes in place and can bind keys.

    show.live(lambda: read_joints(robot))
    show.live(lambda: read_gripper(robot), keys={"o": open_gripper, "c": close_gripper})

The reader returns rows (a list of dicts or dataclass instances), a single dict or dataclass
instance, or a dict of titled sections that contain these. Ctrl+C or q ends the loop. With SHOW_ONCE=1 the function prints one frame
and returns. The tests use this mode.
"""

from __future__ import annotations

import dataclasses
import os
import select
import sys
import termios
import threading
import time
import tty
from collections.abc import Callable, Mapping, Sequence

Rows = Sequence[Mapping[str, object]]
Data = Rows | Mapping[str, object] | object  # dataclass instances are drawn like dicts


def cell(value: object) -> str:
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return f"{value:+.3f}"
    if isinstance(value, (list, tuple)) and all(isinstance(v, (int, float)) for v in value):
        return " ".join(f"{v:+.3f}" for v in value)
    return str(value)


def table(rows: Rows) -> str:
    if not rows:
        return "  (empty)"
    columns = list(rows[0].keys())
    cells = [[cell(row.get(column, "")) for column in columns] for row in rows]
    widths = [max(len(column), *(len(line[i]) for line in cells)) for i, column in enumerate(columns)]
    head = "  ".join(column.ljust(width) for column, width in zip(columns, widths))
    body = ["  ".join(text.rjust(width) if i else text.ljust(width) for i, (text, width) in enumerate(zip(line, widths)))
            for line in cells]
    return "\n".join([head, "  ".join("-" * width for width in widths), *body])


def fields(values: Mapping[str, object]) -> str:
    width = max((len(key) for key in values), default=0)
    return "\n".join(f"{key.ljust(width)}  {cell(value)}" for key, value in values.items())


def plain(data: object) -> object:
    """Turns dataclass instances into dicts, also inside lists and titled sections."""
    if dataclasses.is_dataclass(data) and not isinstance(data, type):
        return {field.name: getattr(data, field.name) for field in dataclasses.fields(data)}
    if isinstance(data, Mapping):
        return {key: plain(value) for key, value in data.items()}
    if isinstance(data, list):
        return [plain(value) for value in data]
    return data


def render(data: Data) -> str:
    data = plain(data)
    if isinstance(data, Mapping):
        sections = [value for value in data.values() if isinstance(value, (Mapping, list, tuple))
                    and not all(isinstance(v, (int, float)) for v in value)]
        if sections and len(sections) == len(data):
            return "\n\n".join(f"{title}\n{render(value)}" for title, value in data.items())
        return fields(data)
    return table(data)


def live(read: Callable[[], Data], hz: float = 5.0, keys: Mapping[str, Callable[[], object]] | None = None,
         help: str | Callable[[], str] = "") -> None:
    """Calls read() at hz and redraws the result. keys maps a single key to a function.

    A key's function runs on its own thread, so the table keeps redrawing while a blocking GDK
    call is under way. Only one function runs at a time. Keys pressed during that time are ignored.
    """
    if os.environ.get("SHOW_ONCE") or not sys.stdin.isatty():
        print(render(read()))
        return
    keys = keys or {}
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    message = ""
    worker: threading.Thread | None = None

    def call(key: str) -> None:
        nonlocal message
        try:
            result = keys[key]()
            message = f"{key}: {result if result is not None else 'done'}"
        except Exception as error:  # Show the GDK error below the table and keep the loop running.
            message = f"{key}: {type(error).__name__}: {error}"

    try:
        tty.setcbreak(fd)
        print("\033[2J", end="")
        while True:
            footer = f"{help() if callable(help) else help}   q quit".strip()
            print(f"\033[H\033[J{render(read())}\n\n{footer}\n{message}", flush=True)
            ready, _, _ = select.select([sys.stdin], [], [], 1.0 / hz)
            if not ready:
                continue
            key = os.read(fd, 1).decode(errors="ignore")
            if key == "q":
                return
            if key in keys and not (worker and worker.is_alive()):
                message = f"{key}: running"
                worker = threading.Thread(target=call, args=(key,), daemon=True)
                worker.start()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print()
        if worker and worker.is_alive():
            print("waiting for the move to finish")
            worker.join()


class Rate:
    """Sleeps until the next tick of a fixed schedule.

    A slightly slow iteration does not delay later ticks. After a stall of more than one period
    the schedule restarts from the current time, so the loop never runs faster than hz.
    """

    def __init__(self, hz: float) -> None:
        self.period = 1.0 / hz
        self.next = time.monotonic() + self.period

    def sleep(self) -> None:
        now = time.monotonic()
        if now > self.next + self.period:
            self.next = now
        time.sleep(max(0.0, self.next - now))
        self.next += self.period
