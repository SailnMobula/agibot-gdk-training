"""Exercise 13, mapping. Builds a new map while you drive, then switches between maps.

    python3 exercises/13_mapping/mapping.py

1 starts mapping, 2 stops and saves the map, 3 cancels, 4 asks for a map id and switches to it.
Enter alone switches back to the map that was current when the script started. The drive keys
work as in exercise 12. After 2 the grid of the new map is saved as map_<id>.png next to this
script.
"""

from __future__ import annotations

import contextlib
import os
import sys
import time
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import agibot_gdk
import cv2

from gdk_training import maps, show
from gdk_training.drive import Drive

NEW_MAP_TIMEOUT_S = 10.0

# TODO: The SLAM state codes and their names. Watch the code before, during and after mapping
# and after a cancel, then write down what each code means.
SLAM_STATES: dict[int, str] = {}


@dataclass(frozen=True, kw_only=True)
class MapEntry:
    id: int
    current: bool  # the GDK returns "" as the name of every map


def get_maps(map_api: agibot_gdk.Map) -> list[MapEntry]:
    """Returns every map on the robot."""
    # TODO: get_all_map returns objects with id and is_curr_map.
    raise NotImplementedError


def slam_state(slam: agibot_gdk.Slam) -> str:
    """Returns the SLAM state as code and name, e.g. "1 mapping"."""
    # TODO: get_slam_state returns the code. Look the name up in SLAM_STATES, "?" when it is missing.
    raise NotImplementedError


def start(slam: agibot_gdk.Slam) -> None:
    """Starts mapping."""
    # TODO: start_mapping.
    raise NotImplementedError


def stop(slam: agibot_gdk.Slam, map_api: agibot_gdk.Map, before: list[MapEntry]) -> MapEntry:
    """Stops mapping, waits for the new map and returns it.

    Args:
        before: The maps before mapping started.

    Raises:
        TimeoutError: No new current map appeared within NEW_MAP_TIMEOUT_S.
    """
    # TODO: stop_mapping returns nothing. Poll get_maps until a map appears that is not in before
    # and is current.
    raise NotImplementedError


def cancel(slam: agibot_gdk.Slam) -> None:
    """Cancels mapping. No map is saved."""
    # TODO: cancel_mapping.
    raise NotImplementedError


def switch(map_api: agibot_gdk.Map, map_id: int) -> None:
    """Makes a map the current map. Relocalize on the HMI afterwards."""
    # TODO: switch_map.
    raise NotImplementedError


@contextlib.contextmanager
def quiet() -> Iterator[None]:
    """Hides what the GDK writes to stderr. stop_mapping prints a lot of noise."""
    saved = os.dup(2)
    with open(os.devnull, "w") as null:
        os.dup2(null.fileno(), 2)
        try:
            yield
        finally:
            os.dup2(saved, 2)
            os.close(saved)


def main() -> None:
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        sys.exit("gdk_init failed")
    slam, map_api, pnc = agibot_gdk.Slam(), agibot_gdk.Map(), agibot_gdk.Pnc()
    time.sleep(2.0)
    drive = Drive(pnc)
    workshop_map = map_api.get_curr_map().id
    session = {"before": get_maps(map_api), "started": None}

    def on_start() -> str:
        session["before"] = get_maps(map_api)
        start(slam)
        session["started"] = time.monotonic()
        return "mapping, drive around slowly"

    def on_stop() -> str:
        with quiet():
            new = stop(slam, map_api, session["before"])
        session["started"] = None
        path = Path(__file__).with_name(f"map_{new.id}.png")
        cv2.imwrite(str(path), maps.grid_image(map_api.get_map(new.id)))
        return f"new map {new.id}, image of the map saved to {path}"

    def on_cancel() -> str:
        cancel(slam)
        session["started"] = None
        return "canceled"

    def on_switch(text: str) -> str:
        map_id = int(text) if text else workshop_map
        switch(map_api, map_id)
        return f"switched to map {map_id}, relocalize on the HMI"

    def read() -> dict:
        started = session["started"]
        return {"slam": {"state": slam_state(slam), "mapping s": time.monotonic() - started if started else 0.0,
                         "workshop map": workshop_map},
                "maps": get_maps(map_api)}

    keys = {"1": on_start, "2": on_stop, "3": on_cancel,
            "4": show.Line("switch to map id:", on_switch), **drive.keys}
    try:
        show.live(read, hz=2.0, keys=keys, help=lambda: f"1 start  2 stop  3 cancel  4 switch map   {drive.help()}")
    finally:
        drive.release()
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
