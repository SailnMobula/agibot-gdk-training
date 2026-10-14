"""One call at a time to a GDK object.

    map_api = serial(agibot_gdk.Map())

The GDK bindings are not thread-safe: when two threads are in the same call of the same object,
the first one fails with "std::future_error: Broken promise" or the process crashes. In the live
displays the table reads on the main thread while a key's function runs on its own thread, so
both can ask the same object. serial() wraps the object and holds a lock around every call.
"""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Any


class Serial:
    """Passes every method call on to the wrapped GDK object, one at a time."""

    def __init__(self, api: object) -> None:
        self._api = api
        self._lock = threading.Lock()

    # Any: the GDK ships no types, every method of the wrapped object passes through here.
    def __getattr__(self, name: str) -> Any:
        attribute = getattr(self._api, name)
        if not callable(attribute):
            return attribute

        def call(*args: object, **kwargs: object) -> object:
            with self._lock:
                return attribute(*args, **kwargs)

        return call


def serial[T](api: T) -> T:
    """Returns api behind a lock. The result has the methods of api, so it is typed as api."""
    wrapped: Callable[[object], Any] = Serial
    return wrapped(api)
