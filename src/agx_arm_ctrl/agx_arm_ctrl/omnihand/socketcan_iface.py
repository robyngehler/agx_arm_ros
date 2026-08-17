"""Select the vendor SDK's CAN interface without leaving process state behind.

The vendor SocketCAN backend reads ``OMNIHAND_SOCKETCAN_IFACE`` and nothing
else, once, inside ``OpenDevice()`` — which runs from the SDK object's
constructor. Setting that variable and leaving it set makes the interface a
property of the process rather than of the backend: two hand bridges in one
process would each overwrite the other's choice, and whichever constructed
second would decide the bus for both.

Scoping the variable to the construction call keeps the interface an argument.
The lock makes the set/construct/restore sequence atomic against a concurrent
construction, since the window is process-wide however short it is.
"""

from __future__ import annotations

import os
import threading
from contextlib import contextmanager
from typing import Iterator

ENV_VAR = "OMNIHAND_SOCKETCAN_IFACE"

# Process-wide, because what it guards is process-wide.
_lock = threading.Lock()


@contextmanager
def socketcan_interface(can_interface: str) -> Iterator[str]:
    """Hold ``can_interface`` in the environment for one SDK construction.

    Yields the interface actually in effect. An empty ``can_interface`` changes
    nothing and lets the vendor default apply, so a caller that has not resolved
    an interface does not silently claim one.
    """
    if not can_interface:
        yield os.environ.get(ENV_VAR, "")
        return

    with _lock:
        previous = os.environ.get(ENV_VAR)
        os.environ[ENV_VAR] = can_interface
        try:
            yield can_interface
        finally:
            if previous is None:
                os.environ.pop(ENV_VAR, None)
            else:
                os.environ[ENV_VAR] = previous
