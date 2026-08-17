"""The hand's CAN interface is an argument, not process state.

The vendor SocketCAN backend reads ``OMNIHAND_SOCKETCAN_IFACE`` once, inside the
SDK constructor. Setting it and leaving it set made the interface a property of
the process: two hand bridges in one process each overwrite the other, and
whichever constructs second decides the bus for both.
"""

import os
import threading
import time

from agx_arm_ctrl.omnihand.socketcan_iface import ENV_VAR, socketcan_interface


def test_the_interface_is_visible_while_the_session_is_opened():
    with socketcan_interface("hand_right"):
        assert os.environ[ENV_VAR] == "hand_right"


def test_nothing_is_left_behind_afterwards():
    os.environ.pop(ENV_VAR, None)
    with socketcan_interface("hand_right"):
        pass
    assert ENV_VAR not in os.environ


def test_a_pre_existing_value_is_restored_not_clobbered():
    os.environ[ENV_VAR] = "someone_elses_bus"
    try:
        with socketcan_interface("hand_left"):
            assert os.environ[ENV_VAR] == "hand_left"
        assert os.environ[ENV_VAR] == "someone_elses_bus"
    finally:
        os.environ.pop(ENV_VAR, None)


def test_an_unresolved_interface_claims_nothing():
    """A caller without an interface must not silently claim the vendor default."""
    os.environ.pop(ENV_VAR, None)
    with socketcan_interface("") as active:
        assert active == ""
        assert ENV_VAR not in os.environ


def test_two_hands_in_one_process_each_see_their_own_bus():
    """The failure this exists to prevent, at the only moment it can happen.

    Both threads spend real time inside the window, so an unguarded env-var
    write would interleave and each would read the other's interface.
    """
    seen: dict[str, list[str]] = {"left": [], "right": []}
    overlapped: list[bool] = []
    inside = threading.Semaphore(0)
    depth = {"n": 0}
    depth_lock = threading.Lock()

    def construct(side: str, iface: str) -> None:
        with socketcan_interface(iface):
            with depth_lock:
                depth["n"] += 1
                overlapped.append(depth["n"] > 1)
            # Stand in for the vendor constructor, which reads the variable
            # while it opens the socket — long enough for a racing writer to
            # land in between the two reads.
            seen[side].append(os.environ[ENV_VAR])
            inside.release()
            time.sleep(0.05)
            seen[side].append(os.environ[ENV_VAR])
            with depth_lock:
                depth["n"] -= 1

    threads = [
        threading.Thread(target=construct, args=("left", "hand_left")),
        threading.Thread(target=construct, args=("right", "hand_right")),
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=10.0)

    assert seen["left"] == ["hand_left", "hand_left"], seen
    assert seen["right"] == ["hand_right", "hand_right"], seen
    assert not any(overlapped), "two constructions held the window at once"
