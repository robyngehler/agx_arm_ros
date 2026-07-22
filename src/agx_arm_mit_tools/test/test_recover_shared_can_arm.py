"""Dry-run ordering test for scripts/recover_shared_can_arm.sh.

Guards the shared-CAN step-and-settle plan (section 2, validation 6.1): the
recovery helper must stop the hand and cancel the arm trajectory BEFORE the CAN
link reset, so pending hand retries are not left hammering the bus and are not
killed mid-command by the down/up. Runs the script in DRY_RUN so no ROS graph or
CAN interface is touched.
"""

import subprocess
from pathlib import Path

_SCRIPT = (
    Path(__file__).resolve().parents[3] / "scripts" / "recover_shared_can_arm.sh"
)


def _run(*args, **env):
    full_env = {"DRY_RUN": "1", "PATH": "/usr/bin:/bin"}
    full_env.update(env)
    result = subprocess.run(
        ["bash", str(_SCRIPT), *args],
        capture_output=True,
        text=True,
        env=full_env,
        timeout=30,
    )
    # All diagnostics go to stderr; stdout carries only service responses.
    return result.returncode, result.stderr


def test_script_exists_and_is_executable():
    assert _SCRIPT.is_file()


def test_dry_run_orders_hand_stop_and_cancel_before_link_reset():
    code, out = _run("right", ARM_NS="right_arm")
    assert code == 0, out

    def pos(needle: str) -> int:
        idx = out.find(needle)
        assert idx != -1, f"missing step: {needle}\n{out}"
        return idx

    cancel = pos("/right_arm/mit_controller/cancel_trajectory")
    hand_stop = pos("/right_arm/control/omnihand/stop")
    arm_estop = pos("/right_arm/emergency_stop")
    link_reset = pos("ip link set can_nero_right down/up")

    # cancel -> hand stop -> arm e-stop -> link reset
    assert cancel < hand_stop < arm_estop < link_reset


def test_dry_run_left_side_uses_left_interface():
    code, out = _run("left", ARM_NS="left_arm")
    assert code == 0, out
    assert "can_nero_left" in out
    assert "/left_arm/control/omnihand/stop" in out


def test_unprefixed_namespace_for_teach_setup():
    code, out = _run("right")  # no ARM_NS -> root namespace
    assert code == 0, out
    assert "/mit_controller/cancel_trajectory" in out
    assert "/right_arm/" not in out


def test_bad_side_argument_is_rejected():
    code, out = _run("bogus")
    assert code == 2
    assert "usage" in out.lower()
