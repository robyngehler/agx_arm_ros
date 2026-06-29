# Working Plan for real Hardware Integration into ROS Stack and first Demo Usecase

## SDK Details
- minimal path is python centered around `AgibotHandO12.create_hand()`
- sub-methods which are useful: `set/get_all_active_joint_angles, get_tactile_sensor_data, get_all_current_reports, set_all_current_threasholds`
- joint mappings:
```bash
LOCAL_TO_VENDOR_LEFT = {
    "left_thumb_roll_joint": 1,
    "left_thumb_abad_joint": 2,
    "left_thumb_mcp_joint": 3,
    "left_index_abad_joint": 4,
    "left_index_pip_joint": 5,
    "left_middle_pip_joint": 6,
    "left_ring_abad_joint": 7,
    "left_ring_pip_joint": 8,
    "left_pinky_abad_joint": 9,
    "left_pinky_pip_joint": 10,
}

LOCAL_TO_VENDOR_RIGHT = {
    "right_thumb_roll_joint": 1,
    "right_thumb_abad_joint": 2,
    "right_thumb_mcp_joint": 3,
    "right_index_abad_joint": 4,
    "right_index_pip_joint": 5,
    "right_middle_pip_joint": 6,
    "right_ring_abad_joint": 7,
    "right_ring_pip_joint": 8,
    "right_pinky_abad_joint": 9,
    "right_pinky_pip_joint": 10,
}

LEFT_VENDOR_ORDER = [
    "left_thumb_roll_joint",
    "left_thumb_abad_joint",
    "left_thumb_mcp_joint",
    "left_index_abad_joint",
    "left_index_pip_joint",
    "left_middle_pip_joint",
    "left_ring_abad_joint",
    "left_ring_pip_joint",
    "left_pinky_abad_joint",
    "left_pinky_pip_joint",
]

RIGHT_VENDOR_ORDER = [
    "right_thumb_roll_joint",
    "right_thumb_abad_joint",
    "right_thumb_mcp_joint",
    "right_index_abad_joint",
    "right_index_pip_joint",
    "right_middle_pip_joint",
    "right_ring_abad_joint",
    "right_ring_pip_joint",
    "right_pinky_abad_joint",
    "right_pinky_pip_joint",
]
```
- as table:
```bash
1  thumb_roll
2  thumb_abad
3  thumb_mcp
4  index_abad
5  index_pip
6  middle_pip
7  ring_abad
8  ring_pip
9  pinky_abad
10 pinky_pip
```

## ROS Implementation
- around `omnihand_bridge_node.py`
- main methods: `get_joint_names, read_joint_states, read_status, read_tactile, apply_joint_targets, apply_trajectory, stop`

## Recommended Integration Order
- keep the public ROS contract unchanged while moving from `backend_type:=mock` to `backend_type:=sdk`
- first executable target remains `body + right arm + right OmniHand` through `execution_profile:=right_hand`
- keep shared arm-plus-hand command on `control/joint_states`
- keep combined follow state on `feedback/joint_states`
- keep hand-only debug and diagnostics on `feedback/omnihand/*`
- keep `control/omnihand/joint_trajectory` only as a compatibility/debug surface during the first SDK phase

### Step 1: Safe SDK readback only
- add `backend_type:=sdk` behind the existing bridge surface in `agx_arm_ctrl`
- instantiate the SDK backend but start with readback only: joint states, status, tactile
- validate that `feedback/omnihand/joint_states`, `feedback/omnihand/status`, and `feedback/omnihand/tactile_raw` update on live hardware without sending motion commands
- keep `agx_arm_ctrl_single_node` responsible for merging hand feedback into combined `feedback/joint_states`

### Step 2: Guarded active joint control
- enable `apply_joint_targets()` first for shared `control/joint_states`
- preserve partial-command behavior: arm-only `JointState` updates must still be ignored by the OmniHand backend
- gate first motion with `tactile_stop_threshold`, `current_stop_threshold`, and per-joint current thresholds
- treat `stop()` as hold-current-pose in the first iteration instead of adding a new low-level mode switch

### Step 3: Trajectory compatibility and first demo
- map `control/omnihand/joint_trajectory` onto the same backend after joint-state control is stable
- use RViz soft targets and simple open/close presets before attempting MoveIt-driven hand motion
- only after stable open/close cycles should we test a guarded contact-close demo using tactile and current stop conditions

## Bridge Parameters To Add
- `device_id`
- `canfd_id`
- `tactile_stop_threshold`
- `current_stop_threshold`
- `current_thresholds`

These should stay launch- and parameter-level bridge settings. They should not leak into the public ROS topic contract.

## Validation Ladder
1. vendor-only Python probe on live hardware
2. `ros2 launch agx_arm_ctrl start_agx_arm_components.launch.py mode:=debug_soft_target execution_profile:=right_hand omnihand_backend_type:=sdk ...`
3. `ros2 topic echo /feedback/omnihand/joint_states`
4. `ros2 topic echo /feedback/joint_states`
5. guarded open/close over `control/joint_states`
6. guarded `control/omnihand/joint_trajectory` compatibility check

## Current Repo Baseline
- `debug_soft_target execution_profile:=right_hand` now resolves the staged Duo URDF and SRDF cleanly enough for RViz debug work
- the prefixed follow-state path keeps arm joints as `right_arm_joint*` while preserving OmniHand joints as `right_*`
- this means the next SDK step can focus on real hand readback and guarded command execution, not on TF or naming repair

## Handling for tactile Information
```bash
TACTILE_SLICES = {
    "thumb":   slice(0, 16),
    "index":   slice(16, 32),
    "middle":  slice(32, 48),
    "ring":    slice(48, 64),
    "little":  slice(64, 80),
    "palm":    slice(80, 105),
    "dorsum":  slice(105, 130),
}

TACTILE_LAYOUT_NAME = "thumb16_index16_middle16_ring16_little16_palm25_dorsum25"
TACTILE_TOTAL_LEN = 16 + 16 + 16 + 16 + 16 + 25 + 25  # 130
```

## Possible sample Scripts:
### Python Probe:
```python
#!/usr/bin/env python3
import os
import time
from dataclasses import dataclass

from agibot_hand import AgibotHandO12, EFinger, EHandType


LEFT_OPEN = [
    0.0,   # left_thumb_roll_joint
    0.10,  # left_thumb_abad_joint
    0.0,   # left_thumb_mcp_joint
    0.0,   # left_index_abad_joint
    0.0,   # left_index_pip_joint
    0.0,   # left_middle_pip_joint
    0.0,   # left_ring_abad_joint
    0.0,   # left_ring_pip_joint
    0.0,   # left_pinky_abad_joint
    0.0,   # left_pinky_pip_joint
]

LEFT_CLOSED = [
    -0.35,  # left_thumb_roll_joint
    0.95,   # left_thumb_abad_joint
    -0.55,  # left_thumb_mcp_joint
    0.05,   # left_index_abad_joint
    1.10,   # left_index_pip_joint
    1.10,   # left_middle_pip_joint
    -0.05,  # left_ring_abad_joint
    1.10,   # left_ring_pip_joint
    -0.05,  # left_pinky_abad_joint
    1.10,   # left_pinky_pip_joint
]


@dataclass
class ContactSnapshot:
    thumb_sum: int
    index_sum: int
    middle_sum: int
    ring_sum: int
    little_sum: int
    palm_sum: int
    dorsum_sum: int
    max_current: int

    @property
    def close_contact_score(self) -> int:
        return max(self.thumb_sum, self.index_sum, self.middle_sum, self.ring_sum, self.little_sum, self.palm_sum)


def lerp_vec(start, end, alpha):
    return [s + (e - s) * alpha for s, e in zip(start, end)]


def read_contact_snapshot(hand: AgibotHandO12) -> ContactSnapshot:
    thumb = hand.get_tactile_sensor_data(EFinger.THUMB)
    index = hand.get_tactile_sensor_data(EFinger.INDEX)
    middle = hand.get_tactile_sensor_data(EFinger.MIDDLE)
    ring = hand.get_tactile_sensor_data(EFinger.RING)
    little = hand.get_tactile_sensor_data(EFinger.LITTLE)
    palm = hand.get_tactile_sensor_data(EFinger.PALM)
    dorsum = hand.get_tactile_sensor_data(EFinger.DORSUM)
    currents = hand.get_all_current_reports()

    return ContactSnapshot(
        thumb_sum=sum(thumb),
        index_sum=sum(index),
        middle_sum=sum(middle),
        ring_sum=sum(ring),
        little_sum=sum(little),
        palm_sum=sum(palm),
        dorsum_sum=sum(dorsum),
        max_current=max(currents) if currents else 0,
    )


def print_snapshot(label: str, snap: ContactSnapshot) -> None:
    print(
        f"{label} "
        f"thumb={snap.thumb_sum} "
        f"index={snap.index_sum} "
        f"middle={snap.middle_sum} "
        f"ring={snap.ring_sum} "
        f"little={snap.little_sum} "
        f"palm={snap.palm_sum} "
        f"dorsum={snap.dorsum_sum} "
        f"max_current={snap.max_current}"
    )


def open_hand(hand: AgibotHandO12, pose, settle_s=1.0):
    hand.set_all_active_joint_angles(pose)
    time.sleep(settle_s)


def close_until_contact(
    hand: AgibotHandO12,
    open_pose,
    closed_pose,
    tactile_stop_threshold: int = 180,
    current_stop_threshold: int = 900,
    steps: int = 30,
    step_sleep_s: float = 0.08,
):
    for step in range(steps + 1):
        alpha = step / steps
        cmd = lerp_vec(open_pose, closed_pose, alpha)
        hand.set_all_active_joint_angles(cmd)
        time.sleep(step_sleep_s)

        snap = read_contact_snapshot(hand)
        print_snapshot(f"[step {step:02d}/{steps}]", snap)

        if snap.close_contact_score >= tactile_stop_threshold:
            print(f"stop: tactile threshold reached ({snap.close_contact_score} >= {tactile_stop_threshold})")
            return cmd, snap, "tactile"

        if snap.max_current >= current_stop_threshold:
            print(f"stop: current threshold reached ({snap.max_current} >= {current_stop_threshold})")
            return cmd, snap, "current"

    return closed_pose, read_contact_snapshot(hand), "full_close"


def main():
    if "OMNIHAND_SOCKETCAN_IFACE" not in os.environ:
        raise RuntimeError("OMNIHAND_SOCKETCAN_IFACE is not set")

    hand = AgibotHandO12.create_hand(
        device_id=1,
        canfd_id=0,
        hand_type=EHandType.LEFT,
    )

    hand.show_data_details(False)

    # Fallback safety on motor current thresholds.
    hand.set_all_current_thresholds([900] * 10)

    print("opening hand")
    open_hand(hand, LEFT_OPEN, settle_s=1.0)
    print("open angles:", hand.get_all_active_joint_angles())

    print("closing until contact")
    stop_pose, stop_snapshot, stop_reason = close_until_contact(
        hand,
        LEFT_OPEN,
        LEFT_CLOSED,
        tactile_stop_threshold=180,
        current_stop_threshold=900,
        steps=30,
        step_sleep_s=0.08,
    )

    print("stop reason:", stop_reason)
    print("stop pose:", stop_pose)
    print_snapshot("[stop]", stop_snapshot)

    time.sleep(0.8)

    print("re-opening hand")
    open_hand(hand, LEFT_OPEN, settle_s=1.0)
    print("final angles:", hand.get_all_active_joint_angles())


if __name__ == "__main__":
    main()
```

Shell Structures:
```bash
cd ~/workspace/agx_arm_ros/vendor/OmniHand-Pro-2025

export PYTHONPATH=$PWD/build/agibot_hand_pkg
export LD_LIBRARY_PATH=$PWD/build/agibot_hand_pkg/agibot_hand:$LD_LIBRARY_PATH
export OMNIHAND_SOCKETCAN_IFACE=can0

python3.10 /path/to/your/omnihand_grasp_probe.py
```

### Non-mock Backend:
```python
from __future__ import annotations

import os
import math
from dataclasses import dataclass

from agibot_hand import AgibotHandO12, EFinger, EHandType


LEFT_JOINT_NAMES = [
    "left_thumb_roll_joint",
    "left_thumb_abad_joint",
    "left_thumb_mcp_joint",
    "left_index_abad_joint",
    "left_index_pip_joint",
    "left_middle_pip_joint",
    "left_ring_abad_joint",
    "left_ring_pip_joint",
    "left_pinky_abad_joint",
    "left_pinky_pip_joint",
]

RIGHT_JOINT_NAMES = [
    "right_thumb_roll_joint",
    "right_thumb_abad_joint",
    "right_thumb_mcp_joint",
    "right_index_abad_joint",
    "right_index_pip_joint",
    "right_middle_pip_joint",
    "right_ring_abad_joint",
    "right_ring_pip_joint",
    "right_pinky_abad_joint",
    "right_pinky_pip_joint",
]


@dataclass
class OmniHandStatusSnapshot:
    backend_name: str
    control_mode: str
    connected: bool
    initialized: bool
    is_mock: bool
    communication_fault: bool
    active_joint_temperatures_c: list[float]
    active_joint_currents_a: list[float]
    active_joint_stalled: list[bool]
    active_joint_over_temperature: list[bool]
    active_joint_over_current: list[bool]
    status_text: str


@dataclass
class OmniHandTactileSnapshot:
    backend_name: str
    layout_name: str
    values: list[float]


class SdkOmniHandBackend:
    def __init__(
        self,
        hand_side: str,
        device_id: int = 1,
        canfd_id: int = 0,
        tactile_stop_threshold: int = 0,
        current_stop_threshold: int = 0,
        current_thresholds: list[int] | None = None,
    ) -> None:
        if hand_side not in ("left", "right"):
            raise ValueError("hand_side must be left or right")

        self.hand_side = hand_side
        self.backend_name = "vendor_sdk"
        self.control_mode = "joint_state"
        self.connected = False
        self.initialized = False
        self.communication_fault = False
        self.status_text = "not initialized"

        self.tactile_stop_threshold = tactile_stop_threshold
        self.current_stop_threshold = current_stop_threshold
        self.current_thresholds = current_thresholds or [900] * 10

        hand_type = EHandType.LEFT if hand_side == "left" else EHandType.RIGHT
        self.joint_names = LEFT_JOINT_NAMES if hand_side == "left" else RIGHT_JOINT_NAMES
        self.last_commanded_positions = [0.0] * 10

        self.hand = AgibotHandO12.create_hand(
            device_id=device_id,
            canfd_id=canfd_id,
            hand_type=hand_type,
        )
        self.hand.show_data_details(False)
        self.hand.set_all_current_thresholds(self.current_thresholds)

        self.connected = True
        self.initialized = True
        self.status_text = "sdk backend ready"

    def get_joint_names(self) -> list[str]:
        return list(self.joint_names)

    def apply_joint_targets(self, target_map: dict[str, float], control_mode: str) -> int:
        current = self.read_joint_state()
        matched = 0

        for idx, name in enumerate(self.joint_names):
            if name in target_map:
                current[idx] = float(target_map[name])
                matched += 1

        if matched == 0:
            raise ValueError("received command with no recognized OmniHand joints")

        self._guarded_set_angles(current)

        self.last_commanded_positions = list(current)
        self.control_mode = control_mode
        self.status_text = f"applied sdk {control_mode} command with {matched} commanded joints"
        return matched

    def apply_trajectory(self, msg) -> None:
        if not msg.points:
            raise ValueError("received JointTrajectory with no points")

        final_point = msg.points[-1]
        if len(final_point.positions) != len(msg.joint_names):
            raise ValueError("joint_names and final point positions length mismatch")

        target_map = {
            name: float(pos)
            for name, pos in zip(msg.joint_names, final_point.positions, strict=True)
        }
        self.apply_joint_targets(target_map, "joint_trajectory")

    def stop(self) -> None:
        self.control_mode = "stopped"
        self.status_text = "sdk stop requested"
        # For a first iteration, hold current joint angles.
        self.last_commanded_positions = self.read_joint_state()

    def read_joint_state(self) -> list[float]:
        try:
            angles = self.hand.get_all_active_joint_angles()
            return [float(v) for v in angles]
        except Exception as exc:
            self.communication_fault = True
            self.status_text = f"joint read failed: {exc}"
            return list(self.last_commanded_positions)

    def read_status(self) -> OmniHandStatusSnapshot:
        temps = self._safe_get_temperatures()
        currents_raw = self._safe_get_currents()
        errors = self._safe_get_errors()

        return OmniHandStatusSnapshot(
            backend_name=self.backend_name,
            control_mode=self.control_mode,
            connected=self.connected,
            initialized=self.initialized,
            is_mock=False,
            communication_fault=self.communication_fault,
            active_joint_temperatures_c=[float(v) for v in temps],
            active_joint_currents_a=[float(v) for v in currents_raw],
            active_joint_stalled=[bool(e.stalled) for e in errors],
            active_joint_over_temperature=[bool(e.overheat) for e in errors],
            active_joint_over_current=[bool(e.over_current) for e in errors],
            status_text=self.status_text,
        )

    def read_tactile(self) -> OmniHandTactileSnapshot:
        tactile_vectors = [
            self._safe_get_tactile(EFinger.THUMB),
            self._safe_get_tactile(EFinger.INDEX),
            self._safe_get_tactile(EFinger.MIDDLE),
            self._safe_get_tactile(EFinger.RING),
            self._safe_get_tactile(EFinger.LITTLE),
            self._safe_get_tactile(EFinger.PALM),
            self._safe_get_tactile(EFinger.DORSUM),
        ]

        flat = [float(v) for vec in tactile_vectors for v in vec]

        return OmniHandTactileSnapshot(
            backend_name=self.backend_name,
            layout_name="thumb16_index16_middle16_ring16_little16_palm25_dorsum25",
            values=flat,
        )

    def _guarded_set_angles(self, angles: list[float]) -> None:
        self._check_contact_stop()
        self.hand.set_all_active_joint_angles(angles)
        self._check_contact_stop()

    def _check_contact_stop(self) -> None:
        if self.tactile_stop_threshold <= 0 and self.current_stop_threshold <= 0:
            return

        thumb = self._safe_get_tactile(EFinger.THUMB)
        index = self._safe_get_tactile(EFinger.INDEX)
        middle = self._safe_get_tactile(EFinger.MIDDLE)
        ring = self._safe_get_tactile(EFinger.RING)
        little = self._safe_get_tactile(EFinger.LITTLE)
        palm = self._safe_get_tactile(EFinger.PALM)
        currents = self._safe_get_currents()

        tactile_score = max(
            sum(thumb),
            sum(index),
            sum(middle),
            sum(ring),
            sum(little),
            sum(palm),
        )
        max_current = max(currents) if currents else 0

        if self.tactile_stop_threshold > 0 and tactile_score >= self.tactile_stop_threshold:
            raise RuntimeError(
                f"tactile stop triggered: score={tactile_score} threshold={self.tactile_stop_threshold}"
            )

        if self.current_stop_threshold > 0 and max_current >= self.current_stop_threshold:
            raise RuntimeError(
                f"current stop triggered: current={max_current} threshold={self.current_stop_threshold}"
            )

    def _safe_get_tactile(self, finger) -> list[int]:
        try:
            return list(self.hand.get_tactile_sensor_data(finger))
        except Exception as exc:
            self.communication_fault = True
            self.status_text = f"tactile read failed: {exc}"
            return []

    def _safe_get_currents(self) -> list[int]:
        try:
            return list(self.hand.get_all_current_reports())
        except Exception as exc:
            self.communication_fault = True
            self.status_text = f"current read failed: {exc}"
            return [0] * 10

    def _safe_get_temperatures(self) -> list[int]:
        try:
            return list(self.hand.get_all_temperature_reports())
        except Exception as exc:
            self.communication_fault = True
            self.status_text = f"temperature read failed: {exc}"
            return [0] * 10

    def _safe_get_errors(self):
        try:
            return list(self.hand.get_all_error_reports())
        except Exception as exc:
            self.communication_fault = True
            self.status_text = f"error read failed: {exc}"
            return []
```

Switch in `omnihand_bridge_node.py`
```python
if self.backend_type == "mock":
    self.backend = MockOmniHandBackend(
        hand_side=self.hand_side,
        tactile_sample_count=self.tactile_sample_count,
    )
elif self.backend_type == "sdk":
    self.backend = SdkOmniHandBackend(
        hand_side=self.hand_side,
        device_id=int(self.get_parameter("device_id").value),
        canfd_id=int(self.get_parameter("canfd_id").value),
        tactile_stop_threshold=int(self.get_parameter("tactile_stop_threshold").value),
        current_stop_threshold=int(self.get_parameter("current_stop_threshold").value),
    )
else:
    raise ValueError(f"unsupported backend_type: {self.backend_type}")
```
