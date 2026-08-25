from agx_arm_coordination.arm_executor import ArmConfig

from types import SimpleNamespace

import pytest

from agx_arm_mit_demos.teach_manager import (
    TeachManagerNode,
    _allow_bare_joint_match,
    _build_transition_targets,
    _hand_delivery_verdict,
    _hand_side_for_arm_name,
    _load_hand_gestures,
    _recording_namespace,
    _resolve_topic_for_namespace,
    _transition_robot_ids,
    _update_gesture_in_config,
)


def _status(*, pending: bool, failed: bool = False, attempts: int = 1) -> SimpleNamespace:
    return SimpleNamespace(
        command_pending=pending,
        command_delivery_failed=failed,
        command_attempts=attempts,
    )


def test_delivery_verdict_waits_on_stale_or_pending_status():
    # No sample yet, or one from before the command, or still retrying -> wait.
    assert _hand_delivery_verdict(None, fresh=False, saw_pending=False, elapsed_s=1.0) == "wait"
    assert (
        _hand_delivery_verdict(
            _status(pending=False), fresh=False, saw_pending=False, elapsed_s=1.0
        )
        == "wait"
    )
    assert (
        _hand_delivery_verdict(
            _status(pending=True), fresh=True, saw_pending=True, elapsed_s=1.0
        )
        == "wait"
    )


def test_delivery_verdict_confirms_only_after_pending_or_grace():
    cleared = _status(pending=False)
    # Fresh cleared sample right after publish, never seen pending -> guard against
    # mistaking the pre-command status for instant success.
    assert (
        _hand_delivery_verdict(cleared, fresh=True, saw_pending=False, elapsed_s=0.1)
        == "wait"
    )
    # Seen pending, now cleared -> delivered.
    assert (
        _hand_delivery_verdict(cleared, fresh=True, saw_pending=True, elapsed_s=0.1)
        == "delivered"
    )
    # Never saw pending but grace elapsed (one-shot delivery) -> delivered.
    assert (
        _hand_delivery_verdict(cleared, fresh=True, saw_pending=False, elapsed_s=0.5)
        == "delivered"
    )


def test_delivery_verdict_reports_bridge_giveup():
    assert (
        _hand_delivery_verdict(
            _status(pending=False, failed=True), fresh=True, saw_pending=True, elapsed_s=1.0
        )
        == "failed"
    )


_GESTURE_YAML = (
    "omnihand_model: o12_pro\n"
    "omnihand_active_joint_order:\n"
    "  - thumb_roll_joint\n"
    "  - index_mcp_joint\n"
    "omnihand_gestures:\n"
    "  # bootstrap\n"
    "  zero: [0.0, 0.0]\n"
)


def _config() -> ArmConfig:
    return ArmConfig.from_dict({
        "arm_executor": {
            "groups": {
                "both_arms": {
                    "planning_group": "both_arms",
                    "joint_names": ["l1", "l2", "r1", "r2"],
                },
                "left_arm": {
                    "planning_group": "left_arm",
                    "joint_names": ["l1", "l2"],
                },
                "right_arm": {
                    "planning_group": "right_arm",
                    "joint_names": ["r1", "r2"],
                },
            },
            "poses": {
                "Idle_L": [0.0, 0.1],
                "Idle_R": [0.2, 0.3],
                "Pre_Grip_L": [1.0, 1.1],
                "Pre_Grip_R": [1.2, 1.3],
                "Place_R": [2.0, 2.1],
            },
        }
    })


def _config_explicit() -> ArmConfig:
    # Resource stored explicitly via robot_id (the new form): both_arms is ONE
    # 4-DoF entry, single sides carry robot_id, and names are free of _L/_R.
    return ArmConfig.from_dict({
        "arm_executor": {
            "groups": {
                "both_arms": {
                    "planning_group": "both_arms",
                    "joint_names": ["l1", "l2", "r1", "r2"],
                },
                "left_arm": {"planning_group": "left_arm", "joint_names": ["l1", "l2"]},
                "right_arm": {"planning_group": "right_arm", "joint_names": ["r1", "r2"]},
            },
            "poses": {
                "handoff": {"robot_id": "both_arms", "q": [0.0, 0.1, 0.2, 0.3]},
                "tee_lifted": {"robot_id": "right_arm", "q": [2.0, 2.1]},
                "park": {"robot_id": "left_arm", "q": [3.0, 3.1]},
            },
        }
    })


def test_transition_robot_ids_default_to_right_arm_for_un_namespaced_session():
    assert _transition_robot_ids([""]) == ("right_arm",)


def test_build_transition_targets_uses_explicit_both_arms_entry():
    targets = _build_transition_targets(_config_explicit(), ["left_arm", "right_arm"])
    by_label = {t.label: t for t in targets}
    # One explicit 14-DoF-style both_arms entry, named freely (no _L/_R pairing).
    assert "both_arms:handoff" in by_label
    both = by_label["both_arms:handoff"]
    assert both.pose_names == ("handoff",)
    assert both.target_positions == (0.0, 0.1, 0.2, 0.3)
    # Single sides resolve by stored robot_id, not by name suffix.
    assert by_label["tee_lifted"].robot_id == "right_arm"
    assert by_label["park"].robot_id == "left_arm"


def test_build_transition_targets_single_side_uses_explicit_robot_id():
    # An un-suffixed right_arm pose is picked up for a right-only session.
    targets = _build_transition_targets(_config_explicit(), [""])
    labels = [t.label for t in targets]
    assert labels == ["tee_lifted"]
    assert targets[0].robot_id == "right_arm"


def test_update_pose_writes_explicit_robot_id_and_round_trips(tmp_path):
    from agx_arm_mit_demos.capture_anchor_pose import update_pose_in_config

    cfg_path = tmp_path / "arm_config.yaml"
    cfg_path.write_text(
        "arm_executor:\n  poses:\n    seed_R: [0.0, 0.0]\n", encoding="utf-8"
    )
    update_pose_in_config(cfg_path, "handoff", [0.1, 0.2, 0.3, 0.4], 3, robot_id="both_arms")
    assert "handoff: {robot_id: both_arms, q: [0.100, 0.200, 0.300, 0.400]}" in cfg_path.read_text()

    cfg = ArmConfig.from_file(cfg_path)
    assert cfg.poses["handoff"] == (0.1, 0.2, 0.3, 0.4)
    assert cfg.pose_robot_id("handoff") == "both_arms"

    # Without robot_id it stays a legacy bare list (side from the _R suffix).
    update_pose_in_config(cfg_path, "legacy_R", [5.0, 6.0], 3)
    cfg2 = ArmConfig.from_file(cfg_path)
    assert cfg2.pose_robot_id("legacy_R") == "right_arm"


def test_load_hand_gestures_reads_order_and_vectors(tmp_path):
    path = tmp_path / "gestures.yaml"
    path.write_text(_GESTURE_YAML, encoding="utf-8")
    order, gestures = _load_hand_gestures(path)
    assert order == ["thumb_roll_joint", "index_mcp_joint"]
    assert gestures == {"zero": [0.0, 0.0]}


def test_load_hand_gestures_missing_file_is_empty(tmp_path):
    order, gestures = _load_hand_gestures(tmp_path / "nope.yaml")
    assert order == [] and gestures == {}


def test_update_gesture_inserts_and_round_trips(tmp_path):
    path = tmp_path / "gestures.yaml"
    path.write_text(_GESTURE_YAML, encoding="utf-8")

    note = _update_gesture_in_config(path, "open_flat", [0.1, 0.2], 3)
    assert "inserted" in note
    _, gestures = _load_hand_gestures(path)
    assert gestures["open_flat"] == [0.1, 0.2]
    assert gestures["zero"] == [0.0, 0.0]  # header + existing gesture preserved

    # Re-capturing the same name updates in place, not duplicates.
    note2 = _update_gesture_in_config(path, "open_flat", [0.3, 0.4], 3)
    assert "updated" in note2
    _, gestures2 = _load_hand_gestures(path)
    assert gestures2["open_flat"] == [0.3, 0.4]


def test_build_transition_targets_for_single_right_arm_filters_right_targets_only():
    targets = _build_transition_targets(_config(), [""])

    assert [target.label for target in targets] == ["Idle_R", "Place_R", "Pre_Grip_R"]
    assert all(target.robot_id == "right_arm" for target in targets)


def test_build_transition_targets_for_duo_session_includes_both_and_per_arm_targets():
    targets = _build_transition_targets(_config(), ["left_arm", "right_arm"])

    assert [target.label for target in targets] == [
        "both_arms:Idle",
        "both_arms:Pre_Grip",
        "Idle_L",
        "Pre_Grip_L",
        "Idle_R",
        "Place_R",
        "Pre_Grip_R",
    ]
    both_target = targets[0]
    assert both_target.pose_names == ("Idle_L", "Idle_R")
    assert both_target.target_positions == (0.0, 0.1, 0.2, 0.3)

def test_discover_mit_namespaces_finds_complete_stacks_only():
    from agx_arm_mit_demos.teach_manager import _discover_mit_namespaces

    class FakeNode:
        def get_service_names_and_types(self):
            required = [
                "set_normal_mode",
                "mit_controller/enable",
                "mit_controller/freedrive",
                "mit_controller/hold_current",
            ]
            services = []
            # complete namespaced duo stacks (right listed first: sorting must fix it)
            for ns in ("/right_arm", "/left_arm"):
                services.extend((f"{ns}/{name}", ["t"]) for name in required)
            # incomplete stack must NOT count
            services.append(("/broken_arm/set_normal_mode", ["t"]))
            # unrelated service noise
            services.append(("/rosout/get_parameters", ["t"]))
            return services

    assert _discover_mit_namespaces(FakeNode()) == ["left_arm", "right_arm"]


def test_discover_mit_namespaces_reports_unnamespaced_stack_as_empty_string():
    from agx_arm_mit_demos.teach_manager import _discover_mit_namespaces

    class FakeNode:
        def get_service_names_and_types(self):
            return [
                ("/set_normal_mode", ["t"]),
                ("/mit_controller/enable", ["t"]),
                ("/mit_controller/freedrive", ["t"]),
                ("/mit_controller/hold_current", ["t"]),
            ]

    assert _discover_mit_namespaces(FakeNode()) == [""]


def test_resolve_topic_for_namespace_prefixes_relative_topics():
    assert (
        _resolve_topic_for_namespace("left_arm", "feedback/omnihand/joint_states")
        == "/left_arm/feedback/omnihand/joint_states"
    )
    assert (
        _resolve_topic_for_namespace("", "feedback/omnihand/joint_states")
        == "feedback/omnihand/joint_states"
    )


def test_resolve_topic_for_namespace_preserves_absolute_topics():
    assert (
        _resolve_topic_for_namespace("right_arm", "/shared/omnihand/joint_states")
        == "/shared/omnihand/joint_states"
    )


def test_hand_side_for_arm_name_prefers_left_and_defaults_to_right():
    assert _hand_side_for_arm_name("left_arm") == "left"
    assert _hand_side_for_arm_name("/left_arm") == "left"
    assert _hand_side_for_arm_name("right_arm") == "right"
    assert _hand_side_for_arm_name("") == "right"


def test_recording_namespace_reads_stored_owner():
    assert _recording_namespace({"namespace": "left_arm"}) == "left_arm"
    assert _recording_namespace({}) == ""
    assert _recording_namespace(None) == ""


def test_allow_bare_joint_match_uses_recorded_owner_in_duo():
    assert _allow_bare_joint_match(
        recording_namespace="left_arm", arm_namespace="left_arm", arm_count=2
    ) is True
    assert _allow_bare_joint_match(
        recording_namespace="left_arm", arm_namespace="right_arm", arm_count=2
    ) is False
    assert _allow_bare_joint_match(
        recording_namespace="", arm_namespace="left_arm", arm_count=2
    ) is False
    assert _allow_bare_joint_match(
        recording_namespace="", arm_namespace="", arm_count=1
    ) is True


# --- duo playback dispatch ---------------------------------------------------

class _RecordingPub:
    """Publisher that appends to a shared event log."""

    def __init__(self, log: list, label: str) -> None:
        self._log = log
        self._label = label

    def publish(self, msg) -> None:
        self._log.append(f"publish:{self._label}")


def _dispatch_log(monkeypatch, *, repetitions: int, interval: float) -> list:
    import agx_arm_mit_demos.teach_manager as teach_manager

    log: list = []
    node = TeachManagerNode.__new__(TeachManagerNode)
    node.args = SimpleNamespace(
        publish_repetitions=repetitions, publish_interval=interval
    )
    monkeypatch.setattr(
        teach_manager.rclpy, "spin_once", lambda *_a, **_k: log.append("spin")
    )
    monkeypatch.setattr(
        teach_manager.time, "sleep", lambda seconds: log.append(f"sleep:{seconds}")
    )
    slices = [
        (SimpleNamespace(trajectory_pub=_RecordingPub(log, side)), object())
        for side in ("left", "right")
    ]
    node._dispatch_slices(slices)
    return log


def test_duo_dispatch_publishes_every_arm_before_it_spins_or_sleeps(monkeypatch):
    """Whatever sits between two publishes becomes a start-time offset.

    A controller starts the trajectory when the message arrives, so a sleep or
    a spin inside the per-arm loop desynchronises the arms by exactly its
    duration. That is what put 671 ms between the two arms on a duo playback.
    """
    log = _dispatch_log(monkeypatch, repetitions=1, interval=0.0)

    assert log == ["publish:left", "publish:right", "spin"]


def test_a_repetition_restarts_both_arms_together(monkeypatch):
    """A republish restarts the trajectory rather than reinforcing delivery.

    So it may never be per-arm: repeating one arm's slice alone would restart
    that arm while the other kept running.
    """
    log = _dispatch_log(monkeypatch, repetitions=2, interval=0.2)

    assert log == [
        "publish:left", "publish:right", "spin", "sleep:0.2",
        "publish:left", "publish:right", "spin", "sleep:0.2",
    ]


def test_hand_window_is_derived_from_the_declared_topology(monkeypatch):
    """The handshake quiesces an arm for the duration of a hand command. On
    dedicated per-device buses that arm shares no bus with the hand, so the
    window costs motion for nothing — and the teach manager used to take it
    unconditionally while every other node derived it."""
    import sys

    from agx_arm_ctrl.motion_registry import handshake_required
    from agx_arm_mit_demos import teach_manager

    monkeypatch.setattr(sys, "argv", ["teach_manager"])
    assert teach_manager.parse_args().hand_window is handshake_required()


def test_a_hand_window_flag_against_the_registry_is_refused(monkeypatch):
    """Two truths about one wiring loom: the registry wins, loudly."""
    import sys

    import pytest

    from agx_arm_ctrl.motion_registry import handshake_required
    from agx_arm_mit_demos import teach_manager

    contradicting = "--no-hand-window" if handshake_required() else "--hand-window"
    monkeypatch.setattr(sys, "argv", ["teach_manager", contradicting])
    with pytest.raises(ValueError, match="contradicts bus_topology"):
        teach_manager.parse_args()


# --- callback-driven recording -------------------------------------------


def _endpoint(joints=("joint1", "joint2")):
    """An _ArmEndpoint with its capture state, without a ROS graph."""
    from agx_arm_mit_demos.teach_manager import _ArmEndpoint

    arm = _ArmEndpoint.__new__(_ArmEndpoint)
    arm.source_joints = list(joints)
    arm.node = SimpleNamespace(_callbacks_served=0)
    arm.latest = None
    arm.feedback_messages = 0
    arm.feedback_frames = 0
    arm._last_feedback_stamp = None
    arm._capturing = False
    arm._capture = []
    arm._capture_origin = None
    arm._capture_wall_origin = 0.0
    arm._capture_uses_stamp = True
    arm._capture_threshold = 0.0
    arm._capture_moved = False
    arm._capture_last_motion = 0.0
    return arm


def _state(stamp_sec, stamp_nsec, positions, names=("joint1", "joint2")):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_sec, nanosec=stamp_nsec)),
        name=list(names),
        position=list(positions),
    )


def test_a_capture_stores_one_sample_per_changed_read(monkeypatch):
    """A read whose positions have not changed is not a sample. The stamp tracks
    the last CAN frame to touch the driver cache, not the position content, so a
    stalled cache arrives with an advancing stamp."""
    monkeypatch.setattr(
        "agx_arm_mit_demos.teach_manager.compute_flange_pose_from_mdh",
        lambda positions, robot: None,
    )
    arm = _endpoint()
    arm.start_capture(movement_threshold=0.001)
    for index, (nsec, value) in enumerate(
        [(0, 0.0), (0, 0.0), (10_000_000, 0.1), (20_000_000, 0.2), (20_000_000, 0.3)]
    ):
        arm._on_feedback(_state(100, nsec, [value, value]))
    samples = arm.stop_capture()

    assert [round(s.time_from_start - 100.0, 4) for s in samples] == [0.0, 0.01, 0.02]
    assert [s.positions[0] for s in samples] == [0.0, 0.1, 0.2]
    assert arm.capture_moved is True
    assert arm.capture_uses_stamp is True
    assert arm.capture_stalled == 0


def test_a_stalled_cache_is_not_stored_as_a_sample(monkeypatch):
    """The stall becomes a gap, so playback interpolates the catch-up across it
    instead of commanding it as one step."""
    monkeypatch.setattr(
        "agx_arm_mit_demos.teach_manager.compute_flange_pose_from_mdh",
        lambda positions, robot: None,
    )
    arm = _endpoint()
    arm.start_capture(movement_threshold=0.001)
    # Stamp advances every 10 ms; the positions freeze for three of them and
    # then catch up, which is what a late CAN frame looks like.
    for index, value in enumerate([0.0, 0.01, 0.01, 0.01, 0.01, 0.08, 0.09]):
        arm._on_feedback(_state(100, index * 10_000_000, [value, value]))
    samples = arm.stop_capture()

    assert [s.positions[0] for s in samples] == [0.0, 0.01, 0.08, 0.09]
    assert arm.capture_stalled == 3
    # The gap carries the stall, so the catch-up is spread over 40 ms, not 10.
    assert round(samples[2].time_from_start - samples[1].time_from_start, 4) == 0.04


def test_the_worst_implied_velocity_is_reported():
    """A joint speed above its limit is a stalled cache catching up, not motion
    the arm performed."""
    from agx_arm_mit_demos.teach_manager import TeachManagerNode
    from agx_arm_mit_demos.leader_trajectory_recorder import RecorderSnapshot

    samples = [
        RecorderSnapshot(time_from_start=t, positions=p, efforts=[0.0], flange_pose=None)
        for t, p in ((0.0, [0.0]), (0.01, [0.002]), (0.02, [0.05]), (0.03, [0.052]))
    ]
    worst, joint, when = TeachManagerNode._worst_implied_velocity(samples)
    assert joint == 0
    assert when == pytest.approx(0.02)
    assert worst == pytest.approx(4.8, rel=1e-3)


def test_a_publisher_without_a_stamp_falls_back_to_arrival_time(monkeypatch):
    monkeypatch.setattr(
        "agx_arm_mit_demos.teach_manager.compute_flange_pose_from_mdh",
        lambda positions, robot: None,
    )
    now = {"t": 10.0}
    monkeypatch.setattr("agx_arm_mit_demos.teach_manager.time.monotonic", lambda: now["t"])
    arm = _endpoint()
    arm.start_capture(movement_threshold=0.001)
    for index, value in enumerate((0.0, 0.1, 0.2)):
        now["t"] = 10.0 + index * 0.01
        arm._on_feedback(_state(0, 0, [value, value]))
    samples = arm.stop_capture()

    assert arm.capture_uses_stamp is False
    assert len(samples) == 3
    assert [round(s.time_from_start - 10.0, 4) for s in samples] == [0.0, 0.01, 0.02]


def test_a_capture_ignores_a_frame_whose_stamp_went_backwards(monkeypatch):
    """The retiming pipeline requires strictly increasing times."""
    monkeypatch.setattr(
        "agx_arm_mit_demos.teach_manager.compute_flange_pose_from_mdh",
        lambda positions, robot: None,
    )
    arm = _endpoint()
    arm.start_capture(movement_threshold=0.001)
    for nsec, value in [(20_000_000, 0.0), (10_000_000, 0.1), (30_000_000, 0.2)]:
        arm._on_feedback(_state(100, nsec, [value, value]))
    times = [s.time_from_start for s in arm.stop_capture()]

    assert times == sorted(times)
    assert len(times) == 2


def test_arms_are_rebased_onto_one_time_axis():
    """Each arm's first frame lands at a different instant; a duo recording has
    to be one time axis or the merge skews the arms against each other."""
    from agx_arm_mit_demos.teach_manager import TeachManagerNode
    from agx_arm_mit_demos.leader_trajectory_recorder import RecorderSnapshot

    left, right = _endpoint(), _endpoint()
    left._capture_uses_stamp = right._capture_uses_stamp = True
    node = TeachManagerNode.__new__(TeachManagerNode)
    node.arms = [left, right]

    def samples(start):
        return [
            RecorderSnapshot(time_from_start=start + i * 0.01, positions=[0.0], efforts=[0.0], flange_pose=None)
            for i in range(3)
        ]

    rebased = node._rebase_capture_times({left: samples(100.05), right: samples(100.00)})
    assert [round(s.time_from_start, 6) for s in rebased[right]] == [0.0, 0.01, 0.02]
    assert [round(s.time_from_start, 6) for s in rebased[left]] == [0.05, 0.06, 0.07]


def test_mixed_capture_clocks_are_refused():
    import pytest

    from agx_arm_mit_demos.teach_manager import TeachManagerNode

    left, right = _endpoint(), _endpoint()
    left._capture_uses_stamp, right._capture_uses_stamp = True, False
    node = TeachManagerNode.__new__(TeachManagerNode)
    node.arms = [left, right]
    with pytest.raises(RuntimeError, match="different clocks"):
        node._rebase_capture_times({left: [], right: []})


# --- move to a replay's start pose ---------------------------------------


def _recording(joint_names, first_positions):
    from agx_arm_mit_controller.trajectory_io import (
        RecordedTrajectory,
        RecordedTrajectoryPoint,
    )

    n = len(joint_names)
    return RecordedTrajectory(
        name="wave", robot="duo", joint_names=list(joint_names), sample_rate_hz=100.0,
        recorded_at="", metadata={},
        points=[
            RecordedTrajectoryPoint(0.0, list(first_positions), [0.0] * n, [0.0] * n),
            RecordedTrajectoryPoint(0.01, list(first_positions), [0.0] * n, [0.0] * n),
        ],
    )


def _manager_with_arms(namespaces, config):
    from agx_arm_mit_demos.teach_manager import TeachManagerNode

    node = TeachManagerNode.__new__(TeachManagerNode)
    node.arm_config = config
    node.arms = []
    for ns in namespaces:
        arm = _endpoint(joints=["j1", "j2"])
        arm.namespace = ns
        node.arms.append(arm)
    return node


def test_a_duo_start_pose_fills_the_group_in_registry_side_order():
    """The planning group declares left joints then right; a target built in the
    wrong order sends each arm the other's pose."""
    config = _config()  # groups: both_arms -> l1,l2,r1,r2
    node = _manager_with_arms(["left_arm", "right_arm"], config)
    left, right = node.arms
    recording = _recording(
        ["left_arm_j1", "left_arm_j2", "right_arm_j1", "right_arm_j2"],
        [0.1, 0.2, 0.3, 0.4],
    )
    # Deliberately hand them over right-first: the target must still be ordered.
    target = node._start_pose_target(recording, [(right, [2, 3]), (left, [0, 1])])
    assert target.robot_id == "both_arms"
    assert target.planning_group == "both_arms"
    assert target.joint_names == ("l1", "l2", "r1", "r2")
    assert target.target_positions == (0.1, 0.2, 0.3, 0.4)


def test_a_single_arm_start_pose_uses_that_side_group():
    config = _config()
    node = _manager_with_arms(["right_arm"], config)
    recording = _recording(["j1", "j2"], [1.5, 1.6])
    target = node._start_pose_target(recording, [(node.arms[0], [0, 1])])
    assert target.robot_id == "right_arm"
    assert target.planning_group == "right_arm"
    assert target.target_positions == (1.5, 1.6)


def test_a_start_pose_target_refuses_a_group_size_mismatch():
    config = _config()
    node = _manager_with_arms(["right_arm"], config)
    # A 4-joint recording against the 2-joint right_arm group.
    recording = _recording(["j1", "j2", "j3", "j4"], [0.0, 0.1, 0.2, 0.3])
    with pytest.raises(RuntimeError, match="planning group"):
        node._start_pose_target(recording, [(node.arms[0], [0, 1, 2, 3])])


def test_start_offsets_name_the_arm_and_joint():
    """A duo refusal takes its maximum over fourteen joints, so the number alone
    does not say which arm to move."""
    node = _manager_with_arms(["left_arm", "right_arm"], _config())
    left, right = node.arms
    left.latest = _state(0, 0, [0.10, 0.20], names=("j1", "j2"))
    right.latest = _state(0, 0, [0.90, 0.30], names=("j1", "j2"))
    recording = _recording(
        ["left_arm_j1", "left_arm_j2", "right_arm_j1", "right_arm_j2"],
        [0.1, 0.2, 0.3, 0.4],
    )
    offsets = node._start_offsets(recording, [(left, [0, 1]), (right, [2, 3])])
    worst = max(offsets, key=lambda item: item[2])
    assert worst[0] == "right_arm"
    assert worst[1] == 0
    assert worst[2] == pytest.approx(0.6)


# --- pre-motion trim ------------------------------------------------------


def _snapshots(times, positions):
    from agx_arm_mit_demos.leader_trajectory_recorder import RecorderSnapshot

    return [
        RecorderSnapshot(time_from_start=t, positions=list(p), efforts=[0.0], flange_pose=None)
        for t, p in zip(times, positions)
    ]


class _QuietLogger:
    """The node is built with __new__, so rclpy never attached a real one."""

    def info(self, *_args, **_kwargs):
        pass

    warn = error = info


def _manager_for_trim(pre_roll, arms):
    from agx_arm_mit_demos.teach_manager import TeachManagerNode

    node = TeachManagerNode.__new__(TeachManagerNode)
    node.args = SimpleNamespace(pre_roll_sec=pre_roll)
    node.arms = arms
    node._logger = _QuietLogger()
    node.get_logger = lambda: node._logger
    return node


def test_the_still_interval_before_the_first_move_is_dropped():
    """Arming the recorder and starting to move are seconds apart; persisting
    that interval makes every replay start with a dead hold."""
    arm = _endpoint()
    arm.namespace = "right_arm"
    arm._capture_first_motion = 5.0
    node = _manager_for_trim(0.25, [arm])

    times = [i * 0.01 for i in range(800)]          # 0 .. 7.99 s
    samples = _snapshots(times, [[0.0]] * 800)
    kept = node._trim_pre_motion({arm: samples})[arm]

    assert kept[0].time_from_start == pytest.approx(4.75, abs=0.011)
    assert kept[-1].time_from_start == pytest.approx(times[-1])


def test_the_pre_roll_keeps_the_start_of_the_motion():
    arm = _endpoint()
    arm._capture_first_motion = 2.0
    for pre_roll, expected in ((0.0, 2.0), (0.5, 1.5)):
        node = _manager_for_trim(pre_roll, [arm])
        times = [i * 0.01 for i in range(500)]
        kept = node._trim_pre_motion({arm: _snapshots(times, [[0.0]] * 500)})[arm]
        assert kept[0].time_from_start == pytest.approx(expected, abs=0.011)


def test_both_arms_are_cut_at_the_same_instant():
    """One cut for the pair, taken from the earliest onset: cutting each arm at
    its own first move would shift them against each other."""
    left, right = _endpoint(), _endpoint()
    left.namespace, right.namespace = "left_arm", "right_arm"
    left._capture_first_motion = 3.0     # left moves first
    right._capture_first_motion = 4.2
    node = _manager_for_trim(0.25, [left, right])

    times = [i * 0.01 for i in range(700)]
    trimmed = node._trim_pre_motion({
        left: _snapshots(times, [[0.0]] * 700),
        right: _snapshots(times, [[0.0]] * 700),
    })
    assert trimmed[left][0].time_from_start == pytest.approx(trimmed[right][0].time_from_start)
    assert trimmed[left][0].time_from_start == pytest.approx(2.75, abs=0.011)


def test_trimming_preserves_the_phase_between_the_arms():
    """The relative offset between two arms' motion must survive trim + re-base,
    or a duo replay drifts apart."""
    left, right = _endpoint(), _endpoint()
    left.namespace, right.namespace = "left_arm", "right_arm"
    left._capture_first_motion = 3.0
    right._capture_first_motion = 4.2
    left._capture_uses_stamp = right._capture_uses_stamp = True
    node = _manager_for_trim(0.25, [left, right])

    times = [i * 0.01 for i in range(700)]
    # A marker each arm carries at its own onset, to measure the offset with.
    left_pos = [[1.0 if t >= 3.0 else 0.0] for t in times]
    right_pos = [[1.0 if t >= 4.2 else 0.0] for t in times]
    trimmed = node._trim_pre_motion({
        left: _snapshots(times, left_pos), right: _snapshots(times, right_pos)
    })
    rebased = node._rebase_capture_times(trimmed)

    def onset(samples):
        return next(s.time_from_start for s in samples if s.positions[0] > 0.5)

    assert onset(rebased[right]) - onset(rebased[left]) == pytest.approx(1.2, abs=0.011)
    assert onset(rebased[left]) == pytest.approx(0.25, abs=0.011)


def test_an_arm_that_never_moved_is_not_trimmed_to_a_stub():
    """The other arm may have moved much later; this one still has to start
    where it was standing, and the retiming needs four samples."""
    left, right = _endpoint(), _endpoint()
    left.namespace, right.namespace = "left_arm", "right_arm"
    left._capture_first_motion = 6.0
    right._capture_first_motion = None
    node = _manager_for_trim(0.25, [left, right])

    times = [i * 0.01 for i in range(100)]          # right stops at 0.99 s
    trimmed = node._trim_pre_motion({
        left: _snapshots([i * 0.01 for i in range(800)], [[0.0]] * 800),
        right: _snapshots(times, [[0.0]] * 100),
    })
    assert len(trimmed[right]) >= 4
