from agx_arm_coordination.arm_executor import ArmConfig

from types import SimpleNamespace

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
