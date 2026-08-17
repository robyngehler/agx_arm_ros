#!/usr/bin/env python3
"""OmniHand skill controller — semantic hand skills above the bridge.

This node (one per side, launched as ``/left_hand/omnihand_skill_controller`` /
``/right_hand/...``) turns a *semantic* ``skill_name`` into a vendor-agnostic
motion confirmed by tactile feedback, and holds/releases according to policy.
It is the bottom of the coordinator stack.

Contract. The active architecture reference is the refactor contract —
``docs/sprint_refactor/planning/integration_plan.md`` (phase 2C, hand
arbitration) and ``AGENTS.md`` "ROS Contract Rules"; the ``skill_name ->
backend motion`` mapping design is
``docs/sprint6/planning/hand_skill_backend_mapping.md``.

- this is one of the hand's **two** production motion primitives. Reactive
  contact-seeking motion (here) and trajectory execution
  (``<side>_omnihand_controller/follow_joint_trajectory``) are mutually
  exclusive, and what makes them so is device authority, not topic separation.
  Neither is a debug surface.
- the controller **claims the hand before commanding it** via
  ``control/omnihand/claim_device``, declaring itself ``<primitive>:<node>``,
  and releases afterwards. The bridge is fail-closed: an unclaimed hand executes
  nothing. Claim and release advance the device epoch, so a command from the
  previous owner cannot execute after a handover.
- the public layer carries only ``skill_name``; the ``skill_name -> backend
  motion + target preset`` mapping lives in ``config/omnihand_skills.yaml`` and
  in :mod:`agx_arm_ctrl.omnihand.skills`. Swapping the vendor mapping must not
  change any activity graph.
- behaviour is data: ``completion_policy`` / ``fallback_policy`` are interpreted
  here, never sent to the SDK.
- the controller does NOT open its own SDK session. It commands the existing
  OmniHand bridge over the shared ``control/joint_states`` topic and consumes
  ``feedback/omnihand/{tactile_raw,status,joint_states}`` — so it works against
  both the mock and the SDK backend, per the bridge contract.
- a confirmed grasp holds INTERNALLY and **keeps the claim** — the hold *is* this
  primitive still owning the device. Hold monitoring watches contact and does
  **not** periodically republish the grasp target; republishing made this node a
  second commander of a device the trajectory action also commands. Hold is not a
  coordinator action, so it never blocks arm+hand resources.

Transport: hand skills ride on ``agx_arm_msgs/action/PerformAction`` (no
dedicated HandSkill.action). The goal's ``metadata_json`` carries
``skill_name``, ``contact_sensors``, ``contact_threshold``, ``stable_samples``,
``timeout_sec``, ``completion_policy`` and ``fallback_policy``.
"""

from __future__ import annotations

import json
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from agx_arm_msgs.action import PerformAction
from agx_arm_msgs.msg import (
    DeviceCommandStamp,
    HandJointTarget,
    OmniHandStatus,
    OmniHandTactileRaw,
    RobotEvent,
)
from agx_arm_msgs.srv import ClaimDevice

from agx_arm_ctrl.omnihand.models import DEFAULT_HAND_MODEL, get_hand_model
from agx_arm_ctrl.omnihand.skills import (
    MOTION_CLOSE_UNTIL_CONTACT,
    MOTION_FREEZE,
    MOTION_OPEN,
    MOTION_POSE,
    STATE_CLOSING_UNTIL_CONTACT,
    STATE_FAILED,
    STATE_GRASP_HOLDING,
    STATE_IDLE,
    STATE_OPENING,
    STATE_RELEASING,
    STATE_SHAPING,
    contact_score,
    load_skill_catalogue,
    parse_tactile,
    step_toward,
    within_tolerance,
)
from agx_arm_ctrl.omnihand_bridge_node import (
    HAND_CLAIM_SERVICE,
    build_joint_names,
    resolve_gesture_presets,
)


def _skill_config_share_path() -> str:
    """Best-effort path to the installed skill catalogue YAML."""
    from pathlib import Path

    try:
        return str(
            Path(get_package_share_directory("agx_arm_ctrl"))
            / "config"
            / "omnihand_skills.yaml"
        )
    except Exception:
        return ""


class OmniHandSkillController(Node):

    def __init__(self) -> None:
        super().__init__("omnihand_skill_controller")

        self.declare_parameter("omnihand_type", "right")
        self.declare_parameter("hand_model", DEFAULT_HAND_MODEL)
        self.declare_parameter("skill_config_path", "")
        self.declare_parameter("command_topic", "control/joint_states")
        self.declare_parameter("action_name", "perform")

        self.hand_side = str(self.get_parameter("omnihand_type").value)
        if self.hand_side not in ("left", "right"):
            raise ValueError("omnihand_type must be 'left' or 'right'")
        self.hand_model = get_hand_model(str(self.get_parameter("hand_model").value))
        command_topic = str(self.get_parameter("command_topic").value)
        action_name = str(self.get_parameter("action_name").value)

        skill_config_path = str(self.get_parameter("skill_config_path").value).strip()
        if not skill_config_path:
            skill_config_path = _skill_config_share_path() or None
        self.catalogue = load_skill_catalogue(skill_config_path)
        self.defaults = self.catalogue.defaults

        self.joint_names = build_joint_names(self.hand_side, self.hand_model)
        self.presets = resolve_gesture_presets(self.hand_side, self.hand_model)

        # Latest feedback, written by subscription callbacks (reentrant group),
        # read by the action loop on its own thread.
        self._lock = threading.Lock()
        self._feedback_positions: dict[str, float] = {}
        self._tactile_layout = ""
        self._tactile_values: list[float] = []
        self._tactile_stamp_s = 0.0
        self._hand_fault = False
        self._last_command: list[float] | None = None

        # Internal-hold state (set after a confirmed grasp).
        self._holding = False
        self._hold_target: list[float] | None = None
        self._hold_confirmed_score = 0.0
        self._hold_sensors: list[str] = []
        self._hold_aggregation = self.defaults.contact_aggregation
        self._hold_on_contact_loss = "warn"
        self._hold_warned = False
        self._state = STATE_IDLE

        callback_group = ReentrantCallbackGroup()
        self.command_pub = self.create_publisher(JointState, command_topic, 10)
        # The authority-carrying surface (4D). The reactive loop emits a next
        # target each cycle and cannot be time-parameterized, so it gets a target
        # message rather than being forced through the trajectory contract.
        self.target_pub = self.create_publisher(
            HandJointTarget, "control/omnihand/joint_target", 10
        )
        # Both generations come from the claim response; the sequence restarts
        # with each claim, since a claim opens a new era for the device.
        self._device_epoch = 0
        self._unit_safety_epoch = 0
        self._sequence = 0
        # The owner_id declares the motion primitive first, then the node: the
        # bridge tells the two production primitives apart by it, and uses the
        # node half to notice a commander that died still holding a claim.
        self.owner_id = f"reactive:{self.get_name()}"
        self.claim_service_name = HAND_CLAIM_SERVICE
        self.declare_parameter("claim_timeout_s", 5.0)
        self.claim_timeout_s = float(self.get_parameter("claim_timeout_s").value)
        self.claim_client = self.create_client(
            ClaimDevice, self.claim_service_name, callback_group=callback_group
        )
        self.event_pub = self.create_publisher(RobotEvent, "events", 10)

        self.create_subscription(
            OmniHandTactileRaw,
            "feedback/omnihand/tactile_raw",
            self._tactile_callback,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            OmniHandStatus,
            "feedback/omnihand/status",
            self._status_callback,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            JointState,
            "feedback/omnihand/joint_states",
            self._joint_state_callback,
            10,
            callback_group=callback_group,
        )

        # The hold tick WATCHES a grasp; it does not command one. It used to
        # republish the grasp target at the control rate, which made this node a
        # second commander of a device the trajectory action also commands — and
        # since the bridge keeps exactly one pending target, a hold republish
        # could replace an in-flight trajectory target and let that goal read the
        # hold's verification as its own delivery.
        #
        # Its rate follows the tactile publication, not the control rate: contact
        # is what it looks at, and a faster tick only re-reads the same sample.
        self.declare_parameter("hold_monitor_rate_hz", 5.0)
        monitor_rate = float(self.get_parameter("hold_monitor_rate_hz").value)
        hold_period = 1.0 / monitor_rate if monitor_rate > 0.0 else 0.2
        self.create_timer(hold_period, self._hold_tick, callback_group=callback_group)

        self.action_server = ActionServer(
            self,
            PerformAction,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )

        self.get_logger().info(
            f"OmniHand skill controller up: side={self.hand_side}, "
            f"model={self.hand_model.name} ({len(self.joint_names)} joints), "
            f"command_topic={command_topic}, action={action_name}, "
            f"skills={sorted(self.catalogue.skills)}"
        )

    # --- subscriptions -------------------------------------------------------

    def _tactile_callback(self, msg: OmniHandTactileRaw) -> None:
        with self._lock:
            self._tactile_layout = msg.layout_name
            self._tactile_values = list(msg.values)
            self._tactile_stamp_s = time.monotonic()

    def _status_callback(self, msg: OmniHandStatus) -> None:
        with self._lock:
            self._hand_fault = bool(msg.communication_fault) or not bool(msg.connected)

    def _joint_state_callback(self, msg: JointState) -> None:
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._feedback_positions[name] = float(position)

    # --- helpers -------------------------------------------------------------

    def _current_positions(self) -> list[float]:
        """Best available current pose, ordered to joint_names.

        Prefers the last command (so the ramp is smooth and deterministic), then
        the bridge feedback, then zeros.
        """
        with self._lock:
            if self._last_command is not None:
                return list(self._last_command)
            feedback = dict(self._feedback_positions)
        if all(name in feedback for name in self.joint_names):
            return [feedback[name] for name in self.joint_names]
        return [0.0] * len(self.joint_names)

    def _feedback_pose(self) -> list[float] | None:
        with self._lock:
            feedback = dict(self._feedback_positions)
        if all(name in feedback for name in self.joint_names):
            return [feedback[name] for name in self.joint_names]
        return None

    def _publish_command(self, target: list[float]) -> None:
        positions = [float(value) for value in target]

        # Stamped with the claim this motion runs under. The bridge admits on
        # this one; the plain JointState is published alongside for subscribers
        # that have not migrated off the shared command topic.
        with self._lock:
            self._sequence += 1
            stamp = DeviceCommandStamp()
            stamp.owner_id = self.owner_id
            stamp.device_epoch = self._device_epoch
            stamp.unit_safety_epoch = self._unit_safety_epoch
            stamp.sequence = self._sequence
        authorized = HandJointTarget()
        authorized.authority = stamp
        authorized.joint_names = list(self.joint_names)
        authorized.positions = positions
        self.target_pub.publish(authorized)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = positions
        self.command_pub.publish(msg)
        with self._lock:
            self._last_command = list(target)

    def _read_contact(self, sensors: list[str], aggregation: str) -> tuple[float, float]:
        """Return (contact_score, tactile_age_s)."""
        with self._lock:
            layout = self._tactile_layout
            values = list(self._tactile_values)
            stamp = self._tactile_stamp_s
        reading = parse_tactile(layout, values, self.defaults.normal_force_offset)
        score = contact_score(reading, sensors, aggregation)
        age = time.monotonic() - stamp if stamp > 0.0 else float("inf")
        return score, age

    def _emit_event(
        self,
        event_type: str,
        *,
        activity_id: str = "",
        action_id: str = "",
        state: str = "",
        score: float = 0.0,
        message: str = "",
    ) -> None:
        event = RobotEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.source = "omnihand_skill_controller"
        event.robot_id = f"{self.hand_side}_hand"
        event.activity_id = activity_id
        event.action_id = action_id
        event.event_type = event_type
        event.state = state
        event.contact_score = float(score)
        event.message = message
        self.event_pub.publish(event)

    def _resolve_preset(self, preset_name: str) -> list[float]:
        try:
            return list(self.presets[preset_name])
        except KeyError:
            raise ValueError(
                f"preset '{preset_name}' not defined for model {self.hand_model.name}; "
                f"available: {sorted(self.presets)}"
            ) from None

    # --- action plumbing -----------------------------------------------------

    def _goal_callback(self, goal_request: PerformAction.Goal) -> GoalResponse:
        del goal_request
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> PerformAction.Result:
        goal = goal_handle.request
        result = PerformAction.Result()
        try:
            metadata = json.loads(goal.metadata_json) if goal.metadata_json else {}
            if not isinstance(metadata, dict):
                raise ValueError("metadata_json must encode a JSON object")
        except (ValueError, json.JSONDecodeError) as exc:
            return self._fail(goal_handle, result, f"bad metadata_json: {exc}")

        skill_name = str(metadata.get("skill_name", "")).strip()
        if not skill_name:
            return self._fail(goal_handle, result, "metadata_json missing skill_name")

        try:
            skill = self.catalogue.resolve(skill_name)
        except ValueError as exc:
            return self._fail(goal_handle, result, str(exc))

        self.get_logger().info(
            f"[{self.hand_side}] perform {goal.action_id or skill_name} "
            f"(skill={skill_name}, motion={skill.motion})"
        )
        self._emit_event(
            "started",
            activity_id=goal.activity_id,
            action_id=goal.action_id,
            state=self._state,
            message=f"skill={skill_name}",
        )

        # The bridge is fail-closed, so commanding starts by owning the hand.
        # A refusal here means the trajectory primitive holds it, and the right
        # answer is to fail the action rather than to interleave with it.
        claimed, claim_detail = self._claim_hand()
        if not claimed:
            return self._fail(
                goal_handle, result, f"could not take the hand: {claim_detail}"
            )

        try:
            if skill.motion == MOTION_OPEN:
                return self._run_open(goal_handle, goal, metadata, skill, result)
            if skill.motion == MOTION_POSE:
                return self._run_pose(goal_handle, goal, metadata, skill, result)
            if skill.motion == MOTION_CLOSE_UNTIL_CONTACT:
                return self._run_close_until_contact(goal_handle, goal, metadata, skill, result)
            if skill.motion == MOTION_FREEZE:
                return self._run_freeze(goal_handle, goal, result)
            return self._fail(goal_handle, result, f"unhandled motion '{skill.motion}'")
        finally:
            # A grasp that ended holding keeps the hand: the hold IS the reactive
            # primitive still owning the device, and handing back only to reclaim
            # for the next hold would open a window where nobody owns a hand that
            # is physically gripping something. Released on any other exit.
            if not self._holding:
                self._release_hand()

    def _publish_feedback(self, goal_handle, state: str, progress: float, score: float) -> None:
        feedback = PerformAction.Feedback()
        feedback.state = state
        feedback.progress = float(max(0.0, min(1.0, progress)))
        feedback.contact_score = float(score)
        goal_handle.publish_feedback(feedback)

    def _fail(self, goal_handle, result: PerformAction.Result, message: str) -> PerformAction.Result:
        self._state = STATE_FAILED
        result.success = False
        result.message = message
        result.final_state = STATE_FAILED
        self.get_logger().warn(f"[{self.hand_side}] skill failed: {message}")
        self._emit_event("failed", state=STATE_FAILED, message=message)
        goal_handle.abort()
        return result

    # --- motions -------------------------------------------------------------

    def _run_open(self, goal_handle, goal, metadata, skill, result) -> PerformAction.Result:
        # Opening / releasing clears any internal hold first.
        opening_state = STATE_RELEASING if "release" in skill.skill_name else STATE_OPENING
        return self._ramp_to_preset(
            goal_handle, goal, metadata, skill, result,
            motion_state=opening_state,
            step_rad=self.defaults.open_step_rad,
            label="open",
        )

    def _run_pose(self, goal_handle, goal, metadata, skill, result) -> PerformAction.Result:
        """Reproduce a taught hand shape verbatim (no tactile gating).

        The deterministic counterpart to ``close_until_contact``: the preset was
        measured on the real object, so the shape itself *is* the grasp and no
        calibrated contact threshold is needed. Uses the smaller ``pose_step_rad``
        because a taught grip preset presses into the object.
        """
        return self._ramp_to_preset(
            goal_handle, goal, metadata, skill, result,
            motion_state=STATE_SHAPING,
            step_rad=self.defaults.pose_step_rad,
            label="pose",
        )

    def _ramp_to_preset(
        self, goal_handle, goal, metadata, skill, result, *,
        motion_state: str, step_rad: float, label: str,
    ) -> PerformAction.Result:
        """Bounded per-cycle ramp from the current pose to ``skill.target_preset``.

        Shared by ``open``/``release`` and ``pose``; they differ only in the
        reported state, the step size, and the wording. Any internal (tactile)
        hold is cleared first — the new command supersedes it either way.
        """
        self._clear_hold()
        self._state = motion_state

        target = self._resolve_preset(skill.target_preset)
        timeout = float(metadata.get("timeout_sec", self.defaults.open_settle_timeout_sec))
        period = 1.0 / self.defaults.control_rate_hz if self.defaults.control_rate_hz > 0 else 0.05
        deadline = time.monotonic() + max(timeout, period)

        current = self._current_positions()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._handle_cancel(goal_handle, metadata, result, hold_ok=False)

            current = step_toward(current, target, step_rad)
            self._publish_command(current)

            reached_command = within_tolerance(current, target, 1e-6)
            feedback_pose = self._feedback_pose()
            reached_feedback = (
                feedback_pose is not None
                and within_tolerance(feedback_pose, target, self.defaults.open_tolerance_rad)
            )
            progress = 0.5 if not reached_command else (1.0 if reached_feedback else 0.8)
            self._publish_feedback(goal_handle, motion_state, progress, 0.0)

            if reached_command and (reached_feedback or feedback_pose is None):
                self._state = STATE_IDLE
                result.success = True
                result.message = f"{skill.skill_name}: {label} reached"
                result.final_state = STATE_IDLE
                self._emit_event(
                    "completed",
                    activity_id=goal.activity_id,
                    action_id=goal.action_id,
                    state=STATE_IDLE,
                )
                goal_handle.succeed()
                return result

            if time.monotonic() > deadline:
                # Command ramp done but feedback never confirmed the pose: treat
                # the commanded pose as good enough (the bridge clamps to limits,
                # and a taught grip preset is deliberately past the object surface
                # so feedback legitimately stops short) unless the hand is faulted.
                if reached_command and not self._hand_fault:
                    self._state = STATE_IDLE
                    result.success = True
                    result.message = (
                        f"{skill.skill_name}: {label} commanded (feedback unconfirmed)"
                    )
                    result.final_state = STATE_IDLE
                    self._emit_event(
                        "completed", action_id=goal.action_id, state=STATE_IDLE,
                        message=f"{label} commanded; feedback unconfirmed",
                    )
                    goal_handle.succeed()
                    return result
                return self._fail(goal_handle, result, f"{skill.skill_name}: {label} timed out")

            time.sleep(period)
        return self._fail(goal_handle, result, f"shutdown during {label}")

    def _run_close_until_contact(self, goal_handle, goal, metadata, skill, result) -> PerformAction.Result:
        self._clear_hold()
        self._state = STATE_CLOSING_UNTIL_CONTACT

        target = self._resolve_preset(skill.target_preset)
        sensors = [str(s) for s in metadata.get("contact_sensors", [])]
        aggregation = str(metadata.get("contact_aggregation", self.defaults.contact_aggregation))
        threshold = float(metadata.get("contact_threshold", 0.0))
        stable_samples = int(metadata.get("stable_samples", 1))
        timeout = float(metadata.get("timeout_sec", 4.0))
        completion = metadata.get("completion_policy", {}) or {}
        fallback = metadata.get("fallback_policy", {}) or {}
        passive_monitoring = bool(completion.get("passive_contact_monitoring", True))
        on_success = str(completion.get("on_success", "hold_internal"))

        period = 1.0 / self.defaults.control_rate_hz if self.defaults.control_rate_hz > 0 else 0.05
        deadline = time.monotonic() + max(timeout, period)
        current = self._current_positions()
        stable_count = 0
        last_score = 0.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._handle_cancel(goal_handle, metadata, result, hold_ok=True)

            if self._hand_fault:
                return self._fail(goal_handle, result, "hand reported fault during grasp")

            score, tactile_age = self._read_contact(sensors, aggregation)
            last_score = score
            if tactile_age > self.defaults.tactile_stale_sec:
                return self._fail(
                    goal_handle, result,
                    f"tactile stale ({tactile_age:.2f}s > {self.defaults.tactile_stale_sec}s)",
                )

            if threshold > 0.0 and score >= threshold:
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= max(1, stable_samples):
                return self._confirm_grasp(
                    goal_handle, goal, result, current, score, sensors,
                    aggregation, on_success, passive_monitoring, fallback,
                )

            current = step_toward(current, target, self.defaults.close_step_rad)
            self._publish_command(current)
            progress = min(score / threshold, 1.0) if threshold > 0.0 else 0.0
            self._publish_feedback(goal_handle, STATE_CLOSING_UNTIL_CONTACT, progress, score)

            if time.monotonic() > deadline:
                on_timeout = str(fallback.get("on_timeout", "report_failure"))
                if on_timeout == "stop_and_hold":
                    self.get_logger().warn(
                        f"[{self.hand_side}] grasp timed out; stop_and_hold per fallback_policy"
                    )
                    return self._confirm_grasp(
                        goal_handle, goal, result, current, last_score, sensors,
                        aggregation, "hold_internal", passive_monitoring, fallback,
                        success=False, message="grasp timed out; holding per fallback_policy",
                    )
                return self._fail(goal_handle, result, "grasp did not reach contact before timeout")

            time.sleep(period)
        return self._fail(goal_handle, result, "shutdown during grasp")

    def _confirm_grasp(
        self, goal_handle, goal, result, hold_target, score, sensors,
        aggregation, on_success, passive_monitoring, fallback,
        *, success: bool = True, message: str = "",
    ) -> PerformAction.Result:
        # Stop motion: hold the current commanded pose.
        self._publish_command(hold_target)
        if on_success == "hold_internal":
            with self._lock:
                self._holding = True
                self._hold_target = list(hold_target)
                self._hold_confirmed_score = float(score)
                self._hold_sensors = list(sensors)
                self._hold_aggregation = aggregation
                self._hold_on_contact_loss = str(fallback.get("on_contact_loss", "warn"))
                self._hold_warned = False
            self._state = STATE_GRASP_HOLDING
            final_state = STATE_GRASP_HOLDING
        else:
            self._state = STATE_IDLE
            final_state = STATE_IDLE
        if not passive_monitoring:
            with self._lock:
                self._hold_on_contact_loss = "warn"  # still warn, never auto-abort

        result.success = success
        result.message = message or f"grasp confirmed (contact_score={score:.3f})"
        result.final_state = final_state
        result.final_contact_score = float(score)
        self._emit_event(
            "completed" if success else "warning",
            activity_id=goal.activity_id, action_id=goal.action_id,
            state=final_state, score=score, message=result.message,
        )
        self.get_logger().info(
            f"[{self.hand_side}] {result.message}; state={final_state}"
        )
        if success:
            goal_handle.succeed()
        else:
            goal_handle.succeed()  # held, not a hard failure; coordinator decides on score
        return result

    def _run_freeze(self, goal_handle, goal, result) -> PerformAction.Result:
        hold = self._feedback_pose() or self._current_positions()
        self._publish_command(hold)
        # Freeze does not clear an existing grasp hold; it just pins the pose.
        self._state = STATE_GRASP_HOLDING if self._holding else STATE_IDLE
        result.success = True
        result.message = "stop_hand: pose frozen"
        result.final_state = self._state
        self._emit_event(
            "completed", activity_id=goal.activity_id, action_id=goal.action_id,
            state=self._state, message="freeze",
        )
        goal_handle.succeed()
        return result

    def _handle_cancel(self, goal_handle, metadata, result, *, hold_ok: bool) -> PerformAction.Result:
        fallback = metadata.get("fallback_policy", {}) or {}
        on_cancel = str(fallback.get("on_cancel", "stop_motion"))
        hold = self._feedback_pose() or self._current_positions()
        self._publish_command(hold)
        if hold_ok and on_cancel == "stop_and_hold":
            with self._lock:
                self._holding = True
                self._hold_target = list(hold)
            self._state = STATE_GRASP_HOLDING
            result.final_state = STATE_GRASP_HOLDING
        else:
            self._state = STATE_IDLE
            result.final_state = STATE_IDLE
        result.success = False
        result.message = f"canceled ({on_cancel})"
        self._emit_event("warning", state=result.final_state, message=result.message)
        goal_handle.canceled()
        return result

    # --- internal hold + slip monitoring -------------------------------------

    def _claim_hand(self) -> tuple[bool, str]:
        return self._call_claim(claim=True)

    def _release_hand(self) -> None:
        accepted, detail = self._call_claim(claim=False)
        if not accepted:
            self.get_logger().warn(f"releasing the hand failed: {detail}")

    def _call_claim(self, *, claim: bool) -> tuple[bool, str]:
        """Take or give up this hand's device authority."""
        if not self.claim_client.wait_for_service(timeout_sec=self.claim_timeout_s):
            return False, f"{self.claim_service_name} is not available"
        request = ClaimDevice.Request()
        request.owner_id = self.owner_id
        request.claim = claim
        future = self.claim_client.call_async(request)
        deadline = time.monotonic() + self.claim_timeout_s
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False, f"{self.claim_service_name} did not answer"
            time.sleep(0.02)
        response = future.result()
        if response is None:
            return False, f"{self.claim_service_name} returned nothing"
        if response.accepted and claim:
            with self._lock:
                self._device_epoch = int(response.device_epoch)
                self._unit_safety_epoch = int(response.unit_safety_epoch)
                self._sequence = 0
        return bool(response.accepted), response.message or response.reason

    def _clear_hold(self) -> None:
        """Drop the internal hold state only.

        Deliberately does NOT release the claim: every caller is a motion that
        has just claimed the hand and is about to command it, so releasing here
        would drop the device out from under the command that follows. The claim
        is released where it was taken — in `_execute`, once the action is over
        and no hold survives it.
        """
        with self._lock:
            self._holding = False
            self._hold_target = None
            self._hold_confirmed_score = 0.0
            self._hold_warned = False

    def _hold_tick(self) -> None:
        with self._lock:
            if not self._holding or self._hold_target is None:
                return
            target = list(self._hold_target)
            confirmed = self._hold_confirmed_score
            sensors = list(self._hold_sensors)
            aggregation = self._hold_aggregation
            on_contact_loss = self._hold_on_contact_loss
            already_warned = self._hold_warned

        # Deliberately does NOT re-command the target. The grasp pose was sent
        # once when the grasp was confirmed, and the bridge holds a command
        # pending until a readback verifies it — so re-sending was a second,
        # unverified retry loop layered on top of a verified one, on a different
        # topic, for as long as the grasp lasted. The hand's own controllers keep
        # the commanded position; nothing here needs to restate it.
        del target

        if confirmed <= 0.0 or not sensors:
            return  # no calibrated baseline to compare slip against
        score, _ = self._read_contact(sensors, aggregation)
        warn_level = confirmed * self.defaults.slip_warn_factor
        critical_level = confirmed * self.defaults.slip_critical_factor

        if score < critical_level and on_contact_loss == "abort_activity":
            self._emit_event(
                "contact_lost", state=STATE_GRASP_HOLDING, score=score,
                message=f"contact_score {score:.3f} < critical {critical_level:.3f}",
            )
        elif score < warn_level and not already_warned:
            with self._lock:
                self._hold_warned = True
            self._emit_event(
                "slip_warning", state=STATE_GRASP_HOLDING, score=score,
                message=f"contact_score {score:.3f} < warn {warn_level:.3f}",
            )
        elif score >= warn_level and already_warned:
            with self._lock:
                self._hold_warned = False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OmniHandSkillController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
