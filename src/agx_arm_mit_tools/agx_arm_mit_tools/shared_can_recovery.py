"""Programmatic, hand-coordinated shared-CAN recovery service.

The ROS-callable twin of ``scripts/recover_shared_can_arm.sh`` (shared-CAN
step-and-settle plan section 2.2, deliverable 2): a coordinator or supervisor
can invoke the disconnect-safe stop-and-recover sequence in-process, instead of
only an operator running the bash helper. Same ordering contract as the script:

  1. cancel the active MIT trajectory + request an MIT hold
  2. stop the OmniHand BEFORE any link reset — pending hand retries must not keep
     hammering the shared side bus and must not be killed mid-command by a
     down/up
  3. arm ``emergency_stop`` (Trigger): a damped MIT zero, VERIFIED in feedback,
     that self-escalates to an electronic stop and finally a forced bus-recovery
     link reset when the stop cannot be verified. The link reset is owned by the
     arm ctrl node, so this service stays sudo-free and never fights the driver
     for the interface.
  4. wait for feedback to resume after any recovery
  5. force + verify normal mode (the service returns a readback-checked result)
  6. re-check the hand backend after the link reset (survival across a down/up is
     hardware-dependent, plan 6.2.5)

Fault-lockout handoff (deliberate design): a forced recovery latches the arm
node's fault lockout. This service NEVER clears it — re-arming motion is a
deliberate decision that belongs to the initiator, not to the recovery step. The
Trigger message reports ``fault_lockout=latched`` so the caller (supervisor,
coordinator, or operator) can choose to call ``clear_fault_lockout``.

The bash helper remains the sudo-capable, ROS-graph-independent operator
fallback (and can force an unconditional link reset); this node is the in-graph
path for automation while the executor is alive.
"""

from __future__ import annotations

import time

try:
	from std_srvs.srv import Empty, Trigger
except ModuleNotFoundError:
	Empty = None
	Trigger = None

try:
	import rclpy
	from rclpy.callback_groups import ReentrantCallbackGroup
	from rclpy.executors import MultiThreadedExecutor
	from rclpy.node import Node
except ModuleNotFoundError:
	rclpy = None
	ReentrantCallbackGroup = None
	MultiThreadedExecutor = None
	Node = object

from agx_arm_mit_tools.duo_soft_estop import (
	arm_service_path,
	mit_service_path,
	normalize_relative_namespace,
	recover_service_name,
)


def recovery_service_sequence(namespace: str) -> list[tuple[str, str, str]]:
	"""Ordered (step, absolute-service-path, srv-type) plan for one side.

	This is the single source of truth for the recovery ordering and is unit
	tested without a ROS graph. ``srv-type`` is ``"empty"`` or ``"trigger"``.
	The critical invariants: the hand is stopped BEFORE the arm emergency stop
	(which may force a link reset), and normal mode is only forced/verified
	after the stop.
	"""
	return [
		("cancel_trajectory", mit_service_path(namespace, "cancel_trajectory"), "empty"),
		("hold_current", mit_service_path(namespace, "hold_current"), "empty"),
		("hand_stop", arm_service_path(namespace, "control/omnihand/stop"), "trigger"),
		("emergency_stop", arm_service_path(namespace, "emergency_stop"), "trigger"),
		("set_normal_mode", arm_service_path(namespace, "set_normal_mode"), "trigger"),
		("hand_recheck", arm_service_path(namespace, "control/omnihand/stop"), "trigger"),
	]


def feedback_topic(namespace: str) -> str:
	resolved = normalize_relative_namespace(namespace)
	return f"/{resolved}/feedback/joint_states" if resolved else "/feedback/joint_states"


class SharedCanRecoveryService(Node):
	"""Hosts one ``recover_<token>`` Trigger service per configured arm side."""

	def __init__(self) -> None:
		if Trigger is None or Empty is None:
			raise RuntimeError("std_srvs is required to run the shared-CAN recovery service")
		super().__init__("shared_can_recovery")

		self.declare_parameter("arm_namespaces", ["left_arm", "right_arm"])
		self.declare_parameter("service_timeout", 5.0)
		self.declare_parameter("feedback_timeout", 10.0)
		self.declare_parameter("poll_period", 0.05)

		configured = [
			normalize_relative_namespace(value)
			for value in self.get_parameter("arm_namespaces").value
		]
		self.arm_namespaces = [value for value in configured if value]
		if not self.arm_namespaces:
			raise ValueError("arm_namespaces must contain at least one namespace")
		self.service_timeout = float(self.get_parameter("service_timeout").value)
		self.feedback_timeout = float(self.get_parameter("feedback_timeout").value)
		self.poll_period = float(self.get_parameter("poll_period").value)

		# Reentrant group + MultiThreadedExecutor so a recovery callback can
		# block on the results of services it calls on OTHER nodes without
		# starving the client responses (the coordinator uses the same pattern).
		self._cb_group = ReentrantCallbackGroup()

		self._empty_clients: dict[str, dict[str, object]] = {}
		self._trigger_clients: dict[str, dict[str, object]] = {}
		self._last_feedback_monotonic: dict[str, float] = {}
		for namespace in self.arm_namespaces:
			self._empty_clients[namespace] = {}
			self._trigger_clients[namespace] = {}
			for _step, path, srv_type in recovery_service_sequence(namespace):
				if srv_type == "empty" and path not in self._empty_clients[namespace]:
					self._empty_clients[namespace][path] = self.create_client(
						Empty, path, callback_group=self._cb_group
					)
				elif srv_type == "trigger" and path not in self._trigger_clients[namespace]:
					self._trigger_clients[namespace][path] = self.create_client(
						Trigger, path, callback_group=self._cb_group
					)
			# Track feedback liveness so step 4 can confirm the bus came back.
			try:
				from sensor_msgs.msg import JointState

				self.create_subscription(
					JointState,
					feedback_topic(namespace),
					lambda _msg, ns=namespace: self._on_feedback(ns),
					10,
					callback_group=self._cb_group,
				)
			except ModuleNotFoundError:
				pass

			self.create_service(
				Trigger,
				recover_service_name(namespace),
				lambda request, response, ns=namespace: self._recover_callback(
					request, response, ns
				),
				callback_group=self._cb_group,
			)

		self.get_logger().info(
			"shared-CAN recovery ready for arm namespaces: "
			+ ", ".join(recover_service_name(ns) for ns in self.arm_namespaces)
		)

	def _on_feedback(self, namespace: str) -> None:
		self._last_feedback_monotonic[namespace] = time.monotonic()

	def _call_empty(self, client, label: str) -> str:
		if client is None or not client.wait_for_service(timeout_sec=self.service_timeout):
			return f"{label}: unavailable (continuing)"
		future = client.call_async(Empty.Request())
		if self._await(future, label) is None:
			return f"{label}: timed out (continuing)"
		return f"{label}: done"

	def _call_trigger(self, client, label: str) -> tuple[bool | None, str]:
		"""Returns (success, message). ``success`` is None when the service is
		absent (best-effort steps continue rather than fail the whole sequence)."""
		if client is None or not client.wait_for_service(timeout_sec=self.service_timeout):
			return None, f"{label}: unavailable"
		future = client.call_async(Trigger.Request())
		resp = self._await(future, label)
		if resp is None:
			return False, f"{label}: timed out"
		return bool(resp.success), resp.message or ""

	def _await(self, future, label: str):
		deadline = time.monotonic() + self.service_timeout
		while rclpy.ok() and not future.done():
			if time.monotonic() > deadline:
				self.get_logger().warn(f"{label}: no response within {self.service_timeout:.1f}s")
				return None
			time.sleep(self.poll_period)
		return future.result()

	def _wait_for_feedback(self, namespace: str) -> bool:
		"""Bounded wait for a feedback sample newer than the call start."""
		start = time.monotonic()
		while rclpy.ok() and time.monotonic() - start < self.feedback_timeout:
			last = self._last_feedback_monotonic.get(namespace)
			if last is not None and last >= start:
				return True
			time.sleep(self.poll_period)
		return False

	def _recover_callback(self, request, response, namespace: str):
		del request
		empties = self._empty_clients[namespace]
		triggers = self._trigger_clients[namespace]
		notes: list[str] = []
		self.get_logger().error(
			f"shared-CAN recovery requested for '{namespace}' — running "
			"disconnect-safe stop+recover sequence"
		)

		# 1. cancel the MIT trajectory + hold (best-effort)
		notes.append(self._call_empty(
			empties.get(mit_service_path(namespace, "cancel_trajectory")), "cancel_trajectory"
		))
		notes.append(self._call_empty(
			empties.get(mit_service_path(namespace, "hold_current")), "hold_current"
		))

		# 2. stop the hand BEFORE any link reset
		hand_stop_path = arm_service_path(namespace, "control/omnihand/stop")
		_ok, msg = self._call_trigger(triggers.get(hand_stop_path), "hand_stop")
		notes.append(f"hand_stop: {msg}")

		# 3. verified, self-escalating arm emergency stop (owns the link reset)
		estop_ok, estop_msg = self._call_trigger(
			triggers.get(arm_service_path(namespace, "emergency_stop")), "emergency_stop"
		)
		notes.append(f"emergency_stop: {estop_msg}")
		lockout_latched = "fault_lockout=latched" in (estop_msg or "")

		# 4. wait for feedback to resume (a forced recovery cycles the link)
		feedback_live = self._wait_for_feedback(namespace)
		notes.append("feedback: live" if feedback_live else "feedback: NOT resumed")

		# 5. force + verify normal mode via the readback-checked service
		nm_ok, nm_msg = self._call_trigger(
			triggers.get(arm_service_path(namespace, "set_normal_mode")), "set_normal_mode"
		)
		notes.append(f"set_normal_mode: {nm_msg}")

		# 6. re-check the hand backend after the reset
		_ok2, hr_msg = self._call_trigger(triggers.get(hand_stop_path), "hand_recheck")
		notes.append(f"hand_recheck: {hr_msg}")

		# Success gate: the arm is confirmed stopped OR (feedback is live AND
		# normal mode verified after a recovery). Anything less is INCOMPLETE.
		verified_stop = estop_ok is True
		recovered = feedback_live and nm_ok is True
		response.success = bool(verified_stop or recovered)

		lockout_note = ""
		if lockout_latched or (response.success and not verified_stop):
			# A forced recovery latched the arm's fault lockout; the initiator
			# owns clearing it. This service never clears it.
			lockout_note = (
				" fault_lockout=latched — call clear_fault_lockout to re-arm motion "
				"deliberately; this service does not clear it."
			)

		summary = f"[{namespace}] " + "; ".join(notes) + lockout_note
		if response.success:
			response.message = "RECOVERY OK — " + summary
			self.get_logger().warn(response.message)
		else:
			response.message = (
				"RECOVERY INCOMPLETE — do NOT re-arm arm motion; use the physical "
				"e-stop if it still moves. " + summary
			)
			self.get_logger().error(response.message)
		return response


def main() -> None:
	if rclpy is None or Trigger is None:
		raise RuntimeError("ROS runtime dependencies are required to run the shared-CAN recovery service")
	rclpy.init()
	node = SharedCanRecoveryService()
	executor = MultiThreadedExecutor()
	executor.add_node(node)
	try:
		executor.spin()
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


__all__ = [
	"SharedCanRecoveryService",
	"feedback_topic",
	"main",
	"recover_service_name",
	"recovery_service_sequence",
]
