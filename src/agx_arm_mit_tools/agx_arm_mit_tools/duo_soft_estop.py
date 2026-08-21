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


def normalize_relative_namespace(value: object) -> str:
	text = str(value).strip() if value is not None else ""
	return text.strip("/")


def join_relative_namespaces(*parts: object) -> str:
	normalized_parts = [normalize_relative_namespace(part) for part in parts]
	return "/".join(part for part in normalized_parts if part)


def mit_service_path(namespace: str, service_name: str) -> str:
	resolved_namespace = normalize_relative_namespace(namespace)
	resolved_service = str(service_name).strip().strip("/")
	service_path = join_relative_namespaces(resolved_namespace, "mit_controller", resolved_service)
	return f"/{service_path}" if service_path else f"/{resolved_service}"


def arm_service_path(namespace: str, service_name: str) -> str:
	"""Absolute path of an arm-driver service that lives at the arm-namespace
	root (a sibling of the ``mit_controller`` subnamespace), e.g.
	``/right_arm/emergency_stop`` — matching the arm ctrl node layout used by
	``scripts/recover_shared_can_arm.sh``."""
	resolved_namespace = normalize_relative_namespace(namespace)
	resolved_service = str(service_name).strip().strip("/")
	service_path = join_relative_namespaces(resolved_namespace, resolved_service)
	return f"/{service_path}" if service_path else f"/{resolved_service}"


def hold_service_name(namespace: str) -> str:
	service_token = normalize_relative_namespace(namespace).replace("/", "_") or "arm"
	return f"hold_{service_token}"


def recover_service_name(namespace: str) -> str:
	"""Root-level Trigger service name for the hand-coordinated recovery of one
	side, e.g. ``recover_right_arm`` — hosted by the ``shared_can_recovery``
	node and called by the duo e-stop as the hard-escalation step."""
	service_token = normalize_relative_namespace(namespace).replace("/", "_") or "arm"
	return f"recover_{service_token}"


class DuoSoftEstopCoordinator(Node):
	"""Central duo e-stop with two levels.

	``hold_<side>`` is the SOFT hold for one arm: cancel the MIT trajectory and
	request a hold, nothing heavier. ``/emergency_stop`` is the HARD duo stop: it
	issues the soft hold on every arm for instant responsiveness, then escalates
	each side to the hand-coordinated ``recover_<side>`` service (verified arm
	emergency stop that self-escalates to a bus-recovery link reset). If the
	recovery node is not running it falls back to calling the arm driver's own
	verified ``emergency_stop`` directly, so the operator stop still escalates to
	bus recovery. All services return a Trigger result so a supervisor sees the
	verified/unverified outcome, and any latched fault lockout is reported back —
	this node never clears the lockout (re-arming is the initiator's decision).
	"""

	def __init__(self) -> None:
		if Empty is None or Trigger is None:
			raise RuntimeError("std_srvs is required to run the duo soft e-stop coordinator")
		super().__init__("duo_soft_estop")

		self.declare_parameter("arm_namespaces", ["left_arm", "right_arm"])
		self.declare_parameter("service_timeout", 5.0)
		self.declare_parameter("poll_period", 0.05)
		configured_namespaces = [
			normalize_relative_namespace(value)
			for value in self.get_parameter("arm_namespaces").value
		]
		self.arm_namespaces = [value for value in configured_namespaces if value]
		if not self.arm_namespaces:
			raise ValueError("arm_namespaces must contain at least one namespace")
		self.service_timeout = float(self.get_parameter("service_timeout").value)
		self.poll_period = float(self.get_parameter("poll_period").value)

		# Reentrant group so a Trigger callback can block on the futures of the
		# recovery/estop services it calls on other nodes under a
		# MultiThreadedExecutor without deadlocking on the client responses.
		self._cb_group = ReentrantCallbackGroup()

		self.pending_futures = []
		self.cancel_clients = {
			namespace: self.create_client(
				Empty, mit_service_path(namespace, "cancel_trajectory"),
				callback_group=self._cb_group,
			)
			for namespace in self.arm_namespaces
		}
		self.hold_clients = {
			namespace: self.create_client(
				Empty, mit_service_path(namespace, "hold_current"),
				callback_group=self._cb_group,
			)
			for namespace in self.arm_namespaces
		}
		# Hard-escalation clients: the hand-coordinated recovery service, and the
		# arm driver's own verified e-stop as a fallback if it is not running.
		self.recover_clients = {
			namespace: self.create_client(
				Trigger, recover_service_name(namespace), callback_group=self._cb_group,
			)
			for namespace in self.arm_namespaces
		}
		self.estop_clients = {
			namespace: self.create_client(
				Trigger, arm_service_path(namespace, "emergency_stop"),
				callback_group=self._cb_group,
			)
			for namespace in self.arm_namespaces
		}

		self.create_service(
			Trigger, "emergency_stop", self._estop_all_callback, callback_group=self._cb_group,
		)
		for namespace in self.arm_namespaces:
			self.create_service(
				Trigger,
				hold_service_name(namespace),
				lambda request, response, arm_namespace=namespace: self._hold_single_callback(
					request,
					response,
					arm_namespace,
				),
				callback_group=self._cb_group,
			)

		self.get_logger().info(
			"Duo e-stop ready for MIT namespaces: " + ", ".join(self.arm_namespaces)
		)

	def _track_future(self, future) -> None:
		self.pending_futures.append(future)

		def _cleanup(done_future) -> None:
			try:
				self.pending_futures.remove(done_future)
			except ValueError:
				pass

		future.add_done_callback(_cleanup)

	def _dispatch_empty(self, client, namespace: str, service_name: str) -> bool:
		if not client.wait_for_service(timeout_sec=0.0):
			self.get_logger().warn(
				f"Skipped {service_name} for '{namespace}' because the service is not available"
			)
			return False

		future = client.call_async(Empty.Request())
		self._track_future(future)
		return True

	def _call_trigger_sync(self, client, label: str) -> tuple[bool | None, str]:
		"""Call a Trigger service and wait (bounded). ``success`` is None when the
		service is absent, so the caller can fall back."""
		if not client.wait_for_service(timeout_sec=self.service_timeout):
			return None, f"{label}: unavailable"
		future = client.call_async(Trigger.Request())
		deadline = time.monotonic() + self.service_timeout
		while rclpy.ok() and not future.done():
			if time.monotonic() > deadline:
				return False, f"{label}: timed out"
			time.sleep(self.poll_period)
		resp = future.result()
		if resp is None:
			return False, f"{label}: no response"
		return bool(resp.success), resp.message or ""

	def _request_soft_hold(self, namespace: str) -> bool:
		cancel_scheduled = self._dispatch_empty(
			self.cancel_clients[namespace],
			namespace,
			"cancel_trajectory",
		)
		hold_scheduled = self._dispatch_empty(
			self.hold_clients[namespace],
			namespace,
			"hold_current",
		)
		if cancel_scheduled or hold_scheduled:
			self.get_logger().warn(f"Issued soft e-stop hold for '{namespace}'")
			return True
		return False

	def _hard_recover(self, namespace: str) -> tuple[bool, str]:
		"""Escalate one side to a verified stop + bus recovery.

		Prefers the hand-coordinated recovery service; falls back to the arm
		driver's own verified e-stop (which self-escalates to a bus-recovery link
		reset) when the recovery node is not running.
		"""
		ok, msg = self._call_trigger_sync(
			self.recover_clients[namespace], f"recover[{namespace}]"
		)
		if ok is not None:
			return bool(ok), f"recover: {msg}"
		# Recovery node absent — go straight to the arm's verified e-stop.
		ok, msg = self._call_trigger_sync(
			self.estop_clients[namespace], f"emergency_stop[{namespace}]"
		)
		if ok is None:
			return False, "no recovery service and no arm emergency_stop available"
		return bool(ok), f"recovery node absent, arm emergency_stop: {msg}"

	def _estop_all_callback(self, request, response):
		del request
		all_ok = True
		lockout = False
		notes: list[str] = []
		for namespace in self.arm_namespaces:
			# Instant soft hold first, then the hard verified escalation.
			self._request_soft_hold(namespace)
			ok, msg = self._hard_recover(namespace)
			all_ok = all_ok and ok
			if "fault_lockout=latched" in msg:
				lockout = True
			notes.append(f"[{namespace}] {msg}")
		response.success = bool(all_ok)
		summary = "; ".join(notes)
		if lockout:
			summary += (
				" — fault_lockout latched; call clear_fault_lockout to re-arm "
				"deliberately (this node does not clear it)"
			)
		if all_ok:
			response.message = "duo e-stop verified on all arms: " + summary
			self.get_logger().warn(response.message)
		else:
			response.message = (
				"duo e-stop NOT verified on all arms — cut arm power. " + summary
			)
			self.get_logger().error(response.message)
		return response

	def _hold_single_callback(self, request, response, arm_namespace: str):
		del request
		scheduled = self._request_soft_hold(arm_namespace)
		response.success = bool(scheduled)
		response.message = (
			f"soft hold issued for '{arm_namespace}'"
			if scheduled
			else f"soft hold services unavailable for '{arm_namespace}'"
		)
		return response


def main() -> None:
	if rclpy is None or Empty is None or Trigger is None:
		raise RuntimeError("ROS runtime dependencies are required to run the duo soft e-stop coordinator")
	rclpy.init()
	node = DuoSoftEstopCoordinator()
	executor = MultiThreadedExecutor()
	executor.add_node(node)
	try:
		executor.spin()
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


__all__ = [
	"DuoSoftEstopCoordinator",
	"arm_service_path",
	"hold_service_name",
	"join_relative_namespaces",
	"main",
	"mit_service_path",
	"normalize_relative_namespace",
	"recover_service_name",
]