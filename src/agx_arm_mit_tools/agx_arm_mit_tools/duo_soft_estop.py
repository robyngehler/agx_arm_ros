from __future__ import annotations

try:
	from std_srvs.srv import Empty
except ModuleNotFoundError:
	Empty = None

try:
	import rclpy
	from rclpy.node import Node
except ModuleNotFoundError:
	rclpy = None
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


def hold_service_name(namespace: str) -> str:
	service_token = normalize_relative_namespace(namespace).replace("/", "_") or "arm"
	return f"hold_{service_token}"


class DuoSoftEstopCoordinator(Node):
	def __init__(self) -> None:
		if Empty is None:
			raise RuntimeError("std_srvs is required to run the duo soft e-stop coordinator")
		super().__init__("duo_soft_estop")

		self.declare_parameter("arm_namespaces", ["left_arm", "right_arm"])
		configured_namespaces = [
			normalize_relative_namespace(value)
			for value in self.get_parameter("arm_namespaces").value
		]
		self.arm_namespaces = [value for value in configured_namespaces if value]
		if not self.arm_namespaces:
			raise ValueError("arm_namespaces must contain at least one namespace")

		self.pending_futures = []
		self.cancel_clients = {
			namespace: self.create_client(Empty, mit_service_path(namespace, "cancel_trajectory"))
			for namespace in self.arm_namespaces
		}
		self.hold_clients = {
			namespace: self.create_client(Empty, mit_service_path(namespace, "hold_current"))
			for namespace in self.arm_namespaces
		}

		self.create_service(Empty, "emergency_stop", self._hold_all_callback)
		for namespace in self.arm_namespaces:
			self.create_service(
				Empty,
				hold_service_name(namespace),
				lambda request, response, arm_namespace=namespace: self._hold_single_callback(
					request,
					response,
					arm_namespace,
				),
			)

		self.get_logger().info(
			"Duo soft e-stop ready for MIT namespaces: " + ", ".join(self.arm_namespaces)
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

	def _request_soft_hold(self, namespace: str) -> None:
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
			self.get_logger().warn(
				f"Issued soft e-stop hold for '{namespace}'"
			)

	def _hold_all_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
		del request
		for namespace in self.arm_namespaces:
			self._request_soft_hold(namespace)
		return response

	def _hold_single_callback(
		self,
		request: Empty.Request,
		response: Empty.Response,
		arm_namespace: str,
	) -> Empty.Response:
		del request
		self._request_soft_hold(arm_namespace)
		return response


def main() -> None:
	if rclpy is None or Empty is None:
		raise RuntimeError("ROS runtime dependencies are required to run the duo soft e-stop coordinator")
	rclpy.init()
	node = DuoSoftEstopCoordinator()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


__all__ = [
	"DuoSoftEstopCoordinator",
	"hold_service_name",
	"join_relative_namespaces",
	"main",
	"mit_service_path",
	"normalize_relative_namespace",
]