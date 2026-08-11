"""Arm test double for the L2 integration harness.

The production arm driver (``agx_arm_ctrl_single``) always talks to the vendor
SDK — it has no mock backend — so an end-to-end run without hardware needs a
stand-in that offers exactly the driver surface the coordinator depends on:

* ``<arm_ns>/prepare_hand_window``   (Trigger) — quiesce the arm for a hand window
* ``<arm_ns>/resume_arm_control``    (Trigger) — reopen the arm afterwards
* ``<arm_ns>/emergency_stop``        (Trigger) — hard stop escalation
* ``<mit_ns>/cancel_trajectory``     (Empty)   — safe-stop, primary
* ``<mit_ns>/hold_current``          (Empty)   — safe-stop, pin the arm

Every call is echoed on ``~/calls`` as ``<side>:<service>`` so a test can assert
the *order* of the handoff, not just that it happened. That ordering is the
contract the V02 refactor is about to change, which is why it is pinned here
before the change rather than after.

This double deliberately implements no behaviour beyond answering: it is a
contract fixture, not a simulator. Failure injection is driven from the outside
via the ``fail_services`` parameter.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger


class ArmDouble(Node):
    """Answers the arm-driver and MIT service surface for both sides."""

    def __init__(self) -> None:
        super().__init__("l2_arm_double")

        self.declare_parameter("sides", ["left", "right"])
        self.declare_parameter("arm_service_template", "/{side}_arm")
        self.declare_parameter("mit_controller_template", "/{side}_arm/mit_controller")
        # Service labels ("left:prepare_hand_window", ...) that must answer
        # success=False, so a test can drive the failure paths without patching.
        self.declare_parameter("fail_services", [""])
        # Append-only record of every call, one label per line. A file rather
        # than a topic: the test reads the order while the graph is still up,
        # and reading a live process's stdout would block.
        self.declare_parameter("call_log_path", "")

        sides = list(self.get_parameter("sides").value)
        arm_tpl = str(self.get_parameter("arm_service_template").value)
        mit_tpl = str(self.get_parameter("mit_controller_template").value)
        self._failing = {s for s in self.get_parameter("fail_services").value if s}
        self._call_log_path = str(self.get_parameter("call_log_path").value).strip()

        self.calls: list[str] = []
        self._calls_pub = self.create_publisher(String, "~/calls", 50)

        self._services = []
        for side in sides:
            arm_ns = arm_tpl.format(side=side)
            mit_ns = mit_tpl.format(side=side)
            for name in ("prepare_hand_window", "resume_arm_control", "emergency_stop"):
                self._services.append(
                    self.create_service(
                        Trigger,
                        f"{arm_ns}/{name}",
                        self._trigger_handler(side, name),
                    )
                )
            for name in ("cancel_trajectory", "hold_current"):
                self._services.append(
                    self.create_service(
                        Empty,
                        f"{mit_ns}/{name}",
                        self._empty_handler(side, name),
                    )
                )

        self.get_logger().info(
            f"arm double ready for sides={sides} "
            f"(failing: {sorted(self._failing) or 'none'})"
        )

    def _record(self, side: str, name: str) -> str:
        label = f"{side}:{name}"
        self.calls.append(label)
        msg = String()
        msg.data = label
        self._calls_pub.publish(msg)
        if self._call_log_path:
            with open(self._call_log_path, "a", encoding="utf-8") as handle:
                handle.write(f"{label}\n")
                handle.flush()
        return label

    def _trigger_handler(self, side: str, name: str):
        def _handler(_request, response):
            label = self._record(side, name)
            if label in self._failing:
                response.success = False
                response.message = f"{name} forced to fail by the L2 harness"
            else:
                response.success = True
                response.message = f"{name} ok"
            return response

        return _handler

    def _empty_handler(self, side: str, name: str):
        def _handler(_request, response):
            self._record(side, name)
            return response

        return _handler


def main() -> None:
    rclpy.init()
    node = ArmDouble()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
