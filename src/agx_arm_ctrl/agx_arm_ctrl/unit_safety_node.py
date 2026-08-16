#!/usr/bin/env python3
"""The one process that allocates the unit's safety generations.

Phase 1A of the V02 refactor. Every device kept its own `UnitSafety` counter, so
`unit_safety_epoch` on the left arm and on the right arm were unrelated integers
that happened to match, and a command stamp carrying that field could not mean
anything across devices.

This node is deliberately dull. It holds no model, plans nothing, talks to no
hardware, and imports nothing from the vendor SDK — because it has to be more
available than the things it protects. It is *not* inside the coordinator for
the same reason: that process carries MoveIt, the catalogue and the activity
DAG, and restarts for reasons that have nothing to do with safety.

**What this is not.** The generation is command arbitration inside this software
stack, not a protective stop. Every part of the path — ROS, this node, the
drivers, CAN, the Jetson — can fail. A device stops itself without asking anyone
(that is the fast, unilateral path in the driver, and it must keep working with
this node dead); what needs a writer is only the *unit-wide* statement that a
new safety era has begun. The independent hardware emergency stop is a separate
open question in `docs/open_questions.md` and nothing here addresses it.
"""

from __future__ import annotations

import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger

from agx_arm_msgs.msg import AgxUnitSafety
from agx_arm_msgs.srv import RequestUnitStop

from agx_arm_ctrl.device_authority import UnitSafety


class UnitSafetyNode(Node):
    """Allocates unit safety generations and publishes them latched."""

    def __init__(self) -> None:
        super().__init__("unit_safety")

        self.declare_parameter("writer_id", "unit_safety")
        # Republished periodically as well as on change. Latching covers a late
        # subscriber; the heartbeat covers a subscriber that was connected while
        # this node restarted and would otherwise sit on a generation from the
        # previous instance without ever being told.
        self.declare_parameter("heartbeat_period_s", 2.0)

        self.writer_id = str(self.get_parameter("writer_id").value)
        # One identity per run of this process. Observers order incarnations by
        # start time and fail closed when they see a new one, which is what
        # makes a restart survivable: this node cannot know what happened while
        # it was down, so it must not present itself as a continuation.
        self.incarnation = uuid.uuid4().hex
        self.started_ns = time.time_ns()
        heartbeat_period_s = max(
            0.0, float(self.get_parameter("heartbeat_period_s").value)
        )

        self._safety = UnitSafety(
            self.writer_id,
            incarnation=self.incarnation,
            started_ns=self.started_ns,
        )

        self._publisher = self.create_publisher(
            AgxUnitSafety,
            "unit_safety",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._safety.add_listener(self._publish)

        self.create_service(
            RequestUnitStop, "unit_safety/request_stop", self._request_stop
        )
        self.create_service(Trigger, "unit_safety/rearm", self._rearm)

        if heartbeat_period_s > 0.0:
            self.create_timer(heartbeat_period_s, self._heartbeat)

        self._publish(self._safety.snapshot())
        self.get_logger().info(
            f"Unit safety writer '{self.writer_id}' up as incarnation "
            f"'{self.incarnation}'; this allocates the only unit_safety_epoch on "
            "this unit. Observers that already followed an earlier incarnation "
            "hold a stop until an explicit rearm."
        )

    # -- publication ----------------------------------------------------

    def _publish(self, snapshot) -> None:
        try:
            msg = AgxUnitSafety()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.epoch = snapshot.epoch
            msg.stopped = snapshot.stopped
            msg.reason = snapshot.reason
            msg.writer_id = snapshot.writer_id
            msg.writer_incarnation = snapshot.incarnation
            msg.writer_started_ns = snapshot.started_ns
            self._publisher.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"publishing unit safety failed: {exc}")

    def _heartbeat(self) -> None:
        self._publish(self._safety.snapshot())

    # -- services -------------------------------------------------------

    def _request_stop(self, request, response):
        """Allocate a stop generation. Idempotent while already stopped.

        A second request during an existing stop does not advance the
        generation: the stop is already in force, and minting a new one would
        invalidate commands that were already refused, for no gain.
        """
        requester = request.requester or "unknown"
        reason = request.reason or "no reason given"

        if self._safety.stopped:
            snapshot = self._safety.snapshot()
            response.accepted = True
            response.epoch = snapshot.epoch
            response.message = (
                f"unit already stopped at generation {snapshot.epoch} "
                f"({snapshot.reason})"
            )
            self.get_logger().warn(
                f"'{requester}' requested a unit stop ({reason}); already "
                f"stopped at generation {snapshot.epoch}"
            )
            return response

        snapshot = self._safety.stop(f"{requester}: {reason}")
        response.accepted = True
        response.epoch = snapshot.epoch
        response.message = f"unit stopped at generation {snapshot.epoch}"
        self.get_logger().error(
            f"UNIT STOP generation {snapshot.epoch} allocated on request from "
            f"'{requester}': {reason}"
        )
        return response

    def _rearm(self, request, response):
        """Declare the unit armed. Operator surface, never automatic.

        This always allocates a generation, even when this process does not
        believe a stop is in force. After a restart it is exactly the case where
        the writer thinks nothing is wrong that matters: observers that followed
        the previous incarnation are holding a stop this process has no record
        of, and only a new generation from this writer can clear them. Returning
        "nothing to do" here would strand them stopped for good.
        """
        del request
        snapshot = self._safety.rearm("operator rearm")
        response.success = True
        response.message = f"unit rearmed at generation {snapshot.epoch}"
        self.get_logger().warn(
            f"Unit rearmed at generation {snapshot.epoch}. Devices land in "
            "standby, not ready — each still needs its own verified rearm."
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnitSafetyNode()
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
