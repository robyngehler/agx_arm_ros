"""Ctrl+C in the client's terminal has to reach the hardware too.

The coordinator escalates to the emergency stop on a second interrupt. This
client is the process the operator actually holds Ctrl+C in during a demo, and
it used to answer the second one by printing advice and exiting -- leaving the
arms to whatever the coordinator managed on its own. It now takes the same
ladder: cancel, then the unit emergency stop, then say plainly when the stop
could not be verified.

The node needs ROS to construct, so tests build a bare instance via ``__new__``.
"""

from agx_arm_coordination.run_activity_client import (
    ARM_ESTOP_SERVICES,
    DUO_ESTOP_SERVICE,
    RunActivityClient,
)


class _RecordingLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg, *_a, **_k):
        self.messages.append(("info", str(msg)))

    def warn(self, msg, *_a, **_k):
        self.messages.append(("warn", str(msg)))

    def error(self, msg, *_a, **_k):
        self.messages.append(("error", str(msg)))


def _client(outcomes):
    """Client stub whose per-service stop returns ``outcomes[service]``.

    ``None`` stands for a service that is not on the graph at all.
    """
    node = RunActivityClient.__new__(RunActivityClient)
    node._logger = _RecordingLogger()
    node.get_logger = lambda: node._logger
    node._interrupted = False
    node._interrupt_count = 0
    node.called = []

    def _call_estop(service, timeout_sec):
        node.called.append(service)
        result = outcomes.get(service)
        if result is None:
            return None
        return [(service, result[0], result[1])]

    node._call_estop = _call_estop
    return node


def _errors(node):
    return [m for level, m in node._logger.messages if level == "error"]


def test_the_duo_stop_is_tried_first():
    node = _client({DUO_ESTOP_SERVICE: (True, "duo e-stop verified on all arms")})

    assert node._emergency_stop("second interrupt") == 130
    assert node.called == [DUO_ESTOP_SERVICE]
    assert not any("CUT ARM POWER" in m for m in _errors(node))


def test_a_missing_duo_stop_falls_back_to_the_per_arm_stops():
    """A stack brought up without the duo e-stop node still has to be stoppable."""
    node = _client({
        service: (True, "stop=verified") for service in ARM_ESTOP_SERVICES
    })

    node._emergency_stop("second interrupt")

    assert node.called == [DUO_ESTOP_SERVICE, *ARM_ESTOP_SERVICES]
    assert not any("CUT ARM POWER" in m for m in _errors(node))


def test_an_unverified_stop_demands_cutting_power():
    """A stop that cannot be verified is not a stop.

    This unit has no mechanical emergency stop, so the only remaining one is
    removing arm power -- and that drops the arm, which is why it is said out
    loud rather than left for the operator to work out.
    """
    node = _client({DUO_ESTOP_SERVICE: (False, "right_arm stop=commanded_unverifiable")})

    node._emergency_stop("second interrupt")

    errors = _errors(node)
    assert any("CUT ARM POWER" in m for m in errors)
    assert any("commanded_unverifiable" in m for m in errors)


def test_no_stop_service_at_all_still_demands_cutting_power():
    node = _client({})

    node._emergency_stop("second interrupt")

    assert node.called == [DUO_ESTOP_SERVICE, *ARM_ESTOP_SERVICES]
    assert any("CUT ARM POWER" in m for m in _errors(node))
