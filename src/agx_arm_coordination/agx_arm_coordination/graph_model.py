"""ROS-free activity-graph model, resource model, and scheduler.

This is the heart of the coordinator, kept free of rclpy so the DAG validation,
resource serialization, and sync-flag barrier logic can be unit-tested without a
running ROS graph. The coordinator node drives it; the performer executes the
batches it hands back.

Adapted from the cetibar Activity-DAG coordinator (see
docs/sprint6/reference/) onto our robot_id / resource set.

Model:

- an **Action** is one catalogue entry: ``action_id`` + ``actiontype_id``
  (Gripper | Trajectory) + ``robot_id`` + free-form ``metadata``.
- an **ActivityGraph** is a DAG of nodes (``action_no`` -> ``action_id``) plus
  edges; nodes sharing a ``sync_flag`` form a barrier group.
- **resources**: each ``robot_id`` occupies a set of physical units; two actions
  conflict when their unit sets intersect. ``both_arms`` occupies both per-arm
  units, so it conflicts with ``left_arm`` and ``right_arm``. Whether a side's
  arm and hand also conflict is **derived from the declared CAN topology**, not
  fixed here: on ``dedicated_per_device`` every device owns its bus and
  same-side arm and hand motion may overlap; on ``shared_per_side`` they hold
  one ``<side>_can_bus`` token and are serialized (graph doc §Resources).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from agx_arm_coordination.gripper_closure import (
    CLOSURE_KEY,
    GRIPPER_SIDES,
    ClosureError,
    closure_to_width,
)


TOPOLOGY_DEDICATED = "dedicated_per_device"
TOPOLOGY_SHARED = "shared_per_side"

# robot_id -> set of physical units it occupies. Intersection => resource
# conflict. Both tables give a side's arm and hand a bus token; what differs is
# whether it is the SAME token.
#
# Shared: two mttcan channels, one per side, and the hand rides the arm's. The
# side bus is one unit, so same-side arm and hand are never scheduled
# concurrently — the Step-and-Settle rule, where the arm owns the bus and the
# hand only gets explicit windows.
ROBOT_UNITS_SHARED: dict[str, frozenset[str]] = {
    "left_arm": frozenset({"left_arm", "left_can_bus"}),
    "right_arm": frozenset({"right_arm", "right_can_bus"}),
    "both_arms": frozenset(
        {"left_arm", "right_arm", "left_can_bus", "right_can_bus"}
    ),
    "left_hand": frozenset({"left_hand", "left_can_bus"}),
    "right_hand": frozenset({"right_hand", "right_can_bus"}),
    "left_gripper": frozenset({"left_gripper", "left_can_bus"}),
    "right_gripper": frozenset({"right_gripper", "right_can_bus"}),
}

# Dedicated: four buses, one per device. The arm and the hand of one side hold
# different tokens, so they no longer conflict and may run in parallel. The
# tokens are kept rather than dropped, because they still serialize a device
# against itself and against ``both_arms``.
ROBOT_UNITS_DEDICATED: dict[str, frozenset[str]] = {
    "left_arm": frozenset({"left_arm", "left_arm_bus"}),
    "right_arm": frozenset({"right_arm", "right_arm_bus"}),
    "both_arms": frozenset(
        {"left_arm", "right_arm", "left_arm_bus", "right_arm_bus"}
    ),
    "left_hand": frozenset({"left_hand", "left_hand_bus"}),
    "right_hand": frozenset({"right_hand", "right_hand_bus"}),
    # The parallel gripper is not freed by the dedicated topology: it rides its
    # arm's bus and its arm's SDK session by vendor design, so it holds the arm's
    # token in both tables and is serialized against that arm either way. Freeing
    # it is a hardware question (concurrent arm motion + gripper command), not a
    # wiring one — see docs/sprint_piper/checklist.md.
    "left_gripper": frozenset({"left_gripper", "left_arm_bus"}),
    "right_gripper": frozenset({"right_gripper", "right_arm_bus"}),
}

# The conservative reading is the default everywhere the topology is not stated.
# Scheduling a hand action while the arm still holds the same bus is the failure
# this table exists to prevent, so an unstated topology must not unlock it.
ROBOT_UNITS: dict[str, frozenset[str]] = ROBOT_UNITS_SHARED

# Which robot ids exist is a property of the machine, not of how it is wired.
# Both tables carry the same keys; checking membership against one of them made
# a naming check look like a topology decision.
VALID_ROBOT_IDS: frozenset[str] = frozenset(ROBOT_UNITS_SHARED)


def robot_units(topology: str) -> dict[str, frozenset[str]]:
    """Resource table for a declared CAN topology.

    Anything other than ``dedicated_per_device`` — including an unknown value —
    reads as shared. A topology nobody recognised is not a licence to run a hand
    beside its arm.
    """
    if topology == TOPOLOGY_DEDICATED:
        return ROBOT_UNITS_DEDICATED
    return ROBOT_UNITS_SHARED


ACTIONTYPE_GRIPPER = "Gripper"
ACTIONTYPE_TRAJECTORY = "Trajectory"
VALID_ACTIONTYPES = (ACTIONTYPE_GRIPPER, ACTIONTYPE_TRAJECTORY)


class GraphError(ValueError):
    """Raised when an activity graph or catalogue is structurally invalid."""


def units_for(
    robot_id: str, units: dict[str, frozenset[str]] | None = None
) -> frozenset[str]:
    """Physical units a robot_id occupies (empty set for an unknown id)."""
    return (units if units is not None else ROBOT_UNITS).get(robot_id, frozenset())


def conflicts(
    robot_a: str, robot_b: str, units: dict[str, frozenset[str]] | None = None
) -> bool:
    """True when two robot_ids cannot run simultaneously (shared units)."""
    return bool(units_for(robot_a, units) & units_for(robot_b, units))


# Optional action metadata: the arm on this action's side switches to its
# configured payload gravity model (attach) or back to the unloaded one (detach)
# once the action succeeds. Absent means the payload state is left alone.
PAYLOAD_UPDATE_KEY = "payload_update"
VALID_PAYLOAD_UPDATES: frozenset[str] = frozenset({"attach", "detach"})


@dataclass(frozen=True)
class Action:
    """One catalogue action."""

    action_id: str
    actiontype_id: str
    robot_id: str
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.actiontype_id not in VALID_ACTIONTYPES:
            raise GraphError(
                f"action '{self.action_id}': actiontype_id '{self.actiontype_id}' "
                f"invalid; expected one of {VALID_ACTIONTYPES}"
            )
        if self.robot_id not in VALID_ROBOT_IDS:
            raise GraphError(
                f"action '{self.action_id}': unknown robot_id '{self.robot_id}'; "
                f"expected one of {sorted(VALID_ROBOT_IDS)}"
            )
        # Rejected at load, not at dispatch: a mistyped value that silently did
        # nothing would run the lift under the unloaded gravity model.
        if PAYLOAD_UPDATE_KEY in self.metadata:
            value = self.metadata[PAYLOAD_UPDATE_KEY]
            if value not in VALID_PAYLOAD_UPDATES:
                raise GraphError(
                    f"action '{self.action_id}': {PAYLOAD_UPDATE_KEY} '{value}' "
                    f"invalid; expected one of {sorted(VALID_PAYLOAD_UPDATES)}"
                )
        if self.robot_id in GRIPPER_SIDES:
            self._validate_closure()

    def _validate_closure(self) -> None:
        """A parallel-gripper action carries a closure in [0, 1], checked at load.

        The catalogue speaks normalized closure; a missing or malformed one would
        otherwise surface as a dispatch failure mid-activity, with the arm
        already somewhere. A gripper executes nothing else — there is no path
        that would take a Trajectory on two jaws.
        """
        if self.actiontype_id != ACTIONTYPE_GRIPPER:
            raise GraphError(
                f"action '{self.action_id}': a parallel gripper executes only "
                f"{ACTIONTYPE_GRIPPER} actions, not '{self.actiontype_id}'"
            )
        target = self.metadata.get("target") or {}
        if not isinstance(target, dict) or CLOSURE_KEY not in target:
            raise GraphError(
                f"action '{self.action_id}': gripper action needs "
                f"metadata.target.{CLOSURE_KEY} in [0.0, 1.0]"
            )
        try:
            closure_to_width(target[CLOSURE_KEY])
        except ClosureError as exc:
            raise GraphError(f"action '{self.action_id}': {exc}") from exc

    @property
    def closure(self) -> float:
        """Normalized closure of a gripper action (validated at construction)."""
        return float(self.metadata["target"][CLOSURE_KEY])

    @property
    def payload_update(self) -> str:
        """``attach``, ``detach``, or ``""`` when the action changes nothing."""
        return str(self.metadata.get(PAYLOAD_UPDATE_KEY, ""))


@dataclass(frozen=True)
class GraphNode:
    action_no: int
    action_id: str
    sync_flag: int = 0  # 0 => not part of any barrier group


@dataclass
class ActivityGraph:
    activity_id: str
    nodes: dict[int, GraphNode]
    edges: list[tuple[int, int]]

    def successors(self, action_no: int) -> list[int]:
        return [dst for src, dst in self.edges if src == action_no]

    def predecessors(self, action_no: int) -> list[int]:
        return [src for src, dst in self.edges if dst == action_no]

    def sync_group(self, sync_flag: int) -> list[int]:
        if sync_flag == 0:
            return []
        return [n.action_no for n in self.nodes.values() if n.sync_flag == sync_flag]


# --- parsing -----------------------------------------------------------------

def parse_action(action_id: str, spec: dict[str, Any]) -> Action:
    spec = spec or {}
    return Action(
        action_id=action_id,
        actiontype_id=str(spec.get("actiontype_id", "")),
        robot_id=str(spec.get("robot_id", "")),
        metadata=dict(spec.get("metadata", {}) or {}),
    )


def parse_catalogue(data: dict[str, Any]) -> dict[str, Action]:
    raw = (data or {}).get("actions") or {}
    return {str(name): parse_action(str(name), spec) for name, spec in raw.items()}


def parse_activity(activity_id: str, data: dict[str, Any]) -> ActivityGraph:
    data = data or {}
    raw_nodes = data.get("nodes") or []
    nodes: dict[int, GraphNode] = {}
    for entry in raw_nodes:
        action_no = int(entry["action_no"])
        if action_no in nodes:
            raise GraphError(f"activity '{activity_id}': duplicate action_no {action_no}")
        nodes[action_no] = GraphNode(
            action_no=action_no,
            action_id=str(entry["action_id"]),
            sync_flag=int(entry.get("sync_flag", 0) or 0),
        )
    edges: list[tuple[int, int]] = []
    for pair in data.get("edges") or []:
        if len(pair) != 2:
            raise GraphError(f"activity '{activity_id}': edge {pair} must have 2 entries")
        edges.append((int(pair[0]), int(pair[1])))
    return ActivityGraph(activity_id=activity_id, nodes=nodes, edges=edges)


# --- validation --------------------------------------------------------------

def validate_activity(
    graph: ActivityGraph,
    catalogue: dict[str, Action],
    units: dict[str, frozenset[str]],
) -> list[str]:
    """Return a list of problems; empty list means the activity is runnable.

    Checks: every node's action_id exists in the catalogue; every edge references
    existing nodes; the graph is acyclic; and each sync group is internally
    resource-consistent (its members can actually run in parallel).

    ``units`` is the resource table of the topology the activity will actually
    run under, and it is **required**. It used to default to the shared-bus
    table while the scheduler used the configured one, so the two disagreed
    about the same machine: under ``dedicated_per_device`` a synchronized
    ``left_arm + left_hand`` pair was rejected here as sharing a bus, while the
    scheduler would have run it in parallel quite happily. Whichever table is
    right, validation and scheduling have to be reading the same one.
    """
    problems: list[str] = []

    for node in graph.nodes.values():
        if node.action_id not in catalogue:
            problems.append(
                f"node {node.action_no} references unknown action_id '{node.action_id}'"
            )

    for src, dst in graph.edges:
        if src not in graph.nodes:
            problems.append(f"edge ({src},{dst}) has unknown source node {src}")
        if dst not in graph.nodes:
            problems.append(f"edge ({src},{dst}) has unknown target node {dst}")

    if _has_cycle(graph):
        problems.append("activity graph is cyclic (must be a DAG)")

    # Sync groups must be runnable in parallel: their members must not contend
    # for the same physical units (else the barrier can never satisfy resources).
    flags = {n.sync_flag for n in graph.nodes.values() if n.sync_flag}
    for flag in flags:
        members = graph.sync_group(flag)
        for i, a_no in enumerate(members):
            for b_no in members[i + 1:]:
                a, b = graph.nodes[a_no], graph.nodes[b_no]
                act_a = catalogue.get(a.action_id)
                act_b = catalogue.get(b.action_id)
                if act_a and act_b and conflicts(
                    act_a.robot_id, act_b.robot_id, units
                ):
                    problems.append(
                        f"sync_flag {flag}: actions {a.action_id} and {b.action_id} "
                        f"share resources ({act_a.robot_id}/{act_b.robot_id}) and "
                        "cannot run in parallel"
                    )
    return problems


def _has_cycle(graph: ActivityGraph) -> bool:
    WHITE, GRAY, BLACK = 0, 1, 2
    color = {n: WHITE for n in graph.nodes}

    def visit(node: int) -> bool:
        color[node] = GRAY
        for nxt in graph.successors(node):
            if nxt not in color:
                continue
            if color[nxt] == GRAY:
                return True
            if color[nxt] == WHITE and visit(nxt):
                return True
        color[node] = BLACK
        return False

    return any(color[n] == WHITE and visit(n) for n in graph.nodes)


# --- scheduling --------------------------------------------------------------

@dataclass(frozen=True)
class DispatchItem:
    action_no: int
    action_id: str
    sync_flag: int


class Scheduler:
    """Frontier scheduler with resource serialization and sync-flag barriers.

    Pure state machine: the coordinator feeds it ``completed`` and ``running``
    sets and asks for the next batch to dispatch. It never blocks or sleeps.

    ``units`` is the resource table, which the caller derives from the declared
    CAN topology. It is a constructor argument rather than a module lookup so
    this stays testable without a registry on disk — and so a caller that never
    thought about the topology gets the serializing table, not the parallel one.
    """

    def __init__(
        self,
        graph: ActivityGraph,
        catalogue: dict[str, Action],
        units: dict[str, frozenset[str]] | None = None,
    ) -> None:
        self.graph = graph
        self.catalogue = catalogue
        self.units = units if units is not None else ROBOT_UNITS

    def _node_ready(self, action_no: int, completed: set[int]) -> bool:
        return all(pred in completed for pred in self.graph.predecessors(action_no))

    def is_complete(self, completed: set[int]) -> bool:
        return len(completed) >= len(self.graph.nodes)

    def next_batch(self, completed: set[int], running: set[int]) -> list[DispatchItem]:
        """Actions that may start now, given what is done and what is running.

        A node is dispatchable when: all predecessors completed; it is not
        already running/completed; if it belongs to a sync group, every member
        of that group is also ready (barrier); and its physical units do not
        conflict with anything running or already selected in this batch.
        """
        pending = [
            n for no, n in self.graph.nodes.items()
            if no not in completed and no not in running and self._node_ready(no, completed)
        ]

        # Enforce sync-group barriers: drop a member unless its whole group is
        # ready in the same tick (and none of its members already running).
        ready_nos = {n.action_no for n in pending}
        eligible: list[GraphNode] = []
        for node in pending:
            if node.sync_flag:
                group = self.graph.sync_group(node.sync_flag)
                group_pending_or_done = all(
                    (g in ready_nos) or (g in completed) for g in group
                )
                group_none_running = not any(g in running for g in group)
                # all not-yet-done members must be ready together
                not_done = [g for g in group if g not in completed]
                all_ready = all(g in ready_nos for g in not_done)
                if not (group_pending_or_done and group_none_running and all_ready):
                    continue
            eligible.append(node)

        # Resource admission. The unit of admission is a *synchronization
        # group*, not an action: a group is admitted whole or not at all.
        #
        # Admitting members one at a time looked equivalent and was not. With an
        # independent left_arm action competing against a synced
        # left_arm + right_arm pair, the independent one takes the left arm, the
        # pair's left member is skipped for conflicting, and its right member is
        # dispatched alone — half a barrier, which is the one outcome sync_flag
        # exists to forbid. The group is what has to fit, so the group is what
        # gets tested against the held units.
        held: set[str] = set()
        for action_no in sorted(running):
            node = self.graph.nodes.get(action_no)
            if node:
                held |= units_for(self.catalogue[node.action_id].robot_id, self.units)

        singles: list[list[GraphNode]] = []
        by_flag: dict[int, list[GraphNode]] = {}
        for node in eligible:
            if node.sync_flag:
                by_flag.setdefault(node.sync_flag, []).append(node)
            else:
                singles.append([node])

        # Deterministic order, and a group sorts by its lowest member so the
        # ordering does not depend on dict iteration.
        candidates = singles + list(by_flag.values())
        candidates.sort(key=lambda members: min(m.action_no for m in members))

        batch: list[DispatchItem] = []
        for members in candidates:
            member_units = [
                units_for(self.catalogue[m.action_id].robot_id, self.units)
                for m in members
            ]
            group_units: set[str] = set()
            self_conflict = False
            for unit_set in member_units:
                if unit_set & group_units:
                    # Members of one group contend with each other, so the
                    # barrier can never be satisfied. validate_activity rejects
                    # this before it runs; if it arrives anyway, stalling is the
                    # honest outcome — admitting the group would put two
                    # commanders on one device.
                    self_conflict = True
                    break
                group_units |= unit_set
            if self_conflict or (group_units & held):
                continue
            held |= group_units
            for node in sorted(members, key=lambda n: n.action_no):
                batch.append(
                    DispatchItem(
                        action_no=node.action_no,
                        action_id=node.action_id,
                        sync_flag=node.sync_flag,
                    )
                )
        return batch
