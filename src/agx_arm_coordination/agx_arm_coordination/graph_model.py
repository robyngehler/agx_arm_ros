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
  units, so it conflicts with ``left_arm`` and ``right_arm``. Each side's arm and
  hand also share one physical CAN bus (``left_can_bus`` / ``right_can_bus``), so
  same-side arm and hand actions conflict too (graph doc §Resources).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


# robot_id -> set of physical units it occupies. Intersection => resource
# conflict. Each side's arm and hand share ONE physical CAN bus (only two mttcan
# channels, one per arm), so same-side arm and hand actions conflict on the shared
# ``*_can_bus`` unit and are never scheduled concurrently — this encodes the
# Step-and-Settle rule that the arm owns the side bus and the hand only gets
# explicit windows (docs/sprint6/planning/shared_can_step_and_settle_integration_plan.md
# §1.7, §4). both_arms holds both per-arm units and both side buses.
ROBOT_UNITS: dict[str, frozenset[str]] = {
    "left_arm": frozenset({"left_arm", "left_can_bus"}),
    "right_arm": frozenset({"right_arm", "right_can_bus"}),
    "both_arms": frozenset(
        {"left_arm", "right_arm", "left_can_bus", "right_can_bus"}
    ),
    "left_hand": frozenset({"left_hand", "left_can_bus"}),
    "right_hand": frozenset({"right_hand", "right_can_bus"}),
}

ACTIONTYPE_GRIPPER = "Gripper"
ACTIONTYPE_TRAJECTORY = "Trajectory"
VALID_ACTIONTYPES = (ACTIONTYPE_GRIPPER, ACTIONTYPE_TRAJECTORY)


class GraphError(ValueError):
    """Raised when an activity graph or catalogue is structurally invalid."""


def units_for(robot_id: str) -> frozenset[str]:
    """Physical units a robot_id occupies (empty set for an unknown id)."""
    return ROBOT_UNITS.get(robot_id, frozenset())


def conflicts(robot_a: str, robot_b: str) -> bool:
    """True when two robot_ids cannot run simultaneously (shared units)."""
    return bool(units_for(robot_a) & units_for(robot_b))


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
        if self.robot_id not in ROBOT_UNITS:
            raise GraphError(
                f"action '{self.action_id}': unknown robot_id '{self.robot_id}'; "
                f"expected one of {sorted(ROBOT_UNITS)}"
            )


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

def validate_activity(graph: ActivityGraph, catalogue: dict[str, Action]) -> list[str]:
    """Return a list of problems; empty list means the activity is runnable.

    Checks: every node's action_id exists in the catalogue; every edge references
    existing nodes; the graph is acyclic; and each sync group is internally
    resource-consistent (its members can actually run in parallel).
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
                if act_a and act_b and conflicts(act_a.robot_id, act_b.robot_id):
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
    """

    def __init__(self, graph: ActivityGraph, catalogue: dict[str, Action]) -> None:
        self.graph = graph
        self.catalogue = catalogue

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

        # Resource serialization: greedily admit in deterministic action_no order;
        # skip anything that conflicts with running or already-admitted units.
        held: set[str] = set()
        for action_no in sorted(running):
            node = self.graph.nodes.get(action_no)
            if node:
                held |= units_for(self.catalogue[node.action_id].robot_id)

        batch: list[DispatchItem] = []
        for node in sorted(eligible, key=lambda n: n.action_no):
            robot = self.catalogue[node.action_id].robot_id
            node_units = units_for(robot)
            if node_units & held:
                continue
            held |= node_units
            batch.append(
                DispatchItem(
                    action_no=node.action_no,
                    action_id=node.action_id,
                    sync_flag=node.sync_flag,
                )
            )
        return batch
