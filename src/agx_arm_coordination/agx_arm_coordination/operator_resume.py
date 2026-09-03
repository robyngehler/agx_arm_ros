"""Resuming an activity from an operator step.

An operator counts steps, not graph nodes: one dispatch batch is one step, so a
synchronized pair is a single step even though the graph holds it as two nodes.
:func:`agx_arm_coordination.graph_model.operator_steps` collapses a graph that
way, and this module turns a requested step number into the set of nodes the
coordinator must treat as already done.

Resuming is refused where the first re-dispatched step is a taught replay. A
replay commands the joint angles it was taught, starting from wherever the arm
happens to stand; an anchor move plans from the current state and is
collision-checked. The refusal names the nearest earlier step that is safe, so
the operator gets a usable number rather than a rejection.
"""

from __future__ import annotations

from agx_arm_coordination.arm_executor import is_replay
from agx_arm_coordination.graph_model import Action, ActivityGraph, operator_steps


RESUME_KEY = "resume"
FROM_STEP_KEY = "from_step"


class ResumeError(ValueError):
    """Raised when a requested resume step cannot be honoured."""


def parse_from_step(value) -> int:
    """One operator step number from a metadata value. 1-based."""
    if isinstance(value, bool) or not isinstance(value, int):
        try:
            number = int(str(value).strip())
        except (TypeError, ValueError):
            raise ResumeError(
                f"{RESUME_KEY}.{FROM_STEP_KEY} '{value}' is not a step number"
            ) from None
    else:
        number = value
    if number < 1:
        raise ResumeError(
            f"{RESUME_KEY}.{FROM_STEP_KEY} is 1-based; got {number}"
        )
    return number


def step_of(steps, action_no: int) -> int:
    """1-based operator step a node belongs to, or 0 when it is in none."""
    for index, batch in enumerate(steps, 1):
        if any(item.action_no == action_no for item in batch):
            return index
    return 0


def resumable_steps(steps, actions: dict[str, Action]) -> list[int]:
    """The 1-based steps a resume may start on: those with no taught replay."""
    return [
        index
        for index, batch in enumerate(steps, 1)
        if not any(
            _is_replay_action(actions.get(item.action_id)) for item in batch
        )
    ]


def resume_seed(
    graph: ActivityGraph,
    actions: dict[str, Action],
    units,
    from_step: int,
) -> tuple[set[int], int]:
    """Nodes to mark completed so the run starts at ``from_step``.

    Returns ``(completed_seed, total_steps)``. Step 1 seeds nothing and is
    therefore always the whole activity.
    """
    steps = operator_steps(graph, actions, units)
    total = len(steps)
    if from_step > total:
        raise ResumeError(
            f"activity '{graph.activity_id}' has {total} operator steps; "
            f"cannot resume from {from_step}"
        )

    batch = steps[from_step - 1]
    replays = sorted(
        item.action_id
        for item in batch
        if _is_replay_action(actions.get(item.action_id))
    )
    if replays:
        allowed = [step for step in resumable_steps(steps, actions) if step < from_step]
        nearest = (
            f"the nearest earlier step that plans its own approach is {allowed[-1]}"
            if allowed
            else "no earlier step plans its own approach; start the activity from 1"
        )
        raise ResumeError(
            f"step {from_step} of '{graph.activity_id}' replays a taught path "
            f"({', '.join(replays)}), which starts from wherever the arm stands "
            f"rather than planning its way there; {nearest}"
        )

    seed: set[int] = set()
    for earlier in steps[: from_step - 1]:
        seed |= {item.action_no for item in earlier}
    return seed, total


def _is_replay_action(action: Action | None) -> bool:
    # An action the catalogue does not carry is not a replay; the graph loader
    # has already refused an unknown action_id, so this is only reached for a
    # hand action, which is never one.
    return action is not None and is_replay(action)
