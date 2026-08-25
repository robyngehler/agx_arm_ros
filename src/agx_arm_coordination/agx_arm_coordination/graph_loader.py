"""YAML-backed activity/catalogue loader (database-bridge replacement).

The activity graph + catalogue live in version-controlled YAML, behind the same
logical service contract a ``db_bridge`` exposes — ``get_activity_plan`` /
``validate_activity`` / ``get_action_detail``. A real database can replace this
loader later without any coordinator change.

Layout (installed to the package share/config):

    config/catalogue.yaml              resources + actions (shared catalogue)
    config/catalogue.d/<name>.yaml     optional catalogue fragments, merged in
    config/activities/<id>.yaml        one activity graph (nodes + edges)

``catalogue.d/`` exists so a demo whose actions carry bulky taught ``waypoints``
can live in its own file instead of drowning the shared catalogue. Fragments use
the same schema and share one flat ``action_id`` namespace: a fragment that
redefines an existing ``action_id`` is an error, not a silent override.

The coordinator constructs one :class:`ActivityCatalogue` and calls the three
methods below.
"""

from __future__ import annotations

import math
from pathlib import Path

import json

import yaml

from agx_arm_coordination.graph_model import (
    Action,
    ActivityGraph,
    parse_activity,
    parse_catalogue,
    validate_activity,
)


#: Key an action uses instead of inlining ``waypoints``.
RECORDING_KEY = "recording"
#: Where the loader records the joint names a referenced recording was taught on,
#: for the planner to check against the group it is about to command.
RECORDING_JOINTS_KEY = "recording_joint_names"


def load_recording(path: Path) -> tuple[list[str], list[dict]]:
    """Read a taught trajectory sidecar into joint names and catalogue waypoints.

    The sidecar carries only what a replay needs — joint names, times and
    positions. Velocities are recomputed by the retiming, efforts are zeroed at
    playback and the flange pose is diagnostic, and dropping them takes a 2320 KB
    recording to 279 KB and its parse from 30 ms to 4.6 ms.

    A recording is referenced rather than inlined because a catalogue that
    carried it would be unreadable, and decimating it to fit is what a replay
    then cannot undo.

    Everything a replay would otherwise discover on the arm is checked here: a
    non-finite or non-increasing time, a ragged row, a row that is not as wide as
    the declared joint names.
    """
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise ValueError(f"recording '{path}' does not exist") from None
    except json.JSONDecodeError as exc:
        raise ValueError(f"recording '{path}' is not valid JSON: {exc}") from None
    if not isinstance(payload, dict):
        raise ValueError(f"recording '{path}': expected a JSON object")

    joint_names = [str(name) for name in payload.get("joint_names") or []]
    # A full teach recording works too, so a file can be referenced straight from
    # the teach library without being converted first.
    try:
        if "points" in payload:
            points = payload.get("points") or []
            times = [float(point["time_from_start"]) for point in points]
            positions = [[float(v) for v in point["positions"]] for point in points]
        else:
            times = [float(t) for t in payload.get("times") or []]
            positions = [[float(v) for v in row] for row in payload.get("positions") or []]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"recording '{path}': unreadable sample ({exc})") from None

    if not times or len(times) != len(positions):
        raise ValueError(
            f"recording '{path}': {len(times)} times against {len(positions)} position rows"
        )
    for index, when in enumerate(times):
        if not math.isfinite(when):
            raise ValueError(f"recording '{path}': sample {index} has a non-finite time")
        if index and when <= times[index - 1]:
            raise ValueError(
                f"recording '{path}': time goes from {times[index - 1]} to {when} at "
                f"sample {index}; recorded times must strictly increase"
            )
    width = len(positions[0])
    if joint_names and width != len(joint_names):
        raise ValueError(
            f"recording '{path}': {width} positions per sample against "
            f"{len(joint_names)} declared joint names {joint_names}"
        )
    for index, row in enumerate(positions):
        if len(row) != width:
            raise ValueError(
                f"recording '{path}': sample {index} has {len(row)} positions, "
                f"sample 0 has {width}"
            )
        if not all(math.isfinite(value) for value in row):
            raise ValueError(
                f"recording '{path}': sample {index} has a non-finite position"
            )

    return joint_names, [
        {"positions": row, "time_from_start_sec": when}
        for when, row in zip(times, positions)
    ]


def load_recording_waypoints(path: Path) -> list[dict]:
    """Catalogue waypoints from a taught trajectory sidecar."""
    return load_recording(path)[1]


def resolve_recordings(actions: dict[str, Action], config_dir: Path) -> None:
    """Materialise every ``recording:`` reference into inline waypoints.

    Done here, at load, so a missing or unreadable recording stops the
    coordinator coming up instead of failing an activity that is already running.
    The joint names travel with the waypoints; only the planner knows the group
    the action will be commanded on, so that comparison happens there.
    """
    for action_id, action in actions.items():
        reference = (action.metadata or {}).get(RECORDING_KEY)
        if not reference:
            continue
        if action.metadata.get("waypoints"):
            raise ValueError(
                f"action '{action_id}' declares both '{RECORDING_KEY}' and 'waypoints'; "
                "reference one recording or inline the other, not both"
            )
        path = Path(reference)
        if not path.is_absolute():
            path = config_dir / path
        try:
            joint_names, waypoints = load_recording(path)
        except ValueError as exc:
            raise ValueError(f"action '{action_id}': {exc}") from None
        action.metadata["waypoints"] = waypoints
        if joint_names:
            action.metadata[RECORDING_JOINTS_KEY] = joint_names


def validate_playback(actions: dict[str, Action]) -> None:
    """Parse every action's ``playback`` block, so a bad one fails at load.

    A run-level override is checked when the activity goal arrives; this is the
    static half. Prewarming would catch it too, but only once someone runs an
    activity that happens to use the action.
    """
    from agx_arm_coordination.arm_executor import ArmConfigError, playback_spec

    for action_id, action in actions.items():
        metadata = action.metadata or {}
        if "playback" not in metadata:
            continue
        try:
            playback_spec(metadata, action_id)
        except ArmConfigError as exc:
            raise ValueError(str(exc)) from None


class ActivityCatalogue:
    """In-process YAML catalogue exposing the db_bridge-style contract."""

    def __init__(
        self,
        actions: dict[str, Action],
        activities_dir: Path,
        units: dict[str, frozenset[str]],
    ) -> None:
        self._actions = actions
        self._activities_dir = activities_dir
        # The resource table of the topology this runtime is configured for.
        # Required, because validating against a different one than the
        # scheduler uses gives two answers about one machine.
        self._units = units

    # --- construction --------------------------------------------------------

    @classmethod
    def from_config_dir(
        cls, config_dir: str | Path, units: dict[str, frozenset[str]]
    ) -> "ActivityCatalogue":
        config_dir = Path(config_dir)
        catalogue_path = config_dir / "catalogue.yaml"
        data = yaml.safe_load(catalogue_path.read_text(encoding="utf-8")) or {}
        actions = parse_catalogue(data)
        for fragment in sorted((config_dir / "catalogue.d").glob("*.yaml")):
            extra = parse_catalogue(
                yaml.safe_load(fragment.read_text(encoding="utf-8")) or {}
            )
            clashes = sorted(set(extra) & set(actions))
            if clashes:
                raise ValueError(
                    f"catalogue fragment {fragment.name} redefines action_id(s) "
                    f"{clashes}; action_ids are a single flat namespace"
                )
            actions.update(extra)
        resolve_recordings(actions, config_dir)
        validate_playback(actions)
        return cls(
            actions=actions,
            activities_dir=config_dir / "activities",
            units=units,
        )

    # --- db_bridge-style contract -------------------------------------------

    def get_action_detail(self, action_id: str) -> Action:
        try:
            return self._actions[action_id]
        except KeyError:
            raise KeyError(
                f"unknown action_id '{action_id}'; catalogue has {sorted(self._actions)}"
            ) from None

    def get_activity_plan(self, activity_id: str) -> ActivityGraph:
        path = self._activities_dir / f"{activity_id}.yaml"
        if not path.is_file():
            raise KeyError(
                f"unknown activity '{activity_id}'; expected {path}"
            )
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        declared = str(data.get("activity", activity_id))
        if declared != activity_id:
            raise KeyError(
                f"activity file {path} declares '{declared}', expected '{activity_id}'"
            )
        return parse_activity(declared, data)

    def validate_activity(self, activity_id: str) -> list[str]:
        """Return validation problems for an activity ([] => runnable)."""
        try:
            graph = self.get_activity_plan(activity_id)
        except KeyError as exc:
            return [str(exc)]
        return validate_activity(graph, self._actions, self._units)

    # --- introspection -------------------------------------------------------

    @property
    def actions(self) -> dict[str, Action]:
        return dict(self._actions)

    def available_activities(self) -> list[str]:
        if not self._activities_dir.is_dir():
            return []
        return sorted(p.stem for p in self._activities_dir.glob("*.yaml"))
