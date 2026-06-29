"""YAML-backed activity/catalogue loader (the MVP database-bridge replacement).

Architecture decision §8: for the MVP the activity graph + catalogue live in
version-controlled YAML, behind the SAME logical service contract the reference
``db_bridge`` exposes — ``get_activity_plan`` / ``validate_activity`` /
``get_action_detail``. A real database can replace this loader later without any
coordinator change.

Layout (installed to the package share/config):

    config/catalogue.yaml              resources + actions (shared catalogue)
    config/activities/<id>.yaml        one activity graph (nodes + edges)

The coordinator constructs one :class:`ActivityCatalogue` and calls the three
methods below.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from agx_arm_coordination.graph_model import (
    Action,
    ActivityGraph,
    parse_activity,
    parse_catalogue,
    validate_activity,
)


class ActivityCatalogue:
    """In-process YAML catalogue exposing the db_bridge-style contract."""

    def __init__(self, actions: dict[str, Action], activities_dir: Path) -> None:
        self._actions = actions
        self._activities_dir = activities_dir

    # --- construction --------------------------------------------------------

    @classmethod
    def from_config_dir(cls, config_dir: str | Path) -> "ActivityCatalogue":
        config_dir = Path(config_dir)
        catalogue_path = config_dir / "catalogue.yaml"
        data = yaml.safe_load(catalogue_path.read_text(encoding="utf-8")) or {}
        actions = parse_catalogue(data)
        return cls(actions=actions, activities_dir=config_dir / "activities")

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
        return validate_activity(graph, self._actions)

    # --- introspection -------------------------------------------------------

    @property
    def actions(self) -> dict[str, Action]:
        return dict(self._actions)

    def available_activities(self) -> list[str]:
        if not self._activities_dir.is_dir():
            return []
        return sorted(p.stem for p in self._activities_dir.glob("*.yaml"))
