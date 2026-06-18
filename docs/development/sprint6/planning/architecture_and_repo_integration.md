# Architecture & Repo Integration — Coordinated Hefeweizen Pour

How the reference Activity-DAG coordinator pattern (`reference/`, from the cetibar multi-robot
project) maps onto **this** repo, plus the architecture decisions that need a call before
implementation. Companion to `hefeweizen_pour_proposal.md`.

## 1. Layered architecture

```
                 ┌──────────────────────────────────────────┐
   Activity-DAG  │ Coordinator  (agx_arm_coordination)       │  execute_activity action
   (graph +      │  load graph → validate → frontier →       │
    resources)   │  resource rules → sync groups → dispatch  │
                 └───────────────┬──────────────────────────┘
                                 │ PerformAction (per node)
                 ┌───────────────▼──────────────────────────┐
   routing by    │ Performer helper  (agx_arm_coordination)  │
   actiontype/   │  Trajectory+both_arms → arm executor      │
   robot_id      │  Gripper+left/right_hand → hand skill ctrl │
                 └───────┬───────────────────────┬──────────┘
                         │                        │
        ┌────────────────▼─────────┐   ┌──────────▼────────────────────┐
        │ both_arms trajectory      │   │ OmniHand skill controller      │
        │ (existing MoveIt FJT /    │   │ (agx_arm_ctrl)                 │
        │  per-arm MIT split)       │   │  skill_name → vendor gesture,  │
        │                           │   │  tactile-confirmed, hold/​fault │
        └───────────────────────────┘   └──────────┬────────────────────┘
                                                    │ vendor SDK (below ROS)
                                                    ▼  OmniHand on native CAN FD side bus
```

The coordinator never touches a vendor SDK or hardware directly — it only dispatches
database/catalogue-backed actions through the performer helper. This is the key separation that
keeps the demo graph hardware-agnostic.

## 2. Concept mapping (reference → this repo)

| Reference (cetibar) | This repo | Notes |
|---|---|---|
| `coordination` node, `/coord/execute_activity` | `agx_arm_coordination`, `~/execute_activity` | new package (see §3) |
| `performer_helper` + `PerformAction` | performer in `agx_arm_coordination` + `agx_arm_msgs/PerformAction` | may start coordinator-internal for the MVP |
| `db_bridge` (`get_activity_plan`, `validate_activity`, `get_action_detail`) | catalogue/graph loader with the **same service contract** | **YAML-backed for MVP** (see §6) |
| `cetibar_msgs/PerformActivity`, `PerformAction`, `RobotEvent` | `agx_arm_msgs/PerformActivity`, `PerformAction`, `RobotEvent` | repo-owned, agx_arm-centric |
| robots `ur_1`, `portal`, `panda_1` | `both_arms`, `left_arm`, `right_arm`, `left_hand`, `right_hand` | our `robot_id` set |
| `R_UR_PORT`, `R_FRANKA` resources | `R_BOTH_ARMS`, `R_LEFT_ARM`, …, optional `R_LEFT_CAN_BUS` | resource tokens, `config/` |
| actiontypes (gripper/trajectory) | `Trajectory`, `Gripper` | routed by performer |
| Gripper executor (vendor) | OmniHand **skill controller** in `agx_arm_ctrl` | adds tactile + state machine |

## 3. Package & code placement (with rule justification)

- **`src/agx_arm_coordination` (NEW package).** Owns the coordinator, performer routing, scheduler,
  resource model, graph/catalogue loader, launch + config. Justified under the escalation rule: a
  stable orchestration boundary distinct from description/planning/control. Once accepted, record it
  in `docs/project/repository_structure.md` and `.claude/rules/repository-structure.md`.
- **OmniHand skill controller → `src/agx_arm_ctrl`.** Keeps the OmniHand bridge and its skills in
  `agx_arm_ctrl` per the current baseline (do not split the bridge yet). New node:
  `omnihand_skill_controller`, launched per side (`/left_hand/...`, `/right_hand/...`).
- **Messages/actions → `src/agx_arm_msgs`.** `PerformActivity.action`, `PerformAction.action`,
  `RobotEvent.msg`. Hand skills ride on `PerformAction` (metadata) for the MVP — no separate
  `HandSkill.action` yet (decision §8).
- **Arm trajectory execution → existing `both_arms` FJT / per-arm MIT path** (no new package).
- **Graph/catalogue/resources data → `config/` (YAML)** for the MVP (see §6).

## 4. ROS contracts (agx_arm-centric)

- `agx_arm_coordination/execute_activity` (`PerformActivity`) — run a named activity graph.
- `performer .../perform` (`PerformAction`) — run one catalogue action.
- hand skills: routed `Gripper` actions reach `omnihand_skill_controller`; expose tactile-confirmed
  grasp/release with feedback (`state`, `progress`, `contact_score`) and a structured result.
- events: `~/events` (`RobotEvent`) from coordinator and from each executor.
- keep hand-only diagnostics under `feedback/omnihand/*` (existing rule); the skill controller
  consumes the tactile stream the bridge already publishes.

## 5. Hand skill abstraction (public-semantic, backend-owned mapping)

This is the core contract from the proposal (§5.1, §7) and must be respected end-to-end:

- the **public** layer (activity graph, catalogue, action goal) only carries the semantic
  `skill_name` (e.g. `grasp_glass_until_contact`) — **never** a vendor `gesture_id`.
- the **OmniHand backend** owns `skill_name → {vendor gesture | custom preset | joint sequence}`.
  Swapping the vendor mapping later must not change the graph.
- behavior is data, not commands: `completion_policy` (e.g. `on_success: hold_internal`,
  `passive_contact_monitoring: true`) and `fallback_policy` (`on_cancel: stop_and_hold`,
  `on_timeout`, `on_contact_loss: abort_activity`). The controller interprets these.
- **hold is internal**, not a coordinator action — after a confirmed grasp the controller holds and
  publishes status, so the coordinator does not block arm + hand resources on the same side bus.

Controller state machine: `IDLE → OPENING | CLOSING_UNTIL_CONTACT → GRASP_HOLDING → RELEASING`,
plus `FAILED`. Details and the per-skill table live in `hand_skill_backend_mapping.md`.

## 6. Key decision — activity-graph storage: YAML vs DB

The reference uses a SQL database (`DatabaseObjects`, `DB_Interactor`, `CompositionHelper`) behind
`db_bridge`. For a single deterministic demo this is heavy infrastructure.

**Recommendation (MVP): YAML-backed catalogue + graph in `config/`, behind the same service
contract** (`get_activity_plan`, `validate_activity`, `get_action_detail`). Rationale:

- one deterministic graph; version-controlled, diffable, debuggable; no DB service to operate
- the coordinator interface stays identical, so a real DB can replace the loader later with no
  coordinator change
- `hefeweizen_activity_graph.md` already gives the concrete YAML

Trade-off: no CRUD/multi-activity management yet. Revisit a DB when many activities or runtime
authoring are needed. **Decision needed:** confirm YAML-for-MVP vs port the DB now.

## 7. Resource model

Resource tokens (proposal §6) live in `config/` and serialize same-resource actions; independent
resources run in parallel; `sync_flag` groups start as a barrier. Start with arm/hand tokens; add
`R_LEFT_CAN_BUS` / `R_RIGHT_CAN_BUS` only if sprint-5 bus-load validation shows arm+hand contention
on a side bus. Tactile hold is NOT a long-running token holder (see §5).

## 8. Resolved architecture decisions (MVP)

1. **Graph storage: YAML for the MVP**, behind the same service contract
   (`get_activity_plan` / `validate_activity` / `get_action_detail`); a real DB can replace the
   loader later with no coordinator change. (§6)
2. **No dedicated `HandSkill.action` for the MVP** — route hand skills through `PerformAction` with
   the skill fields (`skill_name`, `contact_*`, `*_policy`, …) in action metadata. A typed
   `HandSkill.action` can be promoted later if the metadata contract gets heavy.
3. **Performer is coordinator-internal for the MVP** — a router inside `agx_arm_coordination`, not a
   separate node/process. It can be split out later without changing the catalogue contract.
4. **New package: `agx_arm_coordination`** (confirmed). Record it in
   `docs/project/repository_structure.md` and `.claude/rules/repository-structure.md` once it lands.
5. **CAN-bus resource tokens deferred** — start with arm/hand tokens only; add
   `R_LEFT_CAN_BUS` / `R_RIGHT_CAN_BUS` only if sprint-5 bus-load validation shows arm+hand
   contention on a side bus.

## 9. Build order (maps to proposal §9)

1. `omnihand_skill_controller` in `agx_arm_ctrl` + skill mapping; validate standalone (no coordinator).
2. `agx_arm_msgs` action/message additions; route `Gripper` actions through the performer.
3. Validate the `both_arms` trajectory catalogue actions independently.
4. `agx_arm_coordination` coordinator + YAML loader + resource model; run mini activity graphs.
5. Run `hefeweizen_pour_v1` with the escalation ladder (no objects → dummy → water → beer).

Each step has explicit success criteria in the proposal §9 and a checklist in `../checklist.md`.