# Sprint 6 — Session Handoff (2026-06-29, arm64 Duo host)

Context for the next session. This session ran on the **real arm64 host with hardware**
(right side live: `can_nero_right` UP, hand + right arm via `nero_right_arm`; left side and
`can_nero_left` not connected yet). A previous attempt at this work crashed mid-way; this
handoff reconstructs where we actually are.

## 1. What is already done / verified this session

### Recovered the crashed attempt
- The uncommitted working-tree changes are the **OmniHand vendor SDK path migration**:
  `Omnihand-2025-SDK` / `omnihand_2025` / `AgibotHandO10.create_hand(...)` →
  `OmniHand-Pro-2025` / `agibot_hand` / `AgibotHandO12(device_id, hand_type)`.
  Touched: `src/agx_arm_ctrl/agx_arm_ctrl/omnihand_bridge_node.py`,
  `src/agx_arm_ctrl/launch/start_omnihand_bridge.launch.py`,
  `scripts/omnihand/omnihand_load_test.py`, `scripts/omnihand/phase1_smoke_test.py`,
  plus doc renames. **It is coherent and builds clean** (see §3). The real SDK exposes exactly
  `AgibotHandO12`, `EFinger`, `EHandType` (verified against
  `vendor/OmniHand-Pro-2025/build/agibot_hand_pkg/agibot_hand/__init__.py`).
- Untracked crash artifacts: `debug_motion_error.txt` (an unrelated earlier MoveIt execution
  note about an arm jumping off-plan after an obstacle — keep for later), `logs/arm.pcap`
  (a CAN capture). `scripts/r` does not exist.
- **Not committed yet** — the SDK migration is still in the working tree only. Recommend a
  focused commit once §5 step 0 is done.

### Validated the sprint-6 slice on the arm64 host (was never run on real ROS before)
The prior validation log entry was a Windows `py_compile` slice only. This session ran the
real toolchain:
- `colcon build` of `agx_arm_msgs`, `agx_arm_ctrl`, `agx_arm_coordination` — **clean**.
- `agx_arm_coordination` unit tests — **32 passed** (`test_graph_model`, `test_graph_loader`,
  `test_arm_executor`, `test_performer`).
- Sprint-6 internals are internally consistent: skill names in
  `agx_arm_ctrl/config/omnihand_skills.yaml` ↔ `agx_arm_coordination/config/catalogue.yaml`
  `skill_name` refs ↔ `arm_config.yaml` anchor-pose names all line up.

### Build gotcha found (will bite anyone who skips the wrapper)
- A bare `colcon build` on this host picks up `~/.local` setuptools and dies with
  `error: option --uninstall not recognized` (Python-env drift). The fix is the documented
  wrapper: `bash ./scripts/colcon_build_system_python.sh --packages-select <pkg>` (filters
  conda/miniforge from PATH, sets `PYTHONNOUSERSITE=1`, forces `/usr/bin/python3`).
- The C++ package `agx_arm_msgs` additionally needs a real cmake; a broken `~/.local/bin/cmake`
  shim (`ModuleNotFoundError: No module named 'cmake'`) shadows it. `agx_arm_msgs` was already
  installed from a clean build, so this only matters when the messages change.

## 2. Root cause of the `components.launch` MoveIt error

```
[move_group] Joint 'right_index_mcp_joint' not found in model 'duo_nero_system'  (x4 fingers, looping)
```

This is **O10/O12 description drift**, not a coordinator bug:
- The hand **URDF + SRDF + MoveIt controllers + initial_positions** are all still the **O10**
  layout (`*_abad` / `*_pip`, `thumb_mcp` active with `pip`/`dip` as mimics; 10 active DoF).
- The **control/skill/SDK layer** migrated to **O12 Pro** (`models.py` `O12_PRO`, 12 active DoF:
  `thumb_pip`, `index_mcp`, `middle_mcp`, `ring_mcp`, `pinky_mcp` are now real joints).
- Your error came from a build state no longer on disk — the crashed attempt had partially
  pushed O12 `*_mcp_joint` names into the SRDF/controllers while the URDF was still O10, so the
  model lacked those joints. The committed tree is internally O10-consistent again.

**Important:** in the sprint-6 design the **hand is driven by the `omnihand_skill_controller`,
not by MoveIt** (semantic skill → vendor preset → SDK). MoveIt only plans the **arm**
(`both_arms` group = arm joints only). So this error is **log noise that does not block arm
planning** — the `debug_motion_error.txt` log shows `both_arms` planning succeeding through the
same spam. It still must be fixed for a clean, correct model.

## 3. The two vendor changes and what they mean

### (a) Upstream repo renamed → `AgibotTech/agillink_omnihand_sdk`
(was the OmniHand-Pro-2025 repo). Impact:
- **Short term: no break.** Our local vendor dir is still `vendor/OmniHand-Pro-2025/`, the built
  package `build/agibot_hand_pkg` and the Python module `agibot_hand` are unchanged, and all our
  path constants point at the local dir. Nothing in the runtime path depends on the GitHub name.
- **Risk: clone/setup instructions and docs that name the old repo/URL are now stale**, and a
  future SDK pull may also rename the Python package (`agibot_hand` → ?). We should:
  - pin/record the exact SDK commit/tag we built `agibot_hand_pkg` against,
  - keep the local dir name `OmniHand-Pro-2025` for now (low churn — many code+doc refs), and
    add a one-line note that upstream is now `agillink_omnihand_sdk`,
  - decide a rename (`vendor/agillink_omnihand_sdk/`) only as a deliberate, separate cleanup.

### (b) Official O12 URDF now available — this is the big unblock
The user downloaded it to
`vendor/OmniHand-Pro-2025/description/urdf/o12_hand_description-o12_t3/`. It ships:
- both sides (`assets/urdf/omnihand_pro_{right,left}.urdf`) + xacro decomposition
  (`xacro/{hand,thumb,index,ring_pinky,const,material}.xacro`),
- **separate visual + collision meshes** (`assets/meshes/` + `assets/meshes/collision/*_col.STL`),
  plus a `urdf_mesh_col` variant and MJCF/MuJoCo + RViz configs.
- The joint topology **matches our O12 active set exactly** (12 active + vendor mimic couplings):
  - active: `thumb_roll, thumb_abad, thumb_mcp, thumb_pip, index_abad, index_mcp, index_pip,
    middle_abad, middle_mcp, middle_pip, ring_mcp, pinky_mcp`
  - mimics (verified in the vendor URDF): `thumb_dip = 0.8068·thumb_pip`,
    `index_dip = 1.0362·index_pip`, `middle_dip = 1.0924·middle_pip`,
    ring driven entirely by `ring_mcp` (`ring_pip = 0.877·ring_mcp`, `ring_dip = 0.9394·ring_mcp`),
    pinky driven entirely by `pinky_mcp` (same factors). Ring/pinky have **no abad**.

**Meaning:** the O12 description migration is now **vendor-grounded** (real link frames, real
meshes, real mimic ratios) instead of inventing geometry. This removes the only real blocker
that made "migrate to O12 now" risky. Two adaptation points before integrating:
1. **Prefix convention:** vendor uses `R_`/`L_` (e.g. `R_index_mcp_joint`); our ROS surface and
   the bridge/skills use `right_`/`left_` (e.g. `right_index_mcp_joint`). Joints, links, and mesh
   refs must be reprefixed on import.
2. **Variant `o12_t3`:** confirm `t3` is the fingertip/tactile config of the hand we physically
   own before adopting it as canonical.

## 4. Tooling decisions taken this session (for the Hefeweizen teach loop)

Confirmed reusable, in-place (no new package — per "reuse + minimal overhead"):
- **freedrive / leader mode + trajectory recording:** `agx_arm_mit_demos`
  `leader_trajectory_recorder` (`agx_arm_record_leader_trajectory`). Uses
  `set_leader_mode`/`set_normal_mode`/`enable_agx_arm` + `feedback/leader_joint_angles`, saves
  `RecordedTrajectory` JSON via `agx_arm_mit_controller.trajectory_io`.
- **performer / playback:** `agx_arm_mit_demos` `execute_saved_trajectory`
  (`agx_arm_execute_saved_trajectory`) → publishes `mit_controller/joint_trajectory` +
  `mit_controller/enable`.
- **FJT → MIT adapter:** `agx_arm_mit_tools` `MitFollowJointTrajectoryActionBridge`
  (default action `arm_controller/follow_joint_trajectory`).

**Gap to close (planned, not yet built):** there is **no single-pose capture tool**. The
coordinator's `agx_arm_coordination/config/arm_config.yaml` anchor poses (`Idle_*`, `Pre_Grip_*`,
`grasp_*`, …) are **all-zero placeholders**. We need a small "capture current joint config →
named anchor pose in `arm_config.yaml`" entry point next to the recorder.

**Seam mismatch to resolve:** `arm_executor` targets a `both_arms_controller/follow_joint_trajectory`
action that does not exist on the live right-side system. For right-side-first we wire it to the
existing MIT FJT bridge with `right_arm` joints; mirror to `both_arms` when the left side is up.

## 5. Plan for next session (ordered, minimal overhead, functionality first)

0. **Commit the SDK migration** as its own change (builds clean; right side present) so the
   working tree is clean before the bigger description work. Record the SDK commit/tag.
   -> don't mention co-authors
1. **Integrate the vendor O12 URDF** into `src/agx_arm_sim/agx_arm_description`:
   - import the vendor xacro/meshes (visual + `collision/*_col.STL`), reprefix `R_`/`L_` →
     `right_`/`left_`, wire the mimic couplings as in the vendor URDF, expose as the new
     `omnihand_hand` macro replacing the O10 hand xacro.
   - update **SRDF group** (`agx_arm.srdf.xacro` `omnihand_group`), **MoveIt controllers**
     (`moveit_controllers_omnihand_{right,left}.yaml`), **`initial_positions.yaml`**, and
     **`ros2_control.xacro`** to the **12 active joints** — this is the actual fix for the §2 error.
   - validate: `xacro` expands; `components.launch` / MoveIt loads `duo_nero_system` with **no**
     "joint not found"; both meshes render in RViz.
2. **Right-side coordinator arm path:** point `arm_executor` at the MIT FJT bridge for `right_arm`;
   keep `both_arms`/left mirror-ready. Smoke `arm_dry_run:=true` then a single real right-arm
   anchor move.
3. **Add the pose-capture tool** (`agx_arm_mit_demos`, e.g. `agx_arm_capture_anchor_pose`):
   read current joint state, write/update a named entry in `arm_config.yaml` (right side first).
   Document the teach loop (freedrive → record/playback → capture anchor → fill catalogue).
4. **Right-side hardware smoke:** bridge `backend_type:=sdk omnihand_type:=right`, skill controller
   `open_hand`/`grasp_*_until_contact`/`release_*` (calibrate `contact_*` on the Pro hand — mock
   tactile is all zeros, so grasps only confirm on the SDK backend), recorder + pose capture.
5. **Teach the Hefeweizen anchors/trajectories** on the right side, fill `arm_config.yaml` +
   catalogue waypoints; run the mini graphs (`hands_open_release_v1` first), then escalate.

## 6. Cleanups / smaller follow-ups

- `models.py` still carries the legacy `o10` entry (`sdk_import_package="omnihand_2025"`,
  `sdk_class_name="AgibotHandO10"`); the bridge now hardcodes `agibot_hand`/`AgibotHandO12` in
  `_load_sdk_symbols`/`_ensure_omnihand_importable` rather than reading those registry fields.
  Either drive the SDK import from the model registry, or drop the dead O10 SDK fields and keep
  o10 as mock-only (the docstring already says the O10 SDK submodule was swapped out).
- Reconcile remaining stable-doc drift flagged on 2026-06-25:
  `docs/assets/omnihand_asset_validation.md` ("mock-only") and
  `docs/assets/control/basic_control_scripts.md` ("10 active joints").
- Add a vendor note (repo rename → `agillink_omnihand_sdk`, pinned SDK commit, local dir kept as
  `OmniHand-Pro-2025`) to `docs/assets/omnihand/omnihand_vendor_sdk_aarch64.md`.
</content>
