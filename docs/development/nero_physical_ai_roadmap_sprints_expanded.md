# Nero Physical AI Roadmap — Expanded Sprint Definition

## 1. Purpose

This document defines the expanded sprint-level roadmap for evolving the currently functional MIT-controller-based Nero arm setup into a modular Physical AI research and development workflow.

The roadmap covers the integration of:

- the AgileX Nero arm,
- the AgiBot OmniHand Pro dexterous gripper,
- a custom AGV/static base platform,
- the existing MIT-style low-level controller,
- ROS 2, MoveIt 2, Isaac Sim, Isaac Lab, and later NVIDIA Physical AI workflows.

This document is intentionally not a low-level implementation manual. It defines the semantic structure, fixed technical frames, sprint objectives, interfaces, expected artifacts, decision gates, and local discovery tasks. Each sprint should be supported by one or more dedicated child documents containing implementation details, validation logs, repository-specific findings, configuration files, and experiment results.

The roadmap is dependency-based rather than calendar-based. Sprints describe logical maturity stages and integration gates, not fixed durations.

---

## 2. System Context

The system is treated as a modular mobile-manipulation research platform.

### 2.1 Physical Components

| Component | Role |
|---|---|
| Nero arm | Primary manipulator |
| OmniHand Pro | Dexterous end effector and tactile manipulation platform |
| Custom AGV/base | Static mount first, mobile base later |
| MIT controller | Low-level model-based controller for arm execution |
| RGB-D cameras | Workspace perception and later policy observations |
| Optional wrist camera | Local end-effector perception |
| Optional wrist force/torque sensor | Contact-rich manipulation and pouring support |

### 2.2 Compute Components

| Platform | Intended Role |
|---|---|
| Jetson AGX Orin | Real hardware integration, ROS 2 runtime, controller bridge, hand driver, sensor preprocessing, compact policy inference, logging, HIL edge node |
| RTX workstation or remote RTX GPU system | Isaac Sim, Isaac Lab, rendering, synthetic data generation, heavier RL training, VLA/VLM experiments |
| Jetson Thor-class platform | Future edge platform for larger policies, VLA/VLM inference, multi-camera perception, GPU-accelerated manipulation, higher-throughput local evaluation |

The AGX Orin should not be treated as the primary platform for heavy Isaac Sim rendering, large-scale Isaac Lab training, or foundation-model fine-tuning.

### 2.3 Core Software Stack

| Area | Primary Tooling | Role |
|---|---|---|
| Middleware | ROS 2 | Common communication layer for real and simulated systems |
| Robot modeling | URDF, Xacro, SRDF, USD | Robot description, planning, simulation, controller model generation |
| Nero assets | `agx_arm_sim`, `agx_arm_description` | Expected source for Nero model assets and examples |
| Motion planning baseline | MoveIt 2 | Planning groups, PlanningScene, collision checks, OMPL planning, trajectory generation |
| IK baseline | TRAC-IK | Primary IK solver for the Nero arm planning group |
| Global planning baseline | OMPL | Sampling-based path planning through MoveIt 2 |
| Collision reasoning | MoveIt 2 PlanningScene | Self-collision and environment collision checks |
| Kinematics/debug | Pinocchio | FK, Jacobians, dynamics/model audit, gravity/payload checks |
| Trajectory smoothing/debug | Ruckig | Jerk-limited trajectory generation, fallback smoothing, controller-side validation |
| Simulation | Isaac Sim | USD-based simulation, sensors, ROS 2 bridge, HIL scenes |
| Robot learning | Isaac Lab | RL and policy training environments |
| GPU manipulation, later | Isaac ROS Manipulation, cuMotion | GPU-accelerated planning and manipulation workflows |
| Environment modeling, later | Nvblox | Dense 3D reconstruction and collision-world generation |
| Object pose estimation, later | FoundationPose or equivalent | Pose estimation for manipulation workflows |
| Physical AI workflows, later | NVIDIA Cosmos-related workflows | Synthetic data, simulation replay, visual variation, evaluation workflows |
| VLA/VLM policies, later | GR00T-style, OpenVLA-style, π0-style models | Skill-level or bounded-action policy evaluation |
| Hand interface | OmniHand Pro SDK / ROS 2 wrapper | Hand control, tactile data, kinematics API, status data |

---

## 3. Roadmap Rules

### 3.1 Iterative Modeling Rule

Arm, hand, and AGV/base assets are validated independently before being merged into combined model variants.

The model stack must support:

- simulation,
- planning,
- controller-side dynamics,
- gravity compensation,
- payload handling,
- mounting-pose awareness,
- collision-aware execution.

### 3.2 Shared Interface Rule

Real and simulated systems should expose equivalent high-level interfaces wherever possible.

A command should ideally be executable in this order:

```text
offline validation
→ MoveIt 2 / PlanningScene validation
→ Isaac Sim validation
→ HIL replay
→ low-speed real execution
→ normal constrained execution
```

### 3.3 MoveIt 2 First Rule

MoveIt 2 with TRAC-IK and OMPL is the primary early-stage IK and planning baseline.

Pinocchio and Ruckig remain part of the system, but primarily for:

- model debugging,
- FK/Jacobian verification,
- gravity and payload checks,
- controller-near trajectory generation,
- fallback and comparison tests.

cuMotion and Isaac ROS Manipulation are introduced only after the robot model, collision geometry, perception, and baseline planning requirements are sufficiently mature.

### 3.4 Hand-as-Tool Rule

The OmniHand Pro is initially treated as:

1. a static payload,
2. a simplified collision envelope,
3. a separately controlled end-effector,
4. later an articulated dexterous model.

The full articulated hand should not initially be included in the main arm IK chain.

### 3.5 Skill-Level Control Rule

Learned policies should not directly emit unsafe raw low-level commands to the robot.

Policies should initially operate through:

- skill selection,
- target pose generation,
- bounded Cartesian deltas,
- grasp proposals,
- preshape selection,
- trajectory parameterization,
- high-level recovery decisions.

### 3.6 Local Agent Discovery Rule

Local agents are expected to inspect repositories, assets, configuration files, launch files, SDK examples, and generated models. Their output should refine sprint details without changing the master architecture unless a decision gate explicitly allows it.

Every discovery task should produce one of the following:

```text
CONFIRMED
MISSING
PARTIALLY_AVAILABLE
REPLACED_BY_LOCAL_IMPLEMENTATION
BLOCKED_BY_VENDOR_OR_HARDWARE
```

Discovery outputs should be stored in sprint-specific child documents rather than overwritten into this master roadmap.

---

## 4. Canonical Naming and Interfaces

### 4.1 Model Variants

The following model variants are canonical and should be generated from shared components:

```text
nero_standalone
nero_with_dummy_payload
nero_with_omnihand_static
nero_with_omnihand_articulated
nero_on_static_agv
nero_omnihand_on_static_agv
nero_omnihand_on_mobile_agv
```

Manual duplication of complete URDF files should be avoided. Xacro composition or equivalent model-generation logic is preferred.

### 4.2 Canonical Frame Tree

```text
map
└── odom
    └── agv_base_link
        └── arm_mount_link
            └── nero_base_link
                └── nero_tool0
                    └── wrist_adapter_link
                        └── omnihand_palm_link
                            └── grasp_frame
```

Required semantic frames:

| Frame | Meaning |
|---|---|
| `map` | Global/world frame |
| `odom` | Local odometry frame for future mobile-base operation |
| `agv_base_link` | Main AGV/base reference frame |
| `arm_mount_link` | Mechanical mounting interface between AGV/base and Nero arm |
| `nero_base_link` | Nero arm base frame |
| `nero_tool0` | Nero tool/flange frame |
| `wrist_adapter_link` | Adapter between Nero flange and OmniHand |
| `omnihand_palm_link` | Palm/root frame of the OmniHand Pro |
| `grasp_frame` | Task-level grasp reference frame |

### 4.3 Planning Groups

Initial MoveIt 2 planning groups:

```text
nero_arm
omnihand
nero_arm_with_static_hand
nero_arm_on_static_agv
nero_arm_with_omnihand_on_static_agv
```

Recommended semantics:

| Planning Group | Purpose |
|---|---|
| `nero_arm` | Primary arm IK and motion planning group |
| `omnihand` | Hand control and later finger-level experiments |
| `nero_arm_with_static_hand` | Arm planning with hand treated as attached tool geometry |
| `nero_arm_on_static_agv` | Arm planning with static AGV/base collision environment |
| `nero_arm_with_omnihand_on_static_agv` | Combined collision-aware arm/hand/base planning model |

### 4.4 Command Types

Canonical command interfaces:

```text
JointTrajectoryCommand
JointPositionCommand
EndEffectorPoseCommand
CartesianDeltaCommand
HandPreshapeCommand
SkillCommand
```

The MIT controller should primarily receive:

- joint trajectories,
- joint targets,
- resolved pose targets,
- tool and payload metadata,
- mounting-pose information relevant for gravity compensation.

### 4.5 Skill Command Schema

```text
SkillCommand:
  skill_name
  target_frame
  target_pose
  object_reference
  hand_preshape
  approach_vector
  allowed_contact_mode
  force_limit
  velocity_limit
  timeout_policy
  fallback_policy
```

Canonical initial skills:

```text
MoveToPose
MoveToPregrasp
OpenHand
CloseHandUntilContact
Grasp
Lift
Place
Pour
Retract
Recover
```

---

## 5. Local Agent Discovery Protocol

Local agents may inspect repositories, SDKs, hardware documentation, generated files, and installed packages. They should not silently redefine core naming, frames, planning groups, or interfaces.

### 5.1 Agent Roles

| Agent Role | Responsibility |
|---|---|
| `asset_discovery_agent` | Inspect available URDF, Xacro, SRDF, USD, mesh, CAD, and example scene files |
| `repo_structure_agent` | Map repository structure, packages, launch files, examples, scripts, and dependencies |
| `moveit_config_agent` | Inspect or generate MoveIt 2 configuration packages and planning groups |
| `isaac_asset_agent` | Inspect Isaac Sim USDs, articulation structure, joint drives, sensors, and scene templates |
| `controller_interface_agent` | Inspect MIT controller inputs, trajectory expectations, model assumptions, and safety constraints |
| `hand_sdk_agent` | Inspect OmniHand SDK examples, transport layer, motor indexing, tactile outputs, and ROS 2 support |
| `hardware_inventory_agent` | Track available compute, adapters, cameras, power supplies, calibration tools, and missing hardware |
| `data_schema_agent` | Inspect log formats, synchronization assumptions, data converters, and replay compatibility |

### 5.2 Required Agent Output Format

Each discovery output should include:

```text
component:
repository_or_source:
inspection_date:
status: CONFIRMED | MISSING | PARTIALLY_AVAILABLE | REPLACED_BY_LOCAL_IMPLEMENTATION | BLOCKED_BY_VENDOR_OR_HARDWARE
found_artifacts:
missing_artifacts:
interface_notes:
risks:
recommended_next_action:
related_sprint:
related_child_document:
```

### 5.3 Discovery Boundaries

Agents may propose changes to:

- package names,
- file paths,
- launch structure,
- build system layout,
- asset conversion paths,
- configuration schema,
- validation scripts.

Agents should not change without explicit review:

- canonical frame names,
- core model variant names,
- primary planning group semantics,
- MIT controller safety boundaries,
- high-level sprint ordering,
- real-hardware execution gates.

---

## 6. Expanded Sprint Roadmap

# Sprint 1: Repository and Asset Discovery

## Objective

Establish a reliable inventory of available model, simulation, planning, SDK, and controller assets before creating local abstractions.

## Fixed Scope

- Inspect Nero-related repositories and packages.
- Inspect OmniHand Pro SDK repositories and documentation available locally.
- Inspect existing AGV/base CAD files and export formats.
- Inspect any existing MIT-controller model assumptions, URDF references, launch files, and trajectory interfaces.
- Identify already available Isaac Sim, MoveIt 2, and ROS 2 examples.
- Create a single asset inventory that becomes the source of truth for later sprints.

## Local Agent Discovery Space

Local agents should search for:

```text
URDF
Xacro
SRDF
USD
STEP
STL
DAE
OBJ
MJCF
YAML configs
ROS 2 launch files
MoveIt config packages
Isaac Sim scene files
controller configs
payload configs
SDK examples
README setup steps
Dockerfiles
colcon workspaces
```

Agents should specifically determine whether:

- Nero URDF/Xacro assets are complete enough for MoveIt 2.
- Nero USD assets are complete enough for Isaac Sim.
- OmniHand Pro has a vendor-provided URDF, CAD, USD, MJCF, or kinematics model.
- OmniHand Pro SDK exposes joint-level commands, tactile readings, motor IDs, and ROS 2 examples.
- AGV/base CAD contains useful assembly references or only visual geometry.
- MIT-controller configuration already assumes a specific Nero URDF or gravity frame.

## Required Artifacts

```text
docs/assets/repository_asset_inventory.md
docs/assets/nero_asset_validation.md
docs/assets/omnihand_asset_validation.md
docs/assets/agv_cad_inventory.md
docs/control/mit_controller_model_inventory.md
```

## Expected Outputs

- Confirmed list of available Nero assets.
- Confirmed list of missing or incomplete OmniHand model assets.
- Confirmed list of AGV/base CAD files and usable export formats.
- Confirmed list of existing controller configuration assumptions.
- Initial repository and package map.
- Initial risk list for missing vendor assets.

## Current Local Implementation Snapshot

Working notes for this sprint currently live under:

```text
docs/development/sprint1/
```

Current state as of 2026-05-11:

- Nero URDF/Xacro, meshes, the unified MoveIt package, and the MIT gravity workflow are already present locally.
- Isaac assets are only partially confirmed locally; the older `src/agx_arm_sim/agx_arm_description` tree contains a confirmed `nero_gripper_d435` USD package.
- OmniHand SDK/model assets are now present under `vendor/Omnihand-2025-SDK`, but they are not yet integrated into the local ROS2 planning/control stack and the vendor README currently targets `x86_64` while this host is `aarch64`.
- AGV/base CAD assets are not yet present in the workspace.
- Sprint 1 has now resolved the description source of truth onto `src/agx_arm_sim/agx_arm_description`, removed the former duplicate root package `src/agx_arm_description`, and detached `agx_arm_urdf` into a fixed local Nero/Revo2 asset tree inside the canonical package.

## Decision Gate

Proceed once the team can answer:

- Which Nero model is the planning source of truth?
- Which Nero model is the controller source of truth?
- Which OmniHand model level is available immediately?
- Which AGV/base geometry is suitable for visual use and which must be simplified for collision?
- Which repository components should be reused instead of rewritten?

---

## Early Local Repo Mapping

The detailed sprint decomposition below remains useful for traceability, but the current local repo sequence now executes the early work as merged packages:

- Local Sprint 2 equals the detailed Sprint 2 package-structure work plus the already-landed OmniHand description, MoveIt, and simulation-contract slice.
- Local Sprint 3 equals the detailed Sprint 3, Sprint 4, Sprint 5, and Sprint 6 work, treated as one Nero planning and control hardening phase.
- Local Sprint 4 equals the detailed Sprint 7 and Sprint 8 work, treated as the first Nero plus OmniHand baseline on top of the shared adapter boundary.

This keeps the detailed sections below useful without forcing the repo-level sprint sequence to pretend the work happened in the original first-draft order.

---

# Sprint 2: Common Environment and Package Structure Baseline

## Objective

Create the local common environment that allows independent agents and developers to work without stepping on each other’s toes while sharing one OmniHand and Nero integration contract.

## Fixed Scope

- Define the local workspace structure.
- Define ROS 2 package names.
- Define documentation tree.
- Define configuration directories.
- Define model-variant generation locations.
- Define experiment output locations.
- Define a convention for generated files versus source files.
- Freeze the repo-owned OmniHand public contract for shared arguments, joint naming, and frame semantics.
- Land or document the normalized OmniHand description and MoveIt simulation slice inside that shared structure.
- Keep the OmniHand adapter below ROS and define the bridge boundary that later hardware backends must satisfy.

## Recommended Package Names

```text
nero_description
nero_moveit_config
nero_bringup
nero_control_bridge
nero_isaac_sim
nero_isaac_lab
nero_skill_library
nero_data_tools
nero_hil_tools
omnihand_description
omnihand_driver_ros2
omnihand_skills
agv_base_description
agv_base_bringup
physical_ai_experiments
```

## Recommended Configuration Layout

```text
config/robot_models
config/planning
config/controllers
config/payloads
config/sensors
config/calibration
config/skills
config/datasets
config/experiments
```

## Local Agent Discovery Space

Agents may refine:

- whether packages should be split or merged based on upstream repository structure,
- whether existing MoveIt or Isaac packages can be vendored or referenced,
- whether upstream examples should be imported as submodules, forks, or documentation references,
- whether generated assets should live inside ROS packages or in a dedicated asset repository.

Agents should not rename canonical frames, model variants, or planning groups without explicit review.

## Required Artifacts

```text
docs/project/repository_structure.md
docs/project/package_naming.md
docs/project/generated_vs_source_assets.md
docs/project/local_agent_workflow.md
```

## Expected Outputs

- A reproducible local workspace skeleton.
- A clear place for each model, config, script, document, and experiment.
- A policy for generated assets.
- A policy for upstream repository integration.
- One shared OmniHand and Nero environment contract that later runtime work can reuse.

## Decision Gate

Proceed once a new developer or local agent can locate:

- source robot models,
- generated robot models,
- MoveIt configs,
- Isaac assets,
- controller configs,
- logs,
- documentation,
- experiments.

The local repo should also already expose the agreed OmniHand simulation contract through the canonical description and MoveIt packages before this sprint is considered closed.

---

# Sprint 3: Nero Standalone Model Validation

## Objective

Validate the Nero arm as a standalone robot model for planning, simulation, and controller-side model reasoning.

In the merged local repo sequence, this section is treated as the first detailed sub-phase of local Sprint 3, which continues through the MoveIt, IK, and MIT-controller sections below.

## Fixed Scope

- Load Nero URDF/Xacro.
- Validate link and joint names.
- Validate joint limits, velocity limits, effort limits, and mimic/fixed joints.
- Validate `nero_base_link` and `nero_tool0` semantics.
- Validate mesh availability and path resolution.
- Validate inertial parameters where available.
- Validate whether the model is suitable for MoveIt 2.
- Validate whether the model is suitable for controller-side dynamics or requires a separate controller model.

## Local Agent Discovery Space

Agents should inspect upstream Nero packages for:

- multiple Nero variants,
- end-effector options,
- existing MoveIt 2 configs,
- Isaac Sim USD files,
- launch files,
- example worlds,
- ros2_control configs,
- hardware interface examples,
- assumptions about fixed base or mount orientation.

Agents may discover that upstream packages already provide parts of later sprints. If so, the sprint output should mark those parts as `CONFIRMED` rather than rebuilding them.

## Required Artifacts

```text
docs/assets/nero_asset_validation.md
docs/planning/nero_model_for_moveit2.md
docs/control/nero_model_for_mit_controller.md
docs/simulation/nero_usd_asset_validation.md
```

## Expected Outputs

- Validated `nero_standalone` model variant.
- Confirmed link/joint naming map.
- Confirmed `nero_tool0` frame.
- Known limitations of upstream model assets.
- Recommendation for model source of truth.

## Decision Gate

Proceed once `nero_standalone` can be loaded and inspected in at least:

- ROS 2 model visualization,
- MoveIt 2 model loading path,
- Pinocchio or equivalent FK validation path,
- Isaac Sim asset validation path if USD is available.

---

# Sprint 4: MoveIt 2 and TRAC-IK Arm Baseline

## Objective

Establish MoveIt 2 with TRAC-IK as the primary early-stage arm-level IK and motion planning baseline.

In the merged local repo sequence, this section is not a separate repo sprint. It is part of the same local Sprint 3 planning/control hardening package as Sprint 3 and Sprint 5.

## Fixed Scope

- Create or validate `nero_moveit_config`.
- Define planning group `nero_arm`.
- Generate or validate SRDF.
- Generate or validate the self-collision matrix.
- Configure TRAC-IK as the primary IK solver for `nero_arm`.
- Configure OMPL as the initial global planner through MoveIt 2.
- Validate pose-to-pose planning in RViz.
- Export planned trajectories as `trajectory_msgs/JointTrajectory`.

## Local Agent Discovery Space

Agents should inspect whether upstream already provides:

- a Nero MoveIt 2 config,
- TRAC-IK configuration,
- OMPL planner configs,
- ros2_control configs,
- RViz configs,
- launch files,
- hardware execution examples.

Agents may generate missing configuration files but must document generated assumptions.

## Required Artifacts

```text
docs/planning/moveit2_setup.md
docs/planning/trac_ik_configuration.md
docs/planning/ompl_baseline_configuration.md
docs/planning/self_collision_matrix_initial.md
```

## Expected Outputs

- Functional `nero_moveit_config`.
- TRAC-IK solving for `nero_arm`.
- OMPL-based pose-to-pose plans.
- Joint trajectory output path.
- Known IK failure cases.
- Known planning failure cases.

## Decision Gate

Proceed once the system can plan to a set of representative target poses without manual frame guessing.

Representative targets should include:

```text
home_pose
front_workspace_pose
left_workspace_pose
right_workspace_pose
high_retract_pose
low_pregrasp_pose
```

---

# Sprint 5: MIT Controller Trajectory Bridge

## Objective

Connect the planning baseline to the existing MIT controller while preserving model-based control assumptions and execution safety.

In the merged local repo sequence, this section closes local Sprint 3 rather than standing alone as a separate early sprint.

## Fixed Scope

- Define the `JointTrajectoryCommand` interface.
- Convert MoveIt 2 trajectories into MIT-controller-compatible commands.
- Validate joint ordering and units.
- Validate position, velocity, acceleration, and timing semantics.
- Define trajectory acceptance criteria.
- Define low-speed execution mode.
- Define emergency stop and abort behavior.
- Log desired versus actual trajectory execution.

## Local Agent Discovery Space

Agents should inspect the MIT-controller code and configs for:

- expected command format,
- accepted trajectory timing,
- joint naming or joint ordering assumptions,
- gravity model source,
- payload model support,
- safety limits,
- controller modes,
- existing ROS 2 bridge code,
- logging support.

Agents may propose wrapper nodes but should not modify the low-level controller semantics without review.

## Required Artifacts

```text
docs/control/mit_controller_interface.md
docs/control/joint_trajectory_bridge.md
docs/control/trajectory_execution_policy.md
docs/control/execution_safety_gate.md
```

## Expected Outputs

- `nero_control_bridge` package or equivalent node.
- Confirmed joint name and ordering map.
- Confirmed trajectory command format.
- Low-speed execution path.
- Execution logging path.
- Safety gating policy.

## Decision Gate

Proceed once a MoveIt-generated trajectory can be executed in a controlled mode and the logged actual trajectory can be compared against the desired trajectory.

---

# Sprint 6: Pinocchio and Ruckig Debug Layer

## Objective

Add a lightweight model-audit and fallback trajectory layer that complements MoveIt 2 without becoming the primary planning stack.

In the merged local repo sequence, this section is still part of local Sprint 3 rather than a separate early sprint.

## Fixed Scope

- Load the Nero URDF into Pinocchio.
- Validate FK against MoveIt 2 and robot visualization.
- Validate Jacobian conventions.
- Validate gravity direction and mount-frame assumptions.
- Create simple DLS-IK test scripts for debugging.
- Integrate Ruckig for simple joint-space trajectory generation and smoothing tests.
- Compare Pinocchio/Ruckig outputs against MoveIt-generated trajectories.

## Local Agent Discovery Space

Agents may inspect:

- whether controller code already contains dynamics or kinematics utilities,
- whether existing scripts already compute FK/IK,
- whether Ruckig or equivalent smoothing is already used,
- whether controller logs expose enough data for FK consistency checks.

## Required Artifacts

```text
docs/control/pinocchio_model_audit.md
docs/control/ruckig_trajectory_debug.md
docs/control/gravity_and_payload_debug.md
```

## Expected Outputs

- FK validation scripts.
- Gravity-frame sanity checks.
- Simple fallback joint trajectory generation.
- Comparison notes between MoveIt and Pinocchio outputs.

## Decision Gate

Proceed once Pinocchio agrees with the MoveIt/URDF model within defined tolerances for representative joint states.

---

# Sprint 7: OmniHand Pro Standalone Bring-Up

## Objective

Bring up the OmniHand Pro independently from the arm and define its control, sensing, and model integration interfaces.

## Fixed Scope

- Install and validate the OmniHand Pro SDK.
- Validate communication adapter and transport layer.
- Define motor-index-to-joint-name mapping.
- Define basic open, close, and preshape commands.
- Expose hand control through ROS 2.
- Expose tactile and status data through ROS 2.
- Validate command rate, latency, and safety limits.
- Define hand model levels.

## Hand Model Levels

```text
omnihand_payload_only
omnihand_simplified_collision
omnihand_articulated_urdf
omnihand_dexterous_research_model
```

## Local Agent Discovery Space

Agents should inspect the OmniHand SDK for:

- motor IDs,
- joint names,
- control modes,
- tactile data format,
- examples,
- Python/C++ APIs,
- ROS 2 wrappers,
- kinematics APIs,
- URDF/CAD/USD/MJCF availability,
- vendor documentation links,
- calibration procedures.

Agents should explicitly mark vendor model assets as `CONFIRMED` or `MISSING`.

## Required Artifacts

```text
docs/hand/omnihand_sdk_integration.md
docs/hand/omnihand_joint_mapping.md
docs/hand/omnihand_tactile_logging.md
docs/hand/omnihand_preshape_library.md
docs/assets/omnihand_asset_validation.md
```

## Expected Outputs

- `omnihand_driver_ros2` package or equivalent node.
- `omnihand_joint_mapping.yaml`.
- `omnihand_preshape_library.yaml`.
- `omnihand_payload.yaml`.
- Basic tactile logging path.
- Hand safety limits.

## Decision Gate

Proceed once the hand can execute basic preshapes and produce synchronized tactile/status logs without the arm.

---

# Sprint 8: OmniHand as Nero Tool and Payload

## Objective

Integrate the OmniHand Pro into the Nero model as a physically meaningful tool and payload without requiring full finger-level dexterous planning.

## Fixed Scope

- Add `wrist_adapter_link`.
- Add `omnihand_palm_link`.
- Add `grasp_frame`.
- Represent the hand as a static payload attached to `nero_tool0`.
- Approximate mass, center of mass, and inertia.
- Update MIT-controller payload configuration.
- Add simplified hand collision geometry to MoveIt 2.
- Validate reachability with the attached hand envelope.

## Local Agent Discovery Space

Agents may inspect:

- actual wrist adapter CAD,
- hand mounting hole pattern,
- vendor-provided hand mass/inertia,
- measured hand mass and center of mass,
- existing tool definitions in Nero packages,
- existing end-effector examples in upstream Nero repositories,
- available hand mesh simplification scripts.

Agents may generate provisional payload values, but these must be marked as provisional until measured or confirmed.

## Required Artifacts

```text
docs/control/payload_and_gravity_model.md
docs/assets/wrist_adapter_modeling.md
docs/planning/hand_collision_envelope.md
docs/planning/nero_with_static_hand_moveit.md
```

## Expected Outputs

- `nero_with_omnihand_static` model variant.
- Updated MoveIt 2 configuration for attached hand geometry.
- Updated MIT payload configuration.
- Validated `grasp_frame` transform.
- Reachability and collision notes.

## Decision Gate

Proceed once the arm can plan with the static hand envelope and the controller can execute trajectories with the corresponding payload model.

---

# Sprint 9: Static AGV/Base Model Integration

## Objective

Integrate the AGV/base CAD into the planning and simulation model as a static mounting and collision environment.

## Fixed Scope

- Import AGV/base CAD as visual geometry.
- Create simplified collision primitives for planning.
- Define `agv_base_link`.
- Define `arm_mount_link`.
- Validate the mount transform from AGV/base to Nero base.
- Include the AGV/base as a static collision environment in MoveIt 2.
- Create `nero_on_static_agv` and `nero_omnihand_on_static_agv` model variants.
- Confirm gravity compensation uses the correct arm mounting orientation.

## Local Agent Discovery Space

Agents may inspect:

- CAD assembly hierarchy,
- existing mounting references,
- STEP/STL export quality,
- mechanical drawings,
- collision simplification candidates,
- AGV coordinate conventions,
- whether AGV is only a static support or already has mobile-base software.

Agents may propose multiple collision simplification levels.

## Collision Simplification Levels

```text
agv_collision_minimal: chassis box only
agv_collision_standard: chassis, top plate, wheel zones, forbidden rear volume
agv_collision_detailed: selected mechanical volumes relevant for close manipulation
```

## Required Artifacts

```text
docs/assets/agv_cad_simplification.md
docs/assets/agv_collision_model.md
docs/control/arm_mounting_pose.md
docs/planning/static_base_collision_environment.md
```

## Expected Outputs

- Simplified AGV/base collision model.
- Static mount transform definition.
- Combined Nero + OmniHand + static AGV/base model.
- MoveIt 2 collision environment including the base.
- Mounting-pose validation report.

## Decision Gate

Proceed once common arm motions are checked against the static AGV/base and the mounting orientation is accepted by the controller model.

---

# Sprint 10: Combined Collision and Planning Validation

## Objective

Validate collision-aware planning for arm-only, arm-with-hand, arm-on-base, and arm-hand-base model variants.

## Fixed Scope

- Generate and validate the Allowed Collision Matrix.
- Test self-collision behavior for each canonical planning variant.
- Use MoveIt 2 PlanningScene as the primary collision-checking baseline.
- Test OMPL plans for common manipulation targets.
- Add collision rejection before controller-bound trajectory execution.
- Compare behavior across model variants.
- Define safe trajectory acceptance criteria.

## Local Agent Discovery Space

Agents may inspect:

- existing SRDF collision matrices,
- upstream disabled collision pairs,
- known problematic link pairs,
- mesh quality issues,
- overly conservative or overly permissive collision geometry,
- PlanningScene performance bottlenecks,
- false-positive collisions caused by tool mounting geometry.

Agents may recommend mesh simplification or collision-pair filtering, but every exception must be documented.

## Required Artifacts

```text
docs/planning/collision_matrix_validation.md
docs/planning/planning_scene_validation.md
docs/planning/planning_failure_taxonomy.md
docs/control/collision_aware_execution_policy.md
```

## Expected Outputs

- Validated self-collision matrix.
- Collision-aware planning test suite.
- Representative success and failure cases.
- Safe trajectory acceptance criteria.
- Collision-pair exception list.

## Decision Gate

Proceed once planned trajectories are checked against the correct model variant before execution and collision failures are categorized rather than ignored.

---

# Sprint 11: Isaac Sim Asset Integration

## Objective

Bring validated model variants into Isaac Sim as simulation-ready USD assets.

## Fixed Scope

- Import or validate Nero USD assets.
- Convert or assemble combined robot variants as USD assets.
- Validate articulation structure.
- Validate joint limits and joint drives.
- Validate collision shapes.
- Validate static hand payload geometry.
- Add AGV/base visual and collision assets.
- Align Isaac Sim frames with ROS 2 and MoveIt 2 frames.
- Validate simple command execution in Isaac Sim.

## Local Agent Discovery Space

Agents should inspect:

- upstream Isaac Sim examples,
- existing Nero USD assets,
- existing Isaac scenes,
- ROS 2 bridge examples,
- articulation conversion scripts,
- import warnings,
- drive parameters,
- collision approximation quality,
- frame naming mismatches.

Agents may propose either direct USD reuse or URDF-to-USD conversion depending on available upstream quality.

## Required Artifacts

```text
docs/simulation/isaac_sim_asset_import.md
docs/simulation/usd_articulation_validation.md
docs/simulation/simulation_frame_alignment.md
docs/simulation/isaac_scene_templates.md
```

## Expected Outputs

```text
nero_standalone.usd
nero_with_omnihand_static.usd
nero_omnihand_on_static_agv.usd
```

Additional outputs:

- Isaac Sim scene template.
- Validated articulation report.
- Known simulation discrepancies.
- ROS 2 bridge configuration draft.

## Decision Gate

Proceed once Isaac Sim can load and move the robot variant without articulation errors, frame ambiguity, or obvious collider explosions.

---

# Sprint 12: Isaac Sim ROS 2 Bridge and HIL Replay

## Objective

Create a shared replay and validation workflow between real hardware and simulation.

## Fixed Scope

- Standardize command and state topics for real and simulated Nero.
- Standardize logging for arm, hand, controller, tactile, sensor, and transform streams.
- Replay real trajectories in Isaac Sim.
- Replay simulated trajectories through the MIT-controller interface in controlled mode.
- Compare target and actual trajectories.
- Detect model mismatch in gravity, payload, and mounting configuration.
- Establish safety gates before real execution.

## Local Agent Discovery Space

Agents may inspect:

- existing ROS 2 topic names in upstream examples,
- Isaac ROS bridge topic conventions,
- controller logging format,
- bag recording utilities,
- synchronization issues,
- timestamp sources,
- simulation time versus real time assumptions.

## Required Artifacts

```text
docs/data/logging_schema.md
docs/simulation/ros2_bridge_simulation.md
docs/simulation/hil_replay.md
docs/control/sim_real_trajectory_validation.md
docs/control/safety_gate.md
```

## Expected Outputs

- Unified log format.
- HIL replay toolchain.
- Real-to-sim replay example.
- Sim-to-controller execution test path.
- Sim-real trajectory comparison report.
- Hardware execution safety checklist.

## Decision Gate

Proceed once at least one representative real trajectory can be replayed in simulation and compared against controller logs.

---

# Sprint 13: Perception and Calibration Baseline

## Objective

Establish a repeatable perception and calibration setup for grasping, skill execution, and future policy observations.

## Fixed Scope

- Select initial fixed RGB-D camera setup.
- Define camera frames.
- Calibrate camera-to-robot transform.
- Define calibration target workflow.
- Validate depth quality in the manipulation workspace.
- Define lighting assumptions.
- Define optional wrist camera interface but do not require it initially.

## Local Agent Discovery Space

Agents may inspect:

- available cameras,
- supported ROS 2 drivers,
- Isaac ROS compatibility,
- USB bandwidth limits,
- camera mounting options,
- calibration packages already used locally,
- whether existing AGV CAD includes camera mounts,
- whether wrist camera payload is compatible with the hand and adapter.

## Required Artifacts

```text
docs/hardware/sensor_setup.md
docs/hardware/camera_calibration.md
docs/hardware/workspace_lighting.md
docs/config/sensors/initial_camera_config.md
```

## Expected Outputs

- Fixed camera frame definition.
- Camera calibration procedure.
- Validated camera-to-robot transform.
- Depth quality notes.
- Initial sensor configuration.

## Decision Gate

Proceed once object locations in camera space can be transformed into the robot planning frame with acceptable repeatability for pregrasp planning.

---

# Sprint 14: Grasping MVP

## Objective

Establish a simple but measurable grasping baseline before introducing SOTA dexterous grasping models.

## Fixed Scope

- Define a minimal object set for grasping tests.
- Use calibrated workspace perception.
- Use scripted or geometry-based pregrasp generation.
- Use predefined OmniHand preshapes.
- Execute approach with MoveIt 2 / MIT controller.
- Close the hand using tactile, position, current, or torque thresholds.
- Perform lift tests.
- Log success, slip, contact quality, and failure modes.

## Local Agent Discovery Space

Agents may inspect:

- available object sets,
- perception algorithms already available locally,
- hand tactile thresholds,
- suitable preshapes from SDK examples,
- useful vendor demos,
- grasp examples from related repositories,
- whether object pose estimation can remain simple or needs a stronger model.

## Required Artifacts

```text
docs/hand/omnihand_preshape_library.md
docs/hand/tactile_closure_policy.md
docs/learning/grasp_benchmark_protocol.md
docs/data/grasp_dataset_schema.md
```

## Expected Outputs

- Grasp benchmark protocol.
- Preshape-based grasping baseline.
- Grasp success/failure dataset.
- Initial tactile/contact logging pipeline.
- Failure taxonomy.

## Decision Gate

Proceed once a repeatable grasp-lift-test loop exists and failures are logged in a structured way.

---

# Sprint 15: Deterministic Skill Library

## Objective

Create a deterministic skill library that serves as baseline behavior, data generator, fallback layer, and interface target for learned policies.

## Fixed Scope

- Implement deterministic versions of common manipulation skills.
- Define preconditions, postconditions, termination criteria, and fallback behavior.
- Connect skills to MoveIt 2, the MIT controller, and OmniHand commands.
- Support object-centric and frame-relative parameterization.
- Log skill inputs, outputs, and outcomes.

## Initial Skills

```text
MoveToPose
MoveToPregrasp
OpenHand
CloseHandUntilContact
Grasp
Lift
Place
Retract
PourFixedTrajectory
Recover
```

## Local Agent Discovery Space

Agents may inspect:

- existing scripts for robot movements,
- existing teleoperation tools,
- MoveIt task-level examples,
- controller recovery behaviors,
- hand examples that can be wrapped as skills,
- existing skill definitions from related projects.

Agents may propose additional skills but should not bypass the canonical `SkillCommand` interface.

## Required Artifacts

```text
docs/skills/skill_schema.md
docs/skills/scripted_skill_library.md
docs/skills/skill_validation_protocol.md
docs/skills/skill_failure_recovery.md
```

## Expected Outputs

- `nero_skill_library` package.
- Skill command schema.
- Scripted manipulation demos.
- Skill-level logging.
- Skill-level failure handling.

## Decision Gate

Proceed once deterministic skills can generate useful demonstrations and provide fallback behavior for later learned policies.

---

# Sprint 16: Demonstration and Dataset Pipeline

## Objective

Build a clean data pipeline for imitation learning, diffusion-based skills, RL initialization, and later Physical AI data workflows.

## Fixed Scope

- Record synchronized demonstrations from scripted, teleoperated, simulated, and real executions.
- Store observations, actions, states, tactile data, camera data, transforms, and skill metadata.
- Segment demonstrations by skill.
- Label success, failure, recovery, contact events, and object/task parameters.
- Support replay into simulation.
- Version datasets.

## Local Agent Discovery Space

Agents may inspect:

- available ROS bag formats,
- existing logging code,
- controller log exports,
- tactile log formats,
- camera synchronization options,
- dataset formats used by candidate learning frameworks,
- simulation replay compatibility.

Agents may propose dataset schemas, but every schema must preserve mapping back to the canonical skill, frame, and model variant names.

## Required Artifacts

```text
docs/data/demonstration_dataset.md
docs/data/logging_schema.md
docs/data/skill_segmentation.md
docs/data/dataset_versioning.md
docs/data/replay_compatibility.md
```

## Expected Outputs

- Demonstration dataset schema.
- Skill-segmented recording format.
- Data validation tools.
- Replay-compatible dataset samples.
- Versioning convention.

## Decision Gate

Proceed once demonstrations can be recorded, segmented, validated, replayed, and used by at least one downstream training or evaluation script.

---

# Sprint 17: Dexterous Grasping Model Evaluation

## Objective

Evaluate SOTA-inspired dexterous grasping models and determine which are suitable for the OmniHand Pro setup.

## Fixed Scope

- Identify candidate grasp-generation model families.
- Evaluate compatibility with OmniHand geometry and control.
- Map generated grasps to the OmniHand kinematic structure.
- Filter generated grasps through IK and collision checks.
- Integrate candidate grasps into the existing pregrasp and hand-closure pipeline.
- Compare against the scripted/preshape MVP grasping baseline.

## Candidate Model Families

```text
DexGraspNet-style dexterous grasp generation
AnyDexGrasp-style cross-hand grasp adaptation
VLA-guided dexterous grasping approaches
Task-conditioned or affordance-conditioned grasp selection
```

## Local Agent Discovery Space

Agents may inspect:

- model code availability,
- model licenses,
- input requirements,
- required hand model format,
- required object representation,
- simulator dependencies,
- GPU requirements,
- inference latency,
- compatibility with OmniHand joint structure,
- whether generated grasps are directly executable or need retargeting.

Agents should explicitly distinguish between reproducible candidates and literature-only candidates.

## Required Artifacts

```text
docs/learning/dexterous_grasp_model_survey.md
docs/learning/omnihand_grasp_mapping.md
docs/learning/grasp_candidate_filtering.md
docs/learning/dexterous_grasp_benchmark.md
```

## Expected Outputs

- Dexterous grasp model compatibility report.
- Grasp candidate filtering pipeline.
- Benchmark comparison against MVP baseline.
- Recommendation for continued dexterous grasping development.

## Decision Gate

Proceed once at least one candidate model can produce grasp proposals that can be filtered, mapped, and tested against the existing grasp baseline.

---

# Sprint 18: Isaac Lab Task Development

## Objective

Create Isaac Lab environments for controlled learning experiments using validated Nero model variants.

## Fixed Scope

- Start with state-based tasks.
- Use `nero_standalone` first.
- Introduce `nero_with_omnihand_static` only after simulation validation.
- Introduce `nero_omnihand_on_static_agv` only after collision validation.
- Keep action spaces compatible with MIT-controller execution assumptions.
- Add domain randomization gradually.
- Evaluate policies in simulation before hardware transfer.

## Initial Task Families

```text
NeroReach-v0
NeroPregrasp-v0
NeroGraspHold-v0
NeroLift-v0
NeroPlace-v0
NeroPour-v0
```

## Local Agent Discovery Space

Agents may inspect:

- existing Isaac Lab task templates,
- upstream Nero USD usability,
- action manager patterns,
- observation manager patterns,
- reward design examples,
- RL library compatibility,
- batch simulation performance,
- model variant loading performance.

Agents may propose task-specific environment names, but canonical task family names should remain stable.

## Required Artifacts

```text
docs/learning/isaac_lab_environment_design.md
docs/learning/observation_action_spaces.md
docs/learning/reward_and_termination_design.md
docs/learning/domain_randomization.md
docs/learning/sim_to_real_readiness.md
```

## Expected Outputs

- `nero_isaac_lab` task package.
- Baseline state-based RL environments.
- Baseline training configuration.
- Simulation-only evaluation protocol.
- Sim-to-real readiness checklist.

## Decision Gate

Proceed once at least one simple task can be trained and evaluated consistently in simulation using the validated model assets.

---

# Sprint 19: Diffusion-Based Skill Policies

## Objective

Evaluate diffusion-based policies for adapting recorded skills to varying object poses, start states, and task parameters.

## Fixed Scope

- Start with state-based skill policies.
- Use demonstration data from deterministic and teleoperated skills.
- Train skill-specific policies rather than one monolithic policy.
- Use receding-horizon execution.
- Restrict policy outputs through safety gates.
- Compare learned policies against scripted skills.

## Candidate Skills

```text
GraspAdaptive
PlaceAdaptive
PourAdaptive
RecoverAdaptive
```

## Local Agent Discovery Space

Agents may inspect:

- available diffusion policy implementations,
- dataset format requirements,
- action horizon conventions,
- observation encoding options,
- simulation rollout wrappers,
- inference latency on AGX Orin and future Thor-class hardware,
- whether policies should output joint deltas, Cartesian deltas, or skill parameters.

Agents should keep policy outputs bounded and compatible with the controller interface.

## Required Artifacts

```text
docs/skills/diffusion_skill_policy.md
docs/learning/diffusion_policy_training.md
docs/learning/skill_policy_evaluation.md
docs/control/learned_policy_safety_wrapper.md
docs/control/real_hardware_rollout_protocol.md
```

## Expected Outputs

- Diffusion skill training pipeline.
- Skill-specific policy checkpoints.
- Offline evaluation reports.
- Simulation evaluation reports.
- Real-hardware trial protocol.

## Decision Gate

Proceed once a learned skill improves or meaningfully generalizes beyond the scripted baseline without violating safety constraints.

---

# Sprint 20: GPU-Accelerated Manipulation Evaluation

## Objective

Evaluate when and how Isaac ROS Manipulation, cuMotion, and related GPU-accelerated components should replace or augment the MoveIt 2 baseline.

## Fixed Scope

- Integrate cuMotion with simplified robot variants.
- Evaluate planning speed and robustness for arm-hand-base collision scenarios.
- Evaluate depth-based collision-world generation where appropriate.
- Evaluate Nvblox for environment reconstruction.
- Evaluate FoundationPose or equivalent object pose estimation for manipulation.
- Compare GPU-accelerated planning against MoveIt 2 baseline.

## Local Agent Discovery Space

Agents may inspect:

- cuMotion robot model requirements,
- Isaac ROS examples,
- supported sensor drivers,
- expected compute platform,
- NITROS data paths,
- ROS 2 topic compatibility,
- GPU memory requirements,
- whether existing model variants need conversion.

Agents should document whether GPU acceleration solves an actual bottleneck or merely adds a more expensive way to be confused.

## Required Artifacts

```text
docs/learning/cumotion_integration.md
docs/learning/nvblox_collision_world.md
docs/learning/foundationpose_integration.md
docs/planning/planning_benchmark.md
docs/planning/planning_stack_decision_matrix.md
```

## Expected Outputs

- cuMotion integration prototype.
- GPU planning comparison report.
- Sensor-aware planning prototype if appropriate.
- Recommendation for production or research usage.
- Updated planning-stack decision matrix.

## Decision Gate

Proceed once GPU-accelerated planning shows a concrete advantage over the MoveIt 2 baseline for the relevant use cases.

---

# Sprint 21: VLA and Physical AI Policy Evaluation

## Objective

Evaluate vision-language-action and generalist policy models as high-level or skill-level controllers, not as direct raw low-level controllers.

## Fixed Scope

- Define embodiment mapping for Nero + OmniHand Pro.
- Evaluate candidate VLA/VLM policy families offline first.
- Use simulation rollouts before hardware trials.
- Restrict initial outputs to skill selection, target pose generation, grasp proposal generation, or bounded action deltas.
- Measure inference latency and memory footprint on available edge hardware.
- Compare VLA-guided behaviors against deterministic and diffusion-skill baselines.

## Candidate Policy Families

```text
GR00T-style VLA models
OpenVLA-style policies
π0-style flow-matching policies
Task-conditioned visuomotor policies
```

## Local Agent Discovery Space

Agents may inspect:

- model licenses,
- code availability,
- fine-tuning requirements,
- input modality requirements,
- action representation,
- dataset format,
- compute requirements,
- quantization support,
- edge deployment paths,
- whether the policy can operate at skill-level rather than raw action level.

Agents should distinguish between:

```text
offline-evaluable
simulation-rollout-ready
hardware-rollout-ready
research-only
not-compatible
```

## Required Artifacts

```text
docs/learning/vla_model_survey.md
docs/learning/embodiment_mapping.md
docs/learning/vla_evaluation_protocol.md
docs/learning/edge_deployment.md
docs/control/vla_safety_boundary.md
```

## Expected Outputs

- VLA compatibility report.
- Embodiment/action mapping proposal.
- Simulation evaluation protocol.
- Edge-inference feasibility report.
- Recommendation for follow-up VLA work.

## Decision Gate

Proceed once a VLA-style candidate can be evaluated safely at skill-level or bounded-action level without bypassing the existing safety and planning stack.

---

# Sprint 22: Cosmos and Synthetic Data Workflow

## Objective

Integrate Physical AI data workflows for synthetic data generation, visual variation, simulation replay, and policy evaluation.

## Fixed Scope

- Export simulation data in a format compatible with downstream synthetic-data workflows.
- Use Isaac Sim scenes and recorded demonstrations as structured inputs.
- Evaluate Cosmos-style workflows for visual variation, data augmentation, and world-model-assisted policy evaluation.
- Keep real demonstrations as the anchor dataset.
- Feed failure cases back into the data and training loop.
- Define dataset versioning and provenance.

## Local Agent Discovery Space

Agents may inspect:

- available Isaac Sim data export writers,
- synthetic-data generation examples,
- compatibility with Cosmos-related tooling,
- storage requirements,
- dataset versioning tools,
- how generated data maps back to real demonstrations,
- whether synthetic variations preserve task semantics.

Agents should avoid treating synthetic data as ground truth unless validated against real executions.

## Required Artifacts

```text
docs/data/synthetic_data_workflow.md
docs/data/cosmos_workflow.md
docs/data/simulation_data_export.md
docs/data/dataset_versioning.md
docs/data/failure_case_feedback_loop.md
```

## Expected Outputs

- Synthetic data workflow prototype.
- Simulation-to-augmentation data path.
- Dataset versioning convention.
- Failure-case feedback loop design.
- Recommendation for continued Physical AI data workflows.

## Decision Gate

Proceed once synthetic or augmented data demonstrably improves evaluation, robustness, or training efficiency without breaking the semantics of the original real tasks.

---

# Sprint 23: Mobile AGV Extension

## Objective

Extend the static AGV/base model toward mobile manipulation only after arm, hand, static base, perception, and baseline skills are stable.

## Fixed Scope

- Introduce `odom` semantics.
- Define mobile-base command and state interfaces.
- Define AGV collision model for moving scenes.
- Validate base localization assumptions.
- Define whole-body planning boundaries.
- Keep arm manipulation tasks valid with a static base before enabling mobile manipulation.

## Local Agent Discovery Space

Agents may inspect:

- existing AGV software,
- localization stack,
- odometry source,
- navigation stack,
- base controller interface,
- CAD-to-base-link alignment,
- base safety constraints,
- whether the AGV should be modeled dynamically or kinematically first.

## Required Artifacts

```text
docs/agv/mobile_base_interface.md
docs/agv/agv_localization_assumptions.md
docs/agv/mobile_manipulation_scope.md
docs/planning/whole_body_planning_boundary.md
```

## Expected Outputs

- `nero_omnihand_on_mobile_agv` model variant draft.
- Mobile-base frame and interface definition.
- Mobile-manipulation scope definition.
- Decision on whether whole-body planning is required or deferred.

## Decision Gate

Proceed once static-base manipulation is stable and mobile-base integration does not invalidate existing arm/hand safety assumptions.

---

## 7. Hardware Procurement Planning

Hardware procurement should be tied to functional gates rather than enthusiasm. Enthusiasm is useful, but it has historically purchased many very shiny paperweights.

### 7.1 Minimum Functional Hardware

Required for early integration:

```text
Nero arm
OmniHand Pro
Jetson AGX Orin
MIT controller hardware
Arm communication adapter
OmniHand-compatible CAN-FD or vendor communication adapter
Stable power supplies for arm and hand
Hardware emergency stop
Local NVMe storage for logs
One calibrated RGB-D camera
Calibration target
Controlled lighting
```

### 7.2 Recommended Early Perception Hardware

```text
1x fixed RGB-D camera observing the manipulation workspace
optional 1x second fixed RGB-D camera
calibration target: ChArUco, AprilTag grid, or equivalent
diffuse lighting panels
rigid camera mount
```

The wrist camera should be deferred until payload, cable routing, field of view, and hand mounting constraints are validated.

### 7.3 Optional Contact-Rich Manipulation Hardware

```text
6-axis wrist force/torque sensor
mechanical adapter stack for F/T sensor + OmniHand
force/torque ROS 2 driver
synchronized logging support
```

This becomes more important for pouring, insertion, contact-rich manipulation, and force-aware recovery.

### 7.4 Compute Procurement

| Need | Hardware Direction |
|---|---|
| Real-time ROS 2 integration | Existing AGX Orin |
| Heavy Isaac Sim / Isaac Lab work | RTX workstation or remote RTX GPU system |
| Larger edge inference | Jetson Thor-class platform |
| Multi-camera and GPU manipulation | Thor-class platform or dedicated workstation depending on deployment mode |
| VLA/VLM fine-tuning | Remote or workstation GPU environment, not AGX Orin |

### 7.5 Hardware Decision Gates

| Gate | Procurement Trigger |
|---|---|
| Perception baseline insufficient | Add second fixed RGB-D camera or improve lighting/mounting |
| Workspace occlusion limits grasping | Add second camera or wrist camera |
| Tactile-only contact estimation insufficient | Add wrist force/torque sensor |
| Isaac Sim/Lab iteration too slow | Procure or allocate RTX workstation/remote GPU |
| Edge inference too slow on Orin | Evaluate Thor-class platform |
| Sensor bandwidth unstable | Move from USB-heavy setup to CSI/GMSL or more robust camera architecture |

---

## 8. Documentation Tree

Recommended documentation structure:

```text
docs/
  roadmap/
    nero_physical_ai_roadmap.md
    nero_physical_ai_roadmap_sprints_expanded.md
  project/
    repository_structure.md
    package_naming.md
    generated_vs_source_assets.md
    local_agent_workflow.md
  assets/
    repository_asset_inventory.md
    nero_asset_validation.md
    omnihand_asset_validation.md
    agv_cad_inventory.md
    agv_cad_simplification.md
    agv_collision_model.md
    wrist_adapter_modeling.md
  planning/
    moveit2_setup.md
    trac_ik_configuration.md
    ompl_baseline_configuration.md
    self_collision_matrix_initial.md
    collision_matrix_validation.md
    planning_scene_validation.md
    planning_failure_taxonomy.md
    planning_benchmark.md
    planning_stack_decision_matrix.md
    whole_body_planning_boundary.md
  control/
    mit_controller_interface.md
    joint_trajectory_bridge.md
    trajectory_execution_policy.md
    execution_safety_gate.md
    payload_and_gravity_model.md
    pinocchio_model_audit.md
    ruckig_trajectory_debug.md
    gravity_and_payload_debug.md
    collision_aware_execution_policy.md
    learned_policy_safety_wrapper.md
    real_hardware_rollout_protocol.md
    vla_safety_boundary.md
  simulation/
    isaac_sim_asset_import.md
    usd_articulation_validation.md
    simulation_frame_alignment.md
    isaac_scene_templates.md
    ros2_bridge_simulation.md
    hil_replay.md
  hand/
    omnihand_sdk_integration.md
    omnihand_joint_mapping.md
    omnihand_tactile_logging.md
    omnihand_preshape_library.md
    tactile_closure_policy.md
  skills/
    skill_schema.md
    scripted_skill_library.md
    skill_validation_protocol.md
    skill_failure_recovery.md
    diffusion_skill_policy.md
  learning/
    grasp_benchmark_protocol.md
    dexterous_grasp_model_survey.md
    omnihand_grasp_mapping.md
    grasp_candidate_filtering.md
    dexterous_grasp_benchmark.md
    isaac_lab_environment_design.md
    observation_action_spaces.md
    reward_and_termination_design.md
    domain_randomization.md
    sim_to_real_readiness.md
    diffusion_policy_training.md
    skill_policy_evaluation.md
    cumotion_integration.md
    nvblox_collision_world.md
    foundationpose_integration.md
    vla_model_survey.md
    embodiment_mapping.md
    vla_evaluation_protocol.md
    edge_deployment.md
  data/
    logging_schema.md
    demonstration_dataset.md
    grasp_dataset_schema.md
    skill_segmentation.md
    replay_compatibility.md
    synthetic_data_workflow.md
    cosmos_workflow.md
    simulation_data_export.md
    dataset_versioning.md
    failure_case_feedback_loop.md
  hardware/
    hardware_procurement_plan.md
    sensor_setup.md
    camera_calibration.md
    workspace_lighting.md
  agv/
    mobile_base_interface.md
    agv_localization_assumptions.md
    mobile_manipulation_scope.md
```

---

## 9. Immediate Integration Priorities

The immediate priorities are:

1. Complete repository and asset discovery.
2. Validate the Nero standalone model.
3. Establish MoveIt 2 + TRAC-IK as the primary IK/planning baseline.
4. Connect planned trajectories to the MIT controller.
5. Bring up the OmniHand Pro independently.
6. Integrate the OmniHand first as static payload and collision envelope.
7. Integrate the AGV/base as static mount and collision environment.
8. Validate combined collision behavior before complex policies.
9. Build deterministic skills before adaptive learned skills.
10. Establish logging and replay before large-scale learning.
11. Introduce Isaac Lab, diffusion policies, GPU manipulation, VLA policies, and Cosmos workflows only after model, control, logging, and baseline skills are stable.

---

## 10. High-Level End State

The intended end state is a modular Physical AI workflow in which:

- Nero, OmniHand Pro, and AGV/base exist as validated model variants.
- MoveIt 2 provides a reliable classical planning baseline.
- The MIT controller receives safe, validated, model-aware trajectories and targets.
- Isaac Sim provides digital twin simulation and HIL replay.
- Isaac Lab provides controlled RL and policy-training environments.
- Deterministic skills provide fallback behavior and demonstration data.
- Dexterous grasping models can be evaluated against measurable baselines.
- Diffusion-based policies adapt demonstrated skills to varying task conditions.
- VLA/VLM policies operate at skill level or bounded-action level.
- Cosmos-style data workflows support simulation replay, data augmentation, and Physical AI evaluation.
- Edge hardware runs real-time integration, perception, policy inference, and safety-gated execution.

The system should remain modular enough that individual components can be replaced as better models, sensors, planners, policies, or compute platforms become available.
