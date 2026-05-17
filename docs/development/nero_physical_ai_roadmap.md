# Nero Physical AI Roadmap

## 1. Purpose

This document defines the high-level roadmap for evolving the currently functional MIT-controller-based Nero arm setup into a modular Physical AI research and development workflow.

The roadmap focuses on the integration of the Nero arm, the OmniHand Pro dexterous gripper, and a custom AGV/static base setup into a simulation, planning, control, and learning pipeline. It is intentionally structured as a semantic master plan. Detailed technical procedures, implementation notes, validation protocols, and experiments are expected to live in separate sub-phase documents.

The roadmap is dependency-based rather than calendar-based. Sprints describe logical work packages and integration maturity levels, not fixed durations.

Use this file as the canonical roadmap only.

Companion coordination docs:

- `docs/development/nero_physical_ai_progress.md`: current overall progress, active sprint focus, and cross-sprint blockers
- `docs/development/component_implementation_map.md`: system components, canonical code locations, and where sprint-level details live

---

## 2. System Context

The system consists of the following main components:

- **Manipulator:** AgileX Nero arm
- **End effector:** AgiBot OmniHand Pro dexterous gripper
- **Mobile/static base:** Custom AGV/base platform, with CAD/model assets currently assumed external and not yet imported into this workspace
- **Low-level controller:** MIT-style controller, currently functional for the Nero arm
- **Edge compute:** NVIDIA Jetson AGX Orin
- **Future edge compute option:** NVIDIA Jetson Thor-class platform
- **Simulation and training environment:** NVIDIA Isaac Sim and Isaac Lab
- **Primary ROS middleware:** ROS 2
- **Primary early-stage planning stack:** MoveIt 2, TRAC-IK, OMPL, PlanningScene
- **Secondary/debug kinematics and trajectory stack:** Pinocchio and Ruckig
- **Later GPU-accelerated manipulation stack:** Isaac ROS Manipulation, cuMotion, Nvblox, FoundationPose
- **Later Physical AI data and model stack:** NVIDIA Cosmos, GR00T-related workflows, VLA/VLM policy evaluation

The current control assumption is that high-level planners and policies provide joint trajectories, target poses, or structured skill commands. The MIT controller remains responsible for low-level model-based control, force/torque computation, and execution safety.

---

## 3. Roadmap Principles

### 3.1 Iterative Modeling

Robot modeling is performed incrementally. Arm, hand, and AGV/base assets are validated independently before being merged into combined model variants.

The model chain must support both:

- simulation and planning workflows, and
- controller-side dynamics, gravity compensation, payload handling, and mounting-pose awareness.

### 3.2 Simulation and Real Hardware Share Interfaces

The real robot and simulated robot should expose equivalent high-level interfaces wherever possible. A command or skill should be executable in simulation first, then on real hardware with minimal adaptation.

### 3.3 MoveIt 2 First, cuMotion Later

MoveIt 2 with TRAC-IK and OMPL is the primary early-stage IK and motion-planning baseline. It provides sufficient sophistication for planning groups, collision checking, self-collision matrices, tool frames, static environment collision objects, RViz debugging, and trajectory generation.

cuMotion and the broader Isaac ROS Manipulation stack are introduced once the robot model, collision geometry, sensor setup, and planning requirements are mature enough to benefit from GPU acceleration.

### 3.4 Hand Is Initially Treated as Tool and Payload

The OmniHand Pro is initially modeled as a tool/payload attached to the Nero flange. Full dexterous finger planning is treated as a separate track and should not block arm-level IK, motion planning, or simulation bring-up.

### 3.5 Skill-Level Abstraction

The long-term system should not expose raw model policies directly to hardware. Instead, policies should operate through skill-level abstractions such as `MoveToPregrasp`, `Grasp`, `Lift`, `Place`, `Pour`, and `Recover`.

### 3.6 Parallel Exploration Where Interfaces Are Clear

Grasping, dexterous manipulation, motion primitives, simulation, and learning can proceed in parallel only after shared interfaces and model assumptions are explicitly defined.

---

## 4. Fixed Technical Frames

### 4.1 Core Software Stack

The roadmap assumes the following core libraries and frameworks:

| Area | Primary Tooling | Role |
|---|---|---|
| Robot middleware | ROS 2 | Common communication layer for real and simulated systems |
| Robot description | URDF / Xacro / SRDF / USD | Robot modeling, planning, simulation, and controller-side model generation |
| Nero assets | `agx_arm_sim`, `agx_arm_description` | Source for Nero URDF/Xacro and Isaac-compatible assets |
| Motion planning baseline | MoveIt 2 | Planning groups, PlanningScene, collision checking, trajectory generation |
| IK baseline | TRAC-IK | Primary IK solver for the Nero arm planning group |
| Global planning baseline | OMPL | Sampling-based path planning through MoveIt 2 |
| Collision reasoning | MoveIt 2 PlanningScene | Self-collision and environment-collision checks |
| Kinematics/debug | Pinocchio | FK, Jacobians, dynamics/model audit, gravity/payload checks |
| Trajectory smoothing/debug | Ruckig | Jerk-limited trajectory generation and fallback smoothing |
| Simulation | Isaac Sim | USD-based robot simulation and sensor simulation |
| Robot learning | Isaac Lab | RL and policy training environments |
| GPU manipulation, later | Isaac ROS Manipulation, cuMotion | GPU-accelerated motion generation and sensor-aware planning |
| Dense environment modeling, later | Nvblox | 3D reconstruction and obstacle representation |
| Object pose estimation, later | FoundationPose | Object pose estimation for manipulation workflows |
| Physical AI data/modeling, later | NVIDIA Cosmos | Synthetic data, world-model-driven augmentation, Physical AI workflows |
| VLA/VLM policies, later | GR00T, OpenVLA, π0-style models | Language-conditioned and generalist visuomotor policy evaluation |
| OmniHand interface | AgiBot OmniHand Pro SDK / ROS 2 interface | Hand control, tactile data, kinematics API, low-level communication |

### 4.2 Compute Roles

| Platform | Intended Role |
|---|---|
| Jetson AGX Orin | Real-time edge integration, ROS 2 runtime, MIT-controller bridge, hand interface, sensor preprocessing, policy inference for compact models, logging, HIL node |
| RTX workstation or equivalent remote GPU system | Isaac Sim, Isaac Lab training, rendering, synthetic data generation, heavy RL training, VLA/VLM experimentation |
| Jetson Thor-class platform | Future edge platform for larger perception models, VLA/VLM inference, multi-camera processing, GPU-accelerated manipulation workflows, higher-throughput local policy evaluation |

The AGX Orin should not be treated as the primary system for heavy Isaac Sim rendering, large-scale Isaac Lab training, or foundation-model fine-tuning.

### 4.3 Robot Model Variants

The model architecture should support the following canonical variants:

```text
nero_standalone
nero_with_dummy_payload
nero_with_omnihand_static
nero_with_omnihand_articulated
nero_on_static_agv
nero_omnihand_on_static_agv
nero_omnihand_on_mobile_agv
```

Each variant should be generated from shared Xacro or equivalent model components. Manual duplication of complete URDF files should be avoided.

### 4.4 Canonical Frame Naming

The following frame semantics should be used consistently:

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
| `map` | Global/world reference frame |
| `odom` | Local odometry frame for future mobile-base operation |
| `agv_base_link` | Main AGV/base reference frame |
| `arm_mount_link` | Mechanical mounting interface between AGV/base and Nero arm |
| `nero_base_link` | Nero arm base frame |
| `nero_tool0` | Nero tool/flange frame |
| `wrist_adapter_link` | Mechanical adapter between Nero flange and OmniHand |
| `omnihand_palm_link` | Palm/base frame of the OmniHand Pro |
| `grasp_frame` | Task-level grasp reference frame used by planners and skills |

### 4.5 Planning Groups

Initial MoveIt 2 planning groups should be defined as follows:

```text
nero_arm
omnihand
nero_arm_with_static_hand
nero_arm_on_static_agv
nero_arm_with_omnihand_on_static_agv
```

Recommended early usage:

- `nero_arm`: primary arm IK and motion planning group
- `omnihand`: hand control and later finger-level experiments
- `nero_arm_with_static_hand`: arm planning with hand geometry treated as attached tool geometry
- `nero_arm_on_static_agv`: arm planning with the static AGV/base as collision environment
- `nero_arm_with_omnihand_on_static_agv`: combined collision-aware planning model for arm, hand envelope, and base

The full articulated hand should not initially be part of the arm IK chain.

### 4.6 Control Interfaces

The controller interface should support at least the following command types:

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
- target poses that can be resolved before execution,
- payload and tool configuration metadata,
- mounting-pose information relevant for gravity compensation.

The controller should not initially receive raw unfiltered policy actions directly from learned policies.

### 4.7 Skill Interface

A generic skill command should use a structure similar to:

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

## 5. Sprint Roadmap

### Current Local Sequencing Note

The current local repo execution order no longer matches the first-draft split of the early sprints.

- Sprint 1 is treated as complete for asset audit, source-of-truth ownership, and repo-baseline decisions, aside from hardware-gated checks and AGV assets that are still external to the workspace.
- Sprint 2 now focuses on the common environment merge: package structure baseline, OmniHand adapter boundary, normalized description assets, and the shared ROS/MoveIt semantics already being used locally.
- Sprint 3 now focuses on validating and hardening the existing Nero planning and control baseline, adding missing IK or planning pieces only where they are not already available.
- Sprint 4 now focuses on the first Nero plus OmniHand baseline on top of the shared adapter boundary and common ROS semantics from Sprint 2.

## Sprint 1: Asset Audit and Model Baseline

### Objective

Establish a reliable source of truth for Nero, OmniHand Pro, and AGV/base assets.

### Scope

- Validate existing Nero assets from AgileX-compatible sources.
- Identify available OmniHand Pro assets and SDK interfaces.
- Import or simplify AGV/base CAD assets.
- Define shared model-component ownership and naming.
- Create a model audit document for each physical component.

### Expected Outputs

- `nero_model_audit.md`
- `omnihand_model_audit.md`
- `agv_base_model_audit.md`
- Initial model-component repository structure
- Confirmed list of available and missing model assets

### Follow-up Documents

- Nero asset validation document
- OmniHand asset acquisition and validation document
- AGV CAD simplification document

---

## Sprint 2: Common Environment and Package Structure Merge

### Objective

Consolidate the common environment, package structure, and OmniHand integration contract that later Nero-only and Nero-plus-hand work will share.

### Scope

- Freeze the repo-owned public ROS contract for OmniHand around shared arguments, joint naming, namespaces, and frame semantics.
- Keep the OmniHand adapter below ROS and define the repo-owned bridge boundary above it.
- Reuse and document the canonical package surfaces already active in the workspace: description, MoveIt, MIT controller, control bridge, and vendored SDK.
- Normalize the OmniHand description assets and the landed MoveIt/mock-controller integration into the common workspace structure.
- Define package naming, configuration layout, generated-vs-source policy, and fork/submodule workflow for the shared environment.
- Add the remaining common-environment hooks still needed by later phases: repo-owned bridge skeleton, mock-backend hook, and OmniHand diagnostics/tactile message plan.
- Capture visual interaction docs for the ROS graph, launch flow, file composition, and config dataflow so the shared environment can be understood without rediscovery.
- Ensure one developer or agent can locate source models, generated assets, configs, logs, and ownership boundaries without rediscovery.

### Expected Outputs

- Documented common package structure and naming baseline
- Frozen OmniHand simulation/control contract for shared ROS semantics
- Normalized OmniHand description and MoveIt simulation slice under canonical packages
- Repo-owned OmniHand bridge/backend interface plan, plus first implementation skeleton where practical
- Documented generated-vs-source and fork/submodule policy
- Stable interaction diagrams for runtime nodes, launch flow, file composition, and config dataflow

### Follow-up Documents

- Repository structure document
- Package naming and generated-vs-source policy documents
- Repo interaction diagrams document
- OmniHand ROS integration contract document
- OmniHand wrapper and bridge implementation document

---

## Sprint 3: Nero Planning and Control Baseline Hardening

### Objective

Validate and harden the existing Nero arm planning and control baseline, adding only the IK, planning, and interface pieces that are still missing.

### Scope

- Audit the current Nero arm-only MoveIt, OMPL, and MIT-controller path before adding new stack pieces.
- Confirm the existing `trajectory_msgs/JointTrajectory` to MIT-controller execution path, including joint ordering, timing, and units.
- Verify whether the current KDL and OMPL baseline already covers representative Nero planning tasks.
- Introduce or validate TRAC-IK only where it materially improves the current baseline.
- Resolve remaining gaps between the desired roadmap semantics and the current code paths, especially `arm` versus `nero_arm` and `link7`/`tcp_link` versus `nero_tool0`.
- Use Pinocchio for model sanity checks and FK validation, and use Ruckig as optional smoothing or comparison tooling where it adds value.
- Capture the remaining IK, planning, collision, and execution gaps that still block a reliable Nero standalone baseline.

### Expected Outputs

- Validated Nero standalone planning and control baseline using the existing workspace packages
- Confirmed `JointTrajectory` execution path into the MIT controller
- Documented gap analysis for TRAC-IK, OMPL, naming, collision semantics, and execution safety
- Representative pose-planning checklist and debug scripts for the Nero arm
- Clear list of missing pieces that still need implementation rather than rediscovery

### Follow-up Documents

- MoveIt 2 Nero setup document
- TRAC-IK configuration document, if still needed after audit
- MIT trajectory interface and execution policy documents
- Pinocchio/Ruckig debug-layer document

---

## Sprint 4: Nero Plus OmniHand Common Baseline

### Objective

Build the first Nero plus OmniHand baseline on top of the shared adapter boundary and common ROS semantics established in Sprint 2.

### Scope

- Reuse the repo-owned OmniHand adapter boundary and common ROS naming from Sprint 2.
- Bring the first repo-owned ROS bridge and common hand status/diagnostic interfaces into the combined stack as far as hardware access allows.
- Add `wrist_adapter_link` and `omnihand_palm_link` to the Nero tool chain.
- Add and validate `grasp_frame` on the shared hand contract.
- Represent the OmniHand as a static payload attached to `nero_tool0`.
- Keep full finger-level dexterous planning out of the primary arm IK chain during this baseline phase.
- Define correct hand mass, center of mass, and inertia approximation.
- Update gravity and payload configuration for the MIT controller.
- Add simplified hand collision geometry to MoveIt 2.
- Reuse the landed OmniHand MoveIt profiles as the planning base for the combined baseline.
- Validate reachability and collision behavior with the attached hand envelope.

### Expected Outputs

- `nero_with_omnihand_static` model variant
- First repo-owned OmniHand bridge/common message surface aligned with agx_arm semantics
- Updated MoveIt 2 configuration for arm planning with attached hand geometry and the shared hand contract
- Updated MIT payload configuration
- Validated `grasp_frame` definition

### Follow-up Documents

- Tool payload integration document
- OmniHand bridge and diagnostics document
- Wrist adapter modeling document
- Hand collision-envelope document
- MIT payload/gravity compensation document

---

## Sprint 5: Static AGV/Base Integration

### Objective

Integrate the AGV/base CAD into the planning and simulation model as a static mounting and collision environment.

### Scope

- Import AGV/base CAD as visual geometry.
- Create simplified collision primitives for planning.
- Define `agv_base_link` and `arm_mount_link`.
- Validate the mounting pose between AGV/base and Nero arm.
- Include the AGV/base as a static collision environment in MoveIt 2.
- Create the `nero_on_static_agv` and `nero_omnihand_on_static_agv` model variants.
- Confirm that gravity compensation uses the correct arm mounting orientation.

### Expected Outputs

- Simplified AGV/base collision model
- Static mount transform definition
- Combined Nero + OmniHand + static AGV/base model
- Collision-aware MoveIt 2 planning scene
- Mounting-pose validation report

### Follow-up Documents

- AGV CAD-to-collision conversion document
- Arm mounting-pose document
- Static base collision environment document

---

## Sprint 6: Combined Collision and Planning Validation

### Objective

Validate collision-aware planning for the arm, hand envelope, and static AGV/base model.

### Scope

- Generate and validate the Allowed Collision Matrix.
- Test self-collision behavior for arm-only, arm-with-hand, and arm-on-base variants.
- Use MoveIt 2 PlanningScene as the primary collision-checking baseline.
- Test OMPL plans for common manipulation poses.
- Add collision rejection to controller-bound trajectory execution.
- Compare behavior across model variants.

### Expected Outputs

- Validated self-collision matrix
- Collision-aware planning test suite
- Planning failure category report
- Safe trajectory acceptance criteria

### Follow-up Documents

- Self-collision validation document
- PlanningScene test document
- Collision-aware execution policy document

---

## Sprint 7: Isaac Sim Digital Twin Integration

### Objective

Bring the validated model variants into Isaac Sim as simulation-ready assets.

### Scope

- Import or validate Nero USD assets.
- Convert or assemble combined robot variants as USD assets.
- Validate articulation structure, joint limits, drives, and collision shapes.
- Validate hand payload and simplified hand collision geometry.
- Add AGV/base visual and collision assets.
- Align Isaac Sim frames with ROS 2 and MoveIt 2 frames.
- Validate simple command execution in Isaac Sim.

### Expected Outputs

- `nero_standalone.usd`
- `nero_with_omnihand_static.usd`
- `nero_omnihand_on_static_agv.usd`
- Isaac Sim scene template
- ROS 2 bridge configuration for simulated Nero control

### Follow-up Documents

- Isaac Sim asset import document
- USD articulation validation document
- Isaac Sim ROS 2 bridge document
- Simulation frame-alignment document

---

## Sprint 8: Hardware-in-the-Loop and Replay

### Objective

Create a shared replay and validation workflow between real hardware and simulation.

### Scope

- Standardize logging for joint states, commands, controller states, hand states, tactile data, and sensor streams.
- Replay real trajectories in Isaac Sim.
- Replay simulated trajectories through the MIT-controller interface in a controlled mode.
- Compare target and actual trajectories.
- Detect model mismatch in gravity, payload, and mounting configuration.
- Establish safety gating before real execution.

### Expected Outputs

- Unified log format
- Replay toolchain
- Sim-real trajectory comparison report
- Hardware execution safety checklist

### Follow-up Documents

- Logging format document
- HIL replay document
- Sim-real validation document
- Safety gate document

---

## Sprint 9: Grasping MVP

### Objective

Establish a simple but measurable grasping baseline before introducing SOTA dexterous grasping models.

### Scope

- Define a minimal object set for grasping tests.
- Calibrate workspace perception.
- Use scripted or geometry-based pregrasp generation.
- Use predefined OmniHand preshapes.
- Close the hand using tactile, position, current, or torque thresholds.
- Perform lift tests.
- Log grasp success, slip, contact quality, and failure modes.

### Expected Outputs

- Grasp benchmark protocol
- Preshape-based grasping baseline
- Grasp success/failure dataset
- Initial tactile/contact logging pipeline

### Follow-up Documents

- Grasp benchmark document
- Preshape library document
- Tactile closure policy document
- Grasp failure taxonomy document

---

## Sprint 10: Dexterous Grasping Model Evaluation

### Objective

Evaluate SOTA-inspired dexterous grasping models and determine which are suitable for the OmniHand Pro setup.

### Scope

- Evaluate candidate grasp-generation models for compatibility with OmniHand geometry and control.
- Map generated grasps to the OmniHand kinematic structure.
- Filter generated grasps through IK and collision checks.
- Integrate candidate grasps into the existing pregrasp and hand-closure pipeline.
- Compare against the scripted/preshape baseline.

### Candidate Model Families

- DexGraspNet-style dexterous grasp generation
- AnyDexGrasp-style cross-hand grasp adaptation
- VLA-guided dexterous grasping approaches
- Task- or affordance-conditioned grasp selection

### Expected Outputs

- Dexterous grasp model compatibility report
- Grasp candidate filtering pipeline
- Benchmark comparison against MVP grasping baseline
- Recommendation for continued dexterous grasping development

### Follow-up Documents

- Dexterous grasp model survey
- OmniHand grasp mapping document
- Grasp candidate filtering document
- Dexterous grasp benchmark report

---

## Sprint 11: Scripted Skill Library

### Objective

Create a deterministic skill library that can serve as a baseline, data generator, fallback layer, and interface target for learned policies.

### Scope

- Implement deterministic versions of common manipulation skills.
- Define preconditions, postconditions, termination criteria, and fallback behavior.
- Connect skills to MoveIt 2, the MIT controller, and OmniHand commands.
- Support object-centric and frame-relative parameterization.

### Initial Skills

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

### Expected Outputs

- `nero_skill_library`
- Skill command schema
- Baseline scripted manipulation demos
- Skill-level logging and annotation format

### Follow-up Documents

- Skill schema document
- Scripted skill implementation document
- Skill validation protocol
- Skill failure recovery document

---

## Sprint 12: Demonstration Data Pipeline

### Objective

Build a clean data pipeline for imitation learning, diffusion-based skills, RL initialization, and later Physical AI data workflows.

### Scope

- Record synchronized demonstrations from scripted, teleoperated, and real executions.
- Store observations, actions, states, tactile data, camera data, transforms, and skill metadata.
- Segment demonstrations by skill.
- Label success, failure, recovery, contact events, and object/task parameters.
- Support replay into simulation.

### Expected Outputs

- Demonstration dataset format
- Skill-segmented recordings
- Data validation tools
- Replay-compatible dataset samples

### Follow-up Documents

- Demonstration data schema
- Teleoperation data collection document
- Skill segmentation document
- Dataset validation document

---

## Sprint 13: Isaac Lab Task Development

### Objective

Create Isaac Lab environments for controlled learning experiments using the Nero model variants.

### Scope

- Create simple state-based tasks first.
- Use the combined model only after standalone and static-hand variants are stable.
- Introduce domain randomization gradually.
- Keep action spaces compatible with the MIT-controller execution model.
- Evaluate policies in simulation before hardware transfer.

### Initial Task Families

```text
NeroReach-v0
NeroPregrasp-v0
NeroGraspHold-v0
NeroLift-v0
NeroPlace-v0
NeroPour-v0
```

### Expected Outputs

- Isaac Lab task package
- Baseline RL training configuration
- Simulation-only policy evaluation protocol
- Sim-to-real readiness checklist

### Follow-up Documents

- Isaac Lab environment design document
- Observation/action space document
- Reward and termination design document
- Domain randomization document

---

## Sprint 14: Diffusion-Based Skill Policies

### Objective

Evaluate diffusion-based policies for adapting recorded skills to varying conditions.

### Scope

- Start with state-based skill policies.
- Use demonstration data from deterministic and teleoperated skills.
- Train skill-specific diffusion policies for grasping, placing, and pouring.
- Use receding-horizon execution with safety-gated commands.
- Compare learned policies against scripted skills.

### Candidate Skills

```text
GraspAdaptive
PlaceAdaptive
PourAdaptive
RecoverAdaptive
```

### Expected Outputs

- Diffusion skill training pipeline
- Skill-specific policy checkpoints
- Offline and simulation evaluation reports
- Real-hardware trial protocol

### Follow-up Documents

- Diffusion policy training document
- Skill policy evaluation document
- Real-hardware rollout document
- Safety wrapper document

---

## Sprint 15: GPU-Accelerated Manipulation Evaluation

### Objective

Evaluate when and how Isaac ROS Manipulation, cuMotion, and related GPU-accelerated components should replace or augment the MoveIt 2 baseline.

### Scope

- Integrate cuMotion with simplified robot variants.
- Evaluate planning speed and robustness for arm-hand-base collision scenarios.
- Integrate depth-based environment representations where appropriate.
- Evaluate Nvblox for collision-world generation.
- Evaluate FoundationPose or equivalent object pose estimation for manipulation.
- Compare GPU-accelerated planning against MoveIt 2 baseline.

### Expected Outputs

- cuMotion integration prototype
- GPU planning comparison report
- Recommendation for production or research usage
- Updated planning-stack decision matrix

### Follow-up Documents

- cuMotion integration document
- Nvblox collision-world document
- FoundationPose integration document
- Planning benchmark document

---

## Sprint 16: VLA and Physical AI Policy Evaluation

### Objective

Evaluate vision-language-action and generalist policy models as high-level or skill-level controllers, not as direct raw low-level controllers.

### Scope

- Define embodiment mapping for Nero + OmniHand Pro.
- Evaluate candidate VLA/VLM policy families offline first.
- Use simulation rollouts before hardware trials.
- Restrict initial outputs to skill selection, target pose generation, grasp proposal generation, or bounded action deltas.
- Measure inference latency and memory footprint on available edge hardware.

### Candidate Policy Families

- GR00T-style VLA models
- OpenVLA-style policies
- π0-style flow-matching policies
- Task-conditioned visuomotor policies

### Expected Outputs

- VLA compatibility report
- Embodiment/action mapping proposal
- Simulation evaluation protocol
- Edge-inference feasibility report

### Follow-up Documents

- VLA model survey
- Embodiment mapping document
- VLA evaluation protocol
- Edge deployment document

---

## Sprint 17: Cosmos and Synthetic Data Workflow

### Objective

Integrate Physical AI data workflows for synthetic data generation, visual variation, simulation replay, and policy evaluation.

### Scope

- Export simulation data in a format compatible with downstream synthetic-data workflows.
- Use Isaac Sim scenes and recorded demonstrations as structured inputs.
- Evaluate Cosmos-style workflows for visual variation, data augmentation, and world-model-assisted policy evaluation.
- Keep real demonstrations as the anchor dataset.
- Feed failure cases back into the data and training loop.

### Expected Outputs

- Synthetic data workflow prototype
- Simulation-to-augmentation data path
- Dataset versioning convention
- Physical AI data flywheel design

### Follow-up Documents

- Cosmos workflow document
- Synthetic data export document
- Dataset versioning document
- Failure-case feedback-loop document

---

## 6. Sensor and Hardware Planning

## 6.1 Minimum Hardware Set

The minimum functional setup should include:

- Nero arm
- OmniHand Pro
- Jetson AGX Orin
- MIT controller hardware
- Dedicated communication adapter for the arm
- Dedicated CAN-FD or manufacturer-compatible adapter for the OmniHand Pro
- Stable power supplies for arm and hand
- Hardware emergency stop
- Local logging storage
- At least one calibrated RGB-D camera

## 6.2 Recommended Early Perception Setup

Recommended initial perception hardware:

- One fixed RGB-D camera observing the manipulation workspace
- Optional second RGB-D camera from a different viewpoint
- Calibration target such as ChArUco, AprilTag grid, or equivalent
- Diffuse lighting for repeatable perception
- Optional wrist camera after payload, cable routing, and field-of-view constraints are validated

## 6.3 Later Perception and Manipulation Hardware

Potential later additions:

- Wrist RGB or RGB-D camera
- 6-axis wrist force/torque sensor
- Multi-camera synchronized perception setup
- CSI/GMSL camera setup for Jetson-oriented pipelines
- Higher-throughput edge compute platform such as Jetson Thor
- Dedicated RTX workstation or remote GPU system for simulation and training

## 6.4 Hardware Decision Gates

Hardware procurement should be tied to functional needs:

| Need | Hardware Consideration |
|---|---|
| Stable workspace perception | Fixed RGB-D camera, calibration target, controlled lighting |
| Tactile grasp refinement | OmniHand tactile interface, synchronized logging |
| Contact-rich manipulation | Wrist force/torque sensor |
| Heavy simulation and RL training | RTX workstation or remote RTX GPU environment |
| Larger edge inference | Jetson Thor-class system |
| GPU-accelerated planning with perception | Jetson Thor-class system, Isaac ROS-compatible sensors |

---

## 7. Repository Alignment

This roadmap does not define the current Sprint 2 package names or public ROS2 contract by itself.

Treat roadmap role names as architectural aliases only. The stable repo decisions live in:

- `docs/project/repository_structure.md`
- `docs/project/package_naming.md`
- `docs/project/ros2_development_practices.md`
- `docs/control/omnihand_ros_integration_options.md`
- `docs/control/omnihand_wrapper_integration_plan.md`

---

## 8. Documentation Workflow

The development docs now follow a fixed two-tier model.

### 8.1 Fixed Coordination Docs

Keep exactly one top-level document for each cross-sprint concern:

- `docs/development/nero_physical_ai_roadmap.md`: roadmap, phases, and sprint intent
- `docs/development/nero_physical_ai_progress.md`: overall progress monitoring and active-priority tracking
- `docs/development/component_implementation_map.md`: component ownership, canonical code paths, and document routing

### 8.2 Sprint Working Folders

Keep sprint-local discovery, checklists, implementation details, and issue logs inside `docs/development/sprintN/`.

Each sprint folder should own at least:

- `README.md`
- `checklist.md`
- `errors_and_fixes.md`
- `open_questions.md`

Add optional subfolders such as `assets/`, `control/`, `hand/`, `planning/`, or `simulation/` only when the sprint needs them.

Promote stable outputs into `docs/assets/`, `docs/control/`, or `docs/project/` when they become canonical. Keep `.github/` as the concise agent-facing mirror of those stable docs rather than a competing documentation tree.

---

## 9. Immediate Integration Priorities

The immediate technical priorities are:

1. Confirm and validate the Nero asset pipeline.
2. Consolidate the common package structure, OmniHand adapter boundary, and shared ROS semantics used across planning and control.
3. Validate and harden the current Nero MoveIt plus MIT planning/control baseline, adding missing IK or planning pieces only where needed.
4. Bring up the repo-owned OmniHand bridge and common hand status or diagnostic interfaces above the adapter.
5. Model the OmniHand first as a static tool and payload on top of that shared contract.
6. Integrate the AGV/base as a static mounting and collision environment once the external assets are available locally.
7. Validate combined collision behavior before introducing complex learned policies.
8. Build deterministic skills before training adaptive skills.
9. Use simulation and replay workflows before real hardware trials.
10. Introduce GPU-accelerated manipulation, VLA policies, and Cosmos workflows only after the model, control, logging, and baseline skills are stable.

---

## 10. High-Level End State

The intended end state is a modular Physical AI workflow in which:

- Nero, OmniHand Pro, and the AGV/base exist as validated model variants.
- MoveIt 2 provides a reliable classical planning baseline.
- Isaac Sim provides digital twin simulation.
- Isaac Lab provides RL and policy-training environments.
- Deterministic skills provide safe fallback behavior and demonstration data.
- Dexterous grasping models can be evaluated against measurable baselines.
- Diffusion-based skill policies adapt demonstrated skills to varying conditions.
- VLA/VLM policies operate at skill or bounded-action level.
- Cosmos-style data workflows support simulation replay, data augmentation, and Physical AI evaluation.
- Edge hardware runs real-time integration, perception, policy inference, and safety-gated execution.

The system should remain modular enough that individual components can be replaced as better models, sensors, planners, or compute platforms become available.
