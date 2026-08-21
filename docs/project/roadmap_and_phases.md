# Nero Physical AI Roadmap

## 1. Purpose

This document defines the high-level roadmap for evolving the currently functional MIT-controller-based Nero arm setup into a modular Physical AI research and development workflow.

The roadmap focuses on the integration of the Nero arm, the OmniHand Pro dexterous gripper, and a custom AGV/static base setup into a simulation, planning, control, and learning pipeline. It is intentionally structured as a semantic master plan. Detailed technical procedures, implementation notes, validation protocols, and experiments are expected to live in separate sub-phase documents.

The roadmap is dependency-based rather than calendar-based. Sprints describe logical work packages and integration maturity levels, not fixed durations.

Use this file as the canonical roadmap only.

Companion coordination docs:

- `docs/checklist.md`: current overall sprint status, active focus, and cross-sprint blockers
- `docs/project/components/implementation_map.md`: system components, canonical code locations, and where sprint-level details live

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

The current local execution order is now:

- repo-local Duo body model
- `body + right arm + right OmniHand`
- mirrored left arm and left OmniHand integration
- multi-arm-safe launch, MoveIt, and controller-path generalization
- only after that, broader Isaac and later AGV/mobile expansion

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
duo_body_right_arm_right_omnihand
duo_body_dual_nero_dual_omnihand
nero_on_static_agv
nero_omnihand_on_static_agv
nero_omnihand_on_mobile_agv
```

Each variant should be generated from shared Xacro or equivalent model components. Manual duplication of complete URDF files should be avoided.

### 4.4 Canonical Frame Naming

The current repo-local Duo body staging baseline uses the following frame chain:

```text
body_base_link
├── left_arm_mount_link
│   └── left_arm_base_link
│       └── left_arm_nero_tool0
│           └── left_arm_omnihand_flange
│               └── left_base_link
└── right_arm_mount_link
    └── right_arm_base_link
        └── right_arm_nero_tool0
            └── right_arm_omnihand_flange
                └── right_base_link
```

Later mobile or AGV-mounted variants may wrap this under the larger navigation frame chain:

```text
map
└── odom
    └── agv_base_link
        └── body_base_link
            └── ... current Duo body chain ...
```

Required semantic frames:

| Frame | Meaning |
|---|---|
| `body_base_link` | Current repo-local Duo body reference frame for fixed two-arm body integration |
| `left_arm_mount_link`, `right_arm_mount_link` | Mechanical mounting interfaces between the Duo body and each Nero arm |
| `left_arm_base_link`, `right_arm_base_link` | Prefixed Nero arm base frames for multi-arm-safe composition |
| `left_arm_nero_tool0`, `right_arm_nero_tool0` | Prefixed Nero flange frames in the current Duo body system chain |
| `left_arm_omnihand_flange`, `right_arm_omnihand_flange` | Mechanical adapter frames between each Nero flange and OmniHand |
| `left_base_link`, `right_base_link` | OmniHand base frames provided by the current hand description assets |
| `map` | Global/world reference frame |
| `odom` | Local odometry frame for future mobile-base operation |
| `agv_base_link` | Main AGV/base reference frame for later mobile or larger static-base variants |
| `grasp_frame` | Task-level grasp reference frame used by planners and skills |

### 4.5 Planning Groups

Initial MoveIt 2 planning groups should be defined as follows:

```text
right_arm
right_arm_hand
left_arm
left_arm_hand
both_arms
both_arms_with_hands
```

Recommended early usage:

- `right_arm`: first executable arm IK and motion-planning group for the right-side Duo system slice
- `right_arm_hand`: right-arm planning with the attached hand geometry treated as part of the system envelope
- `left_arm` and `left_arm_hand`: mirrored extension once the right-side chain is validated
- `both_arms`: coordinated dual-arm planning group for shared-body tasks
- `both_arms_with_hands`: later coordinated planning group after self-collision matrices and execution semantics stabilize

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

The execution sprints (`docs/sprintN/`) are the work iterations and have diverged from this
roadmap's thematic phase numbering. Current state (see `docs/checklist.md` for the current
status snapshot):

- Execution sprints 1–4 are **complete**: asset baseline; common environment + OmniHand bridge with
  the vendor SDK backend; Nero planning/control hardening (TRAC-IK + OMPL); and the Duo body
  baseline with shared macro/xacro URDF, dynamic SRDF, arm-count-aware MoveIt, successful OMPL +
  TRAC-IK Duo planning, and a joint arm + OmniHand bringup with small live movements.
- Execution **sprint 5 is complete**: native `mttcan` CAN FD transport (`one-shot`, arm + hand per
  side bus) and `vendor/pyAgxArm` pinning are the settled baseline.
- Execution **sprint 6 is resuming**: coordination, Duo-hand runtime hardening, and the first demo
  task path (tea pour, then Hefeweizen pouring). It was paused for the V02 runtime refactor and
  resumes against the refactored command contracts.
- The **V02 refactor** (`docs/sprint_refactor/`) is the current implementation sprint: device
  authority and epochs, one serialized SDK owner per device, per-device CAN buses with parallel
  same-side arm and hand motion. Its Runtime RC closed 2026-08-17.
  These execution sprints sit ahead of this roadmap's thematic phase 5 (AGV/base), which stays external.
- Simulation and Isaac work stay behind the first validated Duo body system baseline instead of leading it.

### Adjacent Demo Tooling Note

The current workspace also contains a wakeword-triggered motion-demo slice around `agx_arm_wakeword_motion_manager` and the external `wakeword-benchmark` listener. This is not a direct roadmap gate by itself, but it is a useful integration path for later TTS-driven and interaction-driven demos. Keep the historical workflow lineage in `docs/sprint2/evidence/mit_runtime_history.md`.

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

Validate and harden the existing Nero arm planning and control baseline, adding only the IK, planning, interface, and description-composition pieces that are still missing for the first Duo system bringup.

### Scope

- Audit the current Nero arm-only MoveIt, OMPL, and MIT-controller path before adding new stack pieces.
- Confirm the existing `trajectory_msgs/JointTrajectory` to MIT-controller execution path, including joint ordering, timing, and units.
- Verify whether the current KDL and OMPL baseline already covers representative Nero planning tasks.
- Introduce or validate TRAC-IK only where it materially improves the current baseline.
- Resolve remaining gaps between the desired roadmap semantics and the current code paths, especially `arm` versus `nero_arm` and `link7`/`tcp_link` versus `nero_tool0`.
- Land the minimum prefix-safe and side-selectable description groundwork needed so Sprint 4 can compose a body-mounted right-first system without reopening canonical package ownership.
- Use Pinocchio for model sanity checks and FK validation, and use Ruckig as optional smoothing or comparison tooling where it adds value.
- Capture the remaining IK, planning, collision, and execution gaps that still block a reliable Nero standalone baseline.

### Expected Outputs

- Validated Nero standalone planning and control baseline using the existing workspace packages
- Confirmed `JointTrajectory` execution path into the MIT controller
- Documented gap analysis for TRAC-IK, OMPL, naming, collision semantics, and execution safety
- Prefix-safe Duo system description staging layer and a documented handoff into Sprint 4
- Representative pose-planning checklist and debug scripts for the Nero arm
- Clear list of missing pieces that still need implementation rather than rediscovery

### Follow-up Documents

- MoveIt 2 Nero setup document
- TRAC-IK configuration document, if still needed after audit
- MIT trajectory interface and execution policy documents
- Pinocchio/Ruckig debug-layer document

---

## Sprint 4: Duo Body Plus OmniHand System Baseline

### Objective

Build the first Duo body plus Nero plus OmniHand baseline on top of the shared adapter boundary and common ROS semantics established in Sprint 2.

### Scope

- Reuse the repo-owned OmniHand adapter boundary and common ROS naming from Sprint 2.
- Validate `body_base_link`, left/right mount frames, and the staged Duo body package as the current system-assembly surface.
- Bring up `body + right arm + right OmniHand` first as the first executable system slice.
- Keep descriptions and launch surfaces arm-count-aware from the start even while validating only the right side first.
- Mirror the left arm and left OmniHand only after the right-side chain is validated.
- Generalize the current single-arm assumptions in RViz, MoveIt, and controller-facing launch surfaces without forking long-term package ownership.
- Keep full finger-level dexterous planning out of the primary arm IK chain during this baseline phase.
- Add simplified hand collision geometry and body-aware collision reasoning to the planning baseline.
- Define the first coordinated dual-arm planning targets needed for shared-body tasks such as a two-arm pouring workflow.

### Expected Outputs

- `duo_body_right_arm_right_omnihand` model variant
- Configurable `duo_body_dual_nero_dual_omnihand` system description
- Documented and partially validated right-side body-mounted baseline
- Multi-arm-safe launch and MoveIt generalization task list
- First coordinated dual-arm benchmark target, with pouring as the representative reference task

### Follow-up Documents

- Duo system integration and frame-validation document
- Multi-arm launch and MoveIt generalization document
- Wrist adapter and hand collision-envelope document
- MIT payload/gravity compensation document

---

## Sprint 5: Static AGV/Base Integration

### Objective

Integrate the AGV/base CAD into the planning and simulation model as a static mounting and collision environment.

### Scope

- Import AGV/base CAD as visual geometry.
- Create simplified collision primitives for planning.
- Define `agv_base_link`, `body_base_link`, and the wrapper transforms around the current left and right body mount frames.
- Validate the mounting pose between AGV/base, the Duo body, and the left/right Nero arm chains.
- Include the AGV/base as a static collision environment in MoveIt 2.
- Create the `nero_on_static_agv` and `nero_omnihand_on_static_agv` model variants.
- Confirm that gravity compensation uses the correct arm mounting orientation.

### Expected Outputs

- Simplified AGV/base collision model
- Static mount transform definition
- Combined Duo body + Nero + OmniHand + static AGV/base model
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
- Hardware emergency stop (none exists today; the unit has no mechanical e-stop)
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
- `.claude/rules/package-naming.md`
- `.claude/rules/ros2-development.md`
- `docs/assets/omnihand/omnihand_ros_integration_options.md`
- `docs/assets/omnihand/omnihand_wrapper_integration_plan.md`

---

## 8. Documentation Workflow

The development docs now follow a fixed two-tier model.

### 8.1 Fixed Coordination Docs

Keep exactly one top-level document for each cross-sprint concern:

- `docs/project/roadmap_and_phases.md`: roadmap, phases, and sprint intent
- `docs/checklist.md`: overall progress monitoring and active-priority tracking
- `docs/project/components/implementation_map.md`: component ownership, canonical code paths, and document routing

### 8.2 Sprint Working Folders

Use `docs/sprintN/` as the user-facing sprint entrypoints. Keep detailed historical discovery,
checklists, implementation details, and issue logs inside the matching sprint surface itself.

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
