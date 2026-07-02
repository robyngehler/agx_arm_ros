---
paths:
  - "src/**"
  - "vendor/**"
  - "**/package.xml"
  - "**/CMakeLists.txt"
---

# Repository Structure

*Use when creating, moving, or extending package surfaces in agx_arm_ros. Covers the current Duo
package boundaries and where new work belongs.*

Use the current workspace layout as the implementation truth during the active Duo baseline.

## Canonical Package Roles

- `src/agx_arm_sim/agx_arm_description`: canonical long-term description package for Nero, Revo2, and repo-owned OmniHand assets
- `src/duo_body_description`: current Duo staging package for Duo body plus configurable arm-hand system assembly
- `src/agx_arm_moveit`: current MoveIt baseline and simulation path
- `src/agx_arm_ctrl`: runtime arm bridge, launch surfaces, and current end-effector integration point
- `src/agx_arm_mit_controller`: runtime MIT controller node, shared trajectory/gravity libraries, and curated controller configs
- `src/agx_arm_mit_demos`: interactive recorder, playback, and wakeword demo workflows around the MIT stack
- `src/agx_arm_mit_tools`: debug bridges, hold validation, and calibration helpers around the MIT stack
- `src/agx_arm_msgs`: repo-owned ROS messages
- `src/agx_arm_coordination`: Activity-DAG coordinator, performer routing, resource model, and YAML graph/catalogue loader for coordinated dual-arm + dual-hand tasks
- `vendor/OmniHand-Pro-2025`: upstream SDK input, not the public repo contract

## Current Placement Rules

- keep the OmniHand bridge and the OmniHand skill controller in `src/agx_arm_ctrl` for now
- keep Sprint 6 task orchestration (coordinator, performer routing, YAML graph/catalogue loader) in `src/agx_arm_coordination`; route `Trajectory+both_arms` through the existing FollowJointTrajectory path rather than forking arm execution
- keep `src/agx_arm_sim/agx_arm_description` as the canonical long-term description package and use `src/duo_body_description` only as the documented Duo staging surface
- do not fork a second MoveIt package for the same Nero or Duo baseline
- keep production MIT execution ownership in `src/agx_arm_mit_controller`
- place app-layer demos under `src/agx_arm_mit_demos` instead of the controller runtime package
- place debug, calibration, and legacy helper entry points under `src/agx_arm_mit_tools`
- extend `src/agx_arm_msgs` instead of creating an OmniHand-only message package
- use `.claude/` for Claude-Code-native rule, skill, and agent mirrors, not as a replacement for stable project docs
- keep descriptions and bringup surfaces arm-count-aware from the start, with `body + right arm + right OmniHand` as the current executable Duo target

## Documentation Split

- `docs/control/`: how to run the system — bringup launch/argument matrix and the teach loop
- `docs/assets/`: stable factual inventories, validation state, and OmniHand/runtime integration decisions
- `docs/development/`: fixed roadmap, progress, and component-routing docs plus sprint working folders
- `docs/project/`: stable package, naming, and workflow policy
- `.claude/`: concise Claude-Code-native rule, skill, and agent layer

## Escalation Rule

Create a new long-term package only when a stable public ROS contract, dependency boundary, or rebuild
boundary truly requires it. Temporary staging packages are acceptable only when `docs/project/`
documents their role and exit path.
