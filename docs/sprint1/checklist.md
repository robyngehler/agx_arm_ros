# Sprint 1 Checklist

Historical closure summary for Sprint 1.

## Established in Sprint 1

- [x] Canonicalize the Nero description source of truth onto `src/agx_arm_sim/agx_arm_description`.
- [x] Remove the duplicate root description package from workspace discovery.
- [x] Drop the legacy root-source MIT URDF fallback.
- [x] Remove the remaining `agx_arm_urdf` submodule dependency and keep the required Nero/Revo2
	asset tree directly in-repo.
- [x] Restrict the active launch, config, and documentation surface to the Nero-focused baseline.
- [x] Promote the first stable asset and controller inventory docs into `docs/assets/`.
- [x] Decide the near-term OmniHand integration direction: isolated SDK validation first, then a
	thin local wrapper, then a repo-owned ROS layer.

## Handed off beyond Sprint 1

- [x] deeper OmniHand runtime validation moved into later sprints
- [x] AGV/base geometry and mounting data remained missing and stayed open
- [x] broader simulation and Isaac coverage stayed partial and was not treated as closed