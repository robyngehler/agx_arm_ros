# OmniHand Docs

This directory contains the stable OmniHand-specific documentation surfaces.

## Current stable docs

- `omnihand_canfd_setup.md`: validated CAN FD bringup on the current Jetson baseline
- `omnihand_vendor_sdk_aarch64.md`: local SDK baseline and adapter policy for Jetson `aarch64`
- `omnihand_solo_bringup_and_load_test.md`: isolated bridge and vendor-level validation flow
- `omnihand_ros_integration_options.md`: stable architecture and ROS-contract decision record
- `omnihand_wrapper_integration_plan.md`: backend sequencing and wrapper-first rollout plan
- `omnihand_vendor_socketcan_recv_report.md`: the receive-thread spin reported upstream, with the fix carried in our fork

## Historical or engineering-reference docs

- `omnihand_active_joint_map.md`: legacy O10 vendor-declared active-joint baseline, kept only as a historical mapping note
- `omnihand_pro_jetson_socketcan_patches.md`: engineering patch lineage for the Jetson SocketCAN fork path

## Scope rule

- stable runtime claims belong in the current stable docs above
- patch-level history and superseded vendor baselines should remain clearly labelled and should not
  redefine the current repo contract on their own