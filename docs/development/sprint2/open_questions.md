# Sprint 2 Decisions And Open Questions

## Remaining Open Questions

### Current Host Bring-Up Gate

> ✅ **RESOLVED (sprint5).** Historical USB `gs_usb` exploration below. The OmniHand now runs on the
> Jetson **native `mttcan`** CAN FD bus (5 Mbit transceiver), and the bridge has a working vendor-SDK
> backend. See `docs/assets/omnihand/omnihand_canfd_setup.md`.

- The current Ubuntu host now shows a USB CANFD adapter as `can2` via `gs_usb`, while the repo-owned ROS bridge is still mock-only. The next live validation step should therefore stay below ROS and use the isolated vendor SDK path first.
- The vendor SDK's local SocketCAN path has now been patched to read `OMNIHAND_SOCKETCAN_IFACE` instead of assuming `can0`, but it is still unverified against the attached hand on this host.
- A forced `gs_usb` bind for the OmniHand adapter (`a8fa:8598`) now creates `can3` on USB port `1-4.3:1.0`, but the kernel exposes only classic CAN on that interface: `mtu 16`, no `dbitrate`, and `ip link set can3 type can fd on ...` returns `Operation not supported`.
- The vendored ZLG `usbcanfd` kernel module does build on the current Jetson and supports USB IDs `04cc:1240` and `3068:0009`, but not the currently attached adapter `a8fa:8598`.
- The remaining question is therefore no longer whether forced `gs_usb` can be made good enough on this host. The real open question is whether the supplier can provide an aarch64 Linux support package for `a8fa:8598`, or whether Sprint 2 should switch to a supported ZLG-style CAN FD adapter for first live hand validation.
- The current evidence and host-side install options are tracked in `omnihand_canfd_driver_investigation.md`.

### First Real OmniHand Backend

- Should the first non-mock backend target direct SDK access only, or should the repo keep an optional vendor-ROS adapter behind the same bridge surface?

### Long-Term Hand Command Surface

- Should `control/omnihand/joint_trajectory` remain a compatibility surface only, or should the repo promote a more explicit action or controller contract once the non-mock backend exists?

### Package Boundary After Real Backend Validation

- If the real backend adds more dependencies or rebuild churn, does the bridge still belong inside `agx_arm_ctrl`, or does a dedicated package become materially clearer only then?

### Runtime Graph Validation Evidence

- Once a full mock or live launch is run again, should the repo promote a captured runtime graph or launch trace into a stable doc, or keep the current diagrams source-derived only until the next validation pass?