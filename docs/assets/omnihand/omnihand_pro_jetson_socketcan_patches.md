# OmniHand Pro 2025 — Jetson / aarch64 SocketCAN Build Patches

**Target fork:** `robyngehler/OmniHand-Pro-2025`, branch `jetson-orin-socketcan`
**Goal:** build the Pro SDK Python package (`agibot_hand`) on Jetson AGX Orin over
**native SocketCAN**, with no x86 ZLG USB-CANFD library pulled into the aarch64
runtime, and with the CAN interface selectable via `OMNIHAND_SOCKETCAN_IFACE`.

These patches are derived from the verified upstream `main` content (checked
2026-06-24). Apply them on the fork branch. Four files change; one extra
optional guard for the vendor ROS node is noted at the end.

---

## 0. Fork and submodule swap (run with your GitHub)

```bash
# 1) Fork AgibotTech/OmniHand-Pro-2025 -> robyngehler/OmniHand-Pro-2025 (GitHub UI or gh)
gh repo fork AgibotTech/OmniHand-Pro-2025 --clone=false --org robyngehler 2>/dev/null || \
gh repo fork AgibotTech/OmniHand-Pro-2025 --clone=false

# 2) In the agx_arm_ros workspace, remove the old O10 submodule
cd ~/workspace/agx_arm_ros
git submodule deinit -f vendor/OmniHand-Pro-2025
git rm -f vendor/OmniHand-Pro-2025

# 3) Add the Pro fork submodule tracking its DEFAULT branch (main).
#    Do NOT pass -b jetson-orin-socketcan here: that branch does not exist on the
#    fork yet, and `submodule add -b` requires the branch to already be on the
#    remote, so it fails the checkout. Create the branch AFTER cloning (step 4).
git submodule add git@github.com:robyngehler/OmniHand-Pro-2025.git vendor/OmniHand-Pro-2025

# 4) Create the integration branch in the fork and push it
cd vendor/OmniHand-Pro-2025
git switch -c jetson-orin-socketcan
git push -u origin jetson-orin-socketcan
cd ../..

# 5) Point the submodule tracking branch at it (used by `submodule update --remote`)
git config -f .gitmodules submodule.vendor/OmniHand-Pro-2025.branch jetson-orin-socketcan
git add .gitmodules vendor/OmniHand-Pro-2025
```

The old O10 submodule stays recoverable via the tag
`omnihand-o10-bridge-working-2026-06-24`.

---

## Patch 1 — top-level `CMakeLists.txt`: add CAN backend option

`CanfdDevice` must be visible to BOTH `src/` and `python/`; a `set()` inside
`src/CMakeLists.txt` is scoped to that subdirectory only. Define it at the top
level so both `add_subdirectory()` children inherit it.

Insert after the `option(BUILD_CPP_EXAMPLES "Build CPP examples" ON)` line:

```cmake
# --- CAN backend selection (Jetson/aarch64 native SocketCAN) ---------------
# 1 = ZLG USB-CANFD SDK (x86 only); 2 = SocketCAN (Jetson native).
set(OMNIHAND_PRO_CAN_BACKEND "SOCKETCAN" CACHE STRING "CAN backend: SOCKETCAN or ZLG")
set_property(CACHE OMNIHAND_PRO_CAN_BACKEND PROPERTY STRINGS SOCKETCAN ZLG)
if(OMNIHAND_PRO_CAN_BACKEND STREQUAL "ZLG")
  set(CanfdDevice 1)
elseif(OMNIHAND_PRO_CAN_BACKEND STREQUAL "SOCKETCAN")
  set(CanfdDevice 2)
else()
  message(FATAL_ERROR "Unsupported OMNIHAND_PRO_CAN_BACKEND=${OMNIHAND_PRO_CAN_BACKEND}")
endif()
message(STATUS "OmniHand Pro CAN backend: ${OMNIHAND_PRO_CAN_BACKEND} (CanfdDevice=${CanfdDevice})")
```

---

## Patch 2 — `src/CMakeLists.txt`: inherit backend + guard x86 installs

**2a.** Remove the hardcoded device line near the top:

```diff
-#切换基于不同CAN盒和CAN卡的SDK还是socketCAN，1~ZLG周立功的usbcanfd的CAN盒SDK;2~socketCAN
-set(CanfdDevice 1)
+# CanfdDevice is set at the top-level CMakeLists.txt via OMNIHAND_PRO_CAN_BACKEND.
+# (1 = ZLG USB-CANFD x86 SDK, 2 = SocketCAN.)
```

> Leaving `set(CanfdDevice 1)` here would override the inherited value and force
> the x86 backend, so it must be deleted, not just ignored.

**2b.** Guard the two unconditional x86 library installs at the bottom:

```diff
-install(FILES ${PROJECT_SOURCE_DIR}/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/libusbcanfd.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
-install(FILES ${PROJECT_SOURCE_DIR}/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/libusbcanfd.so.1.0.10 DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
+if(CanfdDevice EQUAL 1)
+  install(FILES ${PROJECT_SOURCE_DIR}/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/libusbcanfd.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
+  install(FILES ${PROJECT_SOURCE_DIR}/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/libusbcanfd.so.1.0.10 DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
+endif()
```

**2c.** Add `src/` to the include path for BOTH backends. Upstream only adds it
inside the `if(CanfdDevice EQUAL 1)` (ZLG) branch via
`target_include_directories(... PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})`, so the
SocketCAN build fails to find `kinematics_solver/kinematics_solver.h`. Add it
unconditionally, right after the `set_target_properties(...)` call:

```diff
 set_target_properties(omniHandPro25Can PROPERTIES
   INSTALL_RPATH "$ORIGIN"
   BUILD_WITH_INSTALL_RPATH TRUE)
+
+target_include_directories(omniHandPro25Can PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
```

The existing `${CMAKE_CURRENT_SOURCE_DIR}` line inside the ZLG branch is now a
harmless duplicate and can stay.

---

## Patch 3 — `python/CMakeLists.txt`: don't copy x86 lib into the package

Split the POST_BUILD command so the core `.so` copy stays unconditional and the
x86 `libusbcanfd` copy + symlinks are guarded:

```diff
 add_custom_command(
   TARGET ${CUR_TARGET_NAME}
   POST_BUILD
-  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:omniHandPro25Can> ${PYTHON_PKG_DIR}/agibot_hand/
-  COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/libusbcanfd.so.1.0.10 ${PYTHON_PKG_DIR}/agibot_hand/
-  COMMAND ${CMAKE_COMMAND} -E create_symlink libusbcanfd.so.1.0.10 ${PYTHON_PKG_DIR}/agibot_hand/libusbcanfd.so.1
-  COMMAND ${CMAKE_COMMAND} -E create_symlink libusbcanfd.so.1.0.10 ${PYTHON_PKG_DIR}/agibot_hand/libusbcanfd.so
-)
+  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:omniHandPro25Can> ${PYTHON_PKG_DIR}/agibot_hand/)
+
+if(CanfdDevice EQUAL 1)
+  add_custom_command(
+    TARGET ${CUR_TARGET_NAME}
+    POST_BUILD
+    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/libusbcanfd.so.1.0.10 ${PYTHON_PKG_DIR}/agibot_hand/
+    COMMAND ${CMAKE_COMMAND} -E create_symlink libusbcanfd.so.1.0.10 ${PYTHON_PKG_DIR}/agibot_hand/libusbcanfd.so.1
+    COMMAND ${CMAKE_COMMAND} -E create_symlink libusbcanfd.so.1.0.10 ${PYTHON_PKG_DIR}/agibot_hand/libusbcanfd.so)
+endif()
```

`target_include_directories(... ${USER_PATH})` further up is harmless when
`USER_PATH` is empty under SocketCAN; no change required.

---

## Patch 4 — `src/can_bus_device/socket_can/c_can_bus_device_socket_can.cc`

Make the interface selectable (default `can0`), matching our bridge's
`OMNIHAND_SOCKETCAN_IFACE` convention.

**4a.** Add `<cstdlib>` to the includes (after `#include <cstring>`):

```diff
 #include <cstring>
+#include <cstdlib>
 #include <iostream>
```

**4b.** Replace the hardcoded interface (around lines 57-60):

```diff
-  /*指定can0设备，获取设备索引*/
-  struct ifreq ifr {};
-  strcpy(ifr.ifr_name, "can0");
-  ioctl(fd_sock_, SIOCGIFINDEX, &ifr);
+  /* Interface from OMNIHAND_SOCKETCAN_IFACE env var, default can0. */
+  struct ifreq ifr {};
+  const char* env_iface = std::getenv("OMNIHAND_SOCKETCAN_IFACE");
+  const char* iface = (env_iface && env_iface[0] != '\0') ? env_iface : "can0";
+  std::strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
+  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
+  ioctl(fd_sock_, SIOCGIFINDEX, &ifr);
```

---

## Patch 5 — same file: wait on the socket instead of spinning on it

`RecvFrame` read the **non-blocking** socket in a loop with nothing in between,
so an idle bus cost a full core per hand — measured at 100.0 % with `wchan=0`
while the bus carried 25 frames/s. The comment on that line already named the
constraint (a blocking read would keep the thread from being released at
shutdown); `poll()` with a timeout satisfies it without spinning.

**5a.** Add `<poll.h>` and the timeout constant to the includes:

```diff
 #include <net/if.h>
+#include <poll.h>
 #include <sys/ioctl.h>
 #include <sys/socket.h>
 #include <unistd.h>
+
+static constexpr int kRecvPollTimeoutMs = 20;
```

**5b.** Wait before reading, in `RecvFrame`:

```diff
   while (!IsInterruptRequested()) {
+    struct pollfd pfd {};
+    pfd.fd = fd_sock_;
+    pfd.events = POLLIN;
+    const int ready = poll(&pfd, 1, kRecvPollTimeoutMs);
+    if (ready <= 0) {
+      continue; /* timeout or interrupted: re-check the interrupt request */
+    }
+
     canfd_frame frame{};
-    int ret = read(fd_sock_, &frame, sizeof(frame));  // read接收阻塞式会导致线程无法释放
+    int ret = read(fd_sock_, &frame, sizeof(frame));
```

The socket stays non-blocking on purpose, so a spurious wakeup cannot become a
blocked read. The timeout bounds shutdown, not receive latency: a frame wakes the
poll immediately, and the measured request round trip is unchanged (2.23 ms mean
against 2.18 ms). Bridge process cost falls from 110 % of a core to 13 %.

Reported upstream — see `omnihand_vendor_socketcan_recv_report.md`.

---

## Build on Jetson

> **Build against Python 3.10, not the conda base env.** On this Jetson the
> `(base)` conda environment is Python 3.13, but ROS 2 Humble / `agx_arm_ctrl`
> run on system `python3.10`. A binding built for 3.13 will not import under ROS.
> Build outside conda (`conda deactivate`) and pin the interpreter explicitly.

```bash
cd vendor/OmniHand-Pro-2025
conda deactivate                      # use the system python3.10 that ROS uses
python3.10 -m pip install --upgrade build setuptools wheel pybind11
# python3.10-dev headers must be present: sudo apt install python3.10-dev

./build.sh \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PYTHON_BINDING=ON \
  -DBUILD_CPP_EXAMPLES=OFF \
  -DOMNIHAND_PRO_CAN_BACKEND=SOCKETCAN \
  -DPython3_EXECUTABLE=$(which python3.10)
```

### Acceptance checks

```bash
# package present
ls build/agibot_hand_pkg/agibot_hand/

# no x86 contamination — every runtime .so must be aarch64
find build/agibot_hand_pkg -name "*.so*" -exec file {} \;
# expect: ELF 64-bit LSB shared object, ARM aarch64 ; NO "x86-64"

# import works on the SAME python ROS uses
PYTHONPATH=build/agibot_hand_pkg python3.10 -c "from agibot_hand import AgibotHandO12, EFinger, EHandType, EControlMode; print('import ok')"
```

---

## Known build risk: the vendor `node/` subdirectory

The top-level `CMakeLists.txt` calls `add_subdirectory(node)` unconditionally.
That is the vendor's own ROS/aimrt node, which we do **not** use (our bridge owns
the hardware session). If it fails to configure/build on Jetson (missing aimrt or
ROS deps), add a guard rather than fighting its dependencies:

```diff
-add_subdirectory(node)
+option(BUILD_VENDOR_NODE "Build the vendor ROS node (unused by agx_arm_ctrl)" OFF)
+if(BUILD_VENDOR_NODE)
+  add_subdirectory(node)
+endif()
```

Only apply this fifth patch if the `node/` target actually breaks the build; the
Python binding (`python/`) and core lib (`src/`) are all `agx_arm_ctrl` needs.

---

## Next (after a green build)

Once `from agibot_hand import AgibotHandO12` works on the Jetson, we wire the
`O12ProSdkAdapter` in `agx_arm_ctrl` against the verified API and validate it
read-only first (`pro_hardware_probe.py`), then with low-rate sweeps. The
model-aware bridge (`hand_model:=o12_pro`) and the joint/gesture definitions are
already in place; only the SDK backend remains.
