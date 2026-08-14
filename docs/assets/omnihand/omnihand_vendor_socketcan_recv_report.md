# OmniHand Pro 2025 SDK — SocketCAN receive thread spins on an idle bus

status: REPORTED_UPSTREAM
last_updated: 2026-08-14
component: `src/can_bus_device/socket_can/c_can_bus_device_socket_can.cc`
affected version: `VERSION` 0.9.0, SocketCAN backend
platform observed: Jetson AGX Orin (aarch64), Ubuntu 22.04, kernel 5.15.185-tegra

This is the text we sent to Agibot, kept here so the repo records what was
reported, when, and on what evidence. The fix is carried in our fork
(`jetson-orin-socketcan`, commit `f43a0e9`) until an upstream release includes
one.

---

## Summary

`CanBusDeviceSocketCan::RecvFrame` reads from a **non-blocking** socket in a loop
with no wait between iterations. On an idle or lightly loaded bus this consumes a
full CPU core per open device, regardless of traffic.

Measured with two O12 Pro hands, each on its own SocketCAN FD interface carrying
**25 frames per second**: the receive thread of each device ran at **100.0 % of
one core**, and `/proc/<pid>/task/<tid>/wchan` read `0` — the thread was never
sleeping. Two hands cost two cores to move 50 frames a second.

## Where

`OpenDevice()` sets the socket non-blocking:

```cpp
int flags = fcntl(fd_sock_, F_GETFL, 0);
fcntl(fd_sock_, F_SETFL, flags | O_NONBLOCK);
```

`RecvFrame()` then polls it in a tight loop:

```cpp
void CanBusDeviceSocketCan::RecvFrame() {
  while (!IsInterruptRequested()) {
    canfd_frame frame{};
    int ret = read(fd_sock_, &frame, sizeof(frame));  // read接收阻塞式会导致线程无法释放
    if (ret > 0) {
      ...
```

With no data available, `read()` returns `-1`/`EAGAIN` immediately and the loop
runs again at once.

## Why it is written that way, and why that reason does not require a spin

The comment on the `read()` line is the design note: a *blocking* read would keep
the thread from being released, because `~CanBusDeviceSocketCan()` sets the
interrupt request and then joins the thread — a thread parked in a blocking read
would never observe the request.

That constraint is real, and `poll()` satisfies it. A bounded timeout lets the
thread return to the top of the loop and re-check `IsInterruptRequested()`, so
shutdown stays prompt, while an arriving frame wakes the thread immediately, so
receive latency is unaffected.

## Suggested change

```cpp
#include <poll.h>

static constexpr int kRecvPollTimeoutMs = 20;

void CanBusDeviceSocketCan::RecvFrame() {
  while (!IsInterruptRequested()) {
    struct pollfd pfd {};
    pfd.fd = fd_sock_;
    pfd.events = POLLIN;
    const int ready = poll(&pfd, 1, kRecvPollTimeoutMs);
    if (ready <= 0) {
      continue;  // timeout or interrupted: re-check the interrupt request
    }

    canfd_frame frame{};
    int ret = read(fd_sock_, &frame, sizeof(frame));
    ...
```

The socket deliberately stays non-blocking, so a spurious wakeup cannot turn into
a blocked read. The timeout bounds shutdown only; it is not a polling interval.

## Measured effect

Same hardware, same bring-up, before and after the change:

| | before | after |
| --- | --- | --- |
| receive thread | 100.0 % of a core, `wchan=0` | **0.2 %**, `wchan=do_sys_poll` |
| whole device process | 110.6 % of a core | **12.5 %** |
| `get_all_active_joint_angles` | 2.18 ms mean | 2.23 ms mean |
| `get_tactile_sensor_data` | 2.24 ms mean | 2.15 ms mean |
| CANFD timeouts | 0 | 0 |
| interface RX drops / misses / errors | 0 | 0 |

Request latency is unchanged, which was the property that had to hold. Both hands
were exercised through a `FollowJointTrajectory` action with readback-verified
delivery, four goals, all successful.

## Notes for the maintainers

- The same pattern would apply to any other backend that sets `O_NONBLOCK` and
  reads in a loop; we only measured the SocketCAN one.
- `SendRequestSynch` in `c_can_bus_device.cc` also busy-waits, on the calling
  thread, for up to 50 ms while matching a reply. That one is bounded by the
  request timeout and costs far less, but a condition variable signalled by the
  receive thread would remove it too, and would likely reduce the ~2.2 ms mean
  round trip we measure per request.
