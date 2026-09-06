# Jetson Reset (on-device)

How to bring an AGX Orin demo unit back to a clean, fast state **from the unit
itself** — no host PC, no recovery cable, no display swap. Written for the
bottom unit, which was configured first and carries the most drift.

## What this can and cannot do

A true factory flash of the AGX Orin devkit writes the internal eMMC over USB-C
in recovery mode, and that needs a second machine. Nothing on the device can
replace it.

What the device *can* do to itself is everything short of that: reclaim the
disk, remove the desktop and background services a demo unit does not need,
reset the user account, and reinstall the L4T and ROS packages. On a unit whose
problem is accumulated state rather than a corrupt image — which is the usual
case — the result is indistinguishable from a fresh install.

Reach for a host-PC flash only if the bootloader is broken, the rootfs will not
mount, or a JetPack *major* upgrade is wanted.

## Before you start

- The unit is powered from mains, not mid-demo. Every step below is
  interruptible except the `apt` runs.
- **Arms de-energised.** A reboot with arms powered leaves them on their last
  setpoint until the driver comes back.
- Back up anything not in git:
  ```bash
  ls ~/agx_arm_trajectories        # taught recordings and anchors — NOT in the repo
  ```
  Copy that directory somewhere off the unit first. Losing it costs a teaching
  session per recording.

## Level 1 — reclaim the disk (minutes, no reboot)

The bottom unit runs its rootfs on internal eMMC with no NVMe, so free space is
also write throughput: ext4 slows measurably above ~85 % full.

```bash
df -h /                                   # note the starting point

sudo apt clean                            # ~950 MB of cached .deb
sudo apt autoremove --purge               # old kernels and orphaned deps
sudo rm -rf /var/crash/*                  # ~340 MB of crash dumps
sudo journalctl --vacuum-size=100M

# snap keeps old revisions mounted as loop devices
snap list --all | awk '/disabled/{print $1, $3}' |
  while read -r name rev; do sudo snap remove "$name" --revision="$rev"; done

rm -rf ~/.npm/_cacache ~/.cache/pip
```

Repo-side, safe to delete at any time (`colcon` rebuilds them):

```bash
cd ~/workspace/agx_arm_ros && rm -rf build install log
bash ./scripts/colcon_build_system_python.sh
```

Verify with `df -h /`. Aim for 70 % or below.

## Level 2 — take the desktop off a demo unit (one reboot)

A demo unit does not need a graphical session. `gdm`, `gnome-shell`, `Xorg` and
the tracker indexer were measurably active during a failing run.

```bash
sudo systemctl set-default multi-user.target   # boot to console
sudo systemctl disable --now gdm3
```

Reach the unit over SSH afterwards. To get the desktop back:
`sudo systemctl set-default graphical.target && sudo reboot`.

Background updaters fire on randomized timers and do network plus disk work
without warning — `apt-daily` took 41 s at the last boot:

```bash
sudo systemctl disable --now \
  apt-daily.timer apt-daily-upgrade.timer \
  motd-news.timer update-notifier-download.timer fwupd-refresh.timer \
  packagekit.service
```

Services a headless arm controller does not use:

```bash
sudo systemctl disable --now \
  bluetooth ModemManager avahi-daemon cups-browsed rpcbind \
  colord switcheroo-control kerneloops
```

Docker is failing on this unit and nothing in the stack uses it:

```bash
sudo systemctl disable --now docker.service docker.socket containerd
```

## Level 3 — fix what is actually misconfigured

Two settings on this unit are wrong rather than merely heavy.

**The journal is volatile.** It lives in `/run/log/journal` (RAM), so every
reboot erases it — which is why the demo evening of 2026-09-05 left no system
log to read afterwards.

```bash
sudo mkdir -p /var/log/journal
sudo systemd-tmpfiles --create --prefix /var/log/journal
printf '[Journal]\nStorage=persistent\nSystemMaxUse=500M\n' |
  sudo tee /etc/systemd/journald.conf.d/persistent.conf
sudo systemctl restart systemd-journald
journalctl --disk-usage      # should now name a path under /var/log
```

**The clock is not synchronized.** `timedatectl` reports
`System clock synchronized: no` and `NTP service: inactive`, and `rtc1` reads
1970-01-01. Timestamps across the two units therefore do not compare, and boot
times in logs are unreliable.

```bash
sudo timedatectl set-ntp true
timedatectl                  # expect: System clock synchronized: yes
```

## Level 4 — reinstall the software without reflashing

Reinstall every NVIDIA L4T and ROS package while keeping the bootloader:

```bash
sudo apt update
sudo apt install --reinstall 'nvidia-l4t-*'
sudo apt install --reinstall 'ros-humble-*'
sudo apt full-upgrade
sudo reboot
```

Then rebuild the workspace from a clean tree:

```bash
cd ~/workspace/agx_arm_ros
git status                                  # confirm nothing uncommitted is lost
rm -rf build install log
bash ./scripts/colcon_build_system_python.sh
```

## Level 5 — reset the user account

The heaviest on-device step, for when configuration drift is suspected but not
located. Do it from a *second* account so the target home is not in use:

```bash
sudo adduser resetadmin && sudo usermod -aG sudo resetadmin
# log out, log in as resetadmin
sudo mv /home/user /home/user.old
sudo mkdir /home/user && sudo chown user:user /home/user
sudo cp -a /etc/skel/. /home/user/
```

Then log back in as `user`, restore `~/agx_arm_trajectories` from the backup,
re-clone the workspace, and delete `/home/user.old` once the stack runs. This
resets 6.2 GB of miniforge, 800 MB of VS Code server state and every dotfile —
so plan on redoing the Conda environment from
[environment.md](environment.md).

## Before the next demo

Independent of any reset, on every unit:

```bash
./scripts/jetson_presentation_mode.sh on   # no sleep, no radio parking, MAXN
./scripts/jetson_clock_boost.sh on         # pin clocks — presentation mode does not
```

Presentation mode deliberately leaves the CPU idle states alone, so without the
boost the first motion after an idle period starts on a governor still ramping
up. Check it took:

```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor   # performance
df -h /                                                     # below 85 %
timedatectl | grep synchronized                             # yes
```

## Comparing the two units

Where a unit is slower than its twin, compare rather than guess. Run this on
both and diff the output:

```bash
{ cat /etc/nv_tegra_release
  uname -r
  df -hT /
  lsblk -d -o NAME,SIZE,TYPE,MODEL
  nvpmodel -q
  cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
  systemctl get-default
  systemctl list-units --type=service --state=running --no-legend | awk '{print $1}' | sort
  timedatectl | grep -E "synchronized|NTP"
} > "/tmp/unit-$(hostname).txt"
```

Known state of the bottom unit as of 2026-09-06: L4T R36.2.0 (JetPack 6.0
Developer Preview, Dec 2023), rootfs on eMMC at 89 % full, no NVMe, graphical
target, volatile journal, NTP off. If the top unit differs on the L4T release or
boots from NVMe, that difference outranks everything else in this document.
