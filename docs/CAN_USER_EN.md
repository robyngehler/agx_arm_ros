# CAN Module User Manual

Script Path: `agx_arm_ros/scripts`

**Note**: The CAN module here only supports the CAN module that comes with the robotic arm; other CAN modules are not supported.

Install CAN Tools

```shell
sudo apt update && sudo apt install can-utils ethtool
```

These two tools are used to configure the CAN module.

## 0 Recommended workflow: `prepare_can_interfaces.py`

Prefer the repo-owned role-based preparation script: `scripts/prepare_can_interfaces.py`.

It reads `config/can_interface_roles.json` and automatically handles:

- discovery of Linux CAN interfaces and USB `bus-info`
- role resolution for the current repo roles (`nero`, `effector`, `omnihand`)
- classic CAN or CAN FD bitrate configuration
- interface renaming such as `can_nero`, `can_effector`, and `can_omnihand`
- `restart-ms` and `txqueuelen` setup

Run it from the repository root:

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
python3 scripts/prepare_can_interfaces.py --list
python3 scripts/prepare_can_interfaces.py --roles nero --dry-run
python3 scripts/prepare_can_interfaces.py --roles nero
```

Common examples:

- Prepare only the arm: `python3 scripts/prepare_can_interfaces.py --roles nero`
- Prepare the arm and OmniHand together: `python3 scripts/prepare_can_interfaces.py --roles nero,omnihand`
- Pin a role to a USB bus-info: `python3 scripts/prepare_can_interfaces.py --roles nero --nero-can-interface 3-1.4:1.0`
- Pin a role to the current Linux interface name: `python3 scripts/prepare_can_interfaces.py --roles nero --nero-can-interface can0`
- Try an OmniHand CAN FD candidate without changing the repo baseline: `python3 scripts/prepare_can_interfaces.py --roles omnihand --dry-run --omnihand-can-interface can_omnihand --omnihand-bitrate 1000000 --omnihand-dbitrate 2000000 --omnihand-sample-point 0.75 --omnihand-dsample-point 0.75`

Edit `config/can_interface_roles.json` if you want to change the default target names, bitrates, CAN FD data bitrate, or pre-bound USB bus-info values.

Role-specific `--<role>-bitrate`, `--<role>-dbitrate`, `--<role>-sample-point`, and `--<role>-dsample-point` overrides are intended for bring-up experiments such as Jetson `mttcan` CAN FD tuning. The script still verifies the applied settings against `ip -details link show`, so a quantized or rejected rate will fail fast instead of silently changing the repo baseline.

If you see `ip: command not found` when executing the script, install the `ip` command, typically with:

`sudo apt-get install iproute2`

The legacy `can_activate.sh` procedure remains below for compatibility with older manual USB workflows. For the native arms and OmniHand use `scripts/activate_native_can.sh`.

## 1 Find CAN Modules

Run:

```bash
bash find_all_can_port.sh
```

After entering your password, if the CAN module is plugged into the computer and detected, the output will look like this:

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
```

If multiple CAN modules are present, the output will look like this:

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
Interface can1 is connected to USB port 3-1.1:1.0
```

There will be one line like `Interface can1 is connected to USB port 3-1.1:1.0` for each CAN module detected.

- `can1` is the name of the CAN module found by the system.
- `3-1.1:1.0` is the USB port the CAN module is connected to.

If a CAN module was previously activated with a different name (e.g., `can_piper`), the output will be:

```bash
Both ethtool and can-utils are installed.
Interface can_piper is connected to USB port 3-1.4:1.0
Interface can0 is connected to USB port 3-1.1:1.0
```

If no CAN module is detected, only the following will be printed:

```bash
Both ethtool and can-utils are installed.
```

## 2  Activate a Single CAN Module (using `can_activate.sh`)

### (1) Find the USB port hardware address of the CAN module

Unplug all CAN modules, then plug only the CAN module connected to the robotic arm into the PC. Run:

```shell
bash find_all_can_port.sh
```

Record the `USB port` value, e.g., `3-1.4:1.0`.

### (2) Activate the CAN device

Assuming the `USB port` value is `3-1.4:1.0`, run:

```bash
bash can_activate.sh can_piper 1000000 "3-1.4:1.0"
```

The CAN device plugged into the USB port with hardware ID `3-1.4:1.0` is renamed to `can_piper`, set to a baud rate of 1,000,000, and activated.

### (3) Verify activation

Run `ifconfig` and check if `can_piper` appears. If it does, the CAN module is set up successfully.

### (4) Special note

If only one CAN module is plugged into the computer, you can run directly:

```bash
bash can_activate.sh can0 1000000
```

Here, `can0` can be replaced with any name, and `1000000` is the baud rate.

