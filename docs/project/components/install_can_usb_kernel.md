# Building `peak_usb` for Jetson AGX (5.15.148-tegra)

The stock Jetson kernel only ships `mttcan` + generic CAN core modules — no `peak_usb`. Build it out-of-tree.

## 1. Kernel headers
```bash
sudo apt install nvidia-l4t-kernel-headers
ls -d /usr/src/linux-headers-$(uname -r)*
ls -l /lib/modules/$(uname -r)/build   # symlink should point at the headers dir above
```
If the `build` symlink is missing, create it manually pointing at the headers dir.

## 2. Get the driver source (mainline)
```bash
cd ~
wget https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.15.148.tar.xz
tar xf linux-5.15.148.tar.xz linux-5.15.148/drivers/net/can/usb/peak_usb
cd linux-5.15.148/drivers/net/can/usb/peak_usb
```

## 3. Build out-of-tree
Edit the `Makefile` in that directory — change the `obj-$(CONFIG_CAN_PEAK_USB)` line to:
```makefile
obj-m := peak_usb.o
```
(leave the `peak_usb-y := ...` source list line as-is), then build:
```bash
make -C /lib/modules/$(uname -r)/build M=$(pwd) modules
```

## 4. Install and load
```bash
sudo mkdir -p /lib/modules/$(uname -r)/updates/drivers/net/can/usb/peak_usb
sudo cp peak_usb.ko /lib/modules/$(uname -r)/updates/drivers/net/can/usb/peak_usb/
sudo depmod -a

sudo modprobe can-dev
sudo modprobe peak_usb
```

## 5. Verify + bring up
```bash
lsusb              # PEAK-System Technik, vendor 0c72
dmesg | tail -30   # peak_usb bind + canX creation
ip link show

sudo apt install can-utils
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
candump can0
```

> Note: not part of the NVIDIA package, so it disappears on kernel/L4T upgrades. Consider DKMS if rebuilding repeatedly.