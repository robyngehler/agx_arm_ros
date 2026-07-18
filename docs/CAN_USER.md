# can模块使用手册

脚本路径：`agx_arm_ros/scripts`

注意此处的can模块仅支持机械臂自带的can模块，不支持其它can模块

安装can工具

```shell
sudo apt update && sudo apt install can-utils ethtool
```

这两个工具用于配置 CAN 模块

## 0 当前原生工作流：`activate_native_can.sh`

当前 Duo 真机基线路径优先使用 Jetson 原生 `mttcan` side bus 上的 `scripts/activate_native_can.sh`。这也是 `docs/control/bringups/launches.md` 和当前 arm-plus-hand 运行时采用的路径。

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
sudo bash scripts/activate_native_can.sh
ip -details link show can_nero_right
```

它会应用当前已验证的 side-bus 映射：

- `can0 -> can_nero_right`
- `can1 -> can_nero_left`

`scripts/prepare_can_interfaces.py` 保留用于 USB CAN 或 CAN FD 适配器、fallback bringup 和 bench 场景。像 `can0`、`can_nero` 这样的旧公开运行时名称不应再作为 `can_port` 使用。

## 1 USB 角色化工作流：`prepare_can_interfaces.py`

当你走 USB 适配器路径时，推荐使用仓库内的角色化准备脚本：`scripts/prepare_can_interfaces.py`。

该脚本会读取 `config/can_interface_roles.json` 中的角色配置，并自动完成以下操作：

- 枚举当前 Linux CAN 接口和 USB `bus-info`
- 按角色解析目标接口（当前默认角色包含 `nero`、`effector`、`omnihand`）
- 配置 classic CAN / CAN FD 波特率
- 按配置重命名接口，例如 `can_nero_right`、`can_effector`、`can_omnihand`
- 设置 `restart-ms` 与 `txqueuelen`

建议在仓库根目录执行：

```bash
cd ~/agx_arm_ws/src/agx_arm_ros
python3 scripts/prepare_can_interfaces.py --list
python3 scripts/prepare_can_interfaces.py --roles nero --dry-run
python3 scripts/prepare_can_interfaces.py --roles nero
```

常见用法：

- 单独准备机械臂：`python3 scripts/prepare_can_interfaces.py --roles nero`
- 同时准备机械臂与 OmniHand：`python3 scripts/prepare_can_interfaces.py --roles nero,omnihand`
- 显式绑定某个 USB 口：`python3 scripts/prepare_can_interfaces.py --roles nero --nero-can-interface 3-1.4:1.0`
- 显式绑定当前 Linux 接口名：`python3 scripts/prepare_can_interfaces.py --roles nero --nero-can-interface can0`
- 在不改动仓库默认配置的前提下试验 OmniHand CAN FD 参数：`python3 scripts/prepare_can_interfaces.py --roles omnihand --dry-run --omnihand-can-interface can_omnihand --omnihand-bitrate 1000000 --omnihand-dbitrate 2000000 --omnihand-sample-point 0.75 --omnihand-dsample-point 0.75`

若需修改默认目标名、波特率、CAN FD 数据波特率或预绑定的 USB 口，请编辑 `config/can_interface_roles.json`。

如需做 Jetson `mttcan` 这类 bring-up 试验，可直接使用按角色覆盖的 `--<role>-bitrate`、`--<role>-dbitrate`、`--<role>-sample-point`、`--<role>-dsample-point` 参数。脚本仍会根据 `ip -details link show` 校验结果，若内核量化后的参数与请求值不一致会直接报错，避免把试验值误写成仓库基线。

如果执行脚本时出现 `ip: command not found`，请安装 `ip` 指令，一般是 `sudo apt-get install iproute2`。

以下章节保留旧的 USB `can_activate.sh` 手工流程以兼容既有方式。

## 2 寻找can模块

执行

```bash
bash find_all_can_port.sh
```

输入密码后，如果can模块插入了电脑，并被电脑检测到，输出类似如下：

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
```

如果有多个，输出类似如下：

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
Interface can1 is connected to USB port 3-1.1:1.0
```

有多少个can模块就会有多少行类似`Interface can1 is connected to USB port 3-1.1:1.0`的输出

其中`can1`是系统找到的can模块名字，`3-1.1:1.0`是该can模块所链接的usb端口

如果之前已经激活过can模块并其名为其它名字，这里假设名字为`can_piper`则输出如下

```bash
Both ethtool and can-utils are installed.
Interface can_piper is connected to USB port 3-1.4:1.0
Interface can0 is connected to USB port 3-1.1:1.0
```

如果没有检测到can模块，则只会输出如下：

```bash
Both ethtool and can-utils are installed.
```

## 3 激活单个can模块, **此处使用`can_activate.sh`脚本**

(1) 查看can模块插在usb端口的硬件地址。拔掉所有can模块，只将连接到机械臂的can模块插入PC，执行

```shell
bash find_all_can_port.sh
```

并记录下`USB port`的数值，例如`3-1.4:1.0`

(2) 激活can设备。假设上面的`USB port`数值为`3-1.4:1.0`，执行：

```bash
bash can_activate.sh can_piper 1000000 "3-1.4:1.0"
```

解释：**3-1.4:1.0硬件编码的usb端口插入的can设备，名字被重命名为can_piper，设定波特率为1000000，并激活**

(3) 检查是否激活成功

执行`ifconfig`查看是否有`can_piper`，如果有则can模块设置成功  

(4) 特别提示：

如果电脑只插入了一个can模块

直接执行

```bash
bash can_activate.sh can0 1000000
```

此处`can0`可以改为任意名字，`1000000`为波特率

