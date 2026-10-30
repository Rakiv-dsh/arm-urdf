# EL05 ROS 2 电机控制

这是一个基于 ROS 2 Jazzy 的 RobStride EL05 电机控制工作区，通过 USB-CANHUB 的 SocketCAN 接口与电机通信。仓库包含电机协议库、五自由度控制器、单电机和五电机测试节点、CAN 初始化脚本及 EL05 参考手册。

## 硬件配置

- RobStride EL05，CAN ID 为 `1` 到 `5`
- CAN 主站 ID：`253`（`0xFD`）
- USB-CANHUB：Linux 接口 `can0`
- CAN 波特率：`1 Mbps`
- 位置单位：弧度（rad）

当前机械零点偏移（按 CAN ID 1~5，单位 rad）：

| CAN ID | MechOffset |
|---:|---:|
| 1 | 0.000000 |
| 2 | 0.265762 |
| 3 | 0.000000 |
| 4 | 4.244908 |
| 5 | 0.000000 |

五个电机共用 `can0`。启动硬件前，请确认接线、机械空间、运动方向、关节限位和急停装置均正常。

## 编译

需要安装 ROS 2 Jazzy 和 `colcon`：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/el05_serial_ws
colcon build --symlink-install
source install/setup.bash
```

重新连接 USB-CANHUB 后，先初始化 CAN：

```bash
~/el05_serial_ws/scripts/setup_can0.sh
```

## 电机测试

### 单电机测试（ID 1）

以当前位置为中心进行小幅正弦运动：

```bash
ros2 run rs_motor_ros2 el05_joint1_sine \
  --confirm-hardware --duration 10 \
  --amplitude 0.005 --frequency 0.1 --speed 0.1
```

### 五电机测试

每个电机以自己的实时位置为中心：

```bash
ros2 run rs_motor_ros2 el05_all_sine \
  --confirm-hardware --duration 60 \
  --amplitude 0.02 --frequency 0.1 --speed 0.1
```

### ±50°大幅运动

幅度超过 `0.05 rad` 时，必须增加 `--confirm-large-motion`。以下命令让五个电机同时进行 ±50°运动：

```bash
ros2 run rs_motor_ros2 el05_all_sine \
  --confirm-hardware --confirm-large-motion \
  --duration 60 --amplitude 0.872665 \
  --frequency 0.05 --speed 0.2
```

`0.872665 rad` 表示以当前位置为中心的正负 50°。如果需要总行程 50°，请使用 `0.436332`。

五电机也支持单独设置 2 号电机幅度。例如让 1、3、4、5 号为 40°，2 号为 20°：

```bash
ros2 run rs_motor_ros2 el05_all_sine \
  --confirm-hardware --confirm-large-motion --hold-enabled \
  --duration 60 --amplitude 0.698132 --amplitude-id2 0.349066 \
  --frequency 0.05 --speed 0.2
```

`--speed-id2` 可单独设置 2 号电机速度，单位为 `rad/s`；50 rpm 等于 `5.235988 rad/s`。例如五个电机均 ±30°，2 号电机速度上限为 50 rpm：

```bash
ros2 run rs_motor_ros2 el05_all_sine \
  --confirm-hardware --confirm-large-motion --hold-enabled \
  --duration 60 --amplitude 0.523599 --frequency 0.05 \
  --speed 0.2 --speed-id2 5.235988
```

2 号电机的极限测试（±90°）需要额外的 `--confirm-extreme-motion`。例如其他电机 ±30°、2 号电机 ±90°，并降低 2 号速度：

```bash
ros2 run rs_motor_ros2 el05_all_sine \
  --confirm-hardware --confirm-large-motion --confirm-extreme-motion \
  --monitor-id2-current --hold-enabled --duration 60 \
  --amplitude 0.523599 --amplitude-id2 1.570796 \
  --frequency 0.02 --speed 0.2 --speed-id2 0.05
```

### 跳过指定电机

使用 `--skip-id` 可让指定 ID 保持不动。例如跳过 2 号电机：

```bash
ros2 run rs_motor_ros2 el05_all_sine \
  --confirm-hardware --confirm-large-motion --skip-id 2 \
  --duration 60 --amplitude 0.698132 \
  --frequency 0.05 --speed 0.2
```

按 `Ctrl+C` 可停止。程序在正常退出、中断或控制异常时，会对已创建的电机发送停止和失能命令。

### 监听 2 号电机电流

以下节点只读取 CAN ID 2 的 `0x701A iqf`，即滤波 q 轴电流（A）；不会使能、运动或停止电机：

```bash
ros2 run rs_motor_ros2 el05_joint2_current_monitor --duration 60 --rate 10
```

请不要在位置控制节点运行时同时启动该监听器，以免多个程序接收同一条 CAN 回包。

如果需要在五电机运动时同步监听 2 号电机电流，直接给运动节点增加同进程监听开关：

```bash
ros2 run rs_motor_ros2 el05_all_sine \
  --confirm-hardware --monitor-id2-current \
  --duration 60 --amplitude 0.02 --frequency 0.05 --speed 0.1
```

终端会每约 0.1 秒输出一次 `motor2 iqf`（单位 A）。

如果需要运动结束后仍保持电机上电，可增加 `--hold-enabled`。轨迹完成后程序会持续发送当前位置，电机保持使能但不再运动；按 `Ctrl+C` 才会失能：

```bash
ros2 run rs_motor_ros2 el05_all_sine \
  --confirm-hardware --hold-enabled --duration 10 \
  --amplitude 0.02 --frequency 0.1 --speed 0.1
```

## 目录结构

- `src/robstride_ros_sample`：`rs_motor_ros2` C++ 包和电机协议实现
- `src/el05_serial_tools`：可选的串口监听工具
- `scripts/setup_can0.sh`：SocketCAN 初始化脚本
- `docs/EL05使用说明书2600428.pdf`：EL05 硬件参考手册

`build/`、`install/` 和 `log/` 是本地生成目录，已通过 `.gitignore` 排除，不会提交到仓库。
