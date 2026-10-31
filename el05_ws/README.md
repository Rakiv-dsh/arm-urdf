# 机械臂 EL05 × ROS 2 Jazzy（WSL2）

RobStride EL05 准直驱电机 + USB_CANHUB（candleLight / gs_usb）在 **WSL2** 里跑通 ROS 2 控制的最小可用代码库。
基于两套上游仓库整理，并修掉了一个会让控制器「一使能就停机」的 bug。

- 上游参考：`12345qqqde/el05-ros2-motor-control`、`12345qqqde/robstride-can-ros2-notes`
- 本仓库：ROS 2 包 + 电机协议实现 + 自研工具（虚拟电机 / 只读探针 / 改 CAN ID / 接线拉起）+ 文档

## 目录

| 路径 | 内容 |
|---|---|
| `src/robstride_ros_sample` | 包 `rs_motor_ros2`：EL05 协议库 + 五轴控制器 + 单/五电机测试节点 |
| `src/el05_serial_tools` | 可选的串口监听工具（上游） |
| `tools/el05_vcan_sim.py` | **虚拟 EL05**：协议级应答 + 一阶跟随动力学 + CSV 轨迹，无硬件也能验证整条链路 |
| `tools/demo_virtual.sh` | 一条命令的虚拟演示（vcan + 5 台虚拟电机 + 控制器 + 话题采集） |
| `tools/el05_probe.py` | **只读**探测：某路 CAN 上有哪些 ID 在应答（不使能、电机不动） |
| `tools/el05_set_id.py` | 读/改 CAN ID（带「同 ID 多台」守卫） |
| `tools/can_sniff.py` | 裸监听：不过滤地打印总线上所有帧，用来判断「电气通不通」 |
| `tools/bringup_can.sh` / `setup_can0_wsl.sh` | WSL 内加载 gs_usb、把 can0~can4 拉到 1 Mbps |
| `docs/` | 电机手册 PDF、硬件清单、接线图、修复记录 |

## 环境

- WSL2 Ubuntu-24.04，ROS 2 **Jazzy**（`/opt/ros/jazzy`）
- USB-CANHUB（VID:PID = `1d50:606f`，candleLight/gs_usb），经 **usbipd-win** 过继进 WSL
- 电机：EL05，CAN 1 Mbps，私有协议（29 位扩展帧），主机 ID `0xFD`

## 编译

```bash
source /opt/ros/jazzy/setup.bash
cd ~/el05_serial_ws
colcon build --symlink-install
source install/setup.bash
```

## 快速开始（没有硬件）

```bash
sudo bash tools/demo_virtual.sh 8      # vcan + 5 台虚拟电机 + 五轴控制器，采集 /arm/joint_states
```

## 接真机

```bash
# 1) Windows（管理员，一次）：usbipd bind --busid <BUSID>
# 2) 每次插拔后：usbipd attach --wsl --busid <BUSID>
# 3) WSL 里拉起 CAN 并只读体检
sudo bash tools/bringup_can.sh
python3 tools/el05_probe.py --iface can0
# 4) 改 ID（确认该 ID 上只有一台在应答）
python3 tools/el05_set_id.py --iface can0 --list
python3 tools/el05_set_id.py --iface can0 --from 127 --to 1
# 5) 第一次动：极小幅度（±0.3°）
ros2 run rs_motor_ros2 el05_joint1_sine --confirm-hardware --duration 10 \
     --amplitude 0.005 --frequency 0.1 --speed 0.1
```

## ROS 接口（five_dof_arm_controller）

| 话题 | 类型 | 方向 |
|---|---|---|
| `/arm/enable` | `std_msgs/Bool` | 订阅，`true` 使能 |
| `/arm/sine_enable` | `std_msgs/Bool` | 订阅，`true` 开始正弦自走 |
| `/arm/joint_command` | `sensor_msgs/JointState` | 订阅，关节目标（joint1~joint5，rad） |
| `/arm/joint_states` | `sensor_msgs/JointState` | 发布，实测关节角 |

参数文件：`src/robstride_ros_sample/config/five_dof_controller.yaml`
（`mech_offset` / `joint_direction` / 限位 这几项**必须按自己这台机器重新标定**，仓库里的值来自上游那台臂。）

## 重要提醒

1. **上电/使能前先读安全须知**：第一次一律单电机、`--amplitude 0.005`，手放电源开关。
2. **改 CAN ID 前必须确认「同一 ID 上只有一台电机在应答」**：改 ID 帧是发给该 ID 的，同 ID 的多台会一起被改（本仓库 `el05_set_id.py` 已加守卫，检测到同 ID 多台会拒绝执行）。详见 `docs/FIXES.md`。
3. `build/` `install/` `log/` 已在 `.gitignore` 里，不要提交。
