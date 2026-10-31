# 机械臂 ROS 2 单仓库（ARM + EL05）

两套工作区放在一个仓库里：

| 目录 | 内容 | 对应本地路径 |
|---|---|---|
| `arm_ws/` | 机械臂 URDF、RViz 滑块、限位单一数据源、零位/限位标定工具 | `~/ros2_arm_ws` |
| `el05_ws/` | EL05 电机协议库、五轴控制器、滑块伺服、虚拟电机、只读探针、改 CAN ID | `~/el05_serial_ws` |

## 编译

两个工作区**各自独立**编译（各自有 src/ 和 colcon）：

```bash
source /opt/ros/jazzy/setup.bash

cd ~/el05_serial_ws && colcon build --symlink-install && source install/setup.bash
cd ~/ros2_arm_ws  && colcon build --symlink-install && source install/setup.bash
```

## 跑起来（滑块控制真机）

```bash
source /opt/ros/jazzy/setup.bash
source ~/el05_serial_ws/install/setup.bash
source ~/ros2_arm_ws/install/setup.bash
ros2 launch arm_urdf slider_servo.launch.py max_speed:=0.05 limit_cur:=6.0
# 另一个终端
ros2 topic pub --once /arm/enable std_msgs/msg/Bool "{data: true}"
```

## 硬件与协议要点

- 电机：RobStride EL05 ×5，CAN ID **21~25**，主机 ID **0xFD**，1 Mbps，私有协议（29 位扩展帧）
- 适配器：USB_CANHUB（candleLight / gs_usb），经 usbipd-win 过继进 WSL；每路 3P 端子 H/L/GND，120 Ω 拨码
- 零点：已写进电机（0x7019 在零位读 0），ROS 侧 `mech_offset` 全 0 → 关节角 = 电机读数
- 限位：`arm_ws/src/arm_urdf/config/joint_limits.yaml` 是唯一数据源，`sync_limits.py` 同时改 URDF 与控制器参数

## 目录速查

```
arm_ws/src/arm_urdf/
  urdf/            URDF（关节名 joint1..joint6）
  config/          joint_limits.yaml（限位/电机映射的唯一数据源）
  launch/          display.launch.py（可视化+滑块）
                   slider_servo.launch.py（滑块驱动真机）
  tools/           sync_limits.py  calibrate.py  set_zero_here.py  zero_safe.py
el05_ws/
  src/robstride_ros_sample/   rs_motor_ros2：协议库 + 控制器 + slider_servo
  tools/                      el05_vcan_sim.py  el05_probe.py  el05_set_id.py
                              demo_virtual.sh  bringup_can.sh  can_sniff.py
```

## 安全提醒

- 使能前必须有人扶住或垫住机械臂：任何一台报故障，节点会**全部失能**，臂会掉。
- 第一次试一根轴、幅度 0.05 rad、速度 0.05 rad/s，电流限幅 6 A。
- 电源必须是恒压模式（CV）；之前 CC/限流状态下出现过母线跌落触发欠压故障（故障位 bit2，锁存，需断电清除）。
