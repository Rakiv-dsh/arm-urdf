# 代码备份说明（两套系统都在这里）

这个仓库同时保存了**两套**控制方案。它们是并列的，选一套用即可，**不要同时跑**——两边都会往同一批电机写目标。

---

## 一、旧方案：ROS 2 那套（滑块面板 + C++ 伺服节点）

| 路径 | 内容 |
|---|---|
| `src/arm_urdf/` | 滑块面板、标定工具、运动学/动力学工具、CAN 桥 |
| `src/arm_real/` | 真机模型（URDF + STL）、零位、分析报告 |
| `el05_driver/` | C++ 电机驱动包 `rs_motor_ros2`（含 `el05_slider_servo` 等节点） |

### 启动

```bash
mkdir -p ~/ros2_arm_ws/src && cp -r src/* ~/ros2_arm_ws/src/
cd ~/ros2_arm_ws && colcon build --symlink-install

mkdir -p ~/el05_serial_ws/src && cp -r el05_driver ~/el05_serial_ws/src/robstride_ros_sample
cd ~/el05_serial_ws && colcon build

source /opt/ros/jazzy/setup.bash
source ~/ros2_arm_ws/install/setup.bash
ros2 launch arm_real display.launch.py gui:=false fallback_slider:=true
```

### 关键文件

| 文件 | 作用 |
|---|---|
| `src/arm_urdf/scripts/joint_slider.py` | Qt 滑块面板（直连 CAN，含 ENABLE / STOP） |
| `src/arm_urdf/scripts/estop_button.py` | 独立急停小窗 |
| `src/arm_urdf/tools/can_bridge.py` | 新方案用的 TCP 桥（旧方案不需要） |
| `src/arm_urdf/tools/calibrate.py` | 看位置、设零位、点动 |
| `src/arm_urdf/tools/kinematics.py` | 正逆运动学 |
| `src/arm_urdf/tools/dynamics.py` | 刚体动力学 + 仿真 |
| `src/arm_urdf/config/joint_limits.yaml` | **唯一真源**：电机号/方向/零位/限位/速度 |
| `src/arm_real/urdf/arm_real.urdf` | 真机模型（零位已烘） |
| `el05_driver/src/el05_slider_servo.cpp` | C++ 伺服节点（读 yaml、连续关节环绕、看门狗） |
| `el05_driver/src/el05_state_pub.cpp` | 电机状态 → `/joint_states` |

---

## 二、新方案：hand-host 上位机（PySide6 + MuJoCo）

| 路径 | 内容 |
|---|---|
| `hand_host_app/` | 上位机程序本体（已裁剪成机械臂版，4.8 MB） |
| `src/arm_urdf/hand_host/arm_el05/` | 我们的「机型包」：manifest / 标定 / 驱动 / MJCF 模型 |
| `src/arm_urdf/hand_host/docs/` | 接入说明 |
| `src/arm_urdf/tools/can_bridge.py` | WSL 侧 CAN<->TCP 桥 |

上位机是「空壳 + 机型包」结构：核心代码里没有一行 EL05 或 SocketCAN，换一台机械臂只要换一个包。

### 启动

```bash
# 0. Windows 侧装依赖（一次）
pip install PySide6 mujoco

# 1. 放到纯英文路径（MuJoCo 打不开中文路径）
xcopy /E /I hand_host_app C:\Users\你的用户名\hand-host_arm
#    再把 src/arm_urdf/hand_host/arm_el05 整个目录放到 hand-host_arm\app\resources\hands\arm_el05

# 2. WSL 里起桥
python3 ~/ros2_arm_ws/src/arm_urdf/tools/can_bridge.py --port 5599            # 真机
python3 ~/ros2_arm_ws/src/arm_urdf/tools/can_bridge.py --port 5599 --dry-run  # 假数据

# 3. Windows 上起界面
cd C:\Users\你的用户名\hand-host_arm
python main.py
```

界面：点「连接」→「使能全部电机」→ 拖滑块。侧栏只有 6 页：连接管理 / 关节控制 / 电机标定 / 数据记录 / 动作序列 / 机型管理。

### 对模板做的改动（原模板没有）

1. MJCF 里所有 geom 加 `group="1"` —— 模板 3D 控件只渲染 group 1，放默认组会全黑；
2. `app/main_window.py` 加全局 QSS 深色文字 —— Windows 深色主题下控件文字是白色，白底上看不见；
3. 裁掉「触觉可视化」「仿真遥操作」，原「预设手势」改名「动作序列」，「手型管理」改名「机型管理」；
4. 连接页在无触觉机型下隐藏触觉区块，bridge 协议显示默认桥地址。

### 备份里没放什么

`hand_host_app/` 不含模板自带的 3 个示例手型（hand1/2/3，18 MB）和 `hand_integration_kits/`（19 MB），它们和机械臂无关。

---

## 三、两套的关系

```
                   ┌─ 旧：ROS 滑块面板 ──┐
电机 CAN (WSL) ────┤                     ├── 二选一
                   └─ 新：can_bridge ────┴─ hand-host 界面
```

- **共用**：`joint_limits.yaml`（电机号/零位/限位）、`arm_real` 模型、CAN 链路；
- **不要同时开**：两边都写 `0x7016` 目标位置，会打架；
- 新的桥内部用的就是旧方案里修好的那套帧格式（参数帧 2 字节填充、自动找总线、使能序列、停机广播）。

---

## 四、硬件前提（两套都一样）

1. 5 个 candleLight USB-CAN 插到 Windows，用 `usbipd attach` 转发进 WSL；
2. WSL 里出现 `canX`，`can-auto-up.service` 自动设成 1 Mbps 并 up；
3. 电机总线号每次插拔都会变（can0~can4 都出现过），两套都做了自动探测；
4. 电机电压低于 12 V 会触发欠压故障并锁死，只能断电恢复。
