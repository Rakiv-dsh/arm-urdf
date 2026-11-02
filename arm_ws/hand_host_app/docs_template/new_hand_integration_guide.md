# 新灵巧手完整适配包制作指南

本文面向新手型的硬件、驱动、算法和上位机接入开发者。目标是让开发者准备出一个类似 `hand_integration_kits/hand2` 的完整接入材料包，然后通过上位机的“手型管理 -> 添加新手型”向导导入，最终在上位机中使用连接管理、关节控制、电机标定、触觉可视化、3D 触觉显示、仿真遥操作、数据记录、预设手势和可选力控等功能。

## 1. 接入成果和工作边界

上位机使用两层结构：

- 接入材料包，放在 `hand_integration_kits/<hand_id>/`，供向导读取和复制。`hand2` 就是这种包。
- 已安装手型包，由向导生成到 `app/resources/hands/<hand_id>/`。运行时只读取这里的 `manifest.json`、模型、驱动和适配器。

向导能做的事：

- 读取接入材料包里的 `kit.json`。
- 预填手型 ID、显示名、模型路径、关节表、协议、触觉布局、遥操作配置。
- 复制模型、驱动目录、触觉适配器目录、遥操作适配器目录和可选标定文件。
- 生成运行时 `manifest.json`、`calibration.json` 和内置手势。
- 检查 Python 适配器是否能被导入，检查必要方法是否存在。

向导不能替你完成的事：

- 编写真实电机通信协议。
- 解析真实触觉传感器字节流。
- 建立 MuJoCo 模型、执行器、关节和 mesh。
- 设计遥操作映射算法。
- 确认电流、力矩、位置、速度等硬件遥测单位。
- 在真实硬件上确认机械限位和安全动作。

因此，完整接入的前提是你已经准备好底层适配代码和模型资源。向导负责把这些材料整理成上位机能识别的手型包。

## 2. 推荐目录结构

建议为每个新手准备如下目录：

```text
hand_integration_kits/<hand_id>/
  kit.json
  model/
    hand.xml
    meshes/
      *.STL
      *.obj
  driver_adapter/
    driver.py
    其他驱动依赖文件.py
    厂商 SDK 包装文件.py
  tactile_adapter/
    tactile_adapter.py
    其他触觉解析依赖文件.py
  teleop_adapter/
    teleop_adapter.py
    其他遥操作算法文件.py
    teleop_config.json
    teleop_pair_calibration.json
    motor_to_sim_calibration.json
  gestures/
    可选说明文件或手势素材
```

最少接入只需要：

- `kit.json`
- `model/hand.xml`
- `driver_adapter/driver.py`

完整功能建议准备：

- `model/hand.xml` 和 mesh
- `driver_adapter/driver.py`
- `tactile_adapter/tactile_adapter.py`
- `teleop_adapter/teleop_adapter.py`
- 遥操作和电机到仿真的标定 JSON
- 内置手势
- 关节标定参数

## 3. 适配包准备清单

在打开上位机向导之前，先完成下面的清单。

### 3.1 手型和关节信息

准备一张关节表，每一行代表一个可控关节：

| 字段 | 必填 | 说明 |
| --- | --- | --- |
| `finger.name` | 是 | 手指内部 ID，例如 `index`、`thumb`、`finger_1` |
| `finger.display_name` | 是 | 上位机显示名，例如 `食指` |
| `joint.name` | 是 | 关节内部 ID，例如 `if_mcp` |
| `joint.display_name` | 是 | 关节显示名，例如 `MCP 屈伸` |
| `joint.motor_id` | 是 | 真实电机 ID，必须为正整数且全手唯一 |
| `joint.min_angle` | 是 | 最小角度，单位为度 |
| `joint.max_angle` | 是 | 最大角度，单位为度 |
| `joint.default_angle` | 是 | 默认角度，单位为度 |
| `joint.torque_limit` | 可选 | 力控阈值参考值，默认 150 |
| `joint.actuator_name` | 强烈建议 | MuJoCo actuator 名称 |

关键约定：

- 上位机内部统一使用“度”作为关节角单位。
- `set_all_joint_angles()` 收到的列表顺序与 `fingers[].joints[]` 的展开顺序一致。
- `motor_id` 只用于真实硬件驱动和遥测映射。
- `actuator_name` 用于 MuJoCo 仿真和 3D 显示。如果缺省，上位机会退回使用 `joint.name`。

### 3.2 MuJoCo 模型

模型目录至少包含一个 XML：

```text
model/
  hand.xml
  meshes/
    base_link.STL
    index_link1.STL
```

模型需要满足：

- XML 可以被 `mujoco.MjModel.from_xml_path()` 加载。
- mesh 相对路径在复制整个 `model/` 目录后仍然有效。
- `<actuator>` 下的 actuator 名称与关节表里的 `actuator_name` 一一对应。
- actuator 控制范围合理，单位通常是弧度。上位机会把角度转弧度后写入 MuJoCo 控制量。
- 如果要使用 3D 触觉上色，XML 中需要声明触觉 taxel site。

触觉 site 命名约定：

```text
tx_<site_prefix>_r<row>_c<col>
```

例如触觉区域 `if_tip` 有 6 行 7 列，默认 `site_prefix` 为 `if_tip`，则模型里应有：

```text
tx_if_tip_r0_c0
tx_if_tip_r0_c1
...
tx_if_tip_r5_c6
```

如果模型里的 site 前缀和触觉区域 ID 不一致，可以在 `kit.json` 的触觉 patch 中填写 `site_prefix`。

支持的触觉显示变换：

- `identity`
- `flip_horizontal`
- `flip_vertical`
- `rotate_180`

`display_transform` 影响 2D 热力图显示，`site_transform` 影响 3D taxel site 对应关系。

### 3.3 电机驱动适配器

电机适配器入口必须命名为 `driver.py`。可以放在 `driver_adapter/driver.py`，同目录下可以放额外 Python 文件、厂商 SDK 包装层、标定文件等。

`driver.py` 必须暴露以下入口之一：

```python
def create_driver(profile):
    return HandDriver(profile)
```

或：

```python
class HandDriver:
    ...
```

必须实现的方法：

```python
class HandDriver:
    def connect(self, port: str, baudrate: int, protocol: str | None = None) -> bool:
        ...

    def disconnect(self) -> None:
        ...

    def enable_all(self, enable: bool = True) -> None:
        ...

    def set_motor_angle(self, motor_id: int, angle_deg: float, speed: float = 0.5) -> None:
        ...

    def set_angles(self, angles_dict: dict[int, float]) -> None:
        ...

    def set_all_joint_angles(self, angles_list: list[float]) -> None:
        ...

    def home_all_motors(
        self,
        target_angles: dict[int, float] | None = None,
        speed: float = 0.3,
    ) -> None:
        ...

    def get_latest_telemetry(self) -> dict[int, dict]:
        ...

    def get_status(self, motor_id: int) -> dict:
        ...

    def cleanup(self) -> None:
        ...
```

建议额外实现的方法：

```python
def set_zero_point(self, motor_id: int) -> None:
    ...

def set_max_current(self, motor_id: int, value_q15: int) -> None:
    ...

def find_motor_limits(self, motor_id: int, start_angle: float, config: dict) -> dict:
    return {"min_angle": -30.0, "max_angle": 90.0}
```

或：

```python
def auto_find_limits(self, motor_id: int, start_angle: float, config: dict) -> dict:
    return {"min": -30.0, "max": 90.0}
```

遥测建议字段：

```python
{
    1: {
        "position": 0.0,      # 当前角度，单位为度
        "velocity": 0.0,      # 可选
        "current": 0.0,       # 力控和自动寻限会读取，建议单位保持 mA 或在文档中固定
        "torque": 0.0,        # 可选
        "temperature": 25.0,  # 可选
        "bus_voltage": 24.0,  # 可选
        "online": True,
        "error_code": 0,
        "fsm_state": 1,
        "timestamp": 1710000000.0
    }
}
```

驱动适配器的实现原则：

- `connect()` 只建立连接，不主动让电机运动。
- `set_all_joint_angles()` 的输入长度必须等于 `profile.num_joints`。
- 对高频滑条和遥操作，建议在驱动内部合并下发，避免串口或总线拥塞。
- 如果硬件支持批量控制，优先在 `set_all_joint_angles()` 一帧下发整手命令。
- 如果硬件只支持单电机控制，可以在 `set_all_joint_angles()` 内循环调用 `set_motor_angle()`。
- 所有硬件协议细节都留在驱动内。上位机不应知道帧头、校验、寄存器或厂商 SDK 细节。

最小可运行驱动示例：

```python
import time


class HandDriver:
    def __init__(self, profile):
        self.profile = profile
        self.connected = False
        self.enabled = False
        self.angles = [
            joint.default_angle
            for finger in profile.finger_configs
            for joint in finger.joints
        ]

    def connect(self, port, baudrate, protocol=None):
        self.connected = True
        return True

    def disconnect(self):
        self.connected = False

    def enable_all(self, enable=True):
        self.enabled = bool(enable)

    def set_motor_angle(self, motor_id, angle_deg, speed=0.5):
        index = self.profile.motor_id_to_joint_index.get(int(motor_id))
        if index is not None:
            self.angles[index] = float(angle_deg)

    def set_angles(self, angles_dict):
        for motor_id, angle in angles_dict.items():
            self.set_motor_angle(motor_id, angle)

    def set_all_joint_angles(self, angles_list):
        if len(angles_list) != self.profile.num_joints:
            raise ValueError("angle count mismatch")
        self.angles = [float(v) for v in angles_list]

    def home_all_motors(self, target_angles=None, speed=0.3):
        if target_angles:
            self.set_angles(target_angles)
        else:
            defaults = {
                joint.motor_id: joint.default_angle
                for finger in self.profile.finger_configs
                for joint in finger.joints
            }
            self.set_angles(defaults)

    def get_latest_telemetry(self):
        now = time.time()
        return {
            motor_id: {
                "position": self.angles[index],
                "online": self.connected,
                "current": 0.0,
                "timestamp": now,
            }
            for index, motor_id in self.profile.joint_index_to_motor_id.items()
        }

    def get_status(self, motor_id):
        return self.get_latest_telemetry().get(int(motor_id), {"online": False})

    def cleanup(self):
        self.disconnect()


def create_driver(profile):
    return HandDriver(profile)
```

### 3.4 触觉适配器

如果新手没有触觉，可以在 `kit.json` 中设置：

```json
{
  "tactile": {
    "kind": "none"
  }
}
```

如果新手有触觉，入口文件必须命名为 `tactile_adapter.py`，并暴露以下入口之一：

```python
def create_tactile_adapter(profile):
    return TactileAdapter(profile)
```

或：

```python
class TactileAdapter:
    ...
```

必须实现：

```python
class TactileAdapter:
    def connect(self, port: str, baudrate: int) -> bool:
        ...

    def disconnect(self) -> None:
        ...

    def read_frame(self, timeout: float = 0.05):
        ...

    def cleanup(self) -> None:
        ...
```

推荐返回格式是 `patches`：

```python
{
    "patches": {
        "if_tip": [
            [0, 12, 25, 18, 0, 0, 0],
            [4, 20, 40, 31, 6, 0, 0]
        ],
        "palm": [
            [0, 0, 3],
            [1, 8, 4]
        ]
    }
}
```

也支持单块矩阵：

```python
{"raw": [[0, 10, 30], [4, 8, 12]]}
```

保留兼容格式：

```python
{
    "finger": [[0, 1, 2], [3, 4, 5]],
    "palm": [0, 1, 2, 3]
}
```

触觉适配器的职责：

- 打开触觉串口、CAN 网关、网络连接或 SDK。
- 从字节流中找到完整帧。
- 校验帧头、长度、校验和。
- 把原始数据转换成二维数组。
- 按 `kit.json` 中声明的 patch 名称返回数据。

上位机会统一处理：

- 数值归一化。
- 基线采集。
- 阈值扣除。
- 2D 热力图显示。
- 3D taxel site 上色。

注意事项：

- 如果 `kit.json` 声明了触觉区域尺寸，`read_frame()` 返回的 patch 尺寸必须完全一致。
- 返回值可以是 `0..1` 浮点数，也可以是 `0..255`、`0..1023` 等原始值。通过 `value_scale` 告诉上位机如何归一化。
- 如果触觉硬件不需要用户输入通信端点，把 `requires_endpoint` 设为 `false`，连接页可以留空。

### 3.5 遥操作适配器

如果不需要遥操作，可以设置：

```json
{
  "teleop": {
    "kind": "none"
  }
}
```

如果需要完整功能，建议使用外部 Python 遥操作适配器：

```json
{
  "teleop": {
    "kind": "external_python",
    "adapter_source": "teleop_adapter"
  }
}
```

入口文件必须命名为 `teleop_adapter.py`，并暴露以下入口之一：

```python
def create_teleop_adapter(profile):
    return TeleopAdapter(profile)
```

或：

```python
class TeleopAdapter:
    ...
```

必须实现：

```python
class TeleopAdapter:
    def retarget(self, hand_payload) -> list[float]:
        ...
```

可选实现：

```python
def process_frame(self, frame):
    return processed_frame, hand_payload
```

`retarget()` 的输入通常来自上位机相机检测结果：

- 默认情况下是 MediaPipe 右手 21 个 3D 关键点，形状接近 `21x3`。
- 如果适配器实现了 `process_frame()`，则可以自己处理相机图像，并把任意 `hand_payload` 传给 `retarget()`。

`retarget()` 必须返回：

- `list[float]`
- 长度等于 `profile.num_joints`
- 顺序等于关节表展开顺序
- 单位为度

最小遥操作示例：

```python
class TeleopAdapter:
    def __init__(self, profile):
        self.profile = profile

    def retarget(self, hand_payload):
        return [
            joint.default_angle
            for finger in self.profile.finger_configs
            for joint in finger.joints
        ]
```

如果使用内置 DexPilot 路径，`teleop.kind` 可以设置为 `dexpilot`。此路径要求：

- 手型是类人多指手。
- `teleop.joint_names` 显式声明目标关节名。
- `teleop.tip_link_names` 显式声明指尖 link。
- `teleop.wrist_link_name` 默认是 `base_link`，可修改。
- 模型中的 joint 和 link 名称必须匹配。

对完全新手，推荐优先使用 `external_python`，把遥操作算法全部封装在适配包内。

### 3.6 标定文件

建议准备三类标定：

1. 关节运行范围标定，由向导生成 `calibration.json`，也可以在上位机的“电机标定”页重新保存。
2. 电机原始位置到仿真角度的标定，例如 `motor_to_sim_calibration.json`。
3. 遥操作算法标定，例如 open/fist 样本、配对样本、比例系数等。

`calibration.json` 运行时结构：

```json
{
  "joints": [
    {
      "name": "if_mcp",
      "motor_id": 5,
      "min_angle": -18.0,
      "max_angle": 112.0,
      "default_angle": 0.0
    }
  ]
}
```

`motor_to_sim_calibration.json` 没有固定强制格式。它由你的 `driver.py` 和 `teleop_adapter.py` 自己读取。`hand2` 的做法是把舵机原始位置、中心点、方向、比例和 actuator 名称写在一起，供驱动完成“上位机角度 -> 舵机位置”的换算。

### 3.7 内置手势

内置手势写在 `kit.json` 的 `gestures.builtin` 中。每个手势都是完整关节角度列表：

```json
{
  "gestures": {
    "builtin": {
      "张开": [0, 0, 0, 0],
      "抓握": [60, 20, 80, 70]
    }
  }
}
```

要求：

- 每个列表长度必须等于总关节数。
- 单位为度。
- 顺序与关节表展开顺序一致。

## 4. kit.json 编写规范

`kit.json` 是向导读取的接入材料说明文件，不是运行时 `manifest.json`。向导会把 `kit.json` 转换成运行时 `manifest.json`。

完整示例：

```json
{
  "schema": "dexhand.integration_kit.v1",
  "hand_id": "my_hand",
  "display_name": "我的新灵巧手",
  "model_xml": "model/hand.xml",
  "driver": {
    "source": "driver_adapter"
  },
  "driver_source": "driver_adapter",
  "motor": {
    "kind": "python_adapter",
    "protocols": ["serial", "canfd", "ethercat", "sdk"],
    "default_protocol": "serial",
    "default_baudrates": {
      "serial": 115200,
      "canfd": 1000000,
      "ethercat": 0,
      "sdk": 0
    },
    "actuator_order": ["if_mcp_act", "if_pip_act"],
    "force_control_supported": true,
    "adapter_config": {
      "endpoint_label": "通信端点",
      "requires_endpoint": true,
      "command_debounce_ms": 30,
      "telemetry_period_ms": 100,
      "merge_interval_ms": 30,
      "telemetry_interval_ms": 100,
      "teleop_move_time_ms": 72
    }
  },
  "tactile": {
    "kind": "external_python",
    "default_baudrate": 460800,
    "adapter_source": "tactile_adapter",
    "layout": {
      "patches": [
        {
          "name": "if_tip",
          "display_name": "食指指尖",
          "group": "食指",
          "rows": 6,
          "cols": 7,
          "site_prefix": "if_tip",
          "display_transform": "identity",
          "site_transform": "flip_vertical"
        },
        {
          "name": "palm",
          "display_name": "手掌",
          "group": "手掌",
          "rows": 16,
          "cols": 16,
          "site_transform": "flip_horizontal"
        }
      ]
    },
    "adapter_config": {
      "endpoint_label": "触觉通信端点",
      "requires_endpoint": true,
      "frame_format": "patches",
      "value_scale": 255,
      "baseline_frames": 30,
      "baseline_threshold": 0
    }
  },
  "teleop": {
    "kind": "external_python",
    "adapter_source": "teleop_adapter",
    "config": "teleop_config.json",
    "pair_calibration": "teleop_pair_calibration.json",
    "motor_to_sim_calibration": "motor_to_sim_calibration.json",
    "wrist_link_name": "base_link",
    "tip_link_names": ["thumb_tip", "index_tip", "middle_tip"],
    "adapter_config": {
      "input": "mediapipe_21x3_or_features",
      "output_unit": "degree",
      "smoothing_alpha": 0.55,
      "rate_limit_rad_per_s": 8,
      "paired_k": 5,
      "paired_sigma": 0.45
    }
  },
  "fingers": [
    {
      "name": "index",
      "display_name": "食指",
      "joints": [
        {
          "name": "if_mcp",
          "display_name": "MCP 屈伸",
          "motor_id": 1,
          "min_angle": -20,
          "max_angle": 110,
          "default_angle": 0,
          "torque_limit": 150,
          "actuator_name": "if_mcp_act"
        }
      ]
    }
  ],
  "gestures": {
    "builtin": {
      "张开": [0],
      "抓握": [80]
    }
  }
}
```

字段说明：

| 字段 | 说明 |
| --- | --- |
| `schema` | 固定为 `dexhand.integration_kit.v1` |
| `hand_id` | 手型 ID，英文开头，只包含字母、数字、下划线、短横线 |
| `display_name` | 上位机显示名称 |
| `model_xml` | 相对接入材料目录的 MuJoCo XML 路径 |
| `driver_source` | 驱动目录或 `driver.py` 文件路径 |
| `motor.protocols` | 连接页协议下拉框 |
| `motor.default_baudrates` | 每种协议默认速率。非串口或 SDK 可填 `0` |
| `motor.force_control_supported` | 是否允许上位机显示力控开关 |
| `motor.adapter_config` | 传给驱动的配置，运行时通过 `profile.motor_adapter_config` 读取 |
| `tactile.kind` | `none` 或 `external_python` |
| `tactile.adapter_source` | 触觉适配器目录或文件 |
| `tactile.layout.patches` | 触觉区域声明 |
| `tactile.adapter_config` | 传给触觉适配器和归一化流程的配置 |
| `teleop.kind` | `none`、`external_python` 或 `dexpilot` |
| `teleop.adapter_source` | 遥操作适配器目录或文件 |
| `teleop.config` | 可选，向导复制到运行时包根目录 |
| `teleop.pair_calibration` | 可选，向导复制到运行时包根目录 |
| `teleop.motor_to_sim_calibration` | 可选，向导复制到运行时包根目录 |
| `fingers` | 手指和关节定义 |
| `gestures.builtin` | 内置手势 |

## 5. 向导导入流程

准备好 `hand_integration_kits/<hand_id>/` 后，按以下步骤导入：

1. 启动上位机。

```bash
python main.py
```

2. 打开“手型管理”。
3. 点击“添加新手型”。
4. 在首页点击“载入接入材料目录”。
5. 选择包含 `kit.json` 的目录，例如 `hand_integration_kits/my_hand`。
6. 逐页检查向导预填内容。

第 1 步，基础信息：

- 检查手型 ID。
- 检查显示名称。

第 2 步，模型：

- 检查 XML 路径。
- 如果模型无法解析，先在 MuJoCo 或脚本中修复 XML。

第 3 步，关节表：

- 检查手指顺序。
- 检查关节顺序。
- 检查 `motor_id` 是否唯一。
- 检查角度范围和默认值。
- 检查 actuator 名称是否和 XML 一致。

第 4 步，电机通信适配：

- 选择或确认 `driver_adapter` 目录。
- 检查通信协议列表和默认波特率。
- 对 SDK、EtherCAT、网关类连接，可以把波特率设为 `0`。
- 如果无需通信端点，在 `kit.json` 的 `motor.adapter_config.requires_endpoint` 中设为 `false`。
- 只有驱动能返回可靠电流或力矩遥测时，才勾选力控能力。

第 5 步，触觉接入：

- 没有触觉则选择“不启用触觉”。
- 有触觉则选择“Python 触觉适配器”。
- 检查 patch 名称、行数、列数。
- 检查 `value_scale`、基线帧数和阈值。
- 检查 `site_prefix` 和 3D site 命名是否一致。

第 6 步，遥操作接入：

- 推荐选择“Python 遥操作适配器”。
- 检查 `teleop_adapter.py` 路径。
- 检查遥操作配置和标定文件。
- 如果使用自定义 `process_frame()`，确认相机输入能被你的适配器处理。

第 7 步，接口校验与联机验证：

- 点击“执行接口检查”。
- 默认会生成临时手型包并检查适配器能否导入。
- 如果勾选电机烟测，需要填真实通信端点。测试会连接、调用 `enable_all(False)`、断开。
- 如果勾选触觉烟测，需要能在约 2 秒内读到标准触觉帧。

第 8 步，生成手型包：

- 点击“完成接入”。
- 向导会生成 `app/resources/hands/<hand_id>/manifest.json`。
- 主界面手型列表刷新后，选择新手型。

## 6. 导入后如何验证所有功能

### 6.1 手型列表

在“手型管理”确认：

- 手型 ID 正确。
- 关节数量正确。
- 驱动显示为 `python`。
- 触觉显示为 `external_python` 或 `none`。
- 遥操作显示为 `external_python`、`dexpilot` 或 `none`。

### 6.2 连接管理

验证步骤：

1. 选择通信端点。
2. 选择协议。
3. 检查速率是否自动切到该协议默认值。
4. 点击“连接”。
5. 点击“使能全部电机”。
6. 查看电机在线状态网格。
7. 点击“使能并归位”。

预期结果：

- 日志显示连接成功。
- 在线电机显示为绿色。
- `home_all_motors()` 被调用。
- 未连接或断线时不会导致上位机崩溃。

### 6.3 关节控制

验证步骤：

1. 打开“关节控制”。
2. 缓慢拖动每个关节滑条。
3. 观察真实电机方向和角度是否正确。
4. 点击“全部归零”。

预期结果：

- 每个滑条只控制对应关节。
- 角度方向符合模型和硬件预期。
- 到达 `min_angle` 和 `max_angle` 时不会越界。

如果方向反了，通常在驱动的角度到硬件位置换算中修正，不建议修改上位机核心代码。

### 6.4 电机标定

验证步骤：

1. 连接并使能电机。
2. 打开“电机标定”。
3. 检查当前角度是否刷新。
4. 对每个关节设置最小值、最大值和默认值。
5. 点击“保存标定”或“应用标定”。

如需自动寻限：

- 优先在 `driver.py` 实现 `find_motor_limits()`。
- 驱动内部更了解电流单位、堵转保护和安全策略。
- 未实现时，上位机会使用通用低速试探流程，这只适合角度控制电机。

### 6.5 触觉可视化

验证步骤：

1. 打开“连接管理”。
2. 在触觉通信区域连接触觉适配器，或点击“模拟数据”。
3. 打开“触觉可视化”。
4. 逐个按压触觉区域。

预期结果：

- 每个 patch 出现在正确分组中。
- 按压位置与 2D 热力图方向一致。
- 数值范围合理，没有全黑、全满或剧烈跳变。

如果 2D 方向不对，调整 `display_transform`。

### 6.6 3D 触觉显示

验证步骤：

1. 确认 MuJoCo 可用。
2. 打开“关节控制”右侧 3D 视图。
3. 连接或模拟触觉。
4. 按压某个 taxel。

预期结果：

- 模型加载成功。
- 关节姿态随滑条变化。
- 触觉 site 上色位置与真实按压区域一致。

如果 3D 上色位置不对，优先检查：

- XML 是否有 `tx_<site_prefix>_r<row>_c<col>` site。
- `site_prefix` 是否正确。
- `site_transform` 是否正确。

### 6.7 仿真遥操作

验证步骤：

1. 打开“仿真遥操作”。
2. 点击“加载模型”。
3. 点击“启动仿真”。
4. 设置摄像头源。
5. 点击“启动摄像头”。
6. 点击“启动遥操作”。

预期结果：

- MuJoCo 模型能加载。
- 摄像头画面能显示。
- 检测到手后，`teleop_adapter.py` 输出完整关节角度。
- 仿真和真实电机同步动作。

如果使用外部遥操作适配器：

- `retarget()` 返回长度必须等于 `profile.num_joints`。
- 输出单位必须是度。
- 不要在 `retarget()` 中直接访问上位机 UI。

### 6.8 预设手势

验证步骤：

1. 打开“预设手势”。
2. 点击内置手势。
3. 检查真实手动作。
4. 调整关节后保存自定义手势。

预期结果：

- 内置手势按钮来自 `kit.json` 的 `gestures.builtin`。
- 自定义手势保存到已安装手型包的 `gestures/` 目录。

### 6.9 数据记录

数据记录依赖 `HandState`。只要驱动和触觉适配器持续更新：

- 关节角度会来自关节控制、遥操作或手势。
- 电机遥测会来自 `get_latest_telemetry()`。
- 触觉数据会来自 `read_frame()`。

验证时重点检查时间戳、在线状态、角度和触觉数据是否持续变化。

### 6.10 力控

只有满足以下条件才建议启用力控：

- `motor.force_control_supported` 为 `true`。
- 驱动遥测中有稳定、可信的 `current` 或等价力矩数据。
- `joint.torque_limit` 已根据每个关节设置。
- 电机协议支持安全停止、限流或位置保持。

力控逻辑会读取每个电机当前角度、目标角度、电流和默认角度，并在合拢方向上减速或冻结。若电流单位不是 mA，需要在驱动里转换到一致尺度，或调整 `torque_limit`。

## 7. 常见问题

### 7.1 向导提示缺少 driver.py

原因：

- `driver_source` 指向的目录里没有 `driver.py`。
- 或者你选择了单个文件，但文件名不是 `driver.py`。

解决：

- 把入口文件改名为 `driver.py`。
- 如果有多个驱动文件，把整个目录作为 `driver_source`。

### 7.2 导入时报 adapter is missing required method

原因：

- `HandDriver` 缺少必要方法。
- 方法名拼写不一致。
- `create_driver(profile)` 返回了错误对象。

解决：

- 对照第 3.3 节补齐全部必要方法。

### 7.3 关节数量不匹配

原因：

- `teleop_adapter.py` 返回的角度列表长度不等于 `profile.num_joints`。
- 内置手势列表长度不等于总关节数。
- 标定文件 `calibration.json` 的 joints 数量和关节表不一致。

解决：

- 统一以 `kit.json` 的 `fingers[].joints[]` 为准。
- 所有关节角列表都必须使用同一顺序和长度。

### 7.4 MuJoCo 模型加载失败

常见原因：

- XML 路径错误。
- mesh 文件没有随 `model/` 目录复制。
- XML 引用绝对路径或本机临时路径。
- actuator 名称不存在。

解决：

- 在接入材料包中使用相对路径。
- 把 XML 和 mesh 放在同一个 `model/` 目录体系下。
- 用单独脚本先测试 `mujoco.MjModel.from_xml_path("model/hand.xml")`。

### 7.5 触觉连接成功但没有数据

常见原因：

- 波特率错误。
- 触觉适配器没有从字节流中正确同步帧头。
- `read_frame()` 返回了非 dict。
- 返回 patch 尺寸和 `kit.json` 声明不一致。

解决：

- 在 `tactile_adapter.py` 中加入独立串口读取测试。
- 打印或记录原始帧长度和帧头。
- 确认 `read_frame()` 返回 `{"patches": ...}` 或 `{"raw": ...}`。

### 7.6 触觉 2D 正确但 3D 错位

原因：

- 2D 显示和 3D site 排布方向不同。

解决：

- 保持 `display_transform` 只处理 2D。
- 用 `site_transform` 单独修正 3D site 对应关系。

### 7.7 遥操作动作抖动

常见原因：

- 相机关键点噪声。
- 遥操作输出未平滑。
- 驱动下发频率过高。
- 硬件响应时间和命令周期不匹配。

解决：

- 在 `teleop.adapter_config` 中调小或调大 `smoothing_alpha`。
- 使用 `rate_limit_rad_per_s` 限制角速度。
- 在 `motor.adapter_config` 中增加 `command_debounce_ms` 或 `merge_interval_ms`。
- 在驱动里为遥操作命令设置合适的移动时间。

### 7.8 力控误触发

常见原因：

- `current` 单位不符合 `torque_limit`。
- 默认角度设置错误，导致合拢方向判断反了。
- 正常伺服电流被误判为堵转。

解决：

- 在驱动里统一电流尺度。
- 为每个关节设置合理 `default_angle`。
- 调大 `torque_limit` 或关闭力控能力。

## 8. 最终交付检查表

交付新手适配包前，逐项确认：

- `kit.json` 可以被 JSON 解析。
- `hand_id` 合法且不会和已有手型重复。
- `model_xml` 指向真实存在的 XML。
- XML 和 mesh 复制到其他目录后仍能加载。
- 每个 `motor_id` 是正整数且全手唯一。
- 每个关节都有正确 `min_angle`、`max_angle`、`default_angle`。
- 每个关节的 `actuator_name` 能在 MuJoCo XML 的 actuator 中找到。
- `driver_adapter/driver.py` 能独立导入。
- `driver.py` 实现全部必需方法。
- `connect()` 失败时能返回 `False` 或抛出明确异常。
- `set_all_joint_angles()` 输入完整角度列表后不会越界。
- `get_latest_telemetry()` 返回每个在线电机的 `position` 和 `online`。
- 触觉适配器返回标准 dict。
- 触觉 patch 名称、行数、列数与 `kit.json` 一致。
- 有 3D 触觉时，XML 中的 taxel site 命名完整。
- 遥操作适配器返回完整角度列表，单位为度。
- 内置手势长度等于总关节数。
- 向导第 7 步接口检查通过。
- 导入后连接管理、关节控制、标定、触觉、仿真遥操作、手势功能逐项验证通过。

## 9. 开发建议

建议按以下顺序开发，不要一开始就同时接所有功能：

1. 先完成 `kit.json`、模型和关节表。
2. 编写最小 `driver.py`，让连接管理和关节滑条能工作。
3. 确认真实电机方向、角度范围和默认角。
4. 补齐遥测，支持标定页读取当前角度。
5. 增加归位、零点、自动寻限等可选方法。
6. 接入触觉适配器，先返回 `raw`，再升级为 `patches`。
7. 在 MuJoCo XML 中补触觉 site，修正 `site_transform`。
8. 接入遥操作适配器，先返回默认角，再逐步加入算法。
9. 准备内置手势。
10. 最后启用力控，并在低速和安全夹具下测试。

这个顺序能把问题分层定位。电机控制、触觉、仿真和遥操作都稳定后，再用向导把接入材料包安装成正式手型包。
