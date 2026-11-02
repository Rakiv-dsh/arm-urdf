"""中文手型管理页与新灵巧手接入向导。"""

from __future__ import annotations

import json
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QDoubleSpinBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
    QWizard,
    QWizardPage,
)

from app.hands import EMPTY_HAND_ID, delete_profile, get_profiles, reload_profiles
from app.hands.adapter import load_python_adapter
from app.hands.base import HANDS_DIR, HAND_KITS_DIR
from app.hands.manifest import HAND_ID_RE, load_profile_from_manifest, validate_manifest_payload


MOTOR_ADAPTER_TEMPLATE = '''"""新灵巧手电机通信适配器模板。

把你的底层串口协议、CAN 网关协议或厂家 SDK 封装到这个类里。
上位机只调用这里定义的统一接口。
"""

from __future__ import annotations

import time


class HandDriver:
    def __init__(self, profile):
        self.profile = profile
        self.config = getattr(profile, "teleop_adapter_config", {}) or {}
        self.config = getattr(profile, "motor_adapter_config", {}) or {}
        self.connected = False
        self.enabled = False
        self.angles = [
            joint.default_angle
            for finger in profile.finger_configs
            for joint in finger.joints
        ]

    def connect(self, port: str, baudrate: int, protocol: str | None = None) -> bool:
        # TODO: 在这里打开你的电机串口、CAN 网关或厂家 SDK。
        # 成功返回 True，失败返回 False 或抛出异常。
        self.connected = True
        return True

    def disconnect(self) -> None:
        # TODO: 关闭硬件连接。
        self.connected = False

    def enable_all(self, enable: bool = True) -> None:
        # TODO: 按你的电机协议发送使能 / 失能命令。
        self.enabled = bool(enable)

    def set_motor_angle(self, motor_id: int, angle_deg: float, speed: float = 0.5) -> None:
        # TODO: 将 angle_deg 转换为你的电机目标位置并下发。
        idx = self.profile.motor_id_to_joint_index.get(motor_id)
        if idx is not None:
            self.angles[idx] = float(angle_deg)

    def set_angles(self, angles_dict: dict[int, float]) -> None:
        for motor_id, angle in angles_dict.items():
            self.set_motor_angle(motor_id, angle)

    def set_all_joint_angles(self, angles_list: list[float]) -> None:
        # TODO: 如果你的协议支持批量下发，建议在这里一次发送整手目标。
        if len(angles_list) != self.profile.num_joints:
            return
        for index, angle in enumerate(angles_list):
            motor_id = self.profile.joint_index_to_motor_id[index]
            self.set_motor_angle(motor_id, angle)

    def home_all_motors(self, target_angles: dict[int, float] | None = None, speed: float = 0.3) -> None:
        if target_angles:
            self.set_angles(target_angles)
            return
        defaults = {
            joint.motor_id: joint.default_angle
            for finger in self.profile.finger_configs
            for joint in finger.joints
        }
        self.set_angles(defaults)

    def get_latest_telemetry(self) -> dict[int, dict]:
        # TODO: 返回最近一次遥测。字段名建议保持如下格式。
        now = time.time()
        telemetry = {}
        for index, angle in enumerate(self.angles):
            motor_id = self.profile.joint_index_to_motor_id[index]
            telemetry[motor_id] = {
                "position": angle,
                "velocity": 0.0,
                "current": 0.0,
                "torque": 0.0,
                "temperature": 25.0,
                "bus_voltage": 0.0,
                "online": self.connected,
                "error_code": 0,
                "fsm_state": 1 if self.enabled else 0,
                "timestamp": now,
            }
        return telemetry

    def get_status(self, motor_id: int) -> dict:
        return self.get_latest_telemetry().get(motor_id, {"online": False})

    def set_zero_point(self, motor_id: int) -> None:
        # 可选：如果硬件支持写零点，在这里实现。
        pass

    def cleanup(self) -> None:
        self.disconnect()
'''


TACTILE_ADAPTER_TEMPLATE = '''"""新灵巧手触觉通信适配器模板。"""

from __future__ import annotations


class TactileAdapter:
    def __init__(self, profile):
        self.profile = profile
        self.config = getattr(profile, "tactile_adapter_config", {}) or {}

    def connect(self, port: str, baudrate: int) -> bool:
        # TODO: 打开触觉串口、CAN 网关或厂家 SDK。
        return True

    def disconnect(self) -> None:
        pass

    def read_frame(self, timeout: float = 0.05):
        # 推荐返回 patches：{"patches": {"区域ID": [[...], ...]}}
        # 也可以返回 raw：{"raw": [[...], ...]}，上位机会显示成一个通用二维 patch。
        # 数值可以返回 0..1，也可以返回 0..255；上位机会按配置归一化。
        return None

    def cleanup(self) -> None:
        self.disconnect()
'''


TELEOP_ADAPTER_TEMPLATE = '''"""新灵巧手遥操作适配器模板。

默认输入是上位机相机识别到的人手 21x3 关键点数组。
返回值必须是当前手型的关节角度列表，单位为度。
"""

from __future__ import annotations


class TeleopAdapter:
    def __init__(self, profile):
        self.profile = profile

    def retarget(self, hand_payload) -> list[float]:
        # TODO: 调用你已有的遥操作程序，把 hand_payload 转成关节角度。
        return [
            joint.default_angle
            for finger in self.profile.finger_configs
            for joint in finger.joints
        ]
'''


MOTOR_ADAPTER_TEMPLATE = '''"""通用灵巧手电机驱动适配器模板。

你只需要改这个文件，不需要改上位机源码。上位机会把连接页选择的串口、
波特率、协议名，以及关节表里的 motor_id / 角度范围传给这个类。
如果驱动需要多个 Python 文件、厂家 SDK 包装层或标定 JSON，可以把整个目录
导入接入向导，但入口文件必须命名为 driver.py。

必须遵守的约定：
1. 上位机下发的所有角度都是“度”，顺序与接入向导第 3 步的关节表一致。
2. motor_id 必须与你在关节表中填写的电机 ID 一致。
3. connect 成功返回 True；失败可以返回 False，也可以抛出异常，错误会显示到日志。
4. 遥测 position 建议返回“度”，这样滑条、电机状态和记录回放会保持一致。
"""

from __future__ import annotations

import time


class HandDriver:
    def __init__(self, profile):
        self.profile = profile
        self.config = getattr(profile, "motor_adapter_config", {}) or {}
        self.connected = False
        self.enabled = False
        self.protocol = profile.default_protocol
        self.angles = [
            joint.default_angle
            for finger in profile.finger_configs
            for joint in finger.joints
        ]

    def connect(self, port: str, baudrate: int, protocol: str | None = None) -> bool:
        self.protocol = protocol or self.profile.default_protocol
        # TODO: 在这里打开你的硬件连接。
        # 串口示例：serial.Serial(port, baudrate, timeout=0.05, write_timeout=0.1)
        # CAN/网关/厂家 SDK 也可以在这里初始化。
        self.connected = True
        return True

    def disconnect(self) -> None:
        # TODO: 关闭串口、CAN 网关或 SDK 句柄。
        self.connected = False

    def enable_all(self, enable: bool = True) -> None:
        # TODO: 如果硬件有使能命令，在这里广播或逐电机发送。
        self.enabled = bool(enable)

    def set_motor_angle(self, motor_id: int, angle_deg: float, speed: float = 0.5) -> None:
        # TODO: 将 angle_deg 转换为你的电机协议目标值并发送给 motor_id。
        idx = self.profile.motor_id_to_joint_index.get(int(motor_id))
        if idx is not None:
            self.angles[idx] = float(angle_deg)

    def set_angles(self, angles_dict: dict[int, float]) -> None:
        for motor_id, angle in angles_dict.items():
            self.set_motor_angle(int(motor_id), float(angle))

    def set_all_joint_angles(self, angles_list: list[float]) -> None:
        # 如果协议支持整手批量帧，建议在这里一次发送，滑条和遥操作会更顺。
        if len(angles_list) != self.profile.num_joints:
            raise ValueError(f"需要 {self.profile.num_joints} 个角度，收到 {len(angles_list)} 个")
        for index, angle in enumerate(angles_list):
            motor_id = self.profile.joint_index_to_motor_id[index]
            self.set_motor_angle(motor_id, float(angle))

    def home_all_motors(self, target_angles: dict[int, float] | None = None, speed: float = 0.3) -> None:
        if target_angles:
            self.set_angles(target_angles)
            return
        defaults = {
            joint.motor_id: joint.default_angle
            for finger in self.profile.finger_configs
            for joint in finger.joints
        }
        self.set_angles(defaults)

    def get_latest_telemetry(self) -> dict[int, dict]:
        # TODO: 如果硬件能读位置/电流/温度，请在这里返回真实遥测。
        now = time.time()
        telemetry = {}
        for index, angle in enumerate(self.angles):
            motor_id = self.profile.joint_index_to_motor_id[index]
            telemetry[motor_id] = {
                "position": float(angle),
                "velocity": 0.0,
                "current": 0.0,
                "torque": 0.0,
                "temperature": 25.0,
                "bus_voltage": 0.0,
                "online": self.connected,
                "error_code": 0,
                "fsm_state": 1 if self.enabled else 0,
                "timestamp": now,
            }
        return telemetry

    def get_status(self, motor_id: int) -> dict:
        return self.get_latest_telemetry().get(int(motor_id), {"online": False})

    def set_zero_point(self, motor_id: int) -> None:
        pass

    def set_max_current(self, motor_id: int, value_q15: int) -> None:
        pass

    def cleanup(self) -> None:
        self.disconnect()


def create_driver(profile):
    return HandDriver(profile)
'''


TACTILE_ADAPTER_TEMPLATE = '''"""通用灵巧手触觉适配器模板。

你只需要把触觉硬件的一帧数据转换成下面两种标准格式之一：
1. patches：推荐格式。每个 patch 是一个二维矩阵，可表示任意手指、
   手掌、手背、腕部或其他触觉区域。
2. raw：通用二维矩阵，适合只有一整块触觉阵列的原型。

数值可以返回 0..1，也可以返回 0..255；接入向导里的“数值最大值”
会告诉上位机如何归一化。基线采集、阈值扣除和显示由上位机统一完成。

如果接入向导里配置了触觉区域表，请保证 patches 的 key 与“区域 ID”一致，
矩阵行列数也与表中“行数/列数”一致。上位机只认这个通用契约，不关心底层
是串口、CAN、网关还是厂家 SDK。
"""

from __future__ import annotations


class TactileAdapter:
    def __init__(self, profile):
        self.profile = profile
        self.config = getattr(profile, "tactile_adapter_config", {}) or {}
        self.connected = False

    def connect(self, port: str, baudrate: int) -> bool:
        # TODO: 打开触觉串口或 SDK。
        self.connected = True
        return True

    def disconnect(self) -> None:
        # TODO: 关闭触觉连接。
        self.connected = False

    def read_frame(self, timeout: float = 0.05):
        # TODO: 在 timeout 秒内读取并解析一帧触觉数据；没有新帧就返回 None。
        # 推荐格式：
        # return {
        #     "patches": {
        #         "thumb_tip": [[0, 12, 25], [4, 18, 31]],
        #         "palm_center": [[0] * 8 for _ in range(8)],
        #     }
        # }
        # 简单二维阵列也可以返回：
        # return {"raw": [[0.0, 0.2], [0.1, 0.7]]}
        return None

    def cleanup(self) -> None:
        self.disconnect()


def create_tactile_adapter(profile):
    return TactileAdapter(profile)
'''


TELEOP_ADAPTER_TEMPLATE = '''"""通用灵巧手遥操作适配器模板。

上位机会把相机识别到的人手数据传给 retarget：
- 默认是 21x3 的关键点数组。
- 如果你已有独立遥操作程序，可以把整个程序目录导入向导，并在这里调用它。

返回值必须是 list[float]，长度等于 profile.num_joints，单位为“度”，
顺序与接入向导第 3 步关节表一致。

上位机不要求你的底层算法是什么，也不会读取你的内部文件；它只调用
TeleopAdapter.retarget(hand_payload)。如果算法需要多个 Python 文件、
模型权重或标定文件，请放在同一个适配器目录中，入口文件保持为
teleop_adapter.py。
"""

from __future__ import annotations


class TeleopAdapter:
    def __init__(self, profile):
        self.profile = profile
        self.config = getattr(profile, "teleop_adapter_config", {}) or {}

    def retarget(self, hand_payload) -> list[float]:
        # TODO: 把 hand_payload 转成当前手型的关节角度。
        return [
            joint.default_angle
            for finger in self.profile.finger_configs
            for joint in finger.joints
        ]


def create_teleop_adapter(profile):
    return TeleopAdapter(profile)
'''


def _copy_text_template(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8")


def _json_from_text(text: str, field_name: str) -> dict:
    text = text.strip()
    if not text:
        return {}
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f"{field_name} 不是合法 JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"{field_name} 必须是 JSON 对象")
    return payload


def _copy_optional_file(src_text: str, dst: Path) -> bool:
    src_text = src_text.strip()
    if not src_text:
        return False
    shutil.copy2(Path(src_text), dst)
    return True


def _adapter_entry_exists(src_text: str, entry_name: str) -> tuple[bool, str]:
    src_text = src_text.strip()
    if not src_text:
        return False, f"请选择包含 {entry_name} 的文件或目录。"
    src = Path(src_text)
    if not src.exists():
        return False, f"{src} 不存在。"
    if src.is_dir() and not (src / entry_name).exists():
        return False, f"适配器目录中必须包含入口文件 {entry_name}。"
    return True, ""


def _copy_adapter_source(
    src_text: str,
    package_dir: Path,
    entry_name: str,
    folder_name: str,
    template_text: str | None = None,
) -> str:
    """Copy a single adapter file or a whole adapter directory into a hand package."""
    src_text = src_text.strip()
    if not src_text:
        if template_text is None:
            raise ValueError(f"缺少 {entry_name}")
        dst = package_dir / entry_name
        _copy_text_template(dst, template_text)
        return entry_name

    src = Path(src_text)
    if src.is_dir():
        entry = src / entry_name
        if not entry.exists():
            raise ValueError(f"适配器目录中必须包含入口文件 {entry_name}")
        dst_dir = package_dir / folder_name
        shutil.copytree(
            src,
            dst_dir,
            dirs_exist_ok=False,
            ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".git", ".idea", ".vscode"),
        )
        return f"{folder_name}/{entry_name}"

    dst = package_dir / entry_name
    shutil.copy2(src, dst)
    return entry_name


def _write_new_kit_skeleton(parent_dir: Path, hand_id: str, display_name: str) -> Path:
    kit_dir = parent_dir / hand_id
    if kit_dir.exists():
        raise FileExistsError(f"{kit_dir} 已存在")
    (kit_dir / "model" / "meshes").mkdir(parents=True)
    (kit_dir / "driver_adapter").mkdir()
    (kit_dir / "tactile_adapter").mkdir()
    (kit_dir / "teleop_adapter").mkdir()
    (kit_dir / "gestures").mkdir()
    (kit_dir / "model" / "README.txt").write_text(
        "把 MuJoCo XML 放在本目录，例如 hand.xml；把 mesh 文件放入 meshes/。\n"
        "XML 中 actuator 名称建议与第 3 步关节表中的 actuator_name 一一对应。\n",
        encoding="utf-8",
    )
    _copy_text_template(kit_dir / "driver_adapter" / "driver.py", MOTOR_ADAPTER_TEMPLATE)
    _copy_text_template(kit_dir / "tactile_adapter" / "tactile_adapter.py", TACTILE_ADAPTER_TEMPLATE)
    _copy_text_template(kit_dir / "teleop_adapter" / "teleop_adapter.py", TELEOP_ADAPTER_TEMPLATE)
    (kit_dir / "teleop_adapter" / "README.txt").write_text(
        "遥操作所需的 JSON 标定、模型权重、厂家 SDK 包装文件都放在本目录。\n"
        "teleop_adapter.py 中可以用 Path(__file__).parent 读取它们。\n",
        encoding="utf-8",
    )
    payload = {
        "schema": "dexhand.integration_kit.v1",
        "hand_id": hand_id,
        "display_name": display_name,
        "model_xml": "model/hand.xml",
        "driver": {"source": "driver_adapter"},
        "driver_source": "driver_adapter",
        "motor": {
            "kind": "python_adapter",
            "protocols": ["serial"],
            "default_protocol": "serial",
            "default_baudrates": {"serial": 115200},
            "actuator_order": [],
            "force_control_supported": False,
            "adapter_config": {
                "endpoint_label": "通信端点",
                "requires_endpoint": True,
                "command_debounce_ms": 30,
                "telemetry_period_ms": 100,
                "merge_interval_ms": 30,
                "telemetry_interval_ms": 100,
            },
        },
        "tactile": {
            "kind": "none",
            "default_baudrate": 115200,
            "adapter_source": "tactile_adapter",
            "layout": {"patches": []},
            "adapter_config": {
                "endpoint_label": "触觉通信端点",
                "requires_endpoint": True,
                "frame_format": "patches",
                "value_scale": 255,
                "baseline_frames": 30,
                "baseline_threshold": 0,
            },
        },
        "teleop": {
            "kind": "external_python",
            "adapter_source": "teleop_adapter",
            "wrist_link_name": "base_link",
            "tip_link_names": [],
            "adapter_config": {
                "input": "mediapipe_21x3_or_features",
                "output_unit": "degree",
                "smoothing_alpha": 0.55,
                "rate_limit_rad_per_s": 8.0,
            },
        },
        "fingers": [
            {
                "name": "finger",
                "display_name": "手指",
                "joints": [
                    {
                        "name": "joint_1",
                        "display_name": "关节 1",
                        "motor_id": 1,
                        "min_angle": -90,
                        "max_angle": 90,
                        "default_angle": 0,
                        "torque_limit": 150,
                        "actuator_name": "joint_1_act",
                    }
                ],
            }
        ],
        "gestures": {"builtin": {"张开": [0]}},
    }
    (kit_dir / "kit.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return kit_dir


def _combo_value(combo: QComboBox) -> str:
    value = combo.currentData()
    return str(value if value is not None else combo.currentText())


def _set_combo_value(combo: QComboBox, value: str):
    index = combo.findData(value)
    if index < 0:
        index = combo.findText(value)
    if index >= 0:
        combo.setCurrentIndex(index)


def _xml_actuator_names(xml_path: Path) -> list[str]:
    if not xml_path.exists():
        return []
    root = ET.parse(xml_path).getroot()
    names: list[str] = []
    for actuator in root.iter():
        if actuator.tag.split("}")[-1] != "actuator":
            continue
        for child in list(actuator):
            name = child.attrib.get("name")
            if name:
                names.append(name)
    return names


class IntroPage(QWizardPage):
    def __init__(self):
        super().__init__()
        self.setTitle("新灵巧手接入流程")
        layout = QVBoxLayout(self)
        text = QTextEdit()
        text.setReadOnly(True)
        text.setPlainText(
            "本向导会把一只新手整理成上位机可识别的手型包。\n\n"
            "你需要准备：\n"
            "1. MuJoCo XML 文件以及同目录下的 meshes 文件夹。\n"
            "2. 电机通信协议说明，或已经封装好的厂家 API。\n"
            "3. 触觉通信协议说明，如果这只手带触觉。\n"
            "4. 遥操作程序，如果你希望在仿真遥操作页面直接驱动这只手。\n\n"
            "最终生成目录：app/resources/hands/<hand_id>/\n"
            "其中包含 manifest.json、driver.py、model/、gestures/、calibration.json，"
            "以及可选的 tactile_adapter.py 和 teleop_adapter.py。"
        )
        layout.addWidget(text)

        kit_hint = QLabel(
            "如果你手里已经有按上位机规范整理好的接入材料目录，可以先载入材料。"
            "向导会把手型 ID、XML、关节表、电机/触觉/遥操作适配器和标定文件预填好，"
            "你仍然可以逐步检查和修改。"
        )
        kit_hint.setWordWrap(True)
        layout.addWidget(kit_hint)

        row = QHBoxLayout()
        self.kit_path = QLineEdit()
        self.kit_path.setPlaceholderText(str(HAND_KITS_DIR))
        browse = QPushButton("载入接入材料目录")
        browse.clicked.connect(self._browse_kit)
        create = QPushButton("生成新手材料模板")
        create.clicked.connect(self._create_kit_skeleton)
        row.addWidget(self.kit_path, 1)
        row.addWidget(browse)
        row.addWidget(create)
        layout.addLayout(row)

    def _browse_kit(self):
        start = str(HAND_KITS_DIR if HAND_KITS_DIR.exists() else HANDS_DIR.parent)
        path = QFileDialog.getExistingDirectory(self, "选择包含 kit.json 的接入材料目录", start)
        if not path:
            return
        self.kit_path.setText(path)
        try:
            self.wizard().load_kit(Path(path))
        except Exception as exc:
            QMessageBox.critical(self, "接入材料载入失败", str(exc))

    def _create_kit_skeleton(self):
        hand_id, ok = QInputDialog.getText(self, "新手材料模板", "手型 ID（英文开头，例如 my_hand）：")
        if not ok:
            return
        hand_id = hand_id.strip()
        if not HAND_ID_RE.match(hand_id):
            QMessageBox.warning(self, "手型 ID 不合法", "请使用英文开头，只包含英文、数字、下划线或短横线。")
            return
        display_name, ok = QInputDialog.getText(self, "新手材料模板", "显示名称：", text=hand_id)
        if not ok:
            return
        start = str(HAND_KITS_DIR if HAND_KITS_DIR.exists() else HANDS_DIR.parent)
        parent = QFileDialog.getExistingDirectory(self, "选择模板生成位置", start)
        if not parent:
            return
        try:
            kit_dir = _write_new_kit_skeleton(Path(parent), hand_id, display_name.strip() or hand_id)
        except Exception as exc:
            QMessageBox.critical(self, "模板生成失败", str(exc))
            return
        self.kit_path.setText(str(kit_dir))
        QMessageBox.information(
            self,
            "模板已生成",
            "已生成标准接入材料目录。请把 XML/meshes 放进 model，把电机、触觉、遥操作协议分别封装到对应适配器目录，然后重新载入该目录。",
        )
        try:
            self.wizard().load_kit(kit_dir)
        except Exception:
            pass


class BasicPage(QWizardPage):
    def __init__(self):
        super().__init__()
        self.setTitle("1. 基础信息")
        self.setSubTitle("手型 ID 会成为目录名，建议使用英文、数字、下划线。")
        layout = QFormLayout(self)
        self.hand_id = QLineEdit()
        self.hand_id.setPlaceholderText("例如 my_dex_hand")
        self.display_name = QLineEdit()
        self.display_name.setPlaceholderText("例如 六自由度实验灵巧手")
        layout.addRow("手型 ID", self.hand_id)
        layout.addRow("显示名称", self.display_name)

    def validatePage(self) -> bool:
        hand_id = self.hand_id.text().strip()
        if not HAND_ID_RE.match(hand_id):
            QMessageBox.warning(self, "手型 ID 不合法", "请使用英文开头，只包含英文、数字、下划线或短横线。")
            return False
        if hand_id in get_profiles() or (HANDS_DIR / hand_id).exists():
            QMessageBox.warning(self, "手型已存在", f"{hand_id} 已经存在。")
            return False
        if not self.display_name.text().strip():
            QMessageBox.warning(self, "缺少显示名称", "请输入这只手在上位机中显示的名称。")
            return False
        return True


class ModelPage(QWizardPage):
    def __init__(self):
        super().__init__()
        self.setTitle("2. 导入 XML 与 meshes")
        self.setSubTitle("请选择 MuJoCo XML。向导会复制 XML 所在目录，保留 meshes 相对路径。")
        layout = QVBoxLayout(self)
        row = QHBoxLayout()
        self.xml_path = QLineEdit()
        browse = QPushButton("选择 XML")
        browse.clicked.connect(self._browse)
        row.addWidget(self.xml_path, 1)
        row.addWidget(browse)
        layout.addLayout(row)
        self.preview = QTextEdit()
        self.preview.setReadOnly(True)
        self.preview.setPlainText("选择 XML 后，这里会显示从 <actuator> 中读取到的执行器名称。")
        layout.addWidget(self.preview, 1)

    def _browse(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "选择 MuJoCo XML 文件",
            "",
            "MuJoCo XML (*.xml *.mjcf);;所有文件 (*)",
        )
        if path:
            self.xml_path.setText(path)
            if self.wizard().maybe_load_kit_from_path(Path(path)):
                return
            self._refresh_preview()

    def _refresh_preview(self):
        names = _xml_actuator_names(Path(self.xml_path.text().strip()))
        if names:
            self.preview.setPlainText("读取到的执行器：\n" + "\n".join(names))
        else:
            self.preview.setPlainText("没有读取到执行器。你仍然可以在下一步手动填写 actuator_name。")

    def actuator_names(self) -> list[str]:
        return _xml_actuator_names(Path(self.xml_path.text().strip()))

    def validatePage(self) -> bool:
        path = Path(self.xml_path.text().strip())
        if not path.exists():
            QMessageBox.warning(self, "缺少 XML", "请选择这只手的 MuJoCo XML 文件。")
            return False
        return True


class JointMapPage(QWizardPage):
    COLUMNS = ["手指 ID", "手指名称", "关节 ID", "关节名称", "电机 ID", "最小角", "最大角", "默认角", "力矩阈值", "执行器名"]

    def __init__(self):
        super().__init__()
        self.setTitle("3. 关节、电机与模型执行器映射")
        self.setSubTitle("每一行对应一个上位机控制关节。motor_id 必须和你的电机适配器一致。")
        layout = QVBoxLayout(self)
        toolbar = QHBoxLayout()
        gen_btn = QPushButton("从 XML 执行器生成")
        gen_btn.clicked.connect(self._generate_from_xml)
        add_btn = QPushButton("添加关节")
        add_btn.clicked.connect(self._add_empty_row)
        del_btn = QPushButton("删除选中行")
        del_btn.clicked.connect(self._delete_rows)
        toolbar.addWidget(gen_btn)
        toolbar.addWidget(add_btn)
        toolbar.addWidget(del_btn)
        toolbar.addStretch()
        layout.addLayout(toolbar)
        self.table = QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        layout.addWidget(self.table, 1)

    def initializePage(self):
        if self.table.rowCount() == 0:
            self._generate_from_xml()

    def _generate_from_xml(self):
        names = self.wizard().model.actuator_names()
        self.table.setRowCount(0)
        if not names:
            self._add_empty_row()
            return
        for index, actuator in enumerate(names, start=1):
            finger_id = actuator.split("_", 1)[0] if "_" in actuator else "finger"
            values = [
                finger_id,
                finger_id,
                actuator.replace("act_", "").replace("-", "_"),
                actuator,
                str(index),
                "-180",
                "180",
                "0",
                "150",
                actuator,
            ]
            self._append_row(values)

    def _add_empty_row(self):
        row = self.table.rowCount() + 1
        self._append_row(["finger", "手指", f"joint_{row}", f"关节 {row}", str(row), "-180", "180", "0", "150", ""])

    def _append_row(self, values: list[str]):
        row = self.table.rowCount()
        self.table.insertRow(row)
        for col, value in enumerate(values):
            self.table.setItem(row, col, QTableWidgetItem(value))
        self.table.resizeColumnsToContents()

    def set_fingers(self, fingers: list[dict]):
        self.table.setRowCount(0)
        for finger in fingers:
            finger_id = str(finger.get("name") or "finger")
            finger_name = str(finger.get("display_name") or finger_id)
            for joint in finger.get("joints", []):
                self._append_row([
                    finger_id,
                    finger_name,
                    str(joint.get("name") or ""),
                    str(joint.get("display_name") or joint.get("name") or ""),
                    str(joint.get("motor_id") or ""),
                    str(joint.get("min_angle", -180)),
                    str(joint.get("max_angle", 180)),
                    str(joint.get("default_angle", 0)),
                    str(joint.get("torque_limit", 150)),
                    str(joint.get("actuator_name") or joint.get("name") or ""),
                ])

    def _delete_rows(self):
        rows = sorted({idx.row() for idx in self.table.selectedIndexes()}, reverse=True)
        for row in rows:
            self.table.removeRow(row)

    def fingers(self) -> list[dict]:
        groups: dict[str, dict] = {}
        order: list[str] = []
        for row in range(self.table.rowCount()):
            values = [self.table.item(row, col).text().strip() if self.table.item(row, col) else "" for col in range(len(self.COLUMNS))]
            finger_id, finger_name, joint_id, joint_name = values[:4]
            if finger_id not in groups:
                order.append(finger_id)
                groups[finger_id] = {"name": finger_id, "display_name": finger_name or finger_id, "joints": []}
            groups[finger_id]["joints"].append({
                "name": joint_id,
                "display_name": joint_name or joint_id,
                "motor_id": int(values[4]),
                "min_angle": float(values[5]),
                "max_angle": float(values[6]),
                "default_angle": float(values[7]),
                "torque_limit": float(values[8]),
                "actuator_name": values[9] or joint_id,
            })
        return [groups[key] for key in order]

    def validatePage(self) -> bool:
        try:
            fingers = self.fingers()
        except Exception as exc:
            QMessageBox.warning(self, "映射表有误", f"请检查数字列和空白项：{exc}")
            return False
        payload = {
            "schema": "dexhand.hand.v1",
            "hand_id": "preview",
            "display_name": "preview",
            "driver": {"kind": "python", "path": "driver.py"},
            "fingers": fingers,
        }
        errors = validate_manifest_payload(payload, Path.cwd())
        errors = [err for err in errors if "driver file does not exist" not in err]
        if errors:
            QMessageBox.warning(self, "映射表不完整", "\n".join(errors))
            return False
        return True


class MotorAdapterPage(QWizardPage):
    def __init__(self):
        super().__init__()
        self.setTitle("第 4 步：电机通信适配")
        self.setSubTitle("把新手的电机协议封装成 HandDriver，并告诉连接页应该显示哪些协议和波特率。")
        layout = QVBoxLayout(self)

        info = QTextEdit()
        info.setReadOnly(True)
        info.setMaximumHeight(150)
        info.setPlainText(
            "你需要准备一个 driver.py，也可以导入一个包含 driver.py 和辅助文件的目录。"
            "它不需要修改上位机源码，只要实现模板中的 HandDriver 接口。\n"
            "连接页选择串口后，上位机会调用 driver.connect(port, baudrate, protocol)。\n"
            "关节控制、手势、记录回放、遥操作都会调用 set_all_joint_angles 或 set_motor_angle，角度单位统一为度。\n"
            "如果硬件能读位置、电流、温度，请在 get_latest_telemetry 中返回；不能读也可以返回在线状态和最后目标角。\n"
            "如果你的电机使用编码器 tick、脉冲或厂家内部单位，请在 driver.py 或同目录标定文件中完成“硬件单位 ↔ 角度”的转换。"
        )
        layout.addWidget(info)

        row = QHBoxLayout()
        self.driver_path = QLineEdit()
        browse = QPushButton("导入 driver.py")
        browse.clicked.connect(self._browse)
        browse_dir = QPushButton("导入驱动目录")
        browse_dir.clicked.connect(self._browse_dir)
        save_template = QPushButton("生成接口模板")
        save_template.clicked.connect(self._save_template)
        row.addWidget(self.driver_path, 1)
        row.addWidget(browse)
        row.addWidget(browse_dir)
        row.addWidget(save_template)
        layout.addLayout(row)

        self.placeholder = QCheckBox("只生成演示占位驱动（不会控制真实硬件）")
        layout.addWidget(self.placeholder)

        protocol_hint = QLabel(
            "协议名是连接页下拉框里的选项，也会原样传给 driver.connect(..., protocol=协议名)。"
            "如果你的新手只有一种通信方式，保留一行即可，例如 serial。"
        )
        protocol_hint.setWordWrap(True)
        layout.addWidget(protocol_hint)

        self.protocol_table = QTableWidget(1, 2)
        self.protocol_table.setHorizontalHeaderLabels(["协议名", "默认波特率"])
        self.protocol_table.setItem(0, 0, QTableWidgetItem("serial"))
        self.protocol_table.setItem(0, 1, QTableWidgetItem("115200"))
        self.protocol_table.itemChanged.connect(self._refresh_default_protocols)
        layout.addWidget(self.protocol_table)

        protocol_btns = QHBoxLayout()
        add_protocol = QPushButton("添加协议")
        add_protocol.clicked.connect(self._add_protocol)
        remove_protocol = QPushButton("删除选中协议")
        remove_protocol.clicked.connect(self._remove_protocol)
        self.default_protocol = QComboBox()
        protocol_btns.addWidget(add_protocol)
        protocol_btns.addWidget(remove_protocol)
        protocol_btns.addStretch()
        protocol_btns.addWidget(QLabel("默认协议"))
        protocol_btns.addWidget(self.default_protocol)
        layout.addLayout(protocol_btns)

        form = QFormLayout()
        self.force_control = QCheckBox("当前手型支持按电流/力矩做力控保护")
        self.force_control.setChecked(False)
        self.command_debounce_ms = QSpinBox()
        self.command_debounce_ms.setRange(0, 500)
        self.command_debounce_ms.setValue(30)
        self.command_debounce_ms.setSuffix(" ms")
        self.telemetry_period_ms = QSpinBox()
        self.telemetry_period_ms.setRange(20, 2000)
        self.telemetry_period_ms.setValue(100)
        self.telemetry_period_ms.setSuffix(" ms")
        self._extra_adapter_config: dict[str, object] = {}
        form.addRow("力控能力", self.force_control)
        form.addRow("滑条/遥操作合并下发间隔", self.command_debounce_ms)
        form.addRow("遥测刷新间隔", self.telemetry_period_ms)
        layout.addLayout(form)

        template = QTextEdit()
        template.setPlainText(MOTOR_ADAPTER_TEMPLATE)
        template.setReadOnly(True)
        layout.addWidget(template, 1)
        self._refresh_default_protocols()

    def _browse(self):
        path, _ = QFileDialog.getOpenFileName(self, "选择电机适配器", "", "Python 文件 (*.py);;所有文件 (*)")
        if path:
            self.driver_path.setText(path)
            self.wizard().maybe_load_kit_from_path(Path(path))

    def _browse_dir(self):
        path = QFileDialog.getExistingDirectory(self, "选择包含 driver.py 的驱动目录")
        if path:
            self.driver_path.setText(path)
            self.wizard().maybe_load_kit_from_path(Path(path))

    def _save_template(self):
        path, _ = QFileDialog.getSaveFileName(self, "生成电机适配器模板", "driver.py", "Python 文件 (*.py)")
        if path:
            _copy_text_template(Path(path), MOTOR_ADAPTER_TEMPLATE)
            self.driver_path.setText(path)
            self.wizard().maybe_load_kit_from_path(Path(path))

    def _add_protocol(self):
        row = self.protocol_table.rowCount()
        self.protocol_table.insertRow(row)
        self.protocol_table.setItem(row, 0, QTableWidgetItem(f"protocol_{row + 1}"))
        self.protocol_table.setItem(row, 1, QTableWidgetItem("115200"))
        self._refresh_default_protocols()

    def _remove_protocol(self):
        rows = sorted({index.row() for index in self.protocol_table.selectedIndexes()}, reverse=True)
        for row in rows:
            if self.protocol_table.rowCount() > 1:
                self.protocol_table.removeRow(row)
        self._refresh_default_protocols()

    def _refresh_default_protocols(self):
        current = self.default_protocol.currentText() if hasattr(self, "default_protocol") else ""
        protocols, _baudrates = self.protocol_config()
        self.default_protocol.blockSignals(True)
        self.default_protocol.clear()
        self.default_protocol.addItems(protocols or ["serial"])
        if current in protocols:
            self.default_protocol.setCurrentText(current)
        self.default_protocol.blockSignals(False)

    def set_motor_config(self, motor: dict):
        self.placeholder.setChecked(False)
        protocols = motor.get("protocols")
        if not isinstance(protocols, list) or not protocols:
            protocols = ["serial"]
        baudrates = motor.get("default_baudrates")
        if not isinstance(baudrates, dict):
            baudrates = {}
        self.protocol_table.setRowCount(0)
        for row, protocol in enumerate(protocols):
            self.protocol_table.insertRow(row)
            protocol_name = str(protocol)
            self.protocol_table.setItem(row, 0, QTableWidgetItem(protocol_name))
            self.protocol_table.setItem(row, 1, QTableWidgetItem(str(baudrates.get(protocol_name, 115200))))
        self._refresh_default_protocols()
        default_protocol = str(motor.get("default_protocol") or protocols[0])
        _set_combo_value(self.default_protocol, default_protocol)
        self.force_control.setChecked(bool(motor.get("force_control_supported", False)))
        config = motor.get("adapter_config")
        if isinstance(config, dict):
            self._extra_adapter_config = {
                key: value
                for key, value in config.items()
                if key not in {"command_debounce_ms", "telemetry_period_ms"}
            }
            self.command_debounce_ms.setValue(int(config.get("command_debounce_ms", self.command_debounce_ms.value())))
            self.telemetry_period_ms.setValue(int(config.get("telemetry_period_ms", self.telemetry_period_ms.value())))
        else:
            self._extra_adapter_config = {}

    def protocol_config(self) -> tuple[list[str], dict[str, int]]:
        protocols: list[str] = []
        baudrates: dict[str, int] = {}
        for row in range(self.protocol_table.rowCount()):
            name_item = self.protocol_table.item(row, 0)
            baud_item = self.protocol_table.item(row, 1)
            name = name_item.text().strip() if name_item else ""
            if not name:
                continue
            baud_text = baud_item.text().strip() if baud_item else "115200"
            protocols.append(name)
            baudrates[name] = int(baud_text)
        return protocols, baudrates

    def motor_manifest(self) -> dict:
        protocols, baudrates = self.protocol_config()
        adapter_config = dict(self._extra_adapter_config)
        adapter_config["command_debounce_ms"] = self.command_debounce_ms.value()
        adapter_config["telemetry_period_ms"] = self.telemetry_period_ms.value()
        adapter_config.setdefault("merge_interval_ms", self.command_debounce_ms.value())
        adapter_config.setdefault("telemetry_interval_ms", self.telemetry_period_ms.value())
        return {
            "kind": "python_adapter",
            "protocols": protocols,
            "default_protocol": self.default_protocol.currentText() or protocols[0],
            "default_baudrates": baudrates,
            "force_control_supported": self.force_control.isChecked(),
            "adapter_config": adapter_config,
        }

    def validatePage(self) -> bool:
        if not self.placeholder.isChecked():
            ok, message = _adapter_entry_exists(self.driver_path.text(), "driver.py")
            if not ok:
                QMessageBox.warning(self, "缺少电机适配器", message)
                return False
            src = Path(self.driver_path.text().strip())
            if src.is_file() and src.name != "driver.py":
                QMessageBox.warning(self, "入口文件名不正确", "电机适配器入口文件必须命名为 driver.py。")
                return False
        try:
            protocols, baudrates = self.protocol_config()
        except ValueError:
            QMessageBox.warning(self, "协议配置有误", "波特率必须是整数，例如 115200。")
            return False
        if not protocols:
            QMessageBox.warning(self, "协议配置有误", "至少需要填写一个协议名。")
            return False
        if self.default_protocol.currentText() not in protocols:
            QMessageBox.warning(self, "协议配置有误", "默认协议必须来自协议表。")
            return False
        if any(value < 0 for value in baudrates.values()):
            QMessageBox.warning(self, "协议配置有误", "速率/波特率不能小于 0；填 0 表示由适配器自行处理。")
            return False
        return True


class TactilePage(QWizardPage):
    LAYOUT_COLUMNS = [
        "区域 ID",
        "显示名称",
        "分组",
        "行数",
        "列数",
        "3D site 前缀",
        "2D显示变换",
        "3D site变换",
    ]
    TRANSFORMS = {"", "identity", "flip_horizontal", "flip_vertical", "rotate_180"}

    def __init__(self):
        super().__init__()
        self.setTitle("第 5 步：触觉接入")
        self.setSubTitle("把任意触觉阵列声明成区域表，并用 tactile_adapter.py 输出同名二维矩阵。")
        layout = QVBoxLayout(self)

        info = QTextEdit()
        info.setReadOnly(True)
        info.setMaximumHeight(155)
        info.setPlainText(
            "推荐使用 patches 格式：你在下面表格中定义任意数量的触觉区域，每个区域都有自己的行数和列数。\n"
            "tactile_adapter.py 只需要返回 {'patches': {'区域ID': 二维数组}}，区域 ID 和数组尺寸必须与表格一致。\n"
            "上位机会统一完成归一化、基线采集、阈值扣除、热力图显示和 3D site 上色。"
            "如果你的适配器需要多个 Python 文件，请导入整个目录，入口文件保持为 tactile_adapter.py。"
        )
        layout.addWidget(info)

        form = QFormLayout()
        self.kind = QComboBox()
        self.kind.addItem("不启用触觉", "none")
        self.kind.addItem("Python 触觉适配器（推荐）", "external_python")
        self.kind.currentIndexChanged.connect(self._on_kind_changed)
        self.baud = QSpinBox()
        self.baud.setRange(1200, 3000000)
        self.baud.setValue(115200)
        self.value_scale = QDoubleSpinBox()
        self.value_scale.setRange(1.0, 65535.0)
        self.value_scale.setDecimals(1)
        self.value_scale.setValue(255.0)
        self.baseline_frames = QSpinBox()
        self.baseline_frames.setRange(0, 300)
        self.baseline_frames.setValue(30)
        self.baseline_threshold = QDoubleSpinBox()
        self.baseline_threshold.setRange(0.0, 65535.0)
        self.baseline_threshold.setDecimals(2)
        self.baseline_threshold.setValue(0.0)
        self.frame_format = QComboBox()
        self.frame_format.addItem("由 tactile_adapter.py 解析后返回标准字典", "adapter_dict")
        self.frame_format.addItem("适配器返回 patches 分区矩阵", "patches")
        self.frame_format.addItem("适配器返回单块 raw 二维矩阵", "raw_matrix")
        form.addRow("触觉来源", self.kind)
        form.addRow("默认波特率", self.baud)
        form.addRow("数值最大值", self.value_scale)
        form.addRow("基线采集帧数", self.baseline_frames)
        form.addRow("接触阈值（原始单位）", self.baseline_threshold)
        form.addRow("适配器输出格式", self.frame_format)
        layout.addLayout(form)

        row = QHBoxLayout()
        self.adapter_path = QLineEdit()
        browse = QPushButton("导入 tactile_adapter.py")
        browse.clicked.connect(self._browse)
        browse_dir = QPushButton("导入触觉目录")
        browse_dir.clicked.connect(self._browse_dir)
        save_template = QPushButton("生成接口模板")
        save_template.clicked.connect(self._save_template)
        row.addWidget(self.adapter_path, 1)
        row.addWidget(browse)
        row.addWidget(browse_dir)
        row.addWidget(save_template)
        layout.addLayout(row)

        layout_hint = QLabel(
            "触觉区域表用于告诉上位机每块触觉皮肤如何显示。"
            "区域 ID 要和适配器返回的 patches key 一致；行数/列数要和二维数组一致。"
            "如果 XML 中有触觉 site，默认命名为 tx_<区域ID>_r行_c列；若 site 使用别名前缀，请填写 3D site 前缀。"
            "2D显示变换影响热力图方向，3D site变换影响矩阵坐标到 MuJoCo site 的对应方向。"
        )
        layout_hint.setWordWrap(True)
        layout.addWidget(layout_hint)

        layout_btns = QHBoxLayout()
        add_patch = QPushButton("添加触觉区域")
        add_patch.clicked.connect(self._add_patch_row)
        remove_patch = QPushButton("删除选中区域")
        remove_patch.clicked.connect(self._remove_patch_rows)
        generate_from_fingers = QPushButton("按手指生成基础区域")
        generate_from_fingers.clicked.connect(self._generate_patch_rows_from_fingers)
        layout_btns.addWidget(add_patch)
        layout_btns.addWidget(remove_patch)
        layout_btns.addWidget(generate_from_fingers)
        layout_btns.addStretch()
        layout.addLayout(layout_btns)

        self.layout_table = QTableWidget(0, len(self.LAYOUT_COLUMNS))
        self.layout_table.setHorizontalHeaderLabels(self.LAYOUT_COLUMNS)
        layout.addWidget(self.layout_table, 1)

        template = QTextEdit()
        template.setPlainText(TACTILE_ADAPTER_TEMPLATE)
        template.setReadOnly(True)
        template.setMaximumHeight(185)
        layout.addWidget(template, 1)

    def _on_kind_changed(self):
        kind = _combo_value(self.kind)
        if kind == "external_python":
            self.frame_format.setCurrentIndex(0)

    def _browse(self):
        path, _ = QFileDialog.getOpenFileName(self, "选择触觉适配器", "", "Python 文件 (*.py);;所有文件 (*)")
        if path:
            self.adapter_path.setText(path)
            self.wizard().maybe_load_kit_from_path(Path(path))

    def _browse_dir(self):
        path = QFileDialog.getExistingDirectory(self, "选择包含 tactile_adapter.py 的触觉适配器目录")
        if path:
            self.adapter_path.setText(path)
            self.wizard().maybe_load_kit_from_path(Path(path))

    def _save_template(self):
        path, _ = QFileDialog.getSaveFileName(self, "生成触觉适配器模板", "tactile_adapter.py", "Python 文件 (*.py)")
        if path:
            _copy_text_template(Path(path), TACTILE_ADAPTER_TEMPLATE)
            self.adapter_path.setText(path)
            self.wizard().maybe_load_kit_from_path(Path(path))

    def _add_patch_row(self, values: list[str] | None = None):
        if not isinstance(values, list):
            values = None
        row_number = self.layout_table.rowCount() + 1
        values = values or [f"patch_{row_number}", f"触觉区域 {row_number}", "触觉", "4", "4", "", "identity", "identity"]
        row = self.layout_table.rowCount()
        self.layout_table.insertRow(row)
        padded = list(values[: len(self.LAYOUT_COLUMNS)])
        padded.extend([""] * (len(self.LAYOUT_COLUMNS) - len(padded)))
        for col, value in enumerate(padded):
            self.layout_table.setItem(row, col, QTableWidgetItem(str(value)))

    def _remove_patch_rows(self):
        rows = sorted({index.row() for index in self.layout_table.selectedIndexes()}, reverse=True)
        for row in rows:
            self.layout_table.removeRow(row)

    def _generate_patch_rows_from_fingers(self):
        self.layout_table.setRowCount(0)
        for finger in self.wizard().joints.fingers():
            finger_id = finger.get("name") or "finger"
            finger_name = finger.get("display_name") or finger_id
            self._add_patch_row([f"{finger_id}_tip", f"{finger_name} 指尖", finger_name, "4", "4", "", "identity", "identity"])
        self._add_patch_row(["palm_center", "掌心", "手掌", "8", "8", "", "identity", "identity"])

    def tactile_layout_specs(self) -> list[dict]:
        specs: list[dict] = []
        seen: set[str] = set()
        for row in range(self.layout_table.rowCount()):
            items = [self.layout_table.item(row, col) for col in range(len(self.LAYOUT_COLUMNS))]
            values = [(item.text().strip() if item else "") for item in items]
            (
                name,
                display_name,
                group,
                rows_text,
                cols_text,
                site_prefix,
                display_transform,
                site_transform,
            ) = values
            if not name:
                continue
            if name in seen:
                raise ValueError(f"触觉区域 ID 重复：{name}")
            seen.add(name)
            if display_transform not in self.TRANSFORMS:
                raise ValueError(f"触觉区域 {name} 的 2D显示变换不支持：{display_transform}")
            if site_transform not in self.TRANSFORMS:
                raise ValueError(f"触觉区域 {name} 的 3D site变换不支持：{site_transform}")
            rows = int(rows_text)
            cols = int(cols_text)
            if rows <= 0 or cols <= 0:
                raise ValueError(f"触觉区域 {name} 的行数和列数必须大于 0")
            spec = {
                "name": name,
                "display_name": display_name or name,
                "group": group or "触觉",
                "rows": rows,
                "cols": cols,
            }
            if site_prefix:
                spec["site_prefix"] = site_prefix
            if display_transform:
                spec["display_transform"] = display_transform
            if site_transform:
                spec["site_transform"] = site_transform
            specs.append(spec)
        return specs

    def set_tactile_config(self, tactile: dict):
        _set_combo_value(self.kind, str(tactile.get("kind", "none")))
        self.baud.setValue(int(tactile.get("default_baudrate", self.baud.value()) or self.baud.value()))
        config = tactile.get("adapter_config")
        if isinstance(config, dict):
            _set_combo_value(self.frame_format, str(config.get("frame_format", _combo_value(self.frame_format))))
            self.value_scale.setValue(float(config.get("value_scale", self.value_scale.value())))
            self.baseline_frames.setValue(int(config.get("baseline_frames", self.baseline_frames.value())))
            self.baseline_threshold.setValue(float(config.get("baseline_threshold", self.baseline_threshold.value())))
        self.layout_table.setRowCount(0)
        layout = tactile.get("layout")
        patches = layout.get("patches") if isinstance(layout, dict) else []
        if isinstance(patches, list):
            for patch in patches:
                if not isinstance(patch, dict):
                    continue
                self._add_patch_row([
                    str(patch.get("name") or ""),
                    str(patch.get("display_name") or patch.get("name") or ""),
                    str(patch.get("group") or "触觉"),
                    str(patch.get("rows", 1)),
                    str(patch.get("cols", 1)),
                    str(patch.get("site_prefix") or ""),
                    str(patch.get("display_transform") or "identity"),
                    str(patch.get("site_transform") or "identity"),
                ])

    def tactile_manifest(self, package_dir: Path) -> dict:
        tactile = {
            "kind": _combo_value(self.kind),
            "default_baudrate": self.baud.value(),
            "adapter_config": {
                "frame_format": _combo_value(self.frame_format),
                "value_scale": self.value_scale.value(),
                "baseline_frames": self.baseline_frames.value(),
                "baseline_threshold": self.baseline_threshold.value(),
            },
        }
        layout_specs = self.tactile_layout_specs()
        if layout_specs:
            tactile["layout"] = {"patches": layout_specs}
        if tactile["kind"] == "external_python":
            adapter_rel = _copy_adapter_source(
                self.adapter_path.text(),
                package_dir,
                "tactile_adapter.py",
                "tactile_adapter",
                TACTILE_ADAPTER_TEMPLATE,
            )
            tactile["adapter_path"] = adapter_rel
            tactile["adapter_class"] = "TactileAdapter"
        return tactile

    def validatePage(self) -> bool:
        kind = _combo_value(self.kind)
        try:
            layout_specs = self.tactile_layout_specs()
        except ValueError as exc:
            QMessageBox.warning(self, "触觉区域配置有误", str(exc))
            return False
        if _combo_value(self.frame_format) == "patches" and kind == "external_python" and not layout_specs:
            QMessageBox.warning(
                self,
                "缺少触觉区域表",
                "选择 patches 格式时，请至少添加一个触觉区域；每个区域对应适配器返回的一个二维数组。",
            )
            return False
        if kind == "external_python":
            ok, message = _adapter_entry_exists(self.adapter_path.text(), "tactile_adapter.py")
            if not ok:
                QMessageBox.warning(self, "缺少触觉适配器", message)
                return False
            src = Path(self.adapter_path.text().strip())
            if src.is_file() and src.name != "tactile_adapter.py":
                QMessageBox.warning(self, "入口文件名不正确", "触觉适配器入口文件必须命名为 tactile_adapter.py。")
                return False
        return True


class TeleopPage(QWizardPage):
    def __init__(self):
        super().__init__()
        self.setTitle("第 6 步：遥操作接入")
        self.setSubTitle("把你已有的遥操作程序封装成 TeleopAdapter.retarget，或使用内置重定向方式。")
        layout = QVBoxLayout(self)

        info = QTextEdit()
        info.setReadOnly(True)
        info.setMaximumHeight(145)
        info.setPlainText(
            "遥操作适配器的任务很单纯：输入人手关键点或特征，输出当前手型的关节角度列表。\n"
            "返回角度单位必须是度，长度必须等于第 3 步关节数量，顺序也必须一致。\n"
            "如果你已有独立遥操作程序，请导入整个程序目录，并在 teleop_adapter.py 中调用它；"
            "上位机不关心内部算法，只调用 TeleopAdapter.retarget(hand_payload)。"
        )
        layout.addWidget(info)

        form = QFormLayout()
        self.kind = QComboBox()
        self.kind.addItem("不启用遥操作", "none")
        self.kind.addItem("Python 遥操作适配器（推荐）", "external_python")
        self.smoothing_alpha = QDoubleSpinBox()
        self.smoothing_alpha.setRange(0.0, 1.0)
        self.smoothing_alpha.setSingleStep(0.05)
        self.smoothing_alpha.setValue(0.55)
        self.rate_limit = QDoubleSpinBox()
        self.rate_limit.setRange(0.1, 100.0)
        self.rate_limit.setSingleStep(0.5)
        self.rate_limit.setValue(8.0)
        self.rate_limit.setSuffix(" rad/s")
        self.paired_k = QSpinBox()
        self.paired_k.setRange(1, 50)
        self.paired_k.setValue(5)
        self.paired_sigma = QDoubleSpinBox()
        self.paired_sigma.setRange(0.01, 10.0)
        self.paired_sigma.setSingleStep(0.05)
        self.paired_sigma.setValue(0.45)
        form.addRow("遥操作方式", self.kind)
        form.addRow("平滑系数", self.smoothing_alpha)
        form.addRow("最大角速度", self.rate_limit)
        form.addRow("样本匹配邻居数", self.paired_k)
        form.addRow("样本匹配宽度", self.paired_sigma)
        layout.addLayout(form)

        row = QHBoxLayout()
        self.adapter_path = QLineEdit()
        browse = QPushButton("导入 teleop_adapter.py")
        browse.clicked.connect(self._browse)
        browse_dir = QPushButton("导入遥操作目录")
        browse_dir.clicked.connect(self._browse_dir)
        save_template = QPushButton("生成接口模板")
        save_template.clicked.connect(self._save_template)
        row.addWidget(self.adapter_path, 1)
        row.addWidget(browse)
        row.addWidget(browse_dir)
        row.addWidget(save_template)
        layout.addLayout(row)

        self.config_path = QLineEdit()
        self.pair_path = QLineEdit()
        self.motor_cal_path = QLineEdit()
        self.config_path.setVisible(False)
        self.pair_path.setVisible(False)
        self.motor_cal_path.setVisible(False)
        self._extra_adapter_config: dict[str, object] = {}
        self._teleop_joint_names: list[str] = []
        file_form = QFormLayout()
        resource_hint = QLabel(
            "遥操作只需要导入一个目录。目录里必须包含 teleop_adapter.py；如果你的算法还需要 "
            "JSON 标定、模型权重、第三方 Python 文件或厂家程序，请全部放在同一目录。"
            "上位机会整目录复制，teleop_adapter.py 可用 Path(__file__).parent 读取这些资源。"
        )
        resource_hint.setWordWrap(True)
        file_form.addRow("遥操作资源", resource_hint)
        self.tip_links = QLineEdit()
        self.tip_links.setPlaceholderText("可选，例如 thumb_tip,index_tip,middle_tip,ring_tip")
        self.wrist_link = QLineEdit("base_link")
        file_form.addRow("指尖 link", self.tip_links)
        file_form.addRow("腕部 link", self.wrist_link)
        layout.addLayout(file_form)

        template = QTextEdit()
        template.setPlainText(TELEOP_ADAPTER_TEMPLATE)
        template.setReadOnly(True)
        layout.addWidget(template, 1)

    def _browse(self):
        path, _ = QFileDialog.getOpenFileName(self, "选择遥操作适配器", "", "Python 文件 (*.py);;所有文件 (*)")
        if path:
            self.adapter_path.setText(path)
            self.wizard().maybe_load_kit_from_path(Path(path))

    def _browse_dir(self):
        path = QFileDialog.getExistingDirectory(self, "选择包含 teleop_adapter.py 的遥操作适配器目录")
        if path:
            self.adapter_path.setText(path)
            self.wizard().maybe_load_kit_from_path(Path(path))

    def _save_template(self):
        path, _ = QFileDialog.getSaveFileName(self, "生成遥操作适配器模板", "teleop_adapter.py", "Python 文件 (*.py)")
        if path:
            _copy_text_template(Path(path), TELEOP_ADAPTER_TEMPLATE)
            self.adapter_path.setText(path)
            self.wizard().maybe_load_kit_from_path(Path(path))

    def _file_row(self, line_edit: QLineEdit, title: str, file_filter: str):
        button = QPushButton("选择")
        button.clicked.connect(lambda: self._browse_into(line_edit, title, file_filter))
        row = QHBoxLayout()
        row.addWidget(line_edit, 1)
        row.addWidget(button)
        return row

    def _browse_into(self, line_edit: QLineEdit, title: str, file_filter: str):
        path, _ = QFileDialog.getOpenFileName(self, title, "", file_filter)
        if path:
            line_edit.setText(path)
            self.wizard().maybe_load_kit_from_path(Path(path))

    def set_teleop_config(self, teleop: dict):
        _set_combo_value(self.kind, str(teleop.get("kind", "none")))
        self.wrist_link.setText(str(teleop.get("wrist_link_name") or "base_link"))
        tip_links = teleop.get("tip_link_names")
        if isinstance(tip_links, list):
            self.tip_links.setText(",".join(str(item) for item in tip_links))
        config = teleop.get("adapter_config")
        if isinstance(config, dict):
            self._extra_adapter_config = {
                key: value
                for key, value in config.items()
                if key not in {
                    "input",
                    "output_unit",
                    "smoothing_alpha",
                    "rate_limit_rad_per_s",
                    "paired_k",
                    "paired_sigma",
                }
            }
            self.smoothing_alpha.setValue(float(config.get("smoothing_alpha", self.smoothing_alpha.value())))
            self.rate_limit.setValue(float(config.get("rate_limit_rad_per_s", self.rate_limit.value())))
            self.paired_k.setValue(int(config.get("paired_k", self.paired_k.value())))
            self.paired_sigma.setValue(float(config.get("paired_sigma", self.paired_sigma.value())))
        else:
            self._extra_adapter_config = {}
        joint_names = teleop.get("joint_names")
        self._teleop_joint_names = [str(item) for item in joint_names] if isinstance(joint_names, list) else []

    def teleop_manifest(self, package_dir: Path) -> dict:
        adapter_config = dict(self._extra_adapter_config)
        adapter_config.update({
            "input": "mediapipe_21x3_or_features",
            "output_unit": "degree",
            "smoothing_alpha": self.smoothing_alpha.value(),
            "rate_limit_rad_per_s": self.rate_limit.value(),
            "paired_k": self.paired_k.value(),
            "paired_sigma": self.paired_sigma.value(),
        })
        teleop = {
            "kind": _combo_value(self.kind),
            "wrist_link_name": self.wrist_link.text().strip() or "base_link",
            "adapter_config": adapter_config,
        }
        tip_links = [item.strip() for item in self.tip_links.text().split(",") if item.strip()]
        if tip_links:
            teleop["tip_link_names"] = tip_links
        if self._teleop_joint_names:
            teleop["joint_names"] = list(self._teleop_joint_names)
        if teleop["kind"] == "external_python":
            adapter_src = Path(self.adapter_path.text().strip())
            adapter_rel = _copy_adapter_source(
                self.adapter_path.text(),
                package_dir,
                "teleop_adapter.py",
                "teleop_adapter",
            )
            teleop["adapter_path"] = adapter_rel
            teleop["adapter_class"] = "TeleopAdapter"
            adapter_dir = adapter_src if adapter_src.is_dir() else adapter_src.parent
            for src_text, filename, manifest_key in (
                (self.config_path.text(), "teleop_config.json", "config"),
                (self.pair_path.text(), "teleop_pair_calibration.json", "pair_calibration"),
                (self.motor_cal_path.text(), "motor_to_sim_calibration.json", "motor_to_sim_calibration"),
            ):
                src = Path(src_text.strip()) if src_text.strip() else adapter_dir / filename
                if src.exists() and _copy_optional_file(str(src), package_dir / filename):
                    teleop[manifest_key] = filename
        return teleop

    def validatePage(self) -> bool:
        if _combo_value(self.kind) == "external_python":
            ok, message = _adapter_entry_exists(self.adapter_path.text(), "teleop_adapter.py")
            if not ok:
                QMessageBox.warning(self, "缺少遥操作适配器", message)
                return False
            src = Path(self.adapter_path.text().strip())
            if src.is_file() and src.name != "teleop_adapter.py":
                QMessageBox.warning(self, "入口文件名不正确", "遥操作适配器入口文件必须命名为 teleop_adapter.py。")
                return False
        return True


class VerificationPage(QWizardPage):
    def __init__(self):
        super().__init__()
        self.setTitle("第 7 步：接口校验与联机验证")
        self.setSubTitle("保存手型包之前，先检查 manifest、XML 资源、Python 适配器和可选硬件连接。")
        self._passed = False

        layout = QVBoxLayout(self)
        intro = QLabel(
            "这一步会在临时目录中生成一次完整手型包，然后导入并检查电机、触觉、遥操作适配器。"
            "如果你已经把新手接到电脑上，也可以填写串口并勾选冒烟测试；冒烟测试只执行连接、使能状态检查和断开，不会主动下发运动角度。"
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        form = QFormLayout()
        self.port = QLineEdit()
        self.port.setPlaceholderText("例如 COM3；不做硬件验证可留空")
        self.baud = QLineEdit("115200")
        form.addRow("硬件串口：", self.port)
        form.addRow("波特率：", self.baud)
        layout.addLayout(form)

        self.motor_smoke = QCheckBox("对电机适配器执行连接/断开冒烟测试")
        self.tactile_smoke = QCheckBox("对外部触觉适配器执行连接/断开冒烟测试")
        layout.addWidget(self.motor_smoke)
        layout.addWidget(self.tactile_smoke)

        run_btn = QPushButton("执行接口检查")
        run_btn.clicked.connect(self._run_check)
        layout.addWidget(run_btn)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(220)
        layout.addWidget(self.log, 1)

    def initializePage(self):
        self._passed = False
        self.log.clear()
        self.log.append("请点击“执行接口检查”。检查通过后才能进入最终保存。")
        self.completeChanged.emit()

    def isComplete(self) -> bool:
        return self._passed

    def validatePage(self) -> bool:
        if not self._passed:
            QMessageBox.warning(self, "还未验证", "请先执行接口检查，确认临时手型包可以被上位机加载。")
            return False
        return True

    def _append(self, text: str):
        self.log.append(text)

    def _run_check(self):
        self._passed = False
        self.log.clear()
        try:
            baudrate = int(self.baud.text().strip() or "115200")
        except ValueError:
            QMessageBox.warning(self, "波特率错误", "波特率必须是整数。")
            return

        try:
            with tempfile.TemporaryDirectory(prefix="dexhand_verify_") as temp_dir:
                package_dir = Path(temp_dir) / self.wizard().basic.hand_id.text().strip()
                manifest_path = self.wizard()._write_package(package_dir)
                self._append(f"临时手型包已生成：{manifest_path}")

                profile = load_profile_from_manifest(manifest_path)
                self._append(f"manifest 已加载：{profile.display_name}，共 {profile.num_joints} 个关节。")

                motor_adapter = load_python_adapter(profile)
                self._append("电机 Python 适配器接口检查通过。")
                try:
                    if self.motor_smoke.isChecked():
                        port = self.port.text().strip()
                        if not port:
                            raise RuntimeError("电机冒烟测试需要填写串口。")
                        if motor_adapter.connect(port, baudrate):
                            self._append("电机适配器连接成功。")
                        else:
                            raise RuntimeError("电机适配器 connect 返回 False。")
                        motor_adapter.enable_all(False)
                        self._append("电机适配器使能接口调用成功。")
                    else:
                        self._append("已跳过电机硬件冒烟测试。")
                finally:
                    try:
                        motor_adapter.disconnect()
                    finally:
                        motor_adapter.cleanup()

                if profile.tactile_kind == "external_python":
                    from app.services.tactile_service import _load_external_tactile_adapter

                    tactile_adapter = _load_external_tactile_adapter(profile)
                    self._append("外部触觉适配器接口检查通过。")
                    try:
                        if self.tactile_smoke.isChecked():
                            port = self.port.text().strip()
                            if not port:
                                raise RuntimeError("触觉冒烟测试需要填写串口。")
                            if tactile_adapter.connect(port, baudrate):
                                self._append("触觉适配器连接成功。")
                            else:
                                raise RuntimeError("触觉适配器 connect 返回 False。")
                            sample = None
                            for _ in range(20):
                                sample = tactile_adapter.read_frame(timeout=0.1)
                                if sample:
                                    break
                            if not isinstance(sample, dict):
                                raise RuntimeError("触觉连接成功，但 2 秒内没有读到标准触觉帧，请检查串口、波特率和 read_frame 返回格式。")
                            self._append(f"触觉帧读取成功，字段: {', '.join(sample.keys())}")
                        else:
                            self._append("已跳过触觉硬件冒烟测试。")
                    finally:
                        tactile_adapter.disconnect()
                        tactile_adapter.cleanup()
                else:
                    self._append(f"触觉能力类型为 {profile.tactile_kind}，不需要外部触觉适配器检查。")

                if profile.teleop_kind == "external_python":
                    from app.widgets.sim_teleop_page import ExternalPythonTeleopRetargeter

                    ExternalPythonTeleopRetargeter(profile)
                    self._append("外部遥操作适配器接口检查通过。")
                else:
                    self._append(f"遥操作能力类型为 {profile.teleop_kind}，不需要外部遥操作适配器检查。")

            self._append("全部检查通过，可以进入最终保存。")
            self._passed = True
            self.completeChanged.emit()
        except Exception as exc:
            self._append(f"检查失败：{exc}")
            QMessageBox.critical(self, "接口检查失败", str(exc))
            self.completeChanged.emit()


class SummaryPage(QWizardPage):
    def __init__(self):
        super().__init__()
        self.setTitle("第 8 步：生成手型包")
        layout = QVBoxLayout(self)
        self.summary = QTextEdit()
        self.summary.setReadOnly(True)
        layout.addWidget(self.summary)

    def initializePage(self):
        wizard = self.wizard()
        self.summary.setPlainText(
            f"即将生成：app/resources/hands/{wizard.basic.hand_id.text().strip()}\n"
            f"模型 XML：{wizard.model.xml_path.text().strip()}\n"
            f"关节数量：{sum(len(f['joints']) for f in wizard.joints.fingers())}\n"
            f"电机适配器：{'占位模板' if wizard.motor.placeholder.isChecked() else wizard.motor.driver_path.text().strip()}\n"
            f"触觉区域：{len(wizard.tactile.tactile_layout_specs())} 个\n"
            f"触觉类型：{wizard.tactile.kind.currentText()}\n"
            f"遥操作类型：{wizard.teleop.kind.currentText()}\n\n"
            "点击完成后，上位机会复制资源、写入 manifest.json、校验电机适配器接口，并刷新手型列表。"
        )


class HandIntegrationWizard(QWizard):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("新灵巧手接入向导")
        self.resize(980, 720)
        self._kit_payload: dict | None = None
        self._kit_dir: Path | None = None
        self.intro = IntroPage()
        self.basic = BasicPage()
        self.model = ModelPage()
        self.joints = JointMapPage()
        self.motor = MotorAdapterPage()
        self.tactile = TactilePage()
        self.teleop = TeleopPage()
        self.verify = VerificationPage()
        self.summary = SummaryPage()
        for page in (
            self.intro,
            self.basic,
            self.model,
            self.joints,
            self.motor,
            self.tactile,
            self.teleop,
            self.verify,
            self.summary,
        ):
            self.addPage(page)
        self.setButtonText(QWizard.WizardButton.NextButton, "下一步")
        self.setButtonText(QWizard.WizardButton.BackButton, "上一步")
        self.setButtonText(QWizard.WizardButton.FinishButton, "完成接入")
        self.setButtonText(QWizard.WizardButton.CancelButton, "取消")

    def _find_kit_dir_for_path(self, value: Path) -> Path | None:
        try:
            path = Path(value).resolve()
        except OSError:
            path = Path(value)
        start = path if path.is_dir() else path.parent
        for current in (start, *start.parents):
            if (current / "kit.json").exists():
                return current
        return None

    def maybe_load_kit_from_path(self, value: Path) -> bool:
        kit_dir = self._find_kit_dir_for_path(value)
        if kit_dir is None:
            return False
        if self._kit_dir is not None:
            try:
                if self._kit_dir.resolve() == kit_dir.resolve():
                    return False
            except OSError:
                if self._kit_dir == kit_dir:
                    return False
        try:
            self.load_kit(kit_dir)
        except Exception as exc:
            QMessageBox.warning(self, "接入材料加载失败", str(exc))
            return False
        return True

    def _kit_path(self, value: str | None) -> Path | None:
        if not value:
            return None
        path = Path(value)
        if not path.is_absolute():
            if self._kit_dir is None:
                return None
            path = self._kit_dir / path
        return path

    def load_kit(self, kit_dir: Path):
        kit_dir = Path(kit_dir)
        kit_path = kit_dir / "kit.json"
        if not kit_path.exists():
            raise RuntimeError("接入材料目录中没有 kit.json。")
        payload = json.loads(kit_path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise RuntimeError("kit.json 必须是 JSON 对象。")

        self._kit_dir = kit_dir
        self._kit_payload = payload
        self.basic.hand_id.setText(str(payload.get("hand_id") or kit_dir.name))
        self.basic.display_name.setText(str(payload.get("display_name") or payload.get("hand_id") or kit_dir.name))

        model_xml = self._kit_path(str(payload.get("model_xml") or "model/hand.xml"))
        if model_xml is not None:
            self.model.xml_path.setText(str(model_xml))
            self.model._refresh_preview()

        fingers = payload.get("fingers")
        if isinstance(fingers, list):
            self.joints.set_fingers(fingers)

        motor = payload.get("motor")
        if isinstance(motor, dict):
            self.motor.set_motor_config(motor)
        driver_source = payload.get("driver_source")
        if not driver_source and isinstance(payload.get("driver"), dict):
            driver_source = payload["driver"].get("source")
        driver_path = self._kit_path(str(driver_source)) if driver_source else None
        if driver_path is not None:
            self.motor.driver_path.setText(str(driver_path))

        tactile = payload.get("tactile")
        if isinstance(tactile, dict):
            self.tactile.set_tactile_config(tactile)
            tactile_source = tactile.get("adapter_source") or tactile.get("source")
            tactile_path = self._kit_path(str(tactile_source)) if tactile_source else None
            if tactile_path is not None:
                self.tactile.adapter_path.setText(str(tactile_path))

        teleop = payload.get("teleop")
        if isinstance(teleop, dict):
            self.teleop.set_teleop_config(teleop)
            teleop_source = teleop.get("adapter_source") or teleop.get("source")
            teleop_path = self._kit_path(str(teleop_source)) if teleop_source else None
            if teleop_path is not None:
                self.teleop.adapter_path.setText(str(teleop_path))
            for manifest_key, line_edit in (
                ("config", self.teleop.config_path),
                ("pair_calibration", self.teleop.pair_path),
                ("motor_to_sim_calibration", self.teleop.motor_cal_path),
            ):
                resource_path = self._kit_path(str(teleop.get(manifest_key))) if teleop.get(manifest_key) else None
                if resource_path is not None:
                    line_edit.setText(str(resource_path))

        self.setWindowTitle(f"新灵巧手接入向导 - 已载入 {kit_dir.name}")

    def accept(self):
        manifest_path: Path | None = None
        try:
            manifest_path = self._create_package()
            profile = load_profile_from_manifest(manifest_path)
            if profile.driver_kind == "python":
                load_python_adapter(profile)
        except Exception as exc:
            if manifest_path is not None:
                shutil.rmtree(manifest_path.parent, ignore_errors=True)
            QMessageBox.critical(self, "接入失败", str(exc))
            return
        reload_profiles(profile.hand_id)
        QMessageBox.information(self, "接入完成", f"{profile.display_name} 已加入上位机。")
        super().accept()

    def _create_package(self) -> Path:
        hand_id = self.basic.hand_id.text().strip()
        package_dir = HANDS_DIR / hand_id
        return self._write_package(package_dir)

    def _write_package(self, package_dir: Path) -> Path:
        hand_id = self.basic.hand_id.text().strip()
        package_dir.mkdir(parents=True, exist_ok=False)
        (package_dir / "gestures").mkdir(exist_ok=True)

        if self.motor.placeholder.isChecked():
            driver_rel = _copy_adapter_source(
                "",
                package_dir,
                "driver.py",
                "driver_adapter",
                MOTOR_ADAPTER_TEMPLATE,
            )
        else:
            driver_rel = _copy_adapter_source(
                self.motor.driver_path.text(),
                package_dir,
                "driver.py",
                "driver_adapter",
            )

        model_src = Path(self.model.xml_path.text().strip())
        model_dir = package_dir / "model"
        shutil.copytree(model_src.parent, model_dir, dirs_exist_ok=True)

        tactile = self.tactile.tactile_manifest(package_dir)
        teleop = self.teleop.teleop_manifest(package_dir)

        fingers = self.joints.fingers()
        calibration = {
            "joints": [
                {
                    "name": joint["name"],
                    "motor_id": joint["motor_id"],
                    "min_angle": joint["min_angle"],
                    "max_angle": joint["max_angle"],
                    "default_angle": joint["default_angle"],
                }
                for finger in fingers
                for joint in finger["joints"]
            ]
        }
        (package_dir / "calibration.json").write_text(
            json.dumps(calibration, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

        open_pose = [joint["default_angle"] for finger in fingers for joint in finger["joints"]]
        gestures = {"builtin": {"张开": open_pose}}
        if isinstance(self._kit_payload, dict):
            kit_gestures = self._kit_payload.get("gestures")
            if isinstance(kit_gestures, dict) and isinstance(kit_gestures.get("builtin"), dict):
                gestures = {"builtin": kit_gestures["builtin"]}
        motor = self.motor.motor_manifest()
        motor["actuator_order"] = [
            joint.get("actuator_name") or joint["name"]
            for finger in fingers
            for joint in finger["joints"]
        ]
        manifest = {
            "schema": "dexhand.hand.v1",
            "hand_id": hand_id,
            "display_name": self.basic.display_name.text().strip(),
            "preset": False,
            "removable": True,
            "driver": {"kind": "python", "path": driver_rel, "class": "HandDriver"},
            "motor": motor,
            "tactile": tactile,
            "teleop": teleop,
            "resources": {
                "model": f"model/{model_src.name}",
                "calibration": "calibration.json",
                "gestures": "gestures",
            },
            "fingers": fingers,
            "gestures": gestures,
        }

        errors = validate_manifest_payload(manifest, package_dir)
        if errors:
            shutil.rmtree(package_dir, ignore_errors=True)
            raise RuntimeError("\n".join(errors))

        manifest_path = package_dir / "manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return manifest_path


class HandManagerPage(QWidget):
    def __init__(self, hand_state, parent=None):
        super().__init__(parent)
        self.hand_state = hand_state
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(12)

        top = QHBoxLayout()
        title = QLabel("机型管理")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #333;")
        top.addWidget(title)
        top.addStretch()
        add_btn = QPushButton("添加新手型")
        add_btn.clicked.connect(self._open_wizard)
        refresh_btn = QPushButton("刷新列表")
        refresh_btn.clicked.connect(self.refresh)
        top.addWidget(add_btn)
        top.addWidget(refresh_btn)
        layout.addLayout(top)

        hint = QLabel(
            "上位机初始不内置任何真实手型。每个手型都必须通过接入向导生成独立手型包；"
            "示例接入材料位于项目根目录 hand_integration_kits。删除已接入手型后会移动到 app/resources/hands/.trash。"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #666;")
        layout.addWidget(hint)

        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["手型 ID", "显示名称", "关节数", "驱动", "触觉", "遥操作", "手型包目录"])
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        layout.addWidget(self.table, 1)

        actions = QHBoxLayout()
        delete_btn = QPushButton("删除或隐藏选中手型")
        delete_btn.clicked.connect(self._delete_selected)
        actions.addWidget(delete_btn)
        actions.addStretch()
        layout.addLayout(actions)
        self.refresh()

    def refresh(self):
        profiles = get_profiles()
        self.table.setRowCount(len(profiles))
        for row, profile in enumerate(profiles.values()):
            values = [
                profile.hand_id,
                profile.display_name,
                str(profile.num_joints),
                f"{profile.driver_kind}:{profile.driver_name or ''}",
                profile.tactile_kind,
                profile.teleop_kind,
                str(profile.package_dir or ""),
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                if col == 0:
                    item.setData(Qt.ItemDataRole.UserRole, profile.hand_id)
                self.table.setItem(row, col, item)
        self.table.resizeColumnsToContents()
        if not profiles:
            self.table.setRowCount(1)
            message = QTableWidgetItem("暂无已接入手型，请点击“添加新手型”")
            self.table.setItem(0, 0, message)

    def _selected_hand_id(self) -> str | None:
        row = self.table.currentRow()
        if row < 0:
            return None
        item = self.table.item(row, 0)
        return item.data(Qt.ItemDataRole.UserRole) if item else None

    def _open_wizard(self):
        wizard = HandIntegrationWizard(self)
        if wizard.exec():
            self._notify_main_window()
            self.refresh()

    def _delete_selected(self):
        hand_id = self._selected_hand_id()
        if not hand_id:
            QMessageBox.information(self, "未选择手型", "请先在列表中选择一个手型。")
            return
        if hand_id == self.hand_state.profile.hand_id:
            QMessageBox.warning(self, "当前手型正在使用", "请先切换到其他手型，再删除或隐藏它。")
            return
        reply = QMessageBox.question(
            self,
            "确认删除或隐藏",
            f"确定处理 {hand_id} 吗？普通手型会移动到 .trash，预置手型会从列表隐藏。",
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            delete_profile(hand_id)
        except Exception as exc:
            QMessageBox.critical(self, "操作失败", str(exc))
            return
        self._notify_main_window()
        self.refresh()

    def _notify_main_window(self, preferred_hand_id: str | None = None):
        main_win = self.window()
        if hasattr(main_win, "refresh_hand_profiles"):
            main_win.refresh_hand_profiles(preferred_hand_id)
