"""Shared hand profile types and helpers."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path


APP_DIR = Path(__file__).resolve().parents[1]
PROJECT_DIR = APP_DIR.parent
RESOURCES_DIR = APP_DIR / "resources"
HANDS_DIR = RESOURCES_DIR / "hands"
HAND_KITS_DIR = PROJECT_DIR / "hand_integration_kits"
NUM_JOINTS = 0  # 仅为旧导入保留；新代码必须使用 HandProfile.num_joints。


@dataclass
class JointInfo:
    name: str
    display_name: str
    motor_id: int
    min_angle: float
    max_angle: float
    default_angle: float
    torque_limit: float = 150.0
    actuator_name: str | None = None


@dataclass
class FingerConfig:
    name: str
    display_name: str
    joints: list[JointInfo]


@dataclass
class TactilePatchInfo:
    name: str
    display_name: str
    rows: int
    cols: int
    group: str = "触觉"
    site_prefix: str | None = None
    display_transform: str = "identity"
    site_transform: str = "identity"


@dataclass
class HandProfile:
    hand_id: str
    display_name: str
    finger_configs: list[FingerConfig]
    model_path: Path
    calibration_path: Path
    gestures_dir: Path
    motor_kind: str
    protocols: tuple[str, ...]
    default_protocol: str
    default_baudrates: dict[str, int]
    actuator_order: tuple[str, ...]
    force_control_supported: bool = True
    motor_adapter_config: dict[str, object] = field(default_factory=dict)
    tactile_kind: str = "none"
    tactile_default_baudrate: int = 115200
    tactile_adapter_path: Path | None = None
    tactile_adapter_class: str = "TactileAdapter"
    tactile_adapter_config: dict[str, object] = field(default_factory=dict)
    tactile_layout: tuple[TactilePatchInfo, ...] = field(default_factory=tuple)
    teleop_kind: str = "none"
    teleop_config_path: Path | None = None
    teleop_pair_calibration_path: Path | None = None
    motor_to_sim_calibration_path: Path | None = None
    teleop_adapter_path: Path | None = None
    teleop_adapter_class: str = "TeleopAdapter"
    teleop_adapter_config: dict[str, object] = field(default_factory=dict)
    teleop_joint_names: tuple[str, ...] | None = None
    teleop_tip_link_names: tuple[str, ...] | None = None
    teleop_wrist_link_name: str = "base_link"
    default_gestures: dict[str, list[float]] = field(default_factory=dict)
    package_dir: Path | None = None
    driver_kind: str = "python"
    driver_name: str | None = None
    driver_path: Path | None = None
    driver_class: str = "HandDriver"
    preset: bool = False
    removable: bool = True

    @property
    def joint_names(self) -> list[str]:
        return [j.name for f in self.finger_configs for j in f.joints]

    @property
    def actuator_names(self) -> list[str]:
        return [
            j.actuator_name or j.name
            for f in self.finger_configs
            for j in f.joints
        ]

    @property
    def num_joints(self) -> int:
        return len(self.joint_names)

    @property
    def motor_id_to_joint_index(self) -> dict[int, int]:
        mapping: dict[int, int] = {}
        idx = 0
        for finger in self.finger_configs:
            for joint in finger.joints:
                mapping[joint.motor_id] = idx
                idx += 1
        return mapping

    @property
    def joint_index_to_motor_id(self) -> dict[int, int]:
        return {idx: mid for mid, idx in self.motor_id_to_joint_index.items()}

    def default_baudrate(self, protocol: str | None = None) -> int:
        return self.default_baudrates.get(protocol or self.default_protocol, 230400)


def joint(
    name: str,
    display_name: str,
    motor_id: int,
    min_angle: float = -200.0,
    max_angle: float = 200.0,
    default_angle: float = 0.0,
    actuator_name: str | None = None,
) -> JointInfo:
    return JointInfo(
        name=name,
        display_name=display_name,
        motor_id=motor_id,
        min_angle=min_angle,
        max_angle=max_angle,
        default_angle=default_angle,
        actuator_name=actuator_name,
    )


def joint_from_rad(
    name: str,
    display_name: str,
    motor_id: int,
    actuator_name: str,
    min_rad: float,
    max_rad: float,
    default_rad: float = 0.0,
) -> JointInfo:
    return joint(
        name=name,
        display_name=display_name,
        motor_id=motor_id,
        min_angle=round(math.degrees(min_rad), 2),
        max_angle=round(math.degrees(max_rad), 2),
        default_angle=round(math.degrees(default_rad), 2),
        actuator_name=actuator_name,
    )


def degrees_from_radians(values: list[float]) -> list[float]:
    return [round(math.degrees(v), 2) for v in values]


def apply_calibration_file(profile: HandProfile) -> None:
    if not profile.calibration_path.exists():
        return
    try:
        payload = json.loads(profile.calibration_path.read_text(encoding="utf-8"))
    except Exception:
        return
    joints = payload.get("joints", [])
    if not isinstance(joints, list) or len(joints) != profile.num_joints:
        return

    profile_joints = [j for f in profile.finger_configs for j in f.joints]
    for joint_info, saved in zip(profile_joints, joints):
        if not isinstance(saved, dict):
            continue
        joint_info.min_angle = float(saved.get("min_angle", joint_info.min_angle))
        joint_info.max_angle = float(saved.get("max_angle", joint_info.max_angle))
        joint_info.default_angle = float(saved.get("default_angle", joint_info.default_angle))
