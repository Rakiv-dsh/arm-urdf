"""手部运行状态模型。

核心状态只依赖当前 HandProfile：关节数量、motor_id 映射和触觉 patch 布局都
来自手型包。旧的 finger/palm/raw 触觉接口保留为兼容入口，但不会再给核心
上位机引入固定四指或 6x8 布局假设。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
from PySide6.QtCore import QObject, Signal

from app.hands import (
    HandProfile,
    get_active_profile,
    set_active_profile,
)
from app.models.tactile_layout import (
    empty_tactile_patches_for_profile,
    patch_order_for_profile,
    patch_shapes_for_profile,
)


# Legacy compatibility constants. New code should not use these as hand facts.
NUM_FINGER_TACTILE_SEGMENTS = 0
NUM_FINGERS = 0
NUM_PALM_TACTILE = 0
NUM_TACTILE_TOTAL = 0
TACTILE_ROWS = 0
TACTILE_COLS = 0


@dataclass
class MotorTelemetry:
    """单个电机的遥测数据。"""

    motor_id: int = 0
    position: float = 0.0
    velocity: float = 0.0
    current: float = 0.0
    torque: float = 0.0
    temperature: float = 0.0
    bus_voltage: float = 0.0
    online: bool = False
    error_code: int = 0
    fsm_state: int = 0
    timestamp: float = field(default_factory=time.time)


class HandState(QObject):
    """当前手型的关节、电机遥测和触觉数据状态。"""

    joint_angles_changed = Signal(list)
    single_joint_changed = Signal(int, float)
    motor_telemetry_changed = Signal(object)
    tactile_data_changed = Signal()
    profile_changed = Signal(object)

    def __init__(self, parent=None, profile: HandProfile | None = None):
        super().__init__(parent)
        self.profile: HandProfile = profile or get_active_profile()
        set_active_profile(self.profile.hand_id)
        self._angles = np.zeros(self.profile.num_joints, dtype=np.float64)
        self._motor_telemetry: dict[int, MotorTelemetry] = {}
        self._tactile_patches: dict[str, np.ndarray] = {}
        self._raw_tactile = np.zeros((0, 0), dtype=np.float64)
        self._finger_tactile = np.zeros((0, 0), dtype=np.float64)
        self._palm_tactile = np.zeros(0, dtype=np.float64)
        self._tactile_forces = np.zeros(0, dtype=np.float64)
        self._tactile_centers = np.zeros((0, 3), dtype=np.float64)
        self._reset_for_profile()

    def set_profile(self, profile: HandProfile):
        """Switch the active hand profile and reset runtime state."""

        self.profile = profile
        set_active_profile(profile.hand_id)
        self._reset_for_profile()
        self.profile_changed.emit(profile)

    def _reset_for_profile(self):
        self._angles = np.array(
            [j.default_angle for f in self.profile.finger_configs for j in f.joints],
            dtype=np.float64,
        )
        self._motor_telemetry = {
            mid: MotorTelemetry(motor_id=mid)
            for mid in sorted(self.profile.motor_id_to_joint_index)
        }
        self._raw_tactile = np.zeros((0, 0), dtype=np.float64)
        self._finger_tactile = np.zeros((0, 0), dtype=np.float64)
        self._palm_tactile = np.zeros(0, dtype=np.float64)
        self._tactile_forces = np.zeros(0, dtype=np.float64)
        self._tactile_centers = np.zeros((0, 3), dtype=np.float64)
        self._tactile_patches = empty_tactile_patches_for_profile(self.profile, dtype=np.float64)
        self.joint_angles_changed.emit(self._angles.tolist())
        self.motor_telemetry_changed.emit(
            {mid: t for mid, t in self._motor_telemetry.items()}
        )
        self.tactile_data_changed.emit()

    @property
    def angles(self) -> np.ndarray:
        return self._angles.copy()

    def set_angle(self, index: int, angle_deg: float):
        if 0 <= index < self.profile.num_joints:
            cfg = self._get_joint_config(index)
            clamped = float(np.clip(angle_deg, cfg.min_angle, cfg.max_angle))
            if self._angles[index] != clamped:
                self._angles[index] = clamped
                self.single_joint_changed.emit(index, clamped)
                self.joint_angles_changed.emit(self._angles.tolist())

    def set_all_angles(self, angles: list[float]):
        if len(angles) != self.profile.num_joints:
            return
        for i, a in enumerate(angles):
            cfg = self._get_joint_config(i)
            self._angles[i] = float(np.clip(a, cfg.min_angle, cfg.max_angle))
        self.joint_angles_changed.emit(self._angles.tolist())

    def get_angle(self, index: int) -> float:
        if 0 <= index < self.profile.num_joints:
            return float(self._angles[index])
        return 0.0

    @property
    def tactile_forces(self) -> np.ndarray:
        return self._tactile_forces.copy()

    @property
    def tactile_centers(self) -> np.ndarray:
        return self._tactile_centers.copy()

    def set_tactile(self, forces: np.ndarray, centers: np.ndarray):
        self._tactile_forces = np.asarray(forces, dtype=np.float64).copy()
        self._tactile_centers = np.asarray(centers, dtype=np.float64).copy()
        self.tactile_data_changed.emit()

    def get_motor_telemetry(self, motor_id: int) -> MotorTelemetry:
        return self._motor_telemetry.get(motor_id, MotorTelemetry(motor_id=motor_id))

    def get_all_motor_telemetry(self) -> dict[int, MotorTelemetry]:
        return dict(self._motor_telemetry)

    def set_motor_telemetry(self, motor_id: int, data: dict):
        t = self._motor_telemetry.get(motor_id)
        if t is None:
            t = MotorTelemetry(motor_id=motor_id)
            self._motor_telemetry[motor_id] = t
        for key, val in data.items():
            if hasattr(t, key):
                setattr(t, key, val)
        t.timestamp = time.time()

    def set_all_motor_telemetry(self, telemetry_dict: dict[int, dict]):
        for motor_id, data in telemetry_dict.items():
            self.set_motor_telemetry(motor_id, data)
        self.motor_telemetry_changed.emit(
            {mid: t for mid, t in self._motor_telemetry.items()}
        )

    @property
    def finger_tactile(self) -> np.ndarray:
        return self._finger_tactile.copy()

    @property
    def palm_tactile(self) -> np.ndarray:
        return self._palm_tactile.copy()

    def set_finger_tactile_array(self, data: np.ndarray):
        arr = np.asarray(data, dtype=np.float64)
        if arr.ndim == 2:
            self._finger_tactile = np.clip(arr, 0.0, 1.0)
            self.tactile_data_changed.emit()

    def set_palm_tactile_array(self, data: np.ndarray):
        arr = np.asarray(data, dtype=np.float64).reshape(-1)
        self._palm_tactile = np.clip(arr, 0.0, 1.0)
        self.tactile_data_changed.emit()

    def set_all_tactile(self, finger_data: np.ndarray, palm_data: np.ndarray):
        finger = np.asarray(finger_data, dtype=np.float64)
        palm = np.asarray(palm_data, dtype=np.float64).reshape(-1)
        if finger.ndim == 2:
            self._finger_tactile = np.clip(finger, 0.0, 1.0)
        self._palm_tactile = np.clip(palm, 0.0, 1.0)
        self.tactile_data_changed.emit()

    @property
    def raw_tactile(self) -> np.ndarray:
        return self._raw_tactile.copy()

    def set_raw_tactile(self, data: np.ndarray):
        arr = np.asarray(data, dtype=np.float64)
        if arr.ndim == 2:
            self._raw_tactile = np.clip(arr, 0.0, 1.0)
            self.tactile_data_changed.emit()

    @property
    def tactile_patches(self) -> dict[str, np.ndarray]:
        return {name: patch.copy() for name, patch in self._tactile_patches.items()}

    def set_tactile_patches(self, patches: dict[str, np.ndarray]):
        shapes = patch_shapes_for_profile(self.profile)
        order = patch_order_for_profile(self.profile)
        has_declared_layout = bool(getattr(self.profile, "tactile_layout", ()))
        updated = (
            empty_tactile_patches_for_profile(self.profile, dtype=np.float64)
            if has_declared_layout
            else {}
        )
        valid = False
        iterable = order if has_declared_layout else tuple(patches.keys())
        for name in iterable:
            if name not in patches:
                continue
            arr = np.asarray(patches[name], dtype=np.float64)
            if arr.ndim != 2:
                continue
            if has_declared_layout and name in shapes and arr.shape != shapes[name]:
                continue
            updated[name] = np.clip(arr, 0.0, 1.0)
            valid = True
        if valid or has_declared_layout:
            self._tactile_patches = updated
            self.tactile_data_changed.emit()

    def _get_joint_config(self, index: int):
        i = 0
        for finger in self.profile.finger_configs:
            for joint in finger.joints:
                if i == index:
                    return joint
                i += 1
        raise IndexError(f"关节索引 {index} 超出范围")
