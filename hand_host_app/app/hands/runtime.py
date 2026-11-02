"""Runtime motor-service factory for package-based hand drivers."""

from __future__ import annotations

from PySide6.QtCore import QObject, Signal

from app.hands.base import HandProfile
from app.services.plugin_motor_service import PluginMotorService


class NullMotorService(QObject):
    """Motor service used before a real hand package is installed."""

    connection_changed = Signal(bool)
    motor_status_updated = Signal(object)
    motor_angles_updated = Signal(object)
    motor_telemetry_updated = Signal(object)
    error_occurred = Signal(str)

    def __init__(self, profile: HandProfile, parent=None):
        super().__init__(parent)
        self.profile = profile
        self.hand_id = profile.hand_id

    @property
    def is_connected(self) -> bool:
        return False

    def connect(self, port: str, baudrate: int = 0, protocol: str | None = None) -> bool:
        self.error_occurred.emit("当前还没有接入任何手型，请先到“手型管理”添加新手。")
        return False

    def disconnect(self):
        self.connection_changed.emit(False)

    def enable_all(self, enable: bool = True):
        self.error_occurred.emit("当前还没有接入任何手型。")

    def set_motor_angle(self, motor_id: int, angle_deg: float, speed: float = 0.5):
        return None

    def set_motor_angle_immediate(self, motor_id: int, angle_deg: float, speed: float = 0.5):
        return None

    def set_angles(self, angles_dict: dict[int, float]):
        return None

    def set_all_joint_angles(self, angles_list: list[float]):
        return None

    def home_all_motors(self, target_angles: dict[int, float] | None = None, speed: float = 0.3):
        self.error_occurred.emit("当前还没有接入任何手型。")

    def get_status(self, motor_id: int) -> dict:
        return {"online": False}

    def get_latest_telemetry(self) -> dict[int, dict]:
        return {}

    def has_adapter_method(self, name: str) -> bool:
        return False

    def find_motor_limits(self, motor_id: int, start_angle: float, config: dict) -> dict | None:
        return None

    def cleanup(self):
        self.disconnect()


def create_motor_service(profile: HandProfile, parent=None):
    if profile.num_joints == 0 or profile.motor_kind == "none":
        return NullMotorService(profile, parent)
    if profile.driver_kind == "python":
        return PluginMotorService(profile, parent)

    service = NullMotorService(profile, parent)
    service.error_occurred.emit("该手型没有 Python 驱动适配器，请通过接入向导重新导入。")
    return service
