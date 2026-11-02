"""Qt service wrapper for imported dexterous hand motor adapters."""

from __future__ import annotations

from PySide6.QtCore import QObject, QTimer, Signal

from app.hands.adapter import DexHandMotorAdapterV1, load_python_adapter
from app.hands.base import HandProfile


class PluginMotorService(QObject):
    """Expose a Python adapter through the same signal API as built-in services."""

    connection_changed = Signal(bool)
    motor_status_updated = Signal(object)
    motor_angles_updated = Signal(object)
    motor_telemetry_updated = Signal(object)
    error_occurred = Signal(str)

    def __init__(self, profile: HandProfile, parent=None):
        super().__init__(parent)
        self.profile = profile
        self.hand_id = profile.hand_id
        self._adapter: DexHandMotorAdapterV1 | None = None
        self._connected = False
        self._latest_angles = [
            joint.default_angle
            for finger in profile.finger_configs
            for joint in finger.joints
        ]
        self._pending_angles: list[float] | None = None
        self._pending_kwargs: dict[str, object] = {}
        self._poll_timer = QTimer(self)
        motor_config = getattr(profile, "motor_adapter_config", {}) or {}
        self._poll_timer.setInterval(int(motor_config.get("telemetry_period_ms", 100)))
        self._poll_timer.timeout.connect(self._poll)
        self._command_timer = QTimer(self)
        self._command_timer.setSingleShot(True)
        self._command_timer.setInterval(int(motor_config.get("command_debounce_ms", 30)))
        self._command_timer.timeout.connect(self._flush_pending_angles)

    @property
    def is_connected(self) -> bool:
        return self._connected and self._adapter is not None

    def connect(self, port: str, baudrate: int = 115200, protocol: str | None = None) -> bool:
        self.disconnect()
        try:
            self._adapter = load_python_adapter(self.profile)
            ok = bool(self._adapter.connect(port, baudrate, protocol=protocol))
        except Exception as exc:
            self._adapter = None
            self.error_occurred.emit(f"adapter connect failed: {exc}")
            return False
        self._connected = ok
        self.connection_changed.emit(ok)
        if ok:
            self._poll_timer.start()
            self._poll()
        return ok

    def disconnect(self):
        self._poll_timer.stop()
        self._command_timer.stop()
        self._pending_angles = None
        self._pending_kwargs = {}
        if self._adapter is not None:
            try:
                self._adapter.disconnect()
            except Exception as exc:
                self.error_occurred.emit(f"adapter disconnect failed: {exc}")
            try:
                self._adapter.cleanup()
            except Exception:
                pass
        was_connected = self._connected
        self._adapter = None
        self._connected = False
        if was_connected:
            self.connection_changed.emit(False)

    def enable_all(self, enable: bool = True):
        if self._adapter is None:
            self.error_occurred.emit("not connected")
            return
        try:
            self._adapter.enable_all(enable)
        except Exception as exc:
            self.error_occurred.emit(f"enable failed: {exc}")

    def set_motor_angle(self, motor_id: int, angle_deg: float, speed: float = 0.5):
        if self._adapter is None:
            return
        idx = self.profile.motor_id_to_joint_index.get(int(motor_id))
        if idx is None:
            return
        angles = list(self._pending_angles or self._latest_angles)
        if idx < len(angles):
            angles[idx] = float(angle_deg)
            self._queue_angles(angles)

    def set_motor_angle_immediate(self, motor_id: int, angle_deg: float, speed: float = 0.5):
        """Send one motor command without debounce, for calibration workflows."""
        if self._adapter is None:
            return
        try:
            self._adapter.set_motor_angle(int(motor_id), float(angle_deg), speed=float(speed))
        except Exception as exc:
            self.error_occurred.emit(f"set motor failed: {exc}")

    def set_angles(self, angles_dict: dict[int, float]):
        if self._adapter is None:
            return
        angles = list(self._pending_angles or self._latest_angles)
        for motor_id, angle in angles_dict.items():
            idx = self.profile.motor_id_to_joint_index.get(int(motor_id))
            if idx is not None and idx < len(angles):
                angles[idx] = float(angle)
        self._queue_angles(angles)

    def set_all_joint_angles(self, angles_list: list[float], **kwargs):
        if self._adapter is None or len(angles_list) != self.profile.num_joints:
            return
        self._queue_angles(list(angles_list), kwargs)

    def home_all_motors(self, target_angles: dict[int, float] | None = None, speed: float = 0.3):
        if self._adapter is None:
            self.error_occurred.emit("not connected")
            return
        try:
            self._adapter.home_all_motors(target_angles, speed)
        except Exception as exc:
            self.error_occurred.emit(f"home failed: {exc}")

    def get_latest_telemetry(self) -> dict[int, dict]:
        if self._adapter is None:
            return {}
        try:
            return dict(self._adapter.get_latest_telemetry())
        except Exception as exc:
            self.error_occurred.emit(f"telemetry failed: {exc}")
            return {}

    def get_status(self, motor_id: int) -> dict:
        if self._adapter is None:
            return {"online": False, "position": 0.0}
        try:
            return dict(self._adapter.get_status(motor_id))
        except Exception as exc:
            self.error_occurred.emit(f"status failed: {exc}")
            return {"online": False, "position": 0.0}

    def send_config_command(self, dev_id: int, index: int, value: int = 0):
        method = getattr(self._adapter, "send_config_command", None)
        if callable(method):
            try:
                method(dev_id, index, value)
            except Exception as exc:
                self.error_occurred.emit(f"config command failed: {exc}")

    def set_zero_point(self, motor_id: int):
        method = getattr(self._adapter, "set_zero_point", None)
        if callable(method):
            try:
                method(motor_id)
            except Exception as exc:
                self.error_occurred.emit(f"zero point failed: {exc}")

    def set_max_current(self, motor_id: int, value_q15: int):
        method = getattr(self._adapter, "set_max_current", None)
        if callable(method):
            try:
                method(motor_id, value_q15)
            except Exception as exc:
                self.error_occurred.emit(f"set max current failed: {exc}")

    def has_adapter_method(self, name: str) -> bool:
        return callable(getattr(self._adapter, name, None))

    def find_motor_limits(self, motor_id: int, start_angle: float, config: dict) -> dict | None:
        """Delegate mechanical limit search to the hand driver when available.

        Drivers for CAN, EtherCAT, SDK-based hands, or hands with built-in limit
        discovery can implement either:
          * find_motor_limits(motor_id, start_angle, config) -> {"min": x, "max": y}
          * auto_find_limits(motor_id, start_angle, config) -> {"min": x, "max": y}
        """
        for method_name in ("find_motor_limits", "auto_find_limits"):
            method = getattr(self._adapter, method_name, None)
            if callable(method):
                try:
                    result = method(int(motor_id), float(start_angle), dict(config))
                except TypeError:
                    result = method(int(motor_id), dict(config))
                except Exception as exc:
                    self.error_occurred.emit(f"limit search failed: {exc}")
                    return None
                return result if isinstance(result, dict) else None
        return None

    def cleanup(self):
        self.disconnect()

    def _poll(self):
        telemetry = self.get_latest_telemetry()
        if not telemetry:
            return
        status = {
            mid: {"online": bool(data.get("online", False))}
            for mid, data in telemetry.items()
        }
        angles = {
            mid: float(data.get("position", data.get("angle", 0.0)))
            for mid, data in telemetry.items()
        }
        for motor_id, angle in angles.items():
            idx = self.profile.motor_id_to_joint_index.get(int(motor_id))
            if idx is not None and idx < len(self._latest_angles):
                self._latest_angles[idx] = float(angle)
        self.motor_status_updated.emit(status)
        self.motor_angles_updated.emit(angles)
        self.motor_telemetry_updated.emit(telemetry)

    def _queue_angles(self, angles: list[float], kwargs: dict[str, object] | None = None):
        self._pending_angles = list(angles)
        self._pending_kwargs = dict(kwargs or {})
        if not self._command_timer.isActive():
            self._command_timer.start()

    def _flush_pending_angles(self):
        if self._adapter is None or self._pending_angles is None:
            return
        angles = list(self._pending_angles)
        kwargs = dict(self._pending_kwargs)
        self._pending_angles = None
        self._pending_kwargs = {}
        try:
            try:
                self._adapter.set_all_joint_angles(angles, **kwargs)
            except TypeError:
                self._adapter.set_all_joint_angles(angles)
            self._latest_angles = list(angles)
        except Exception:
            angles_dict = {}
            for idx, angle in enumerate(angles):
                motor_id = self.profile.joint_index_to_motor_id.get(idx)
                if motor_id is not None:
                    angles_dict[motor_id] = angle
            try:
                self._adapter.set_angles(angles_dict)
                self._latest_angles = list(angles)
            except Exception as exc:
                self.error_occurred.emit(f"set angles failed: {exc}")
