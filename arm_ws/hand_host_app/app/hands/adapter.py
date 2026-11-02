"""Adapter contract for package-based dexterous hand drivers."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

from app.hands.base import HandProfile


@runtime_checkable
class DexHandMotorAdapterV1(Protocol):
    """Required API implemented by imported hand driver adapters."""

    def connect(self, port: str, baudrate: int, protocol: str | None = None) -> bool: ...
    def disconnect(self) -> None: ...
    def enable_all(self, enable: bool = True) -> None: ...
    def set_motor_angle(self, motor_id: int, angle_deg: float, speed: float = 0.5) -> None: ...
    def set_angles(self, angles_dict: dict[int, float]) -> None: ...
    def set_all_joint_angles(self, angles_list: list[float]) -> None: ...
    def home_all_motors(self, target_angles: dict[int, float] | None = None, speed: float = 0.3) -> None: ...
    def get_latest_telemetry(self) -> dict[int, dict]: ...
    def get_status(self, motor_id: int) -> dict: ...
    def cleanup(self) -> None: ...


REQUIRED_METHODS = (
    "connect",
    "disconnect",
    "enable_all",
    "set_motor_angle",
    "set_angles",
    "set_all_joint_angles",
    "home_all_motors",
    "get_latest_telemetry",
    "get_status",
    "cleanup",
)


def validate_adapter_object(adapter: Any) -> list[str]:
    errors: list[str] = []
    for name in REQUIRED_METHODS:
        if not callable(getattr(adapter, name, None)):
            errors.append(f"adapter is missing required method: {name}")
    return errors


def load_python_adapter(profile: HandProfile) -> DexHandMotorAdapterV1:
    if profile.driver_path is None:
        raise RuntimeError(f"{profile.hand_id} has no python driver path")
    driver_path = Path(profile.driver_path)
    if not driver_path.exists():
        raise RuntimeError(f"driver file does not exist: {driver_path}")

    module_name = f"dexhand_driver_{profile.hand_id}_{abs(hash(driver_path))}"
    spec = importlib.util.spec_from_file_location(module_name, driver_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import driver: {driver_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    sys.path.insert(0, str(driver_path.parent))
    try:
        spec.loader.exec_module(module)
    finally:
        try:
            sys.path.remove(str(driver_path.parent))
        except ValueError:
            pass

    factory = getattr(module, "create_driver", None) or getattr(module, "create_adapter", None)
    if callable(factory):
        adapter = factory(profile)
    else:
        cls = getattr(module, profile.driver_class, None)
        if cls is None:
            raise RuntimeError(
                f"driver must expose create_driver(profile), create_adapter(profile) or {profile.driver_class}"
            )
        try:
            adapter = cls(profile)
        except TypeError:
            adapter = cls()

    errors = validate_adapter_object(adapter)
    if errors:
        raise RuntimeError("; ".join(errors))
    return adapter
