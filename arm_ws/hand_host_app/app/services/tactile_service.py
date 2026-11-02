"""Generic tactile polling service.

Accepted adapter frame formats:

* ``{"patches": {"patch_id": [[...], ...]}}`` (preferred, fully generic)
* ``{"raw": [[...], ...]}`` (displayed as one generic 2D patch)
* ``{"finger": [[...], ...], "palm": [...]}`` (legacy compatibility only)

Normalization, optional baseline subtraction, and Qt signal dispatch stay in
the core; byte parsing and hardware-specific framing stay in the hand package.
"""

from __future__ import annotations

import importlib.util
import inspect
import sys
import time
from pathlib import Path

import numpy as np
from PySide6.QtCore import QObject, QThread, Signal

from app.models.hand_state import HandState
from app.models.tactile_layout import (
    empty_tactile_patches_for_profile,
    patch_order_for_profile,
    patch_shapes_for_profile,
    patches_to_legacy_matrix,
    reduce_patches_to_finger_palm,
)


def _load_external_tactile_adapter(profile):
    path = getattr(profile, "tactile_adapter_path", None)
    if path is None:
        raise RuntimeError("当前手型没有配置 tactile.adapter_path")
    path = Path(path)
    if not path.exists():
        raise RuntimeError(f"触觉适配器不存在: {path}")

    module_name = f"dexhand_tactile_{profile.hand_id}_{abs(hash(path))}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法导入触觉适配器: {path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    sys.path.insert(0, str(path.parent))
    try:
        spec.loader.exec_module(module)
    finally:
        try:
            sys.path.remove(str(path.parent))
        except ValueError:
            pass

    factory = getattr(module, "create_tactile_adapter", None)
    if callable(factory):
        adapter = factory(profile)
    else:
        cls = getattr(module, getattr(profile, "tactile_adapter_class", "TactileAdapter"), None)
        if cls is None:
            raise RuntimeError("触觉适配器需要提供 create_tactile_adapter(profile) 或 TactileAdapter")
        try:
            init_params = list(inspect.signature(cls).parameters.values())
        except (TypeError, ValueError):
            init_params = []
        wants_profile = bool(
            init_params
            and init_params[0].name in {"profile", "hand_profile", "hand"}
        )
        adapter = cls(profile) if wants_profile else cls()

    for name in ("connect", "disconnect", "read_frame", "cleanup"):
        if not callable(getattr(adapter, name, None)):
            raise RuntimeError(f"触觉适配器缺少方法: {name}")
    return adapter


class TactilePollWorker(QThread):
    """Generate generic synthetic tactile patches for UI testing."""

    all_tactile_updated = Signal(object, object)
    raw_tactile_updated = Signal(object)
    tactile_patches_updated = Signal(object)
    status_updated = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, profile=None, parent=None):
        super().__init__(parent)
        self._profile = profile
        self._running = False
        self._frame_index = 0
        self._patch_order = patch_order_for_profile(profile)
        self._patch_shapes = patch_shapes_for_profile(profile)
        if not self._patch_order:
            self._patch_order = ("simulated",)
            self._patch_shapes = {"simulated": (8, 8)}

    def run(self):
        self._running = True
        self.status_updated.emit("触觉模拟数据已启动")
        while self._running:
            try:
                patches = self._generate_patches()
                self.tactile_patches_updated.emit(patches)
                finger, palm = reduce_patches_to_finger_palm(patches)
                self.all_tactile_updated.emit(finger, palm)
                self.raw_tactile_updated.emit(patches_to_legacy_matrix(patches))
                self._frame_index += 1
            except Exception as exc:
                self.error_occurred.emit(str(exc))
            self.msleep(50)

    def stop(self):
        self._running = False
        self.wait(2000)
        if self.isRunning():
            self.terminate()
            self.wait(500)

    def _generate_patches(self) -> dict[str, np.ndarray]:
        phase = self._frame_index * 0.18
        patches: dict[str, np.ndarray] = {}
        for index, name in enumerate(self._patch_order):
            rows, cols = self._patch_shapes[name]
            rr, cc = np.mgrid[0:rows, 0:cols]
            center_r = (rows - 1) * (0.5 + 0.35 * np.sin(phase + index))
            center_c = (cols - 1) * (0.5 + 0.35 * np.cos(phase * 0.8 + index))
            sigma = max(rows, cols, 2) / 4.0
            blob = np.exp(-(((rr - center_r) ** 2 + (cc - center_c) ** 2) / (2.0 * sigma * sigma)))
            patches[name] = np.clip(blob, 0.0, 1.0).astype(np.float64)
        return patches


class ExternalTactilePollWorker(QThread):
    all_tactile_updated = Signal(object, object)
    raw_tactile_updated = Signal(object)
    tactile_patches_updated = Signal(object)
    status_updated = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, adapter, profile=None, parent=None):
        super().__init__(parent)
        self._adapter = adapter
        self._profile = profile
        self._running = False
        config = getattr(profile, "tactile_adapter_config", {}) or {}
        self._patch_order = patch_order_for_profile(profile)
        self._patch_shapes = patch_shapes_for_profile(profile)
        self._value_scale = max(1.0, float(config.get("value_scale", 255.0) or 255.0))
        self._baseline_target = max(0, int(config.get("baseline_frames", 0) or 0))
        threshold = float(config.get("baseline_threshold", config.get("threshold", 0.0)) or 0.0)
        if threshold > 1.0 and self._value_scale > 1.0:
            threshold = threshold / self._value_scale
        self._baseline_threshold = max(0.0, threshold)
        self._normalize_per_frame = bool(config.get("normalize_per_frame", True))
        self._baseline_kind: str | None = None
        self._baseline_samples: list = []
        self._baseline = None

    def run(self):
        self._running = True
        self.status_updated.emit("外部触觉适配器已启动")
        while self._running:
            try:
                frame = self._adapter.read_frame(timeout=0.05)
                if not frame:
                    self.msleep(5)
                    continue
                if isinstance(frame, dict):
                    self._emit_frame(frame)
                else:
                    self.error_occurred.emit("触觉适配器 read_frame 必须返回 dict 或 None")
                    self.msleep(50)
            except Exception as exc:
                self.error_occurred.emit(f"外部触觉读取失败: {exc}")
                self.msleep(50)

    def stop(self):
        self._running = False
        self.wait(2000)
        if self.isRunning():
            self.terminate()
            self.wait(500)

    def _emit_frame(self, frame: dict):
        patches = frame.get("patches")
        if isinstance(patches, dict):
            normalized = self._normalize_patch_dict(patches)
            if normalized:
                normalized = self._apply_baseline("patches", normalized)
                if normalized is None:
                    return
                self.tactile_patches_updated.emit(normalized)
                finger, palm = reduce_patches_to_finger_palm(normalized)
                self.all_tactile_updated.emit(finger, palm)
                self.raw_tactile_updated.emit(patches_to_legacy_matrix(normalized))
                return

        if "raw" in frame:
            raw = self._normalize_values(frame["raw"])
            if raw.ndim == 2:
                raw = self._apply_baseline("raw", raw)
                if raw is None:
                    return
                patch_name = self._raw_name_for(raw)
                self.raw_tactile_updated.emit(raw)
                self.tactile_patches_updated.emit({patch_name: raw})
                return

        if "finger" in frame and "palm" in frame:
            finger = self._normalize_values(frame["finger"])
            palm = self._normalize_values(frame["palm"])
            if finger.ndim == 2 and palm.ndim in {1, 2}:
                combined = self._apply_baseline(
                    "finger_palm",
                    {"finger": finger, "palm": palm.reshape(1, -1)},
                )
                if combined is None:
                    return
                self.all_tactile_updated.emit(combined["finger"], combined["palm"].reshape(-1))
                self.tactile_patches_updated.emit(combined)
                return

        self.error_occurred.emit("触觉帧格式不被识别，请返回 patches、raw 或 finger+palm")

    def _normalize_patch_dict(self, patches: dict) -> dict[str, np.ndarray]:
        declared = bool(self._patch_shapes)
        names = self._patch_order if declared else tuple(patches.keys())
        normalized = (
            empty_tactile_patches_for_profile(self._profile, dtype=np.float64)
            if declared
            else {}
        )
        valid = False
        for name in names:
            if name not in patches:
                continue
            arr = self._normalize_values(patches[name])
            if arr.ndim != 2:
                continue
            expected = self._patch_shapes.get(name)
            if expected is not None and arr.shape != expected:
                continue
            normalized[name] = arr
            valid = True
        return normalized if valid else {}

    def _raw_name_for(self, raw: np.ndarray) -> str:
        for name, shape in self._patch_shapes.items():
            if tuple(raw.shape) == tuple(shape):
                return name
        return "raw"

    def _normalize_values(self, values) -> np.ndarray:
        arr = np.asarray(values, dtype=np.float64)
        finite = arr[np.isfinite(arr)]
        if finite.size and float(np.nanmax(finite)) > 1.0:
            arr = arr / self._value_scale
        return np.clip(arr, 0.0, 1.0)

    def _apply_baseline(self, kind: str, value):
        if self._baseline_target <= 0:
            return value
        if self._baseline_kind is None:
            self._baseline_kind = kind
        if kind != self._baseline_kind:
            return value

        if self._baseline is None:
            self._baseline_samples.append(self._copy_value(value))
            count = len(self._baseline_samples)
            if count == 1 or count % 10 == 0:
                self.status_updated.emit(f"外部触觉基线采集中: {count}/{self._baseline_target}")
            if count < self._baseline_target:
                return None
            self._baseline = self._median_value(self._baseline_samples)
            self._baseline_samples.clear()
            self.status_updated.emit("外部触觉基线完成，已启用基线扣除")
            return None
        return self._subtract_baseline(value, self._baseline)

    @staticmethod
    def _copy_value(value):
        if isinstance(value, dict):
            return {key: np.asarray(item, dtype=np.float64).copy() for key, item in value.items()}
        return np.asarray(value, dtype=np.float64).copy()

    @staticmethod
    def _median_value(samples):
        first = samples[0]
        if isinstance(first, dict):
            return {
                key: np.median(np.stack([sample[key] for sample in samples], axis=0), axis=0)
                for key in first
            }
        return np.median(np.stack(samples, axis=0), axis=0)

    def _subtract_baseline(self, value, baseline):
        if isinstance(value, dict):
            adjusted = {
                key: np.maximum(
                    np.asarray(value[key], dtype=np.float64)
                    - np.asarray(baseline[key], dtype=np.float64)
                    - self._baseline_threshold,
                    0.0,
                )
                for key in value
            }
        else:
            adjusted = np.maximum(
                np.asarray(value, dtype=np.float64)
                - np.asarray(baseline, dtype=np.float64)
                - self._baseline_threshold,
                0.0,
            )

        if not self._normalize_per_frame:
            if isinstance(adjusted, dict):
                return {key: np.clip(val, 0.0, 1.0) for key, val in adjusted.items()}
            return np.clip(adjusted, 0.0, 1.0)
        return self._stretch_per_frame(adjusted)

    @staticmethod
    def _stretch_per_frame(value):
        if isinstance(value, dict):
            arrays = [np.asarray(arr, dtype=np.float64) for arr in value.values()]
            if not arrays:
                return value
            flat_parts = [arr.reshape(-1) for arr in arrays if arr.size]
            if not flat_parts:
                return value
            flat = np.concatenate(flat_parts)
            if flat.size == 0:
                return value
            low = float(np.min(flat))
            high = float(np.max(flat))
            span = high - low
            if span <= 1e-12:
                return {key: np.zeros_like(np.asarray(val, dtype=np.float64)) for key, val in value.items()}
            return {
                key: np.clip((np.asarray(val, dtype=np.float64) - low) / span, 0.0, 1.0)
                for key, val in value.items()
            }

        arr = np.asarray(value, dtype=np.float64)
        if arr.size == 0:
            return arr
        low = float(np.min(arr))
        high = float(np.max(arr))
        span = high - low
        if span <= 1e-12:
            return np.zeros_like(arr, dtype=np.float64)
        return np.clip((arr - low) / span, 0.0, 1.0)


class TactileSensorService(QObject):
    """Qt-facing tactile service for the current hand profile."""

    connection_changed = Signal(bool)
    tactile_updated = Signal(object, object)
    raw_tactile_updated = Signal(object)
    tactile_patches_updated = Signal(object)
    status_updated = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, hand_state: HandState, parent=None):
        super().__init__(parent)
        self._hand_state = hand_state
        self._connected = False
        self._simulate = False
        self._poll_worker: TactilePollWorker | ExternalTactilePollWorker | None = None
        self._external_adapter = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_simulating(self) -> bool:
        return self._simulate

    def connect(self, port: str = "", baudrate: int = 115200) -> bool:
        self._stop_polling(clean_adapter=True)
        self._reset_state()
        try:
            profile = self._hand_state.profile
            if getattr(profile, "tactile_kind", "") != "external_python":
                self.error_occurred.emit("当前手型没有外部 Python 触觉适配器")
                return False

            self._external_adapter = _load_external_tactile_adapter(profile)
            if not self._external_adapter.connect(port, baudrate):
                self.error_occurred.emit("外部触觉适配器连接失败")
                self._cleanup_adapter()
                return False

            self._simulate = False
            self._start_polling()
            self._connected = True
            self.connection_changed.emit(True)
            self.status_updated.emit("触觉通信已连接，正在读取数据")
            return True
        except Exception as exc:
            self._cleanup_adapter()
            self._connected = False
            self._simulate = False
            self.error_occurred.emit(f"触觉传感器连接失败: {exc}")
            self.connection_changed.emit(False)
            return False

    def start_simulation(self):
        self._stop_polling(clean_adapter=True)
        self._reset_state()
        self._simulate = True
        self._start_polling()
        self._connected = True
        self.connection_changed.emit(True)

    def stop_simulation(self):
        self._stop_polling(clean_adapter=False)
        self._simulate = False
        self._connected = False
        self.connection_changed.emit(False)

    def disconnect(self):
        self._stop_polling(clean_adapter=True)
        self._connected = False
        self._simulate = False
        self.connection_changed.emit(False)

    def _reset_state(self):
        profile = self._hand_state.profile
        self._hand_state.set_tactile_patches(
            empty_tactile_patches_for_profile(profile, dtype=np.float64)
        )
        self._hand_state.set_raw_tactile(np.zeros((0, 0), dtype=np.float64))
        self._hand_state.set_all_tactile(
            np.zeros((0, 0), dtype=np.float64),
            np.zeros(0, dtype=np.float64),
        )

    def _start_polling(self):
        if self._poll_worker is not None:
            self._stop_polling(clean_adapter=False)
        if self._external_adapter is not None and not self._simulate:
            self._poll_worker = ExternalTactilePollWorker(self._external_adapter, self._hand_state.profile)
        else:
            self._poll_worker = TactilePollWorker(self._hand_state.profile)
        self._poll_worker.all_tactile_updated.connect(self._on_tactile_data)
        self._poll_worker.raw_tactile_updated.connect(self._on_raw_tactile)
        self._poll_worker.tactile_patches_updated.connect(self._on_tactile_patches)
        self._poll_worker.status_updated.connect(self.status_updated.emit)
        self._poll_worker.error_occurred.connect(self.error_occurred.emit)
        self._poll_worker.start()

    def _stop_polling(self, clean_adapter: bool = True):
        if self._poll_worker is not None:
            self._poll_worker.stop()
            try:
                self._poll_worker.all_tactile_updated.disconnect()
                self._poll_worker.raw_tactile_updated.disconnect()
                self._poll_worker.tactile_patches_updated.disconnect()
                self._poll_worker.status_updated.disconnect()
                self._poll_worker.error_occurred.disconnect()
            except RuntimeError:
                pass
            self._poll_worker = None
        if clean_adapter:
            self._cleanup_adapter()

    def _cleanup_adapter(self):
        if self._external_adapter is not None:
            try:
                self._external_adapter.disconnect()
            except Exception:
                pass
            try:
                self._external_adapter.cleanup()
            except Exception:
                pass
            self._external_adapter = None

    def _on_tactile_data(self, finger_data, palm_data):
        finger_arr = np.asarray(finger_data, dtype=np.float64)
        palm_arr = np.asarray(palm_data, dtype=np.float64)
        self._hand_state.set_all_tactile(finger_arr, palm_arr)
        self.tactile_updated.emit(finger_arr, palm_arr)

    def _on_raw_tactile(self, raw_data):
        raw_arr = np.asarray(raw_data, dtype=np.float64)
        self._hand_state.set_raw_tactile(raw_arr)
        self.raw_tactile_updated.emit(raw_arr)

    def _on_tactile_patches(self, patches):
        self._hand_state.set_tactile_patches(patches)
        self.tactile_patches_updated.emit(patches)

    def cleanup(self):
        self.disconnect()
