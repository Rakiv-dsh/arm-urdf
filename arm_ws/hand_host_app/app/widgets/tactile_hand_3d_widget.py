"""Qt embedded MuJoCo hand view with live tactile site coloring."""

from __future__ import annotations

import os
import threading
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PySide6.QtCore import QPoint, Qt, QThread, Signal, Slot
from PySide6.QtGui import QImage, QMouseEvent, QPixmap
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget

from app.platform_compat import render_scale, scaled_render_size
from app.models.hand_state import HandState
from app.models.tactile_layout import (
    display_to_site_indices,
    empty_tactile_patches_for_profile,
    patch_order_for_profile,
    patch_site_prefixes_for_profile,
    patch_site_transforms_for_profile,
    patch_shapes_for_profile,
)

try:
    import mujoco

    HAS_MUJOCO = True
    _orig_renderer_del = mujoco.Renderer.__del__

    def _safe_renderer_del(self):
        if hasattr(self, "_gl_context"):
            _orig_renderer_del(self)

    mujoco.Renderer.__del__ = _safe_renderer_del
except ImportError:  # pragma: no cover - runtime dependency
    mujoco = None
    HAS_MUJOCO = False


@dataclass(frozen=True)
class TaxelSite:
    site_id: int
    patch: str
    value_row: int
    value_col: int


def heat_rgb(value: float, gamma: float = 0.45) -> np.ndarray:
    t = float(np.clip(value, 0.0, 1.0))
    if gamma > 0.0:
        t = t ** gamma
    stops = (
        (0.00, np.array([0.00, 0.36, 1.00], dtype=np.float32)),
        (0.22, np.array([0.00, 0.95, 1.00], dtype=np.float32)),
        (0.50, np.array([0.30, 1.00, 0.35], dtype=np.float32)),
        (0.76, np.array([1.00, 0.96, 0.00], dtype=np.float32)),
        (1.00, np.array([1.00, 0.00, 0.00], dtype=np.float32)),
    )
    for index in range(len(stops) - 1):
        left_t, left_color = stops[index]
        right_t, right_color = stops[index + 1]
        if t <= right_t:
            local = (t - left_t) / max(right_t - left_t, 1e-9)
            return (1.0 - local) * left_color + local * right_color
    return stops[-1][1]


class TactileHandRenderWorker(QThread):
    frame_ready = Signal(object)
    error_occurred = Signal(str)
    tactile_stats = Signal(float)

    def __init__(self, model_path: Path, actuator_names: list[str], profile=None, parent=None):
        super().__init__(parent)
        self._model_path = Path(model_path)
        self._actuator_names = list(actuator_names)
        self._profile = profile
        self._patch_order = patch_order_for_profile(profile)
        self._patch_shapes = patch_shapes_for_profile(profile)
        self._site_prefixes = patch_site_prefixes_for_profile(profile)
        self._site_transforms = patch_site_transforms_for_profile(profile)
        self._running = False
        self._lock = threading.RLock()
        self._angles_deg = np.zeros(len(self._actuator_names), dtype=np.float64)
        self._patches = empty_tactile_patches_for_profile(profile, dtype=np.float32)
        self._render_width = 640
        self._render_height = 480
        self._resize_dirty = False
        self._dirty = True
        self._idle_skip_enabled = render_scale() < 1.0
        self._camera_events: list[tuple[str, float, float]] = []

        self._model = None
        self._data = None
        self._renderer = None
        self._camera = None
        self._scene_option = None
        self._taxel_sites: list[TaxelSite] = []
        self._base_site_size = None
        self._actuator_ids: list[int] = []

    def set_angles(self, angles_deg: list[float]):
        with self._lock:
            n = min(len(angles_deg), self._angles_deg.size)
            if n:
                updated = np.asarray(angles_deg[:n], dtype=np.float64)
                if not np.allclose(self._angles_deg[:n], updated, atol=1e-3):
                    self._angles_deg[:n] = updated
                    self._dirty = True

    def set_tactile_patches(self, patches: dict[str, np.ndarray]):
        with self._lock:
            updated = empty_tactile_patches_for_profile(self._profile, dtype=np.float32)
            for name in self._patch_order:
                arr = np.asarray(patches.get(name, updated[name]), dtype=np.float32)
                if arr.shape == self._patch_shapes[name]:
                    updated[name] = np.clip(arr, 0.0, 1.0)
            changed = any(
                not np.allclose(self._patches[name], updated[name], atol=1e-4)
                for name in self._patch_order
            )
            if changed:
                self._patches = updated
                self._dirty = True

    def set_render_size(self, width: int, height: int):
        width, height = scaled_render_size(width, height)
        with self._lock:
            width = int(max(64, width))
            height = int(max(64, height))
            if width != self._render_width or height != self._render_height:
                self._render_width = width
                self._render_height = height
                self._resize_dirty = True
                self._dirty = True

    def queue_mouse_action(self, action: str, dx_pixels: float, dy_pixels: float, viewport_height: int):
        scale = 1.0 / max(float(viewport_height), 1.0)
        event = (action, float(dx_pixels) * scale, float(dy_pixels) * scale)
        with self._lock:
            if self._camera_events and self._camera_events[-1][0] == action:
                last_action, last_dx, last_dy = self._camera_events[-1]
                self._camera_events[-1] = (last_action, last_dx + event[1], last_dy + event[2])
            else:
                self._camera_events.append(event)
            self._dirty = True

    def queue_wheel(self, wheel_delta: int):
        steps = float(wheel_delta) / 120.0
        with self._lock:
            if self._camera_events and self._camera_events[-1][0] == "zoom":
                action, dx, dy = self._camera_events[-1]
                self._camera_events[-1] = (action, dx, dy - 0.05 * steps)
            else:
                self._camera_events.append(("zoom", 0.0, -0.05 * steps))
            self._dirty = True

    def run(self):
        if not HAS_MUJOCO:
            self.error_occurred.emit("MuJoCo 未安装，请执行 pip install mujoco")
            return
        try:
            self._load_model()
            self._create_renderer()
        except Exception as exc:
            self.error_occurred.emit(f"3D 触觉模型加载失败: {exc}")
            self._close_renderer()
            return

        self._running = True
        while self._running:
            try:
                with self._lock:
                    if self._idle_skip_enabled and not self._dirty:
                        should_render = False
                    else:
                        should_render = True
                        self._dirty = False
                    angles_deg = self._angles_deg.copy()
                    patches = {name: self._patches[name].copy() for name in self._patch_order}
                    resize_dirty = self._resize_dirty
                    self._resize_dirty = False
                    events = list(self._camera_events)
                    self._camera_events.clear()

                if not should_render:
                    self.msleep(12)
                    continue

                if resize_dirty:
                    self._create_renderer()

                self._apply_pose(angles_deg)
                max_value = self._apply_tactile_colors(patches)

                self._renderer.update_scene(self._data, self._camera, scene_option=self._scene_option)
                self._apply_camera_events(events)
                if events:
                    self._renderer.update_scene(self._data, self._camera, scene_option=self._scene_option)

                pixels = self._renderer.render()
                self.frame_ready.emit(pixels.copy())
                self.tactile_stats.emit(max_value)
            except Exception as exc:
                self.error_occurred.emit(f"3D 触觉渲染错误: {exc}")
                self.msleep(200)
            self.msleep(16)

        self._close_renderer()

    def stop(self):
        self._running = False
        if not self.wait(3000):
            self.terminate()
            self.wait(1000)

    def _load_model(self):
        if not self._model_path.exists():
            raise FileNotFoundError(self._model_path)

        model_dir = os.path.dirname(os.path.abspath(str(self._model_path)))
        model_file = os.path.basename(str(self._model_path))
        old_cwd = os.getcwd()
        try:
            os.chdir(model_dir)
            self._model = mujoco.MjModel.from_xml_path(model_file)
        finally:
            os.chdir(old_cwd)
        self._data = mujoco.MjData(self._model)
        self._taxel_sites = self._build_taxel_site_map()
        self._base_site_size = self._model.site_size.copy()
        self._actuator_ids = self._resolve_actuators()
        self._camera = self._make_camera()
        self._scene_option = self._make_scene_option()
        self._apply_pose(self._angles_deg)

    def _create_renderer(self):
        self._close_renderer()
        max_w = int(self._model.vis.global_.offwidth)
        max_h = int(self._model.vis.global_.offheight)
        width = min(int(self._render_width), max_w)
        height = min(int(self._render_height), max_h)
        self._renderer = mujoco.Renderer(self._model, height=height, width=width)

    def _close_renderer(self):
        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:
                pass
            self._renderer = None

    def _make_camera(self):
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.fixedcamid = -1
        camera.lookat[:] = np.asarray(self._model.stat.center, dtype=np.float64)
        camera.distance = max(float(self._model.stat.extent) * 1.8, 0.25)
        camera.azimuth = 90.0
        camera.elevation = -25.0
        return camera

    def _make_scene_option(self):
        opt = mujoco.MjvOption()
        if hasattr(opt, "sitegroup"):
            opt.sitegroup[:] = 1
        if hasattr(opt, "geomgroup"):
            opt.geomgroup[:] = 0
            if len(opt.geomgroup) > 1:
                opt.geomgroup[1] = 1
        return opt

    def _resolve_actuators(self) -> list[int]:
        actuator_ids: list[int] = []
        for name in self._actuator_names:
            actuator_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            actuator_ids.append(int(actuator_id))
        return actuator_ids

    def _build_taxel_site_map(self) -> list[TaxelSite]:
        sites: list[TaxelSite] = []
        missing: list[str] = []
        for patch_name in self._patch_order:
            rows, cols = self._patch_shapes[patch_name]
            site_prefix = self._site_prefixes.get(patch_name, patch_name)
            transform = self._site_transforms.get(patch_name, "identity")
            for value_row in range(rows):
                for value_col in range(cols):
                    site_row, site_col = display_to_site_indices(value_row, value_col, rows, cols, transform)
                    site_name = f"tx_{site_prefix}_r{site_row}_c{site_col}"
                    site_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SITE, site_name)
                    if site_id < 0:
                        missing.append(site_name)
                        continue
                    sites.append(TaxelSite(site_id, patch_name, value_row, value_col))

        expected = sum(rows * cols for rows, cols in self._patch_shapes.values())
        if 0 < len(sites) < expected:
            preview = ", ".join(missing[:8])
            raise ValueError(f"触觉 site 不完整: {preview}")
        return sites

    def _apply_pose(self, angles_deg: np.ndarray):
        if self._model.nu <= 0:
            mujoco.mj_forward(self._model, self._data)
            return
        low = self._model.actuator_ctrlrange[:, 0]
        high = self._model.actuator_ctrlrange[:, 1]
        for index, actuator_id in enumerate(self._actuator_ids):
            if actuator_id < 0 or actuator_id >= self._model.nu:
                continue
            value = float(np.radians(angles_deg[index]))
            value = float(np.clip(value, low[actuator_id], high[actuator_id]))
            self._data.ctrl[actuator_id] = value
            joint_id = int(self._model.actuator_trnid[actuator_id, 0])
            if 0 <= joint_id < self._model.njnt:
                qpos_addr = int(self._model.jnt_qposadr[joint_id])
                self._data.qpos[qpos_addr] = value
        mujoco.mj_forward(self._model, self._data)

    def _apply_tactile_colors(self, patches: dict[str, np.ndarray]) -> float:
        if not self._taxel_sites:
            return 0.0
        max_value = 0.0
        alpha_low = 0.38
        alpha_high = 1.0
        for site in self._taxel_sites:
            value = float(np.clip(patches[site.patch][site.value_row, site.value_col], 0.0, 1.0))
            max_value = max(max_value, value)
            color = heat_rgb(value)
            alpha = alpha_low + (alpha_high - alpha_low) * value
            self._model.site_rgba[site.site_id] = (
                float(color[0]),
                float(color[1]),
                float(color[2]),
                float(alpha),
            )
            scaled = self._base_site_size[site.site_id].copy()
            scaled[0] = max(float(self._base_site_size[site.site_id, 0]) * (1.0 + 1.15 * value), 1e-6)
            self._model.site_size[site.site_id] = scaled
        return max_value

    def _apply_camera_events(self, events: list[tuple[str, float, float]]):
        if not events or self._renderer is None:
            return
        action_map = {
            "rotate": mujoco.mjtMouse.mjMOUSE_ROTATE_V,
            "pan": mujoco.mjtMouse.mjMOUSE_MOVE_H,
            "zoom": mujoco.mjtMouse.mjMOUSE_ZOOM,
        }
        for action_name, reldx, reldy in events:
            action = action_map.get(action_name)
            if action is None:
                continue
            mujoco.mjv_moveCamera(self._model, action, reldx, reldy, self._renderer.scene, self._camera)


class TactileHandRenderLabel(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker: TactileHandRenderWorker | None = None
        self._last_pos = QPoint()
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(320, 300)
        self.setMouseTracking(True)
        self.setStyleSheet(
            "background-color: #050505; color: #888; border-radius: 6px; font-size: 13px;"
        )

    def set_worker(self, worker: TactileHandRenderWorker | None):
        self.worker = worker
        if worker is not None:
            worker.set_render_size(self.width(), self.height())

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.worker is not None:
            self.worker.set_render_size(self.width(), self.height())

    def mousePressEvent(self, event: QMouseEvent):
        self._last_pos = event.position().toPoint()

    def mouseMoveEvent(self, event: QMouseEvent):
        if self.worker is None:
            return
        pos = event.position().toPoint()
        delta = pos - self._last_pos
        self._last_pos = pos
        if event.buttons() & Qt.MouseButton.LeftButton:
            self.worker.queue_mouse_action("rotate", delta.x(), delta.y(), self.height())
        elif event.buttons() & Qt.MouseButton.RightButton:
            self.worker.queue_mouse_action("pan", delta.x(), delta.y(), self.height())

    def wheelEvent(self, event):
        if self.worker is not None:
            self.worker.queue_wheel(event.angleDelta().y())


class TactileHand3DWidget(QWidget):
    def __init__(self, hand_state: HandState, parent=None):
        super().__init__(parent)
        self.hand_state = hand_state
        self.worker: TactileHandRenderWorker | None = None
        self._cleaned_up = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        title = QLabel("3D 模型预览")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #333;")
        layout.addWidget(title)

        self.render_label = TactileHandRenderLabel(self)
        layout.addWidget(self.render_label, 1)

        self.hand_state.joint_angles_changed.connect(self._on_angles_changed)
        self.hand_state.tactile_data_changed.connect(self._on_tactile_changed)
        self.hand_state.profile_changed.connect(self._on_profile_changed)

    def cleanup(self):
        self._cleaned_up = True
        try:
            self.hand_state.joint_angles_changed.disconnect(self._on_angles_changed)
            self.hand_state.tactile_data_changed.disconnect(self._on_tactile_changed)
            self.hand_state.profile_changed.disconnect(self._on_profile_changed)
        except RuntimeError:
            pass
        self._stop_worker()

    @Slot(list)
    def _on_angles_changed(self, angles: list[float]):
        if self.worker is not None:
            self.worker.set_angles(angles)

    @Slot()
    def _on_tactile_changed(self):
        if self.worker is not None:
            self.worker.set_tactile_patches(self.hand_state.tactile_patches)

    @Slot(object)
    def _on_profile_changed(self, _profile):
        if not self._cleaned_up and self.isVisible():
            self._start_worker()

    @Slot(object)
    def _on_frame_ready(self, pixels: np.ndarray):
        h, w = pixels.shape[:2]
        image = QImage(pixels.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(image)
        transform_mode = (
            Qt.TransformationMode.FastTransformation
            if render_scale() < 1.0
            else Qt.TransformationMode.SmoothTransformation
        )
        self.render_label.setPixmap(
            pixmap.scaled(
                self.render_label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                transform_mode,
            )
        )

    @Slot(str)
    def _on_error(self, message: str):
        self.render_label.setText(message)

    def _start_worker(self):
        if self._cleaned_up:
            return
        if self.worker is not None:
            return
        self._stop_worker()
        profile = self.hand_state.profile
        self.worker = TactileHandRenderWorker(Path(profile.model_path), profile.actuator_names, profile, self)
        self.worker.frame_ready.connect(self._on_frame_ready)
        self.worker.error_occurred.connect(self._on_error)
        self.render_label.set_worker(self.worker)
        self.worker.set_angles(self.hand_state.angles.tolist())
        self.worker.set_tactile_patches(self.hand_state.tactile_patches)
        self.worker.start()

    def _stop_worker(self):
        if self.worker is not None:
            self.worker.stop()
            try:
                self.worker.frame_ready.disconnect()
                self.worker.error_occurred.disconnect()
            except RuntimeError:
                pass
            self.render_label.set_worker(None)
            self.worker = None

    def showEvent(self, event):
        super().showEvent(event)
        self._start_worker()

    def hideEvent(self, event):
        super().hideEvent(event)
        self._stop_worker()
