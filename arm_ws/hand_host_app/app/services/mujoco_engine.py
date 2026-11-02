"""MuJoCo 仿真引擎 — 独立线程运行物理步进和离屏渲染。"""

import os
import numpy as np
from PySide6.QtCore import QThread, Signal, QMutex

from app.platform_compat import is_wsl, scaled_render_size

try:
    import mujoco
    HAS_MUJOCO = True
    # 修补 Renderer.__del__: 构造若半途失败，_gl_context 可能不存在
    _orig_renderer_del = mujoco.Renderer.__del__
    def _safe_renderer_del(self):
        if hasattr(self, '_gl_context'):
            _orig_renderer_del(self)
    mujoco.Renderer.__del__ = _safe_renderer_del
except ImportError:
    HAS_MUJOCO = False

class MuJoCoEngine(QThread):
    """MuJoCo物理仿真引擎

    Signals:
        frame_ready(np.ndarray): 渲染帧就绪 (H, W, 3) uint8
        sim_info(dict): 仿真信息 (时间、步数等)
        error_occurred(str): 错误信息
    """

    frame_ready = Signal(object)
    sim_info = Signal(dict)
    error_occurred = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False
        self._mutex = QMutex()
        self._model = None
        self._data = None
        self._renderer = None
        self._actuator_names: list[str] = []
        self._actuator_ids: list[int] = []
        self._target_angles = np.zeros(0, dtype=np.float64)
        self._render_width = 800
        self._render_height = 600
        self._resize_dirty = False
        self._sim_step_ms = 8 if is_wsl() else 5  # 125Hz on WSL, 200Hz elsewhere

        # 相机参数
        self._cam_azimuth = 90.0
        self._cam_elevation = -20.0
        self._cam_distance = 1.5
        self._cam_lookat = np.array([0.0, 0.0, 0.3])

    @property
    def is_loaded(self) -> bool:
        return self._model is not None

    @property
    def num_actuators(self) -> int:
        """返回已加载模型的执行器数量"""
        if self._model is not None:
            return self._model.nu
        return 0

    def load_model(self, model_path: str, actuator_names: list[str] | None = None) -> bool:
        if not HAS_MUJOCO:
            self.error_occurred.emit("MuJoCo 未安装，请执行 pip install mujoco")
            return False
        try:
            # MuJoCo C底层不支持非ASCII路径，切换工作目录后用文件名加载
            self._actuator_names = list(actuator_names or [])
            model_dir = os.path.dirname(os.path.abspath(model_path))
            model_file = os.path.basename(model_path)
            old_cwd = os.getcwd()
            try:
                os.chdir(model_dir)
                self._model = mujoco.MjModel.from_xml_path(model_file)
            finally:
                os.chdir(old_cwd)
            self._actuator_ids = self._resolve_actuators()
            self._data = mujoco.MjData(self._model)
            # Renderer 延迟到 run() 中创建，避免主线程 OpenGL 上下文冲突
            self._renderer = None
            return True
        except Exception as e:
            self.error_occurred.emit(f"模型加载失败: {e}")
            return False

    def _resolve_actuators(self) -> list[int]:
        if self._model is None:
            return []
        if not self._actuator_names:
            return list(range(self._model.nu))

        actuator_ids: list[int] = []
        missing: list[str] = []
        for name in self._actuator_names:
            actuator_id = mujoco.mj_name2id(
                self._model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                name,
            )
            if actuator_id < 0:
                missing.append(name)
            actuator_ids.append(int(actuator_id))
        if missing:
            preview = ", ".join(missing[:8])
            raise ValueError(f"missing actuators in model: {preview}")
        return actuator_ids

    def set_target_angles(self, angles: list[float]):
        """设置目标关节角度（度 转 弧度）"""
        self._mutex.lock()
        self._target_angles = np.radians(np.asarray(angles, dtype=np.float64))
        self._mutex.unlock()

    def set_camera(self, azimuth: float = None, elevation: float = None,
                   distance: float = None):
        if azimuth is not None:
            self._cam_azimuth = azimuth
        if elevation is not None:
            self._cam_elevation = elevation
        if distance is not None:
            self._cam_distance = distance

    def set_render_size(self, width: int, height: int):
        """Set the target display size; offscreen rendering may use a scale."""
        width, height = scaled_render_size(width, height)
        width = int(max(160, width))
        height = int(max(160, height))
        if width != self._render_width or height != self._render_height:
            self._render_width = width
            self._render_height = height
            self._resize_dirty = True

    def move_lookat(self, dx: float, dy: float):
        """平移相机观察点（右键拖动）

        拖动方向 = 物体在屏幕上的移动方向：
        鼠标往右 → 物体往右（lookat 往左）
        鼠标往上 → 物体往上（lookat[2] 减小）
        """
        import math
        az_rad = math.radians(self._cam_azimuth)
        scale = self._cam_distance * 0.002
        # 屏幕 dx → 水平平移（反向移动 lookat 使物体同向移动）
        self._cam_lookat[0] += (dx * math.sin(az_rad)) * scale
        self._cam_lookat[1] += (dx * math.cos(az_rad)) * scale
        # 屏幕 dy → 垂直平移
        self._cam_lookat[2] += dy * scale

    def run(self):
        """仿真主循环"""
        if not self.is_loaded:
            self.error_occurred.emit("模型未加载")
            return

        # 在工作线程中创建 Renderer，避免与主线程 Qt OpenGL 上下文冲突
        if self._renderer is None and not self._create_renderer():
            return

        self._running = True
        step_count = 0

        while self._running:
            # 设置控制信号
            if self._resize_dirty:
                self._resize_dirty = False
                if not self._create_renderer():
                    return

            # 设置控制信号
            self._mutex.lock()
            actuator_ids = self._actuator_ids or list(range(self._model.nu))
            n_ctrl = min(len(actuator_ids), len(self._target_angles))
            for i in range(n_ctrl):
                actuator_id = actuator_ids[i]
                if 0 <= actuator_id < self._model.nu:
                    self._data.ctrl[actuator_id] = self._target_angles[i]
            self._mutex.unlock()

            # 物理步进
            mujoco.mj_step(self._model, self._data)
            step_count += 1

            # 每隔若干步渲染一帧（约30fps）
            steps_per_frame = max(1, int(1000 / self._sim_step_ms / 30))
            if step_count % steps_per_frame == 0:
                self._render_frame()
                self.sim_info.emit({
                    "time": self._data.time,
                    "step": step_count,
                })

            self.msleep(self._sim_step_ms)

        # 线程退出前关闭 Renderer，释放 OpenGL 上下文
        # 下次 run() 会在新线程中重新创建
        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:
                pass
            self._renderer = None

    def _create_renderer(self) -> bool:
        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:
                pass
            self._renderer = None
        try:
            max_w = self._model.vis.global_.offwidth
            max_h = self._model.vis.global_.offheight
            w = min(self._render_width, max_w)
            h = min(self._render_height, max_h)
            self._renderer = mujoco.Renderer(self._model, height=h, width=w)
            return True
        except Exception as e:
            self.error_occurred.emit(f"渲染器创建失败: {e}")
            return False

    def _render_frame(self):
        """渲染一帧并发射信号"""
        try:
            cam = mujoco.MjvCamera()
            cam.azimuth = self._cam_azimuth
            cam.elevation = self._cam_elevation
            cam.distance = self._cam_distance
            cam.lookat[:] = self._cam_lookat

            self._renderer.update_scene(self._data, cam)
            pixels = self._renderer.render()
            self.frame_ready.emit(pixels.copy())
        except Exception as e:
            self.error_occurred.emit(f"渲染错误: {e}")

    def stop(self):
        self._running = False
        if not self.wait(3000):
            self.terminate()
            self.wait(1000)
