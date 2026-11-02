"""仿真遥操作页面 — 左侧MuJoCo仿真 + 右侧摄像头遥操作，同时驱动仿真和真实电机"""

import importlib.util
import os
import sys
import time
from pathlib import Path

import numpy as np
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QFileDialog, QSplitter,
    QCheckBox, QDoubleSpinBox, QLineEdit,
)
from PySide6.QtCore import Qt, QThread, Signal, Slot, QPoint
from PySide6.QtGui import QImage, QPixmap, QMouseEvent

from app.models.hand_state import HandState
from app.platform_compat import is_wsl, render_scale
from app.services.force_control import ForceControlService
from app.services.mujoco_engine import MuJoCoEngine
from app.services.video_capture import camera_path, inspect_v4l2_device, open_camera_capture, parse_camera_source

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

# ── DexPilot 延迟导入（避免启动时加载 mediapipe/nlopt 等重型库） ──
_RETARGET_DIR = str(Path(__file__).resolve().parent.parent / "services" / "retargeting")

_dexpilot_imported = False
SingleHandDetector = None
MujocoRobotWrapper = None
MujocoDexPilotOptimizer = None
LPFilter = None


def _ensure_dexpilot_imports():
    """首次调用时才导入 DexPilot 重型依赖"""
    global _dexpilot_imported, SingleHandDetector, MujocoRobotWrapper, MujocoDexPilotOptimizer, LPFilter
    if _dexpilot_imported:
        return True
    if _RETARGET_DIR not in sys.path:
        sys.path.insert(0, _RETARGET_DIR)
    try:
        from detector import SingleHandDetector as _Det
        from dex_retargeting.mujoco_robot_wrapper import MujocoRobotWrapper as _RW
        from dex_retargeting.mujoco_vector_optimizer import MujocoDexPilotOptimizer as _Opt
        from dex_retargeting.optimizer_utils import LPFilter as _LP
        SingleHandDetector = _Det
        MujocoRobotWrapper = _RW
        MujocoDexPilotOptimizer = _Opt
        LPFilter = _LP
        _dexpilot_imported = True
    except ImportError as _e:
        print(f"[sim_teleop] DexPilot 模块导入失败: {_e}")
    return _dexpilot_imported


def _camera_open_precheck(camera_id: int | str) -> tuple[bool, str]:
    """Avoid noisy OpenCV backend warnings when a Linux camera node is absent."""
    source = parse_camera_source(camera_id)
    if not isinstance(source, int):
        return True, ""
    if os.name != "posix":
        return True, ""

    device_path = camera_path(source)
    info = inspect_v4l2_device(device_path)
    if info.is_capture:
        return True, ""

    if is_wsl():
        return (
            False,
            f"WSL2 中 {info.message or f'未发现 {device_path}'}。"
            "外接 USB 摄像头需要先通过 usbipd-win attach 到 WSL；"
            "注意同一个摄像头的部分 /dev/videoN 可能只是 metadata 节点。",
        )
    return False, info.message or f"未发现 Linux 摄像头设备 {device_path}"


# ── DexPilot 重定向所需的 MujocoSeqRetargeting 包装器 ──

class MujocoSeqRetargeting:
    """Sequential retargeting wrapper for MuJoCo optimizer."""

    def __init__(self, optimizer, has_joint_limits=True, lp_filter=None):
        self.optimizer = optimizer
        robot = self.optimizer.robot

        self.has_joint_limits = has_joint_limits
        joint_limits = np.ones_like(robot.joint_limits)
        joint_limits[:, 0] = -1e4
        joint_limits[:, 1] = 1e4
        if has_joint_limits:
            joint_limits[:] = robot.joint_limits[:]
            self.optimizer.set_joint_limit(joint_limits[self.optimizer.idx_pin2target])
        self.joint_limits = joint_limits[self.optimizer.idx_pin2target]

        self.last_qpos = joint_limits.mean(1)[self.optimizer.idx_pin2target].astype(np.float32)
        self.accumulated_time = 0
        self.num_retargeting = 0
        self.filter = lp_filter

    def retarget(self, ref_value, fixed_qpos=np.array([])):
        tic = time.perf_counter()
        if fixed_qpos.size == 0 and len(self.optimizer.idx_pin2fixed) > 0:
            fixed_qpos = self.optimizer.robot.q0[self.optimizer.idx_pin2fixed]

        qpos = self.optimizer.retarget(
            ref_value=ref_value.astype(np.float64),
            fixed_qpos=fixed_qpos.astype(np.float64),
            last_qpos=np.clip(self.last_qpos, self.joint_limits[:, 0], self.joint_limits[:, 1]),
        )

        self.accumulated_time += time.perf_counter() - tic
        self.num_retargeting += 1
        self.last_qpos = qpos

        robot_qpos = np.zeros(self.optimizer.robot.dof)
        robot_qpos[self.optimizer.idx_pin2fixed] = fixed_qpos
        robot_qpos[self.optimizer.idx_pin2target] = qpos

        if self.filter is not None:
            robot_qpos = self.filter.next(robot_qpos)
        return robot_qpos

    def reset(self):
        self.last_qpos = self.joint_limits.mean(1).astype(np.float32)
        self.num_retargeting = 0
        self.accumulated_time = 0

    @property
    def joint_names(self):
        return self.optimizer.robot.dof_joint_names


class DexPilotRetargeter:
    """DexPilot retargeter for profiles that explicitly declare DexPilot names."""

    # DexPilot 10 向量的索引定义
    _VEC_INDICES = np.array([
        [8, 12, 16, 12, 16, 16, 0, 0, 0, 0],   # origin
        [4,  4,  4,  8,  8, 12, 4, 8, 12, 16],  # task
    ])

    def __init__(self, xml_path: str, profile=None):
        if not _ensure_dexpilot_imports():
            raise RuntimeError("DexPilot 依赖未安装 (需要 nlopt、mediapipe 等)")

        self.robot = MujocoRobotWrapper(xml_path, from_string=False)

        target_joint_names = list(getattr(profile, "teleop_joint_names", None) or ())
        finger_tip_link_names = list(getattr(profile, "teleop_tip_link_names", None) or ())
        if not target_joint_names or not finger_tip_link_names:
            raise RuntimeError(
                "DexPilot 需要手型 manifest 显式声明 teleop.joint_names 和 teleop.tip_link_names；"
                "通用上位机不再内置任何特定手型的默认关节名。"
            )
        self._output_joint_names = list(getattr(profile, "joint_names", None) or target_joint_names)
        wrist_link_name = getattr(profile, "teleop_wrist_link_name", "base_link")

        optimizer = MujocoDexPilotOptimizer(
            self.robot,
            target_joint_names,
            finger_tip_link_names=finger_tip_link_names,
            wrist_link_name=wrist_link_name,
            scaling=1.2,
        )

        lp_filter = LPFilter(0.5)
        self.retargeting = MujocoSeqRetargeting(
            optimizer, has_joint_limits=True, lp_filter=lp_filter,
        )
        robot_joint_names = list(self.retargeting.joint_names)
        missing_output = [
            name for name in self._output_joint_names
            if name not in robot_joint_names
        ]
        if missing_output:
            preview = ", ".join(missing_output[:8])
            raise RuntimeError(f"DexPilot output joints missing in model: {preview}")
        self._output_indices = [
            robot_joint_names.index(name)
            for name in self._output_joint_names
        ]
        self._num_joints = getattr(profile, "num_joints", len(self._output_indices))
        print(f"[DexPilot] 初始化完成, 关节: {self.retargeting.joint_names}")

    def retarget(self, joint_pos_3d: np.ndarray, selfie: bool = True) -> list[float]:
        """将 21×3 的人手关键点转换为当前手型的关节角度（度）。"""
        if joint_pos_3d is None:
            return [0.0] * self._num_joints

        # selfie 模式翻转 X 轴
        if selfie:
            joint_pos_3d = joint_pos_3d.copy()
            joint_pos_3d[:, 0] = -joint_pos_3d[:, 0]

        # 计算 10 个 DexPilot 向量
        origin_idx = self._VEC_INDICES[0]
        task_idx = self._VEC_INDICES[1]
        ref_value = joint_pos_3d[task_idx, :] - joint_pos_3d[origin_idx, :]

        # 优化求解
        qpos = self.retargeting.retarget(ref_value)

        # 弧度→度，按当前手型输出关节顺序截取。
        angles_deg = np.degrees(qpos).tolist()
        return [angles_deg[i] for i in self._output_indices][:self._num_joints]


class ExternalPythonTeleopRetargeter:
    """Load a hand package teleoperation adapter from teleop_adapter.py."""

    def __init__(self, profile):
        path = getattr(profile, "teleop_adapter_path", None)
        if path is None:
            raise RuntimeError("当前手型没有配置 teleop.adapter_path")
        path = Path(path)
        if not path.exists():
            raise RuntimeError(f"遥操作适配器不存在: {path}")
        module_name = f"dexhand_teleop_{profile.hand_id}_{abs(hash(path))}"
        spec = importlib.util.spec_from_file_location(module_name, path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"无法导入遥操作适配器: {path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        sys.path.insert(0, str(path.parent))
        try:
            spec.loader.exec_module(module)
            factory = getattr(module, "create_teleop_adapter", None)
            if callable(factory):
                adapter = factory(profile)
            else:
                cls = getattr(module, getattr(profile, "teleop_adapter_class", "TeleopAdapter"), None)
                if cls is None:
                    raise RuntimeError("teleop adapter must provide create_teleop_adapter(profile) or TeleopAdapter")
                try:
                    adapter = cls(profile)
                except TypeError:
                    adapter = cls()
        finally:
            try:
                sys.path.remove(str(path.parent))
            except ValueError:
                pass
        if not callable(getattr(adapter, "retarget", None)):
            raise RuntimeError("遥操作适配器缺少 retarget(hand_payload) 方法")
        self.profile = profile
        self.adapter = adapter

    @property
    def has_frame_processor(self) -> bool:
        return callable(getattr(self.adapter, "process_frame", None))

    def process_frame(self, frame):
        method = getattr(self.adapter, "process_frame", None)
        if not callable(method):
            return frame, None
        result = method(frame)
        if isinstance(result, tuple) and len(result) == 2:
            return result
        return frame, result

    def retarget(self, hand_payload) -> list[float]:
        angles = list(self.adapter.retarget(hand_payload))
        if len(angles) != self.profile.num_joints:
            raise RuntimeError(f"遥操作返回 {len(angles)} 个角度，但当前手型需要 {self.profile.num_joints} 个")
        return [float(v) for v in angles]

    def close(self):
        for method_name in ("close", "cleanup", "close_tracker"):
            method = getattr(self.adapter, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass
                return


class CameraWorker(QThread):
    """摄像头采集 + SingleHandDetector 手部 3D 关键点检测工作线程"""

    frame_ready = Signal(object)          # (bgr_frame, joint_pos_3d_or_None)
    error_occurred = Signal(str)

    def __init__(self, camera_id: int | str = 0, parent=None, payload_processor=None):
        super().__init__(parent)
        self._camera_id = camera_id
        self._payload_processor = payload_processor
        self._running = False
        self._cap = None

    def run(self):
        if not HAS_CV2:
            self.error_occurred.emit("OpenCV 未安装")
            return
        ok, message = _camera_open_precheck(self._camera_id)
        if not ok:
            self.error_occurred.emit(message)
            return
        detector = None
        if self._payload_processor is None:
            if not _ensure_dexpilot_imports():
                self.error_occurred.emit("DexPilot/MediaPipe 模块未安装")
                return

        try:
            self._cap = open_camera_capture(self._camera_id, width=640, height=480)
        except Exception as e:
            self.error_occurred.emit(str(e))
            return

        # 初始化 SingleHandDetector
        if self._payload_processor is None:
            try:
                detector = SingleHandDetector(hand_type="Right", selfie=True)
            except Exception as e:
                self.error_occurred.emit(f"SingleHandDetector 初始化失败: {e}")
                return

        self._running = True
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                self.msleep(30)
                continue

            # 水平镜像，使画面如照镜子般自然
            frame = cv2.flip(frame, 1)

            if self._payload_processor is not None:
                try:
                    processed_frame, hand_payload = self._payload_processor.process_frame(frame)
                except Exception as e:
                    self.error_occurred.emit(f"遥操作帧处理失败: {e}")
                    break
                self.frame_ready.emit((processed_frame if processed_frame is not None else frame, hand_payload))
                self.msleep(33)
                continue

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            num_hands, joint_pos_3d, keypoint_2d, _ = detector.detect(rgb)

            if num_hands > 0 and joint_pos_3d is not None:
                # 在画面上绘制骨架
                frame = detector.draw_skeleton_on_image(frame, keypoint_2d, style="default")
                self.frame_ready.emit((frame, joint_pos_3d))
            else:
                self.frame_ready.emit((frame, None))

            self.msleep(33)  # ~30fps

        self._cap.release()

    def stop(self):
        self._running = False
        self.wait(2000)


class MuJoCoRenderLabel(QLabel):
    """支持鼠标拖拽调整相机的渲染画面标签"""

    def __init__(self, engine: MuJoCoEngine, parent=None):
        super().__init__(parent)
        self.engine = engine
        self._last_pos = QPoint()
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setStyleSheet(
            "background-color: #2d2d2d; color: #888; "
            "font-size: 14px; border-radius: 8px;"
        )
        self.setText("请加载 MuJoCo 模型文件")
        self.engine.set_render_size(self.width(), self.height())

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.engine.set_render_size(self.width(), self.height())

    def mousePressEvent(self, event: QMouseEvent):
        self._last_pos = event.position().toPoint()

    def mouseMoveEvent(self, event: QMouseEvent):
        delta = event.position().toPoint() - self._last_pos
        self._last_pos = event.position().toPoint()
        if event.buttons() & Qt.MouseButton.LeftButton:
            # 左键旋转（与 MuJoCo 原生方向一致）
            self.engine.set_camera(
                azimuth=self.engine._cam_azimuth - delta.x() * 0.5,
                elevation=self.engine._cam_elevation - delta.y() * 0.5,
            )
        elif event.buttons() & Qt.MouseButton.RightButton:
            # 右键平移观察点
            self.engine.move_lookat(delta.x(), delta.y())

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        # 滚轮上滚 = 缩小（distance增大），下滚 = 放大（distance减小）
        new_dist = self.engine._cam_distance + delta * 0.001
        self.engine.set_camera(distance=max(0.3, min(5.0, new_dist)))


class SimTeleopPage(QWidget):
    """仿真遥操作组合页面"""

    def __init__(self, hand_state: HandState, parent=None):
        super().__init__(parent)
        self.hand_state = hand_state
        self.profile = hand_state.profile
        self.engine = MuJoCoEngine(self)
        self.camera_worker: CameraWorker | None = None
        self._teleop_active = False
        self._dexpilot: DexPilotRetargeter | None = None
        self._external_teleop: ExternalPythonTeleopRetargeter | None = None
        self._force_control_enabled = False
        self._force_ctrl = ForceControlService(profile=self.profile)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(8)

        # ── 顶部控制栏 ──
        top_bar = QHBoxLayout()
        title = QLabel("仿真遥操作")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #333;")
        top_bar.addWidget(title)
        top_bar.addStretch()

        # 加载模型
        self.load_btn = QPushButton("加载模型")
        self.load_btn.setStyleSheet(
            "QPushButton { background-color: #0078d4; color: white; border: none; "
            "border-radius: 4px; padding: 6px 16px; }"
            "QPushButton:hover { background-color: #106ebe; }"
        )
        self.load_btn.clicked.connect(self._load_model)
        top_bar.addWidget(self.load_btn)

        # 启动/停止仿真
        self.sim_btn = QPushButton("启动仿真")
        self.sim_btn.setEnabled(False)
        self.sim_btn.setStyleSheet(
            "QPushButton { background-color: #28a745; color: white; border: none; "
            "border-radius: 4px; padding: 6px 16px; }"
            "QPushButton:hover { background-color: #218838; }"
            "QPushButton:disabled { background-color: #ccc; }"
        )
        self.sim_btn.clicked.connect(self._toggle_sim)
        top_bar.addWidget(self.sim_btn)

        # 摄像头选择
        top_bar.addWidget(QLabel("摄像头源:"))
        self.camera_source_edit = QLineEdit("0")
        self.camera_source_edit.setFixedWidth(120)
        self.camera_source_edit.setToolTip("可填摄像头编号，如 0；也可填视频文件或 http/rtsp 地址")
        top_bar.addWidget(self.camera_source_edit)

        # 启动/停止摄像头
        self.cam_btn = QPushButton("启动摄像头")
        self.cam_btn.setStyleSheet(
            "QPushButton { background-color: #17a2b8; color: white; border: none; "
            "border-radius: 4px; padding: 6px 16px; }"
            "QPushButton:hover { background-color: #138496; }"
        )
        self.cam_btn.clicked.connect(self._toggle_camera)
        top_bar.addWidget(self.cam_btn)

        # 启动/停止遥操作
        self.teleop_btn = QPushButton("启动遥操作")
        self.teleop_btn.setEnabled(False)
        self.teleop_btn.setStyleSheet(
            "QPushButton { background-color: #fd7e14; color: white; border: none; "
            "border-radius: 4px; padding: 6px 16px; }"
            "QPushButton:hover { background-color: #e8690b; }"
            "QPushButton:disabled { background-color: #ccc; }"
        )
        self.teleop_btn.clicked.connect(self._toggle_teleop)
        top_bar.addWidget(self.teleop_btn)

        # 力控
        self.force_control_cb = QCheckBox("启用力控")
        self.force_control_cb.toggled.connect(self._on_force_control_toggled)
        top_bar.addWidget(self.force_control_cb)

        top_bar.addWidget(QLabel("力矩缩放:"))
        self.force_threshold_spin = QDoubleSpinBox()
        self.force_threshold_spin.setRange(0.1, 5.0)
        self.force_threshold_spin.setSingleStep(0.1)
        self.force_threshold_spin.setValue(1.0)
        self.force_threshold_spin.setSuffix(" x")
        self.force_threshold_spin.setToolTip(
            "力矩阈值缩放系数\n1.0=使用每关节默认阈值\n<1.0=更敏感\n>1.0=更大力"
        )
        self.force_threshold_spin.setFixedWidth(90)
        top_bar.addWidget(self.force_threshold_spin)

        layout.addLayout(top_bar)

        # ── 中央区域：左右分割 ──
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # 左侧：MuJoCo 渲染
        left_container = QWidget()
        left_layout = QVBoxLayout(left_container)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.addWidget(QLabel("MuJoCo 仿真"))
        self.render_label = MuJoCoRenderLabel(self.engine)
        left_layout.addWidget(self.render_label, 1)

        # 仿真信息
        sim_info_bar = QHBoxLayout()
        self.time_label = QLabel("仿真时间: 0.000s")
        self.step_label = QLabel("步数: 0")
        sim_info_bar.addWidget(self.time_label)
        sim_info_bar.addWidget(self.step_label)
        sim_info_bar.addStretch()
        left_layout.addLayout(sim_info_bar)

        # 右侧：摄像头画面
        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.addWidget(QLabel("摄像头视图"))
        self.camera_label = QLabel("点击「启动摄像头」开始")
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_label.setMinimumSize(320, 240)
        self.camera_label.setStyleSheet(
            "background-color: #1a1a1a; color: #888; "
            "font-size: 14px; border-radius: 8px;"
        )
        right_layout.addWidget(self.camera_label, 1)

        # 状态栏
        cam_info_bar = QHBoxLayout()
        self.hand_detected_label = QLabel("手部检测: --")
        self.teleop_status_label = QLabel("遥操作: 未启动")
        self.force_status_label = QLabel("力控: OFF")
        cam_info_bar.addWidget(self.hand_detected_label)
        cam_info_bar.addWidget(self.teleop_status_label)
        cam_info_bar.addWidget(self.force_status_label)
        cam_info_bar.addStretch()
        right_layout.addLayout(cam_info_bar)

        splitter.addWidget(left_container)
        splitter.addWidget(right_container)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter, 1)

        # 信号连接
        self.engine.frame_ready.connect(self._on_mujoco_frame)
        self.engine.sim_info.connect(self._on_sim_info)
        self.engine.error_occurred.connect(self._on_mujoco_error)
        self.hand_state.joint_angles_changed.connect(self._on_angles_changed)

        if not HAS_CV2:
            self.cam_btn.setEnabled(False)
            self.hand_detected_label.setText("OpenCV 未安装")
        if not self.profile.force_control_supported:
            self.force_control_cb.setEnabled(False)
            self.force_threshold_spin.setEnabled(False)
            self.force_status_label.setText("力控: 当前手型未启用")
            self.force_status_label.setToolTip("当前手型未在接入向导中声明力控能力，或驱动适配器未提供电流/力矩遥测。")

    def _on_force_control_toggled(self, checked: bool):
        if checked and not self.profile.force_control_supported:
            self.force_control_cb.setChecked(False)
            return
        self._force_control_enabled = checked
        if checked:
            scale = self.force_threshold_spin.value()
            self._force_ctrl.config.torque_scale = scale
            self._force_ctrl.reset()
            self.force_status_label.setText(f"力控: ON ({scale:.1f}x)")
            self.force_status_label.setStyleSheet("color: #dc3545;")
        else:
            self.force_status_label.setText("力控: OFF")
            self.force_status_label.setStyleSheet("color: #999;")

    def _get_telemetry(self) -> dict:
        """获取电机遥测数据"""
        main_w = self.window()
        if hasattr(main_w, 'get_page'):
            cp = main_w.get_page("连接管理")
            if cp and hasattr(cp, 'get_motor_service'):
                svc = cp.get_motor_service()
                if svc and svc.is_connected:
                    return svc.get_latest_telemetry()
        return {}

    # ── MuJoCo 控制 ──

    def _ensure_external_teleop(self) -> ExternalPythonTeleopRetargeter | None:
        if self.profile.teleop_kind != "external_python":
            return None
        if self._external_teleop is not None:
            return self._external_teleop
        try:
            self._external_teleop = ExternalPythonTeleopRetargeter(self.profile)
        except Exception as e:
            self._external_teleop = None
            print(f"[sim_teleop] 外部遥操作初始化失败: {e}")
        return self._external_teleop

    def _do_load_model(self, path: str):
        """加载指定路径的模型并初始化 DexPilot"""
        if self.engine.load_model(path, self.profile.actuator_names):
            nu = self.engine.num_actuators
            model_info = f"模型已加载 ({nu}个执行器)"
            self._dexpilot = None
            if self.profile.teleop_kind == "external_python":
                if self._ensure_external_teleop() is not None:
                    model_info += " [external teleop]"
            elif self.profile.teleop_kind == "dexpilot" and _ensure_dexpilot_imports():
                try:
                    self._dexpilot = DexPilotRetargeter(path, self.profile)
                    model_info += " [DexPilot]"
                except Exception as e:
                    self._dexpilot = None
                    print(f"[sim_teleop] DexPilot初始化失败: {e}")
            self.sim_btn.setEnabled(True)
            self.render_label.setText(model_info)
            return True
        else:
            self.render_label.setText("模型加载失败")
            return False

    def _load_model(self):
        """点击"加载模型"：优先加载内置默认模型，否则弹出文件选择"""
        default_model = str(self.profile.model_path)
        if os.path.isfile(default_model):
            self._do_load_model(default_model)
        else:
            path, _ = QFileDialog.getOpenFileName(
                self, "选择 MuJoCo 模型文件", "",
                "MuJoCo 模型 (*.xml *.mjcf);;所有文件 (*)"
            )
            if path:
                self._do_load_model(path)

    def _toggle_sim(self):
        if self.engine.isRunning():
            self.engine.stop()
            self.sim_btn.setText("启动仿真")
        else:
            self.engine.set_target_angles(self.hand_state.angles.tolist())
            self.engine.start()
            self.sim_btn.setText("停止仿真")

    @Slot(object)
    def _on_mujoco_frame(self, pixels: np.ndarray):
        h, w = pixels.shape[:2]
        q_img = QImage(pixels.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(q_img)
        transform_mode = (
            Qt.TransformationMode.FastTransformation
            if render_scale() < 1.0
            else Qt.TransformationMode.SmoothTransformation
        )
        scaled = pixmap.scaled(
            self.render_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            transform_mode,
        )
        self.render_label.setPixmap(scaled)

    @Slot(dict)
    def _on_sim_info(self, info: dict):
        self.time_label.setText(f"仿真时间: {info['time']:.3f}s")
        self.step_label.setText(f"步数: {info['step']}")

    @Slot(str)
    def _on_mujoco_error(self, msg: str):
        self.render_label.setText(f"错误: {msg}")

    @Slot(list)
    def _on_angles_changed(self, angles: list):
        if self.engine.isRunning():
            self.engine.set_target_angles(angles)

    # ── 摄像头 / 遥操作控制 ──

    def _toggle_camera(self):
        if self.camera_worker and self.camera_worker.isRunning():
            self._stop_camera()
        else:
            self._start_camera()

    def _start_camera(self):
        camera_id = self.camera_source_edit.text().strip() or "0"
        payload_processor = self._ensure_external_teleop()
        if payload_processor is not None and not payload_processor.has_frame_processor:
            payload_processor = None
        self.camera_worker = CameraWorker(camera_id, self, payload_processor=payload_processor)
        self.camera_worker.frame_ready.connect(self._on_camera_frame)
        self.camera_worker.error_occurred.connect(self._on_camera_error)
        self.camera_worker.start()
        self.cam_btn.setText("停止摄像头")
        self.teleop_btn.setEnabled(True)

    def _stop_camera(self):
        self._teleop_active = False
        self.teleop_btn.setText("启动遥操作")
        self.teleop_btn.setEnabled(False)
        self.teleop_status_label.setText("遥操作: 未启动")
        if self.camera_worker:
            self.camera_worker.stop()
            self.camera_worker = None
        self.cam_btn.setText("启动摄像头")
        self.camera_label.setText("点击「启动摄像头」开始")

    def _toggle_teleop(self):
        self._teleop_active = not self._teleop_active
        if self._teleop_active:
            self.teleop_btn.setText("停止遥操作")
            self.teleop_status_label.setText("遥操作: 运行中")
        else:
            self.teleop_btn.setText("启动遥操作")
            self.teleop_status_label.setText("遥操作: 已停止")

    @Slot(object)
    def _on_camera_frame(self, data):
        frame, hand_payload = data

        # 显示画面
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        q_img = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(q_img)
        scaled = pixmap.scaled(
            self.camera_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.camera_label.setPixmap(scaled)

        # 手势检测状态
        if hand_payload is not None:
            self.hand_detected_label.setText("手部检测: 已检测到")
            self.hand_detected_label.setStyleSheet("color: #28a745;")
            # 遥操作：同时驱动仿真和真实电机
            if self._teleop_active:
                if self.profile.teleop_kind == "external_python":
                    if self._external_teleop is not None:
                        angles = self._external_teleop.retarget(hand_payload)
                    else:
                        angles = [0.0] * self.profile.num_joints
                elif self._dexpilot is not None:
                    angles = self._dexpilot.retarget(hand_payload)
                else:
                    angles = [0.0] * self.profile.num_joints

                # ── 力控：逐电机Q轴电流堵转检测 ──
                if self._force_control_enabled and self.profile.force_control_supported:
                    self._force_ctrl.config.torque_scale = self.force_threshold_spin.value()
                    telemetry = self._get_telemetry()
                    angles = self._force_ctrl.apply(angles, telemetry)

                self.hand_state.set_all_angles(angles)
                # 使用钳位后的角度发送到真实电机
                clamped_angles = self.hand_state.angles.tolist()
                main_win = self.window()
                if hasattr(main_win, 'get_page'):
                    conn_page = main_win.get_page("连接管理")
                    if conn_page and hasattr(conn_page, 'get_motor_service'):
                        service = conn_page.get_motor_service()
                        if service.is_connected:
                            kwargs = {}
                            move_time_ms = (
                                getattr(self.profile, "motor_adapter_config", {}) or {}
                            ).get("teleop_move_time_ms")
                            if move_time_ms is not None:
                                kwargs["move_time_ms"] = int(move_time_ms)
                            service.set_all_joint_angles(clamped_angles, **kwargs)
        else:
            self.hand_detected_label.setText("手部检测: 未检测到")
            self.hand_detected_label.setStyleSheet("color: #999;")

    @Slot(str)
    def _on_camera_error(self, msg: str):
        self.camera_label.setText(f"错误: {msg}")
        self.hand_detected_label.setText("手部检测: 摄像头不可用")
        self.hand_detected_label.setStyleSheet("color: #dc3545;")
        self._teleop_active = False
        self.teleop_btn.setText("启动遥操作")
        self.teleop_btn.setEnabled(False)
        self.teleop_status_label.setText("遥操作: 未启动")
        self.cam_btn.setText("启动摄像头")
        if self.camera_worker:
            worker = self.camera_worker
            self.camera_worker = None
            worker.stop()

    def cleanup(self):
        self.engine.stop()
        if self.camera_worker:
            self.camera_worker.stop()
            self.camera_worker = None
        if self._external_teleop is not None:
            self._external_teleop.close()
            self._external_teleop = None
