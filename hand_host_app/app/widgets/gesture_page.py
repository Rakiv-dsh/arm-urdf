"""预设手势页面 — 当前手型手势一键执行、自定义保存。"""

import json
import os
from pathlib import Path

import numpy as np
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QGroupBox, QGridLayout, QInputDialog,
    QScrollArea, QFrame, QMessageBox, QCheckBox, QDoubleSpinBox,
)
from PySide6.QtCore import Qt, QTimer

from app.models.hand_state import HandState
from app.services.force_control import ForceControlService, ForceControlConfig

GESTURES_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "gestures")


# 手型包通过 manifest.gestures.builtin 提供内置手势。
BUILTIN_GESTURES: dict[str, list[float]] = {}


class GestureButton(QPushButton):
    """手势按钮"""

    def __init__(self, name: str, is_custom: bool = False, parent=None):
        super().__init__(name, parent)
        self.gesture_name = name
        self.is_custom = is_custom
        self.setFixedHeight(60)
        self.setMinimumWidth(120)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

        color = "#17a2b8" if is_custom else "#0078d4"
        hover_color = "#138496" if is_custom else "#106ebe"
        self.setStyleSheet(f"""
            QPushButton {{
                background-color: {color}; color: white;
                border: none; border-radius: 8px;
                font-size: 14px; font-weight: bold;
            }}
            QPushButton:hover {{ background-color: {hover_color}; }}
            QPushButton:pressed {{ background-color: #333; }}
        """)


class GesturePage(QWidget):
    def __init__(self, hand_state: HandState, parent=None):
        super().__init__(parent)
        self.hand_state = hand_state
        self.profile = hand_state.profile
        self.gestures_dir = str(self.profile.gestures_dir)
        self._builtin_gestures = self.profile.default_gestures
        self._custom_gestures: dict[str, list[float]] = {}

        # 力控
        self._force_ctrl = ForceControlService(profile=self.profile)
        self._force_enabled = False
        self._gesture_target: list[float] | None = None  # 力控执行中的目标角度
        self._gesture_timer = QTimer(self)
        self._gesture_timer.setInterval(50)  # 20Hz 力控循环
        self._gesture_timer.timeout.connect(self._force_ctrl_tick)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)

        # 标题栏
        top_bar = QHBoxLayout()
        title = QLabel("动作序列")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #333;")
        top_bar.addWidget(title)
        top_bar.addStretch()

        # 力控开关
        self.force_cb = QCheckBox("启用力控")
        self.force_cb.toggled.connect(self._on_force_toggled)
        top_bar.addWidget(self.force_cb)

        top_bar.addWidget(QLabel("力矩缩放:"))
        self.force_spin = QDoubleSpinBox()
        self.force_spin.setRange(0.1, 5.0)
        self.force_spin.setSingleStep(0.1)
        self.force_spin.setValue(1.0)
        self.force_spin.setSuffix(" x")
        self.force_spin.setToolTip("力矩阈值缩放系数\n1.0=使用每关节默认阈值\n<1.0=更敏感(force较小就停)\n>1.0=更大力(需要更大力才停)")
        self.force_spin.setFixedWidth(90)
        top_bar.addWidget(self.force_spin)

        self.force_status_label = QLabel("力控: OFF")
        self.force_status_label.setStyleSheet("color: #999; font-size: 12px;")
        top_bar.addWidget(self.force_status_label)
        if not self.profile.force_control_supported:
            self.force_cb.setEnabled(False)
            self.force_spin.setEnabled(False)
            self.force_status_label.setText("力控: 当前手型未启用")
            self.force_status_label.setToolTip("当前手型未在接入向导中声明力控能力，或驱动适配器未提供电流/力矩遥测。")

        top_bar.addSpacing(12)

        save_btn = QPushButton("保存当前姿态为手势")
        save_btn.setStyleSheet(
            "QPushButton { background-color: #28a745; color: white; border: none; "
            "border-radius: 4px; padding: 8px 16px; }"
            "QPushButton:hover { background-color: #218838; }"
        )
        save_btn.clicked.connect(self._save_current_gesture)
        top_bar.addWidget(save_btn)

        layout.addLayout(top_bar)

        # ── 内置手势区域 ──
        builtin_group = QGroupBox("内置手势")
        builtin_group.setStyleSheet("""
            QGroupBox {
                font-size: 14px; font-weight: bold;
                border: 1px solid #ddd; border-radius: 8px;
                margin-top: 8px; padding-top: 16px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }
        """)
        self.builtin_grid = QGridLayout(builtin_group)
        self.builtin_grid.setSpacing(12)

        for i, (name, angles) in enumerate(self._builtin_gestures.items()):
            btn = GestureButton(name)
            btn.clicked.connect(lambda checked, a=angles: self._apply_gesture(a))
            self.builtin_grid.addWidget(btn, i // 4, i % 4)

        layout.addWidget(builtin_group)

        # ── 自定义手势区域 ──
        custom_group = QGroupBox("自定义手势")
        custom_group.setStyleSheet("""
            QGroupBox {
                font-size: 14px; font-weight: bold;
                border: 1px solid #ddd; border-radius: 8px;
                margin-top: 8px; padding-top: 16px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }
        """)
        self.custom_grid = QGridLayout(custom_group)
        self.custom_grid.setSpacing(12)

        self.no_custom_label = QLabel("暂无自定义手势\n点击上方按钮保存当前姿态")
        self.no_custom_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.no_custom_label.setStyleSheet("color: #999; padding: 20px;")
        self.custom_grid.addWidget(self.no_custom_label, 0, 0, 1, 4)

        layout.addWidget(custom_group)
        layout.addStretch()

        # 加载已保存的自定义手势
        self._load_custom_gestures()

    def _apply_gesture(self, angles: list[float]):
        if self._force_enabled and self.profile.force_control_supported:
            # 力控模式：保存目标，启动持续执行循环
            self._gesture_target = list(angles)
            self._force_ctrl.reset()
            self._gesture_timer.start()
            self.force_status_label.setText("力控: 执行中...")
            self.force_status_label.setStyleSheet("color: #dc3545; font-size: 12px;")
            print(f"[力控-手势] 开始力控执行，目标角度已设定")
        else:
            self._gesture_timer.stop()
            self._gesture_target = None
            self.hand_state.set_all_angles(angles)

    def _on_force_toggled(self, checked: bool):
        if checked and not self.profile.force_control_supported:
            self.force_cb.setChecked(False)
            return
        self._force_enabled = checked
        if checked:
            scale = self.force_spin.value()
            self._force_ctrl.config.torque_scale = scale
            self._force_ctrl.reset()
            self.force_status_label.setText(f"力控: ON ({scale:.1f}x)")
            self.force_status_label.setStyleSheet("color: #28a745; font-size: 12px;")
        else:
            self._gesture_timer.stop()
            self._gesture_target = None
            self.force_status_label.setText("力控: OFF")
            self.force_status_label.setStyleSheet("color: #999; font-size: 12px;")

    def _get_telemetry(self) -> dict:
        main_w = self.window()
        if hasattr(main_w, 'get_page'):
            cp = main_w.get_page("连接管理")
            if cp and hasattr(cp, 'get_motor_service'):
                svc = cp.get_motor_service()
                if svc and svc.is_connected:
                    return svc.get_latest_telemetry()
        return {}

    def _get_motor_service(self):
        main_w = self.window()
        if hasattr(main_w, 'get_page'):
            cp = main_w.get_page("连接管理")
            if cp and hasattr(cp, 'get_motor_service'):
                return cp.get_motor_service()
        return None

    def _force_ctrl_tick(self):
        """力控循环：持续向目标角度驱动，受力反馈约束"""
        if self._gesture_target is None:
            self._gesture_timer.stop()
            return

        self._force_ctrl.config.torque_scale = self.force_spin.value()
        telemetry = self._get_telemetry()

        # 力控调整（apply 内部从 telemetry 读取电机实际位置）
        adjusted = self._force_ctrl.apply(self._gesture_target, telemetry)

        # 仅在角度实际变化时更新 hand_state，避免信号风暴
        prev = self.hand_state.angles
        if not np.allclose(prev, adjusted, atol=0.1):
            self.hand_state.set_all_angles(adjusted)

        # 收敛检查：所有电机到位且无力控干预 → 停止循环
        if self._force_ctrl.is_converged(self._gesture_target, telemetry):
            self._gesture_timer.stop()
            self.force_status_label.setText("力控: 到位")
            self.force_status_label.setStyleSheet("color: #28a745; font-size: 12px;")
            print("[力控-手势] 所有关节已到位，停止力控循环")
            return

        # 更新状态显示
        active = self._force_ctrl.get_active_count()
        ramping = self._force_ctrl.get_ramping_count()
        if active > 0:
            self.force_status_label.setText(f"力控: {active}个电机受限")
            self.force_status_label.setStyleSheet("color: #dc3545; font-size: 12px;")
        elif ramping > 0:
            self.force_status_label.setText(f"力控: {ramping}个电机归位中")
            self.force_status_label.setStyleSheet("color: #e67e22; font-size: 12px;")
        else:
            self.force_status_label.setText("力控: 执行中")
            self.force_status_label.setStyleSheet("color: #28a745; font-size: 12px;")

    def _save_current_gesture(self):
        name, ok = QInputDialog.getText(self, "保存手势", "请输入手势名称:")
        if not ok or not name.strip():
            return

        name = name.strip()
        angles = self.hand_state.angles.tolist()
        self._custom_gestures[name] = angles

        os.makedirs(self.gestures_dir, exist_ok=True)
        filepath = os.path.join(self.gestures_dir, f"{name}.json")
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump({"name": name, "angles": angles}, f, indent=2, ensure_ascii=False)

        self._refresh_custom_grid()

    def _load_custom_gestures(self):
        if not os.path.exists(self.gestures_dir):
            return
        for filename in os.listdir(self.gestures_dir):
            if filename.endswith(".json"):
                filepath = os.path.join(self.gestures_dir, filename)
                try:
                    with open(filepath, "r", encoding="utf-8") as f:
                        data = json.load(f)
                    if len(data.get("angles", [])) == self.profile.num_joints:
                        self._custom_gestures[data["name"]] = data["angles"]
                except Exception:
                    pass
        if self._custom_gestures:
            self._refresh_custom_grid()

    def _refresh_custom_grid(self):
        while self.custom_grid.count():
            item = self.custom_grid.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        if not self._custom_gestures:
            self.no_custom_label = QLabel("暂无自定义手势")
            self.no_custom_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.no_custom_label.setStyleSheet("color: #999; padding: 20px;")
            self.custom_grid.addWidget(self.no_custom_label, 0, 0, 1, 4)
            return

        for i, (name, angles) in enumerate(self._custom_gestures.items()):
            btn = GestureButton(name, is_custom=True)
            btn.clicked.connect(lambda checked, a=angles: self._apply_gesture(a))
            self.custom_grid.addWidget(btn, i // 4, i % 4)

    def cleanup(self):
        self._gesture_timer.stop()
