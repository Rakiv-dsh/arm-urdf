"""通用关节控制页面。"""

from __future__ import annotations

from PySide6.QtCore import Qt, Slot
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSlider,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from app.models.hand_state import HandState
from app.widgets.tactile_hand_3d_widget import TactileHand3DWidget


class JointSliderWidget(QWidget):
    """单个关节的滑块控制组件。"""

    def __init__(self, joint_index: int, joint_info, hand_state: HandState, parent=None):
        super().__init__(parent)
        self.joint_index = joint_index
        self.joint_info = joint_info
        self.hand_state = hand_state
        self._updating = False

        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(8)

        name_label = QLabel(f"M{joint_info.motor_id} {joint_info.display_name}")
        name_label.setFixedWidth(120)
        name_label.setStyleSheet("font-size: 13px;")
        layout.addWidget(name_label)

        self.min_label = QLabel(f"{joint_info.min_angle:.0f}")
        self.min_label.setFixedWidth(44)
        self.min_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.min_label.setStyleSheet("color: #888; font-size: 11px;")
        layout.addWidget(self.min_label)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(int(joint_info.min_angle * 10))
        self.slider.setMaximum(int(joint_info.max_angle * 10))
        self.slider.setValue(int(joint_info.default_angle * 10))
        self.slider.setMinimumWidth(180)
        self.slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 6px; background: #ddd; border-radius: 3px;
            }
            QSlider::handle:horizontal {
                width: 16px; height: 16px; margin: -5px 0;
                background: #0078d4; border-radius: 8px;
            }
            QSlider::sub-page:horizontal {
                background: #0078d4; border-radius: 3px;
            }
        """)
        layout.addWidget(self.slider, 1)

        self.max_label = QLabel(f"{joint_info.max_angle:.0f}")
        self.max_label.setFixedWidth(44)
        self.max_label.setStyleSheet("color: #888; font-size: 11px;")
        layout.addWidget(self.max_label)

        self.spinbox = QDoubleSpinBox()
        self.spinbox.setRange(joint_info.min_angle, joint_info.max_angle)
        self.spinbox.setValue(joint_info.default_angle)
        self.spinbox.setDecimals(1)
        self.spinbox.setFixedWidth(96)
        layout.addWidget(self.spinbox)

        self.slider.valueChanged.connect(self._on_slider_changed)
        self.spinbox.valueChanged.connect(self._on_spinbox_changed)
        hand_state.single_joint_changed.connect(self._on_state_changed)
        hand_state.joint_angles_changed.connect(self._on_all_angles_changed)

    def _on_slider_changed(self, value: int):
        if self._updating:
            return
        self._updating = True
        angle = value / 10.0
        self.spinbox.setValue(angle)
        self.hand_state.set_angle(self.joint_index, angle)
        self._updating = False

    def _on_spinbox_changed(self, value: float):
        if self._updating:
            return
        self._updating = True
        self.slider.setValue(int(value * 10))
        self.hand_state.set_angle(self.joint_index, value)
        self._updating = False

    @Slot(int, float)
    def _on_state_changed(self, index: int, angle: float):
        if index != self.joint_index or self._updating:
            return
        self._updating = True
        self.slider.setValue(int(angle * 10))
        self.spinbox.setValue(angle)
        self._updating = False

    @Slot(list)
    def _on_all_angles_changed(self, angles: list):
        if self._updating or self.joint_index >= len(angles):
            return
        angle = angles[self.joint_index]
        self._updating = True
        self.slider.setValue(int(angle * 10))
        self.spinbox.setValue(angle)
        self._updating = False

    def update_range(self, min_angle: float, max_angle: float):
        self._updating = True
        self.slider.setMinimum(int(min_angle * 10))
        self.slider.setMaximum(int(max_angle * 10))
        self.spinbox.setRange(min_angle, max_angle)
        self.min_label.setText(f"{min_angle:.0f}")
        self.max_label.setText(f"{max_angle:.0f}")
        cur = self.spinbox.value()
        clamped = max(min_angle, min(max_angle, cur))
        self.slider.setValue(int(clamped * 10))
        self.spinbox.setValue(clamped)
        self._updating = False


class JointControlPage(QWidget):
    def __init__(self, hand_state: HandState, parent=None):
        super().__init__(parent)
        self.hand_state = hand_state

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(8)

        top_bar = QHBoxLayout()
        title = QLabel("关节控制面板")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #333;")
        top_bar.addWidget(title)
        top_bar.addStretch()

        reset_btn = QPushButton("全部归零")
        reset_btn.setFixedWidth(100)
        reset_btn.setStyleSheet(
            "QPushButton { background-color: #6c757d; color: white; border: none; "
            "border-radius: 4px; padding: 6px 12px; }"
            "QPushButton:hover { background-color: #5a6268; }"
        )
        reset_btn.clicked.connect(self._reset_all)
        top_bar.addWidget(reset_btn)
        layout.addLayout(top_bar)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(10)

        self.joint_widgets: list[JointSliderWidget] = []
        joint_index = 0
        for finger_cfg in self.hand_state.profile.finger_configs:
            group = QGroupBox(finger_cfg.display_name)
            group.setStyleSheet("""
                QGroupBox {
                    font-size: 14px; font-weight: bold;
                    border: 1px solid #ddd; border-radius: 8px;
                    margin-top: 8px; padding-top: 16px;
                }
                QGroupBox::title {
                    subcontrol-origin: margin; left: 12px; padding: 0 6px;
                }
            """)
            group_layout = QVBoxLayout(group)
            group_layout.setSpacing(2)
            for joint_info in finger_cfg.joints:
                widget = JointSliderWidget(joint_index, joint_info, hand_state)
                self.joint_widgets.append(widget)
                group_layout.addWidget(widget)
                joint_index += 1
            scroll_layout.addWidget(group)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        left_layout.addWidget(scroll)

        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        self.hand_widget = TactileHand3DWidget(hand_state)
        right_layout.addWidget(self.hand_widget, 1)

        zero_btn = QPushButton("zero")
        zero_btn.setFixedWidth(80)
        zero_btn.setStyleSheet(
            "QPushButton { background-color: white; color: #0078d4; border: 2px solid #0078d4; "
            "border-radius: 4px; padding: 4px 12px; font-size: 13px; font-weight: bold; }"
            "QPushButton:hover { background-color: #e8f0fe; }"
        )
        zero_btn.clicked.connect(self._reset_all)
        zero_layout = QHBoxLayout()
        zero_layout.addStretch()
        zero_layout.addWidget(zero_btn)
        right_layout.addLayout(zero_layout)

        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 1)
        layout.addWidget(splitter, 1)

        hand_state.single_joint_changed.connect(self._on_joint_to_motor)
        hand_state.joint_angles_changed.connect(self._on_all_joints_to_motor)

    def _get_motor_service(self):
        try:
            conn_page = self.window().get_page("连接管理")
            if conn_page:
                return conn_page.get_motor_service()
        except Exception:
            pass
        return None

    @Slot(int, float)
    def _on_joint_to_motor(self, index: int, angle: float):
        svc = self._get_motor_service()
        if svc and svc.is_connected:
            motor_id = self.hand_state.profile.joint_index_to_motor_id.get(index)
            if motor_id is not None:
                svc.set_motor_angle(motor_id, angle)

    @Slot(list)
    def _on_all_joints_to_motor(self, angles: list):
        svc = self._get_motor_service()
        if svc and svc.is_connected:
            svc.set_all_joint_angles(angles)

    def _reset_all(self):
        defaults = [
            joint.default_angle
            for finger in self.hand_state.profile.finger_configs
            for joint in finger.joints
        ]
        self.hand_state.set_all_angles(defaults)

    def cleanup(self):
        if hasattr(self, "hand_widget") and hasattr(self.hand_widget, "cleanup"):
            self.hand_widget.cleanup()
