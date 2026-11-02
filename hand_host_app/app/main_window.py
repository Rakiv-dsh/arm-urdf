"""主窗口 — 左侧sidebar导航 + 右侧QStackedWidget页面切换"""

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QPushButton, QStackedWidget, QLabel, QButtonGroup,
    QSizePolicy, QComboBox,
)
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QFont

from app.models.hand_state import HandState
from app.config import load_config, save_config
from app.hands import EMPTY_HAND_ID, get_profiles, reload_profiles, set_active_profile


class NavButton(QPushButton):
    """侧边栏导航按钮"""

    def __init__(self, text: str, parent=None):
        super().__init__(text, parent)
        self.setCheckable(True)
        self.setFixedHeight(48)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setStyleSheet("""
            QPushButton {
                border: none;
                border-radius: 6px;
                padding: 8px 16px;
                font-size: 14px;
                text-align: left;
                background-color: transparent;
                color: #333333;
            }
            QPushButton:hover {
                background-color: #e8e8e8;
            }
            QPushButton:checked {
                background-color: #0078d4;
                color: white;
                font-weight: bold;
            }
        """)


class MainWindow(QMainWindow):
    """灵巧手上位机主窗口"""

    # 只保留和机械臂相关的页面（手部专用的触觉可视化 / 仿真遥操作已移除）
    PAGE_NAMES = [
        "连接管理",
        "关节控制",
        "电机标定",
        "数据记录",
        "动作序列",
        "机型管理",
    ]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("机械臂上位机控制系统")
        self.resize(1280, 800)
        self.setMinimumSize(960, 600)

        # 深色系统主题下，未显式设色的控件文字会变成白色，落在白底上看不见；
        # 这里统一给一个深色文字，保证标签/分组框/下拉框可读。
        self.setStyleSheet("""
            QLabel, QGroupBox, QCheckBox, QRadioButton, QComboBox, QSpinBox, QDoubleSpinBox {
                color: #333333;
            }
            QComboBox QAbstractItemView { color: #333333; background: white; }
        """)

        self._config = load_config()
        active_hand = self._config.get("active_hand", EMPTY_HAND_ID)
        self.active_profile = set_active_profile(active_hand)
        self._switching_profile = False

        # 全局共享的手部状态
        self.hand_state = HandState(self, self.active_profile)

        # ── 中心区域 ──
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # ── 侧边栏 ──
        sidebar = QWidget()
        sidebar.setFixedWidth(180)
        sidebar.setStyleSheet("background-color: #f5f5f5; border-right: 1px solid #ddd;")
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(8, 16, 8, 16)
        sidebar_layout.setSpacing(4)

        # 标题
        title_label = QLabel("机械臂控制")
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setStyleSheet("color: #0078d4; padding: 8px 0 16px 0; border: none;")
        sidebar_layout.addWidget(title_label)

        hand_label = QLabel("当前机型")
        hand_label.setStyleSheet("color: #666; font-size: 12px; padding: 2px 8px; border: none;")
        sidebar_layout.addWidget(hand_label)

        self.hand_combo = QComboBox()
        self.hand_combo.setFixedHeight(32)
        self.hand_combo.setStyleSheet("""
            QComboBox {
                border: 1px solid #ccc;
                border-radius: 4px;
                padding: 4px 8px;
                background: white;
                font-size: 13px;
            }
        """)
        self._populate_hand_combo(self.active_profile.hand_id)
        sidebar_layout.addWidget(self.hand_combo)
        sidebar_layout.addSpacing(8)

        # 导航按钮组
        self.nav_group = QButtonGroup(self)
        self.nav_group.setExclusive(True)
        self.nav_buttons: list[NavButton] = []

        for i, name in enumerate(self.PAGE_NAMES):
            btn = NavButton(name)
            self.nav_buttons.append(btn)
            self.nav_group.addButton(btn, i)
            sidebar_layout.addWidget(btn)

        sidebar_layout.addStretch()

        # 底部状态
        self.status_label = QLabel("未连接")
        self.status_label.setStyleSheet(
            "color: #999; font-size: 12px; padding: 8px; border: none;"
        )
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sidebar_layout.addWidget(self.status_label)

        main_layout.addWidget(sidebar)

        # ── 页面栈 ──
        self.stack = QStackedWidget()
        self.stack.setStyleSheet("background-color: white;")
        main_layout.addWidget(self.stack, 1)

        # 创建页面
        self._pages: dict[str, QWidget] = {}
        self._create_pages()

        # 导航信号
        self.nav_group.idClicked.connect(self._on_nav_clicked)
        self.hand_combo.currentIndexChanged.connect(self._on_hand_changed)
        self.nav_buttons[0].setChecked(True)
        self.stack.setCurrentIndex(0)
        if self.active_profile.hand_id == EMPTY_HAND_ID:
            self.nav_buttons[-1].setChecked(True)
            self.stack.setCurrentIndex(len(self.PAGE_NAMES) - 1)
            self.set_connection_status(False, "请先添加手型")

    def _populate_hand_combo(self, target_hand_id: str | None = None):
        self.hand_combo.clear()
        profiles = get_profiles()
        if not profiles:
            self.hand_combo.addItem("未添加手型", EMPTY_HAND_ID)
            self.hand_combo.setCurrentIndex(0)
            self.hand_combo.setEnabled(False)
            return
        self.hand_combo.setEnabled(True)
        for hand_id, profile in profiles.items():
            self.hand_combo.addItem(profile.display_name, hand_id)
        idx = self.hand_combo.findData(target_hand_id)
        if idx < 0:
            idx = 0
        self.hand_combo.setCurrentIndex(idx)

    def _create_pages(self):
        """创建所有功能页面"""
        from app.widgets.connection_page import ConnectionPage
        from app.widgets.joint_control_page import JointControlPage
        from app.widgets.calibration_page import CalibrationPage
        from app.widgets.recorder_page import RecorderPage
        from app.widgets.gesture_page import GesturePage
        from app.widgets.hand_manager_page import HandManagerPage

        page_classes = [
            ("连接管理", ConnectionPage),
            ("关节控制", JointControlPage),
            ("电机标定", CalibrationPage),
            ("数据记录", RecorderPage),
            ("动作序列", GesturePage),
            ("机型管理", HandManagerPage),
        ]

        for name, cls in page_classes:
            page = cls(self.hand_state, self)
            self._pages[name] = page
            self.stack.addWidget(page)

    def _recreate_pages(self, current_page: int | None = None):
        if current_page is None:
            current_page = self.stack.currentIndex()
        for page in self._pages.values():
            if hasattr(page, "cleanup"):
                page.cleanup()
        while self.stack.count():
            widget = self.stack.widget(0)
            self.stack.removeWidget(widget)
            widget.deleteLater()
        self._pages.clear()
        self._create_pages()
        self.stack.setCurrentIndex(min(max(current_page, 0), self.stack.count() - 1))

    def _on_nav_clicked(self, index: int):
        self.stack.setCurrentIndex(index)

    def _on_hand_changed(self, index: int):
        if self._switching_profile:
            return
        hand_id = self.hand_combo.itemData(index)
        if not hand_id or hand_id == EMPTY_HAND_ID or hand_id == self.active_profile.hand_id:
            return

        self._switching_profile = True
        current_page = self.stack.currentIndex()
        try:
            self.active_profile = set_active_profile(hand_id)
            self.hand_state.set_profile(self.active_profile)
            self._config["active_hand"] = hand_id
            save_config(self._config)

            self._recreate_pages(current_page)
            self.set_connection_status(False)
        finally:
            self._switching_profile = False

    def refresh_hand_profiles(self, preferred_hand_id: str | None = None):
        profiles = reload_profiles(preferred_hand_id or self.active_profile.hand_id)
        target = preferred_hand_id or self.active_profile.hand_id
        if not profiles:
            target = EMPTY_HAND_ID
        elif target not in profiles:
            target = next(iter(profiles))

        self._switching_profile = True
        try:
            self._populate_hand_combo(target)
        finally:
            self._switching_profile = False

        if target == EMPTY_HAND_ID:
            self.active_profile = set_active_profile(target)
            self.hand_state.set_profile(self.active_profile)
            self._config["active_hand"] = target
            save_config(self._config)
            self._recreate_pages(len(self.PAGE_NAMES) - 1)
            self.set_connection_status(False, "请先添加手型")
        elif target != self.active_profile.hand_id:
            idx = self.hand_combo.findData(target)
            self._on_hand_changed(idx)

    def set_connection_status(self, connected: bool, text: str = ""):
        if connected:
            self.status_label.setText(f"{self.active_profile.hand_id} 已连接 {text}")
            self.status_label.setStyleSheet(
                "color: #28a745; font-size: 12px; padding: 8px; border: none;"
            )
        else:
            if self.active_profile.hand_id == EMPTY_HAND_ID:
                self.status_label.setText(text or "未添加手型")
            else:
                self.status_label.setText(f"{self.active_profile.hand_id} 未连接")
            self.status_label.setStyleSheet(
                "color: #999; font-size: 12px; padding: 8px; border: none;"
            )

    def get_page(self, name: str) -> QWidget | None:
        return self._pages.get(name)

    def closeEvent(self, event):
        """关闭窗口时清理资源"""
        for page in self._pages.values():
            if hasattr(page, "cleanup"):
                page.cleanup()
        super().closeEvent(event)
