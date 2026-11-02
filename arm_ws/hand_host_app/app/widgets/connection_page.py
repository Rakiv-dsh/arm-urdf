"""连接管理页面 — 串口扫描、连接/断开、电机使能、状态网格、开机归位"""

import json
import os

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QComboBox, QPushButton, QFormLayout,
    QGridLayout, QTextEdit,
)
from PySide6.QtCore import Qt, Slot, QTimer

from app.models.hand_state import HandState
from app.hands.runtime import create_motor_service
from app.services.tactile_service import TactileSensorService

try:
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


class MotorStatusIndicator(QLabel):
    """单个电机状态指示灯"""

    def __init__(self, motor_id: int, parent=None):
        super().__init__(parent)
        self.motor_id = motor_id
        self.setText(f"M{motor_id}")
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedSize(56, 40)
        self.set_online(False)

    def set_online(self, online: bool):
        if online:
            self.setStyleSheet(
                "background-color: #28a745; color: white; border-radius: 6px; "
                "font-size: 12px; font-weight: bold;"
            )
        else:
            self.setStyleSheet(
                "background-color: #ccc; color: #666; border-radius: 6px; "
                "font-size: 12px; font-weight: bold;"
            )


class ConnectionPage(QWidget):
    def __init__(self, hand_state: HandState, parent=None):
        super().__init__(parent)
        self.hand_state = hand_state
        self.profile = hand_state.profile
        self.motor_service = self._create_motor_service()
        if hasattr(self.motor_service, "hand_id"):
            self.motor_service.hand_id = self.profile.hand_id
        self.tactile_service = TactileSensorService(hand_state, self)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)
        motor_config = getattr(self.profile, "motor_adapter_config", {}) or {}
        tactile_config = getattr(self.profile, "tactile_adapter_config", {}) or {}
        motor_endpoint_label = str(
            motor_config.get("endpoint_label")
            or motor_config.get("port_label")
            or "通信端点"
        )
        tactile_endpoint_label = str(
            tactile_config.get("endpoint_label")
            or tactile_config.get("port_label")
            or "通信端点"
        )

        # ── 通信连接区域 ──
        serial_group = QGroupBox("电机通信")
        serial_layout = QFormLayout(serial_group)

        # 通信端点选择：串口、CAN 通道、EtherCAT 网卡名或 SDK 连接字符串均可。
        port_row = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(200)
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.setFixedWidth(60)
        self.refresh_btn.clicked.connect(self._scan_ports)
        port_row.addWidget(self.port_combo)
        port_row.addWidget(self.refresh_btn)
        port_row.addStretch()
        # bridge 适配器不需要用户选端口，直接显示默认地址，避免“无可用端口”的误导
        bridge_endpoint_label = "桥地址"
        bridge_addr = str(motor_config.get("bridge_endpoint", "127.0.0.1:5599"))
        if self._motor_requires_endpoint():
            serial_layout.addRow(f"{motor_endpoint_label}:", port_row)
        else:
            addr_label = QLabel(bridge_addr + "（默认）")
            addr_label.setStyleSheet("color: #666;")
            serial_layout.addRow(f"{bridge_endpoint_label}:", addr_label)

        # 波特率/速率：非串口适配器可以忽略该值。
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["0", "9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"])
        self.baud_combo.setCurrentText(str(self.profile.default_baudrate()))
        if self._motor_requires_endpoint():
            serial_layout.addRow("速率/波特率:", self.baud_combo)

        self.protocol_combo = QComboBox()
        for protocol in self.profile.protocols:
            self.protocol_combo.addItem(protocol, protocol)
        protocol_idx = self.protocol_combo.findData(self.profile.default_protocol)
        if protocol_idx >= 0:
            self.protocol_combo.setCurrentIndex(protocol_idx)
        self.protocol_combo.currentIndexChanged.connect(self._on_protocol_changed)
        serial_layout.addRow("通信协议:", self.protocol_combo)

        # 连接 / 使能按钮
        btn_row = QHBoxLayout()
        self.connect_btn = QPushButton("连接")
        self.connect_btn.setFixedWidth(100)
        self.connect_btn.setStyleSheet(
            "QPushButton { background-color: #0078d4; color: white; border: none; "
            "border-radius: 4px; padding: 8px 16px; font-size: 14px; }"
            "QPushButton:hover { background-color: #106ebe; }"
            "QPushButton:disabled { background-color: #ccc; }"
        )
        self.connect_btn.clicked.connect(self._toggle_connection)

        self.enable_btn = QPushButton("使能全部电机")
        self.enable_btn.setFixedWidth(130)
        self.enable_btn.setEnabled(False)
        self.enable_btn.setStyleSheet(
            "QPushButton { background-color: #28a745; color: white; border: none; "
            "border-radius: 4px; padding: 8px 16px; font-size: 14px; }"
            "QPushButton:hover { background-color: #218838; }"
            "QPushButton:disabled { background-color: #ccc; }"
        )
        self.enable_btn.clicked.connect(self._enable_all)

        self.disable_btn = QPushButton("失能全部电机")
        self.disable_btn.setFixedWidth(130)
        self.disable_btn.setEnabled(False)
        self.disable_btn.setStyleSheet(
            "QPushButton { background-color: #dc3545; color: white; border: none; "
            "border-radius: 4px; padding: 8px 16px; font-size: 14px; }"
            "QPushButton:hover { background-color: #c82333; }"
            "QPushButton:disabled { background-color: #ccc; }"
        )
        self.disable_btn.clicked.connect(self._disable_all)

        self.home_btn = QPushButton("使能并归位")
        self.home_btn.setFixedWidth(130)
        self.home_btn.setEnabled(False)
        self.home_btn.setToolTip(
            "使能所有电机 → 等待编码器初始化 → 缓慢归位到标定的默认位置（0°）"
        )
        self.home_btn.setStyleSheet(
            "QPushButton { background-color: #6f42c1; color: white; border: none; "
            "border-radius: 4px; padding: 8px 16px; font-size: 14px; }"
            "QPushButton:hover { background-color: #5a32a3; }"
            "QPushButton:disabled { background-color: #ccc; }"
        )
        self.home_btn.clicked.connect(self._enable_and_home)

        self.status_indicator = QLabel("● 未连接")
        self.status_indicator.setStyleSheet("color: #999; font-size: 14px;")

        btn_row.addWidget(self.connect_btn)
        btn_row.addWidget(self.enable_btn)
        btn_row.addWidget(self.disable_btn)
        btn_row.addWidget(self.home_btn)
        btn_row.addWidget(self.status_indicator)
        btn_row.addStretch()
        serial_layout.addRow("", btn_row)

        layout.addWidget(serial_group)

        # ── 电机在线状态网格 ──
        status_group = QGroupBox("电机在线状态")
        status_grid = QGridLayout(status_group)
        status_grid.setSpacing(8)

        self.motor_indicators: dict[int, MotorStatusIndicator] = {}
        for col, finger in enumerate(self.profile.finger_configs):
            lbl = QLabel(finger.display_name)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet("font-weight: bold; font-size: 13px;")
            status_grid.addWidget(lbl, 0, col)

        for col, finger in enumerate(self.profile.finger_configs):
            for row, joint in enumerate(finger.joints):
                motor_id = joint.motor_id
                indicator = MotorStatusIndicator(motor_id)
                self.motor_indicators[motor_id] = indicator
                status_grid.addWidget(indicator, row + 1, col, alignment=Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(status_group)

        # ── 触觉通信区域 ──
        tactile_group = QGroupBox("触觉通信")
        tactile_layout = QFormLayout(tactile_group)

        # 触觉通信端点选择：可复用串口，也可填写 CAN/网关/SDK 连接字符串。
        tactile_port_row = QHBoxLayout()
        self.tactile_port_combo = QComboBox()
        self.tactile_port_combo.setEditable(True)
        self.tactile_port_combo.setMinimumWidth(200)
        self.tactile_refresh_btn = QPushButton("刷新")
        self.tactile_refresh_btn.setFixedWidth(60)
        self.tactile_refresh_btn.clicked.connect(self._scan_tactile_ports)
        tactile_port_row.addWidget(self.tactile_port_combo)
        tactile_port_row.addWidget(self.tactile_refresh_btn)
        tactile_port_row.addStretch()
        tactile_layout.addRow(f"{tactile_endpoint_label}:", tactile_port_row)

        # 非串口触觉适配器可以忽略该值。
        self.tactile_baud_combo = QComboBox()
        self.tactile_baud_combo.addItems(["0", "9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"])
        self.tactile_baud_combo.setCurrentText(str(self.profile.tactile_default_baudrate))
        self.tactile_baud_combo.setMinimumWidth(120)
        tactile_layout.addRow("速率/波特率:", self.tactile_baud_combo)

        # 触觉传感器按钮行
        tactile_btn_row = QHBoxLayout()
        self.tactile_connect_btn = QPushButton("连接")
        self.tactile_connect_btn.setFixedWidth(100)
        self.tactile_connect_btn.setStyleSheet(
            "QPushButton { background-color: #17a2b8; color: white; border: none; "
            "border-radius: 4px; padding: 8px 16px; font-size: 14px; }"
            "QPushButton:hover { background-color: #138496; }"
            "QPushButton:disabled { background-color: #ccc; }"
        )
        self.tactile_connect_btn.clicked.connect(self._toggle_tactile_connection)

        self.tactile_sim_btn = QPushButton("模拟数据")
        self.tactile_sim_btn.setFixedWidth(100)
        self.tactile_sim_btn.setStyleSheet(
            "QPushButton { background-color: #ffc107; color: #333; border: none; "
            "border-radius: 4px; padding: 8px 16px; font-size: 14px; }"
            "QPushButton:hover { background-color: #e0a800; }"
        )
        self.tactile_sim_btn.clicked.connect(self._toggle_tactile_simulation)

        self.tactile_status_indicator = QLabel("● 未连接")
        self.tactile_status_indicator.setStyleSheet("color: #999; font-size: 14px;")

        tactile_btn_row.addWidget(self.tactile_connect_btn)
        tactile_btn_row.addWidget(self.tactile_sim_btn)
        tactile_btn_row.addWidget(self.tactile_status_indicator)
        tactile_btn_row.addStretch()
        tactile_layout.addRow("", tactile_btn_row)

        # 没有触觉的手型/机械臂不显示这一块
        tactile_kind = str(getattr(self.profile, "tactile_kind", "none") or "none")
        tactile_group.setVisible(tactile_kind not in ("none", "", "None"))
        layout.addWidget(tactile_group)

        # ── 日志区域 ──
        log_group = QGroupBox("通信日志")
        log_layout = QVBoxLayout(log_group)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        self.log_text.setStyleSheet("font-family: Consolas, monospace; font-size: 12px;")
        log_layout.addWidget(self.log_text)
        layout.addWidget(log_group)

        layout.addStretch()

        # 信号连接
        self.motor_service.connection_changed.connect(self._on_connection_changed)
        self.motor_service.motor_status_updated.connect(self._on_status_updated)
        self.motor_service.motor_telemetry_updated.connect(self._on_telemetry_updated)
        self.motor_service.error_occurred.connect(self._on_error)

        self.tactile_service.connection_changed.connect(self._on_tactile_connection_changed)
        self.tactile_service.status_updated.connect(self._log)
        self.tactile_service.error_occurred.connect(self._on_error)

        # 初始扫描端口
        self._scan_ports()
        self._scan_tactile_ports()

    def _create_motor_service(self):
        return create_motor_service(self.profile, self)

    @staticmethod
    def _requires_endpoint(config: dict) -> bool:
        if "requires_endpoint" in config:
            return bool(config["requires_endpoint"])
        if "requires_port" in config:
            return bool(config["requires_port"])
        return True

    def _motor_requires_endpoint(self) -> bool:
        return self._requires_endpoint(getattr(self.profile, "motor_adapter_config", {}) or {})

    def _tactile_requires_endpoint(self) -> bool:
        return self._requires_endpoint(getattr(self.profile, "tactile_adapter_config", {}) or {})

    @staticmethod
    def _selected_port(combo: QComboBox) -> str:
        text = combo.currentText().strip()
        if text and "无可用端口" not in text:
            return text.split(" - ", 1)[0].strip()
        port = combo.currentData()
        if port:
            return str(port).strip()
        return ""

    def _on_protocol_changed(self):
        protocol = self.protocol_combo.currentData()
        baud = self.profile.default_baudrate(protocol)
        if self.baud_combo.findText(str(baud)) < 0:
            self.baud_combo.addItem(str(baud))
        self.baud_combo.setCurrentText(str(baud))

    def _scan_ports(self):
        previous = self._selected_port(self.port_combo)
        self.port_combo.clear()
        matched_index = -1
        if HAS_SERIAL:
            ports = serial.tools.list_ports.comports()
            for p in ports:
                self.port_combo.addItem(f"{p.device} - {p.description}", p.device)
                if previous and previous == p.device:
                    matched_index = self.port_combo.count() - 1
        if self.port_combo.count() == 0:
            self.port_combo.addItem("无可用端口")
            if previous:
                self.port_combo.setEditText(previous)
        elif matched_index >= 0:
            self.port_combo.setCurrentIndex(matched_index)

    def _toggle_connection(self):
        if self.motor_service.is_connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self._selected_port(self.port_combo)
        if not port and self._motor_requires_endpoint():
            self._log("未填写有效通信端点")
            return
        baudrate = int(self.baud_combo.currentText())
        protocol = self.protocol_combo.currentData()
        endpoint = port or "<adapter-default>"
        self._log(f"正在连接 {self.profile.hand_id} {endpoint} @ {baudrate} ({protocol})...")

        if self.motor_service.connect(port, baudrate, protocol=protocol):
            self._log(f"已连接 {endpoint}")
        else:
            self._log(f"无法连接 {endpoint}")

    def _disconnect(self):
        self.motor_service.disconnect()
        self._log("已断开连接")

    def _enable_all(self):
        self.motor_service.enable_all(True)
        self._log("已发送使能全部电机指令")

    def _disable_all(self):
        self.motor_service.enable_all(False)
        self._log("已发送失能全部电机指令")

    def _enable_and_home(self):
        """使能所有电机并归位到标定的默认位置"""
        self._log("使能并归位: 发送使能指令...")
        self.motor_service.enable_all(True)
        self.home_btn.setEnabled(False)

        # 等待编码器初始化 (约2秒), 然后发送归位指令
        QTimer.singleShot(2500, self._do_homing)

    def _do_homing(self):
        """编码器初始化完成后, 缓慢归位到标定的默认角度"""
        # 从标定文件读取默认角度
        target_angles: dict[int, float] = {}
        cal_path = str(self.profile.calibration_path)

        if os.path.isfile(cal_path):
            try:
                with open(cal_path, "r", encoding="utf-8") as f:
                    cal_data = json.load(f)
                for jd in cal_data.get("joints", []):
                    mid = jd.get("motor_id")
                    if mid:
                        target_angles[mid] = jd.get("default_angle", 0.0)
                self._log(f"归位: 从标定文件加载 {len(target_angles)} 个目标角度")
            except Exception as e:
                self._log(f"归位: 无法加载标定文件: {e}")

        # 未标定的电机默认归位到 0°
        for mid in sorted(self.profile.motor_id_to_joint_index):
            if mid not in target_angles:
                target_angles[mid] = 0.0

        self._log(f"归位: 以低速移动到默认位置...")
        self.motor_service.home_all_motors(target_angles, speed=0.3)
        self._log("归位指令已发送, 电机正在移动到默认位置")
        self.home_btn.setEnabled(True)

    @Slot(bool)
    def _on_connection_changed(self, connected: bool):
        if connected:
            self.connect_btn.setText("断开")
            self.status_indicator.setText("● 已连接")
            self.status_indicator.setStyleSheet("color: #28a745; font-size: 14px;")
            self.port_combo.setEnabled(False)
            self.baud_combo.setEnabled(False)
            self.protocol_combo.setEnabled(False)
            self.enable_btn.setEnabled(True)
            self.disable_btn.setEnabled(True)
            self.home_btn.setEnabled(True)
            main_win = self.window()
            if hasattr(main_win, "set_connection_status"):
                main_win.set_connection_status(True, self.port_combo.currentText())
        else:
            self.connect_btn.setText("连接")
            self.status_indicator.setText("● 未连接")
            self.status_indicator.setStyleSheet("color: #999; font-size: 14px;")
            self.port_combo.setEnabled(True)
            self.baud_combo.setEnabled(True)
            self.protocol_combo.setEnabled(True)
            self.enable_btn.setEnabled(False)
            self.disable_btn.setEnabled(False)
            self.home_btn.setEnabled(False)
            for ind in self.motor_indicators.values():
                ind.set_online(False)
            main_win = self.window()
            if hasattr(main_win, "set_connection_status"):
                main_win.set_connection_status(False)

    @Slot(object)
    def _on_status_updated(self, status: dict):
        for motor_id, info in status.items():
            ind = self.motor_indicators.get(motor_id)
            if ind:
                ind.set_online(info.get('online', False))

    @Slot(object)
    def _on_telemetry_updated(self, telemetry: dict):
        """电机遥测数据到达，更新 HandState"""
        self.hand_state.set_all_motor_telemetry(telemetry)

    @Slot(str)
    def _on_error(self, msg: str):
        self._log(f"[错误] {msg}")

    def _log(self, message: str):
        self.log_text.append(message)
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    # ── 触觉传感器操作 ──

    def _scan_tactile_ports(self):
        previous = self._selected_port(self.tactile_port_combo)
        self.tactile_port_combo.clear()
        matched_index = -1
        if HAS_SERIAL:
            ports = serial.tools.list_ports.comports()
            for p in ports:
                self.tactile_port_combo.addItem(f"{p.device} - {p.description}", p.device)
                if previous and previous == p.device:
                    matched_index = self.tactile_port_combo.count() - 1
        if self.tactile_port_combo.count() == 0:
            self.tactile_port_combo.addItem("无可用端口")
            if previous:
                self.tactile_port_combo.setEditText(previous)
        elif matched_index >= 0:
            self.tactile_port_combo.setCurrentIndex(matched_index)

    def _toggle_tactile_connection(self):
        if self.tactile_service.is_connected and not self.tactile_service.is_simulating:
            self.tactile_service.disconnect()
            self._log("触觉传感器已断开")
        else:
            port = self._selected_port(self.tactile_port_combo)
            if not port and self._tactile_requires_endpoint():
                self._log("未填写有效的触觉通信端点")
                return
            baudrate = int(self.tactile_baud_combo.currentText())
            endpoint = port or "<adapter-default>"
            self._log(f"正在连接触觉通信 {endpoint} @ {baudrate}...")
            if self.tactile_service.connect(port, baudrate):
                self._log(f"触觉通信已连接 {endpoint} @ {baudrate}")
            else:
                self._log(f"触觉通信连接失败 {endpoint}")

    def _toggle_tactile_simulation(self):
        if self.tactile_service.is_simulating:
            self.tactile_service.stop_simulation()
            self.tactile_sim_btn.setText("模拟数据")
            self._log("触觉传感器模拟已停止")
        else:
            self.tactile_service.start_simulation()
            self.tactile_sim_btn.setText("停止模拟")
            self._log("触觉传感器模拟数据已启动")

    @Slot(bool)
    def _on_tactile_connection_changed(self, connected: bool):
        if connected:
            if self.tactile_service.is_simulating:
                self.tactile_connect_btn.setEnabled(True)
                self.tactile_port_combo.setEnabled(True)
                self.tactile_baud_combo.setEnabled(True)
                self.tactile_status_indicator.setText("● 模拟中")
                self.tactile_status_indicator.setStyleSheet("color: #ffc107; font-size: 14px;")
            else:
                self.tactile_connect_btn.setText("断开")
                self.tactile_port_combo.setEnabled(False)
                self.tactile_baud_combo.setEnabled(False)
                self.tactile_status_indicator.setText("● 已连接")
                self.tactile_status_indicator.setStyleSheet("color: #17a2b8; font-size: 14px;")
        else:
            self.tactile_connect_btn.setText("连接")
            self.tactile_connect_btn.setEnabled(True)
            self.tactile_port_combo.setEnabled(True)
            self.tactile_baud_combo.setEnabled(True)
            self.tactile_status_indicator.setText("● 未连接")
            self.tactile_status_indicator.setStyleSheet("color: #999; font-size: 14px;")

    def get_motor_service(self):
        return self.motor_service

    def get_tactile_service(self) -> TactileSensorService:
        return self.tactile_service

    def cleanup(self):
        self.motor_service.cleanup()
        self.tactile_service.cleanup()
