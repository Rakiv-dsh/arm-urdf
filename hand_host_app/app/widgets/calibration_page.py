"""电机标定页面 — 读取电机实时角度，设定各关节运动范围和初始位置，保存/加载标定数据

新增功能:
  - 零点标定: 将当前位置设为电机绝对零点 (CONFIG_UPDATE_FIN_ZERO, index=3)
  - 自动寻限位: 以低速驱动电机, 监控电流突增, 自动确定机械限位
"""

import json
import os
import time

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QDoubleSpinBox, QSpinBox, QGridLayout,
    QScrollArea, QFileDialog, QMessageBox, QProgressDialog,
)
from PySide6.QtCore import Qt, QTimer

from app.models.hand_state import HandState


class CalibrationPage(QWidget):
    """电机标定页面"""

    def __init__(self, hand_state: HandState, parent=None):
        super().__init__(parent)
        self.hand_state = hand_state
        self.profile = hand_state.profile
        self._profile_joints = [j for f in self.profile.finger_configs for j in f.joints]
        self._calibration_file = str(self.profile.calibration_path)

        # 每个关节的 UI 组件引用
        self._rows: list[dict] = []   # [{cur_label, min_spin, max_spin, default_spin, ...}]

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(12)

        # ── 顶部控制栏 ──
        top_bar = QHBoxLayout()
        title = QLabel("电机标定")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #333;")
        top_bar.addWidget(title)
        top_bar.addStretch()

        self.save_btn = QPushButton("保存标定")
        self.save_btn.setStyleSheet(
            "QPushButton { background-color: #28a745; color: white; border: none; "
            "border-radius: 4px; padding: 6px 16px; }"
            "QPushButton:hover { background-color: #218838; }"
        )
        self.save_btn.clicked.connect(self._save_calibration)
        top_bar.addWidget(self.save_btn)

        self.load_btn = QPushButton("加载标定")
        self.load_btn.setStyleSheet(
            "QPushButton { background-color: #0078d4; color: white; border: none; "
            "border-radius: 4px; padding: 6px 16px; }"
            "QPushButton:hover { background-color: #106ebe; }"
        )
        self.load_btn.clicked.connect(self._load_calibration)
        top_bar.addWidget(self.load_btn)

        self.apply_btn = QPushButton("应用标定")
        self.apply_btn.setStyleSheet(
            "QPushButton { background-color: #fd7e14; color: white; border: none; "
            "border-radius: 4px; padding: 6px 16px; }"
            "QPushButton:hover { background-color: #e8690b; }"
        )
        self.apply_btn.clicked.connect(self._apply_calibration)
        top_bar.addWidget(self.apply_btn)

        # 零点标定按钮
        self.zero_cal_btn = QPushButton("一键设零点")
        self.zero_cal_btn.setToolTip(
            "将所有在线电机的当前位置设为绝对零点，保存到电机Flash，掉电不丢失。\n"
            "操作前请手动将机械臂摆到零位姿态！"
        )
        self.zero_cal_btn.setStyleSheet(
            "QPushButton { background-color: #6f42c1; color: white; border: none; "
            "border-radius: 4px; padding: 6px 16px; }"
            "QPushButton:hover { background-color: #5a32a3; }"
        )
        self.zero_cal_btn.clicked.connect(self._zero_calibrate_all)
        top_bar.addWidget(self.zero_cal_btn)

        # 电流阈值设置
        top_bar.addSpacing(16)
        threshold_label = QLabel("电流阈值(mA):")
        threshold_label.setStyleSheet("font-size: 12px; color: #555;")
        top_bar.addWidget(threshold_label)

        self._threshold_spin = QSpinBox()
        self._threshold_spin.setRange(10, 2000)
        self._threshold_spin.setValue(self._LIMIT_CURRENT_THRESHOLD)
        self._threshold_spin.setSuffix(" mA")
        self._threshold_spin.setToolTip("自动寻限时的Q轴电流触发阈值，超过此值即判定到达机械限位")
        self._threshold_spin.setFixedWidth(100)
        self._threshold_spin.valueChanged.connect(self._on_threshold_changed)
        top_bar.addWidget(self._threshold_spin)

        layout.addLayout(top_bar)

        # ── 提示信息 ──
        hint = QLabel(
            "标定流程：\n"
            "1. 连接并使能电机 → 手动将机械臂摆到零位姿态 → 点击「一键设零点」将当前位置设为绝对零点（保存到电机Flash，掉电不丢失）\n"
            "2. 手动将关节移到极限位置 → 点击「设为最小值」/「设为最大值」记录机械限位（或使用「自动寻限」）\n"
            "3.「设为默认值」将当前角度设为开机归位位置 → 「保存标定」保存到文件 → 「应用标定」使运行时生效"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #666; font-size: 12px; padding: 4px 0;")
        layout.addWidget(hint)

        # ── 可滚动的标定表格 ──
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(12)

        joint_idx = 0
        for finger in self.profile.finger_configs:
            group = QGroupBox(f"{finger.display_name} ({finger.name})")
            group.setStyleSheet(
                "QGroupBox { font-weight: bold; font-size: 14px; "
                "border: 1px solid #ddd; border-radius: 6px; margin-top: 8px; padding-top: 16px; }"
                "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }"
            )
            grid = QGridLayout(group)
            grid.setSpacing(6)

            # 表头
            headers = ["关节", "电机ID", "当前角度(°)", "", "最小值(°)", "", "最大值(°)", "", "默认值(°)", ""]
            for col, h in enumerate(headers):
                lbl = QLabel(h)
                lbl.setStyleSheet("font-size: 11px; color: #888; font-weight: normal;")
                lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
                grid.addWidget(lbl, 0, col)

            for row_i, joint in enumerate(finger.joints, start=1):
                row_data = self._create_joint_row(grid, row_i, joint, joint_idx)
                self._rows.append(row_data)
                joint_idx += 1

            scroll_layout.addWidget(group)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll, 1)

        # ── 定时刷新当前角度 ──
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(200)  # 5Hz
        self._poll_timer.timeout.connect(self._refresh_current_angles)
        self._poll_timer.start()

        # 启动时尝试自动加载默认标定文件
        if os.path.isfile(self._calibration_file):
            self._load_from_file(self._calibration_file)

    def _create_joint_row(self, grid: QGridLayout, row: int, joint, joint_idx: int) -> dict:
        """为一个关节创建一行标定 UI"""
        col = 0

        # 关节名
        name_lbl = QLabel(joint.display_name)
        name_lbl.setStyleSheet("font-weight: normal;")
        grid.addWidget(name_lbl, row, col)
        col += 1

        # 电机ID
        id_lbl = QLabel(f"M{joint.motor_id}")
        id_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        id_lbl.setStyleSheet("font-weight: normal;")
        grid.addWidget(id_lbl, row, col)
        col += 1

        # 当前角度 (只读标签)
        cur_label = QLabel("--")
        cur_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        cur_label.setFixedWidth(70)
        cur_label.setStyleSheet(
            "background-color: #f0f0f0; border: 1px solid #ddd; border-radius: 4px; "
            "padding: 2px; font-weight: normal;"
        )
        grid.addWidget(cur_label, row, col)
        col += 1

        # 设为最小值按钮
        set_min_btn = QPushButton("设为最小值")
        set_min_btn.setFixedWidth(80)
        set_min_btn.setStyleSheet(
            "QPushButton { background-color: #17a2b8; color: white; border: none; "
            "border-radius: 3px; padding: 3px 6px; font-size: 11px; font-weight: normal; }"
            "QPushButton:hover { background-color: #138496; }"
        )
        grid.addWidget(set_min_btn, row, col)
        col += 1

        # 最小值 SpinBox
        min_spin = QDoubleSpinBox()
        min_spin.setRange(-360.0, 360.0)
        min_spin.setDecimals(1)
        min_spin.setValue(joint.min_angle)
        min_spin.setFixedWidth(80)
        min_spin.setStyleSheet("font-weight: normal;")
        grid.addWidget(min_spin, row, col)
        col += 1

        # 设为最大值按钮
        set_max_btn = QPushButton("设为最大值")
        set_max_btn.setFixedWidth(80)
        set_max_btn.setStyleSheet(
            "QPushButton { background-color: #dc3545; color: white; border: none; "
            "border-radius: 3px; padding: 3px 6px; font-size: 11px; font-weight: normal; }"
            "QPushButton:hover { background-color: #c82333; }"
        )
        grid.addWidget(set_max_btn, row, col)
        col += 1

        # 最大值 SpinBox
        max_spin = QDoubleSpinBox()
        max_spin.setRange(-360.0, 360.0)
        max_spin.setDecimals(1)
        max_spin.setValue(joint.max_angle)
        max_spin.setFixedWidth(80)
        max_spin.setStyleSheet("font-weight: normal;")
        grid.addWidget(max_spin, row, col)
        col += 1

        # 设为默认值按钮
        set_default_btn = QPushButton("设为默认值")
        set_default_btn.setFixedWidth(80)
        set_default_btn.setStyleSheet(
            "QPushButton { background-color: #6c757d; color: white; border: none; "
            "border-radius: 3px; padding: 3px 6px; font-size: 11px; font-weight: normal; }"
            "QPushButton:hover { background-color: #5a6268; }"
        )
        grid.addWidget(set_default_btn, row, col)
        col += 1

        # 默认值 SpinBox
        default_spin = QDoubleSpinBox()
        default_spin.setRange(-360.0, 360.0)
        default_spin.setDecimals(1)
        default_spin.setValue(joint.default_angle)
        default_spin.setFixedWidth(80)
        default_spin.setStyleSheet("font-weight: normal;")
        grid.addWidget(default_spin, row, col)
        col += 1

        # 归零按钮
        zero_btn = QPushButton("归零")
        zero_btn.setFixedWidth(50)
        zero_btn.setStyleSheet(
            "QPushButton { background-color: #ffc107; color: #333; border: none; "
            "border-radius: 3px; padding: 3px 6px; font-size: 11px; font-weight: normal; }"
            "QPushButton:hover { background-color: #e0a800; }"
        )
        grid.addWidget(zero_btn, row, col)
        col += 1

        # 自动寻限按钮
        find_limit_btn = QPushButton("自动寻限")
        find_limit_btn.setFixedWidth(70)
        find_limit_btn.setToolTip("以低速驱动电机向两个方向转动，检测电流突增确定机械限位")
        find_limit_btn.setStyleSheet(
            "QPushButton { background-color: #6f42c1; color: white; border: none; "
            "border-radius: 3px; padding: 3px 6px; font-size: 11px; font-weight: normal; }"
            "QPushButton:hover { background-color: #5a32a3; }"
        )
        grid.addWidget(find_limit_btn, row, col)

        # 绑定按钮
        row_ref = {
            "joint_idx": joint_idx,
            "motor_id": joint.motor_id,
            "cur_label": cur_label,
            "min_spin": min_spin,
            "max_spin": max_spin,
            "default_spin": default_spin,
        }
        set_min_btn.clicked.connect(lambda _, r=row_ref: self._set_min_from_current(r))
        set_max_btn.clicked.connect(lambda _, r=row_ref: self._set_max_from_current(r))
        set_default_btn.clicked.connect(lambda _, r=row_ref: self._set_default_from_current(r))
        zero_btn.clicked.connect(lambda _, r=row_ref: self._reset_row(r))
        find_limit_btn.clicked.connect(lambda _, r=row_ref: self._auto_find_limits(r))

        return row_ref

    # ── 读取当前角度 ──

    def _get_motor_service(self):
        main_win = self.window()
        if hasattr(main_win, 'get_page'):
            conn_page = main_win.get_page("连接管理")
            if conn_page and hasattr(conn_page, 'get_motor_service'):
                return conn_page.get_motor_service()
        return None

    def _refresh_current_angles(self):
        """定时从电机遥测数据刷新当前角度"""
        svc = self._get_motor_service()
        if svc is None or not svc.is_connected:
            return

        telemetry = svc.get_latest_telemetry()
        for row in self._rows:
            mid = row["motor_id"]
            data = telemetry.get(mid)
            if data and data.get("online"):
                angle = data.get("position", 0.0)
                row["cur_label"].setText(f"{angle:.1f}")
                row["cur_label"].setStyleSheet(
                    "background-color: #d4edda; border: 1px solid #c3e6cb; border-radius: 4px; "
                    "padding: 2px; font-weight: normal;"
                )
            else:
                row["cur_label"].setText("离线")
                row["cur_label"].setStyleSheet(
                    "background-color: #f0f0f0; border: 1px solid #ddd; border-radius: 4px; "
                    "padding: 2px; color: #999; font-weight: normal;"
                )

    def _get_current_angle(self, row: dict) -> float | None:
        """获取指定关节的电机当前角度"""
        svc = self._get_motor_service()
        if svc is None or not svc.is_connected:
            QMessageBox.warning(self, "未连接", "请先在「连接管理」页面连接电机")
            return None
        telemetry = svc.get_latest_telemetry()
        data = telemetry.get(row["motor_id"])
        if not data or not data.get("online"):
            QMessageBox.warning(self, "电机离线", f"电机 M{row['motor_id']} 未在线")
            return None
        return data.get("position", 0.0)

    # ── 按钮动作 ──

    def _set_min_from_current(self, row: dict):
        angle = self._get_current_angle(row)
        if angle is not None:
            row["min_spin"].setValue(round(angle, 1))

    def _set_max_from_current(self, row: dict):
        angle = self._get_current_angle(row)
        if angle is not None:
            row["max_spin"].setValue(round(angle, 1))

    def _set_default_from_current(self, row: dict):
        angle = self._get_current_angle(row)
        if angle is not None:
            row["default_spin"].setValue(round(angle, 1))

    def _reset_row(self, row: dict):
        """归零：将该关节的 min/max/default 恢复为出厂默认"""
        idx = row["joint_idx"]
        joint = self._profile_joints[idx]
        row["min_spin"].setValue(joint.min_angle)
        row["max_spin"].setValue(joint.max_angle)
        row["default_spin"].setValue(joint.default_angle)

    def _on_threshold_changed(self, value: int):
        """更新自动寻限的电流阈值"""
        self._LIMIT_CURRENT_THRESHOLD = value

    # ── 零点标定 ──

    def _zero_calibrate_all(self):
        """将所有在线电机的当前位置设为绝对零点"""
        svc = self._get_motor_service()
        if svc is None or not svc.is_connected:
            QMessageBox.warning(self, "未连接", "请先在「连接管理」页面连接并使能电机")
            return

        reply = QMessageBox.question(
            self, "零点标定确认",
            "即将把所有在线电机的当前位置设为绝对零点（0°）。\n\n"
            "⚠️ 请确保机械臂已摆到零位姿态！\n"
            "零点将保存在电机 Flash 中，掉电不丢失。\n\n"
            "确定继续？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return

        telemetry = svc.get_latest_telemetry()
        count = 0
        for row in self._rows:
            mid = row["motor_id"]
            data = telemetry.get(mid)
            if data and data.get("online"):
                svc.set_zero_point(mid)
                count += 1
                time.sleep(0.02)  # 帧间延迟

        if count > 0:
            QMessageBox.information(
                self, "零点标定完成",
                f"已将 {count} 个在线电机的当前位置设为零点。\n"
                "零点已保存到电机 Flash，掉电不丢失。\n\n"
                "建议同时点击「保存标定」将 min/max/default 保存到文件。"
            )
        else:
            QMessageBox.warning(self, "无在线电机", "没有检测到任何在线电机")

    # ── 自动寻限位 ──

    _LIMIT_SPEED = 0.05        # 寻限速度 (5%)
    _LIMIT_CURRENT_THRESHOLD = 100  # Q轴电流阈值 (mA), 超过认为到达限位
    _LIMIT_STEP_DEG = 2.0      # 每步角度增量 (度)
    _LIMIT_MAX_RANGE = 200.0   # 最大搜索范围 (度)
    _LIMIT_SAFETY_MARGIN = 3.0 # 安全余量 (度)
    _LIMIT_STEP_DELAY_MS = 150 # 每步间隔 (ms)
    _LIMIT_STALL_THRESHOLD = 0.5   # 位置停滞判定阈值 (度), 作为辅助判据
    _LIMIT_STALL_COUNT = 3         # 连续停滞次数即判定到达限位

    def _auto_find_limits(self, row: dict):
        """自动寻找指定电机的机械限位

        以低速向正/负方向缓慢驱动电机，同时监控:
        1. Q轴电流 - 电流超过阈值说明堵转 (主判据)
        2. 位置停滞 - 实际位置不再变化 (辅助判据)
        任一条件满足即判定到达限位。
        """
        svc = self._get_motor_service()
        if svc is None or not svc.is_connected:
            QMessageBox.warning(self, "未连接", "请先连接并使能电机")
            return

        mid = row["motor_id"]
        telemetry = svc.get_latest_telemetry()
        data = telemetry.get(mid)
        if not data or not data.get("online"):
            QMessageBox.warning(self, "电机离线", f"电机 M{mid} 未在线")
            return

        reply = QMessageBox.question(
            self, "自动寻限确认",
            f"即将以低速驱动电机 M{mid} 向两个方向搜索机械限位。\n\n"
            f"⚠️ 搜索过程中请勿触碰机械臂！\n"
            f"电流阈值: {self._LIMIT_CURRENT_THRESHOLD} mA\n"
            f"位置停滞: {self._LIMIT_STALL_THRESHOLD}° × {self._LIMIT_STALL_COUNT} 步\n"
            f"安全余量: {self._LIMIT_SAFETY_MARGIN}°\n\n"
            "确定继续？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return

        start_angle = data.get("position", 0.0)
        found_min = None
        found_max = None
        max_steps = int(self._LIMIT_MAX_RANGE / self._LIMIT_STEP_DEG)
        limit_config = {
            "speed": self._LIMIT_SPEED,
            "current_threshold": self._LIMIT_CURRENT_THRESHOLD,
            "step_deg": self._LIMIT_STEP_DEG,
            "max_range_deg": self._LIMIT_MAX_RANGE,
            "safety_margin_deg": self._LIMIT_SAFETY_MARGIN,
            "step_delay_ms": self._LIMIT_STEP_DELAY_MS,
            "stall_threshold_deg": self._LIMIT_STALL_THRESHOLD,
            "stall_count": self._LIMIT_STALL_COUNT,
        }

        delegated = None
        find_limits = getattr(svc, "find_motor_limits", None)
        if callable(find_limits):
            delegated = find_limits(mid, start_angle, limit_config)
        if isinstance(delegated, dict):
            found_min = delegated.get("min_angle", delegated.get("min"))
            found_max = delegated.get("max_angle", delegated.get("max"))
            self._show_limit_result(row, mid, found_min, found_max)
            return

        # 创建进度对话框
        progress = QProgressDialog(f"正在搜索 M{mid} 限位...", "取消", 0, 100, self)
        progress.setWindowModality(Qt.WindowModality.WindowModal)
        progress.setMinimumDuration(0)
        progress.setValue(0)

        def _send_limit_move(target: float):
            immediate = getattr(svc, "set_motor_angle_immediate", None)
            if callable(immediate):
                immediate(mid, target, speed=self._LIMIT_SPEED)
            else:
                svc.set_motor_angle(mid, target, speed=self._LIMIT_SPEED)

        def _search_direction(direction: int, label: str, progress_offset: int):
            """向指定方向搜索限位, direction: +1 或 -1"""
            target = start_angle
            prev_pos = start_angle
            stall_count = 0
            for step in range(max_steps):
                if progress.wasCanceled():
                    return None
                target += direction * self._LIMIT_STEP_DEG
                _send_limit_move(target)
                time.sleep(self._LIMIT_STEP_DELAY_MS / 1000.0)

                tel = svc.get_latest_telemetry().get(mid, {})
                actual_pos = tel.get("position", target)
                cur = abs(tel.get("current", 0.0))
                delta = abs(actual_pos - prev_pos)

                print(f"[{label}] Step {step}: target={target:.1f}°, "
                      f"actual={actual_pos:.1f}°, current={cur:.0f}mA, "
                      f"delta={delta:.2f}°, stall={stall_count}/{self._LIMIT_STALL_COUNT}")
                progress.setValue(progress_offset + int(step / max_steps * 50))

                # 判据1: 电流超过阈值 (堵转)
                if cur > self._LIMIT_CURRENT_THRESHOLD:
                    limit = actual_pos - direction * self._LIMIT_SAFETY_MARGIN
                    print(f"[{label}] 电流触发限位: current={cur:.0f}mA > {self._LIMIT_CURRENT_THRESHOLD}mA, "
                          f"actual={actual_pos:.1f}° → limit={limit:.1f}°")
                    return limit

                # 判据2: 位置停滞
                if delta < self._LIMIT_STALL_THRESHOLD:
                    stall_count += 1
                    if stall_count >= self._LIMIT_STALL_COUNT:
                        limit = actual_pos - direction * self._LIMIT_SAFETY_MARGIN
                        print(f"[{label}] 位置停滞触发限位: actual={actual_pos:.1f}° → limit={limit:.1f}°")
                        return limit
                else:
                    stall_count = 0
                prev_pos = actual_pos
            return None  # 未找到限位

        # ── 向正方向搜索 max ──
        progress.setLabelText(f"M{mid}: 向正方向搜索最大限位...")
        found_max = _search_direction(+1, "+", 0)

        # 回到起点，等待电机实际到位后再搜索负方向
        _send_limit_move(start_angle)
        for _ in range(50):  # 最多等待 5 秒
            time.sleep(0.1)
            tel = svc.get_latest_telemetry().get(mid, {})
            actual = tel.get("position", start_angle)
            if abs(actual - start_angle) < 1.0:
                break
        time.sleep(0.3)  # 额外稳定时间

        # ── 向负方向搜索 min ──
        if not progress.wasCanceled():
            progress.setLabelText(f"M{mid}: 向负方向搜索最小限位...")
            found_min = _search_direction(-1, "-", 50)

        # 回到起点
        _send_limit_move(start_angle)
        progress.setValue(100)
        progress.close()

        self._show_limit_result(row, mid, found_min, found_max)

    def _show_limit_result(self, row: dict, mid: int, found_min, found_max):
        # ── 更新结果 ──
        result_parts = []
        if found_max is not None:
            value = float(found_max)
            row["max_spin"].setValue(round(value, 1))
            result_parts.append(f"最大限位: {value:.1f}°")
        else:
            result_parts.append("最大限位: 未触发（搜索范围 {:.0f}° 内未检测到限位）".format(
                self._LIMIT_MAX_RANGE))

        if found_min is not None:
            value = float(found_min)
            row["min_spin"].setValue(round(value, 1))
            result_parts.append(f"最小限位: {value:.1f}°")
        else:
            result_parts.append("最小限位: 未触发（搜索范围 {:.0f}° 内未检测到限位）".format(
                self._LIMIT_MAX_RANGE))

        QMessageBox.information(
            self, f"M{mid} 寻限结果",
            "\n".join(result_parts) + f"\n\n安全余量: ±{self._LIMIT_SAFETY_MARGIN}°"
        )

    # ── 保存 / 加载 / 应用 ──

    def _collect_calibration_data(self) -> dict:
        """将 UI 上的标定值收集为 dict"""
        data = {"joints": []}
        idx = 0
        for finger in self.profile.finger_configs:
            for joint in finger.joints:
                row = self._rows[idx]
                data["joints"].append({
                    "name": joint.name,
                    "motor_id": joint.motor_id,
                    "min_angle": row["min_spin"].value(),
                    "max_angle": row["max_spin"].value(),
                    "default_angle": row["default_spin"].value(),
                })
                idx += 1
        return data

    def _save_calibration(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "保存标定文件", self._calibration_file,
            "JSON 文件 (*.json);;所有文件 (*)"
        )
        if not path:
            return
        data = self._collect_calibration_data()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        QMessageBox.information(self, "保存成功", f"标定数据已保存到\n{path}")

    def _load_calibration(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "加载标定文件", os.path.dirname(self._calibration_file),
            "JSON 文件 (*.json);;所有文件 (*)"
        )
        if not path:
            return
        self._load_from_file(path)

    def _load_from_file(self, path: str):
        """从文件加载标定数据到 UI"""
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            QMessageBox.warning(self, "加载失败", f"无法读取标定文件:\n{e}")
            return

        joints = data.get("joints", [])
        if len(joints) != self.profile.num_joints:
            QMessageBox.warning(
                self, "数据不匹配",
                f"标定文件包含 {len(joints)} 个关节，但系统需要 {self.profile.num_joints} 个"
            )
            return

        for i, jd in enumerate(joints):
            row = self._rows[i]
            row["min_spin"].setValue(jd.get("min_angle", -200.0))
            row["max_spin"].setValue(jd.get("max_angle", 200.0))
            row["default_spin"].setValue(jd.get("default_angle", 0.0))

    def _apply_calibration(self):
        """将当前 UI 上的标定值应用到运行时关节配置"""
        idx = 0
        for finger in self.profile.finger_configs:
            for joint in finger.joints:
                row = self._rows[idx]
                joint.min_angle = row["min_spin"].value()
                joint.max_angle = row["max_spin"].value()
                joint.default_angle = row["default_spin"].value()
                idx += 1

        # 自动保存到默认标定文件
        data = self._collect_calibration_data()
        os.makedirs(os.path.dirname(self._calibration_file), exist_ok=True)
        with open(self._calibration_file, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

        # 即时同步关节控制页面的滑块范围
        self._sync_joint_sliders()

        QMessageBox.information(
            self, "应用成功",
            "标定参数已应用到当前系统，并已自动保存。\n"
            "关节控制页面的滑块范围已即时更新。"
        )

    def _sync_joint_sliders(self):
        """将标定后的 min/max 同步到关节控制页面的滑块"""
        main_win = self.window()
        if not hasattr(main_win, 'get_page'):
            return
        joint_page = main_win.get_page("关节控制")
        if joint_page is None or not hasattr(joint_page, 'joint_widgets'):
            return
        idx = 0
        for finger in self.profile.finger_configs:
            for joint in finger.joints:
                if idx < len(joint_page.joint_widgets):
                    joint_page.joint_widgets[idx].update_range(
                        joint.min_angle, joint.max_angle
                    )
                idx += 1

    def cleanup(self):
        self._poll_timer.stop()
