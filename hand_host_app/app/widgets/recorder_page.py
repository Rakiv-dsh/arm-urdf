"""数据记录与回放页面 — CSV 记录任意手型的关节轨迹、回放控制。"""

import csv
import os
import time
from pathlib import Path

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QFileDialog, QGroupBox, QSlider,
    QProgressBar, QDoubleSpinBox, QTableWidget,
    QTableWidgetItem, QHeaderView,
)
from PySide6.QtCore import Qt, QTimer, Slot

from app.models.hand_state import HandState


class RecorderPage(QWidget):
    def __init__(self, hand_state: HandState, parent=None):
        super().__init__(parent)
        self.hand_state = hand_state
        self.profile = hand_state.profile

        self._recording = False
        self._playing = False
        self._record_buffer: list[dict] = []
        self._play_index = 0
        self._play_data: list[dict] = []
        self._record_start_time = 0.0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)

        # ── 录制区域 ──
        rec_group = QGroupBox("数据录制")
        rec_layout = QVBoxLayout(rec_group)

        rec_bar = QHBoxLayout()
        self.rec_btn = QPushButton("开始录制")
        self.rec_btn.setStyleSheet(
            "QPushButton { background-color: #dc3545; color: white; border: none; "
            "border-radius: 4px; padding: 8px 20px; font-size: 14px; }"
            "QPushButton:hover { background-color: #c82333; }"
        )
        self.rec_btn.clicked.connect(self._toggle_recording)
        rec_bar.addWidget(self.rec_btn)

        self.rec_status = QLabel("未录制")
        self.rec_status.setStyleSheet("font-size: 13px; color: #666;")
        rec_bar.addWidget(self.rec_status)
        rec_bar.addStretch()

        self.save_btn = QPushButton("保存录制")
        self.save_btn.setEnabled(False)
        self.save_btn.clicked.connect(self._save_recording)
        rec_bar.addWidget(self.save_btn)

        rec_layout.addLayout(rec_bar)
        layout.addWidget(rec_group)

        # ── 回放区域 ──
        play_group = QGroupBox("数据回放")
        play_layout = QVBoxLayout(play_group)

        load_bar = QHBoxLayout()
        self.load_btn = QPushButton("加载文件")
        self.load_btn.clicked.connect(self._load_file)
        load_bar.addWidget(self.load_btn)

        self.file_label = QLabel("未加载文件")
        self.file_label.setStyleSheet("color: #666;")
        load_bar.addWidget(self.file_label)
        load_bar.addStretch()

        load_bar.addWidget(QLabel("速率:"))
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.1, 5.0)
        self.speed_spin.setValue(1.0)
        self.speed_spin.setSingleStep(0.1)
        self.speed_spin.setSuffix("x")
        self.speed_spin.setFixedWidth(80)
        load_bar.addWidget(self.speed_spin)

        play_layout.addLayout(load_bar)

        # 进度条
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        play_layout.addWidget(self.progress)

        # 播放控制
        ctrl_bar = QHBoxLayout()
        self.play_btn = QPushButton("播放")
        self.play_btn.setEnabled(False)
        self.play_btn.clicked.connect(self._toggle_playback)
        ctrl_bar.addWidget(self.play_btn)

        self.stop_btn = QPushButton("停止")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self._stop_playback)
        ctrl_bar.addWidget(self.stop_btn)

        self.play_time_label = QLabel("0.0s / 0.0s")
        ctrl_bar.addWidget(self.play_time_label)
        ctrl_bar.addStretch()

        play_layout.addLayout(ctrl_bar)
        layout.addWidget(play_group)

        # ── 数据预览 ──
        preview_group = QGroupBox("数据预览（最近10帧）")
        preview_layout = QVBoxLayout(preview_group)

        self._preview_joint_indices = list(range(min(3, self.profile.num_joints)))
        self.table = QTableWidget(0, len(self._preview_joint_indices) + 2)
        self.table.setHorizontalHeaderLabels(
            ["时间(s)"]
            + [self.profile.joint_names[i] for i in self._preview_joint_indices]
            + ["..."]
        )
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.setMaximumHeight(200)
        preview_layout.addWidget(self.table)

        layout.addWidget(preview_group)
        layout.addStretch()

        # 定时器
        self._rec_timer = QTimer(self)
        self._rec_timer.timeout.connect(self._record_frame)

        self._play_timer = QTimer(self)
        self._play_timer.timeout.connect(self._play_frame)

    def _toggle_recording(self):
        if self._recording:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self):
        self._recording = True
        self._record_buffer = []
        self._record_start_time = time.time()
        self._rec_timer.start(50)  # 20Hz
        self.rec_btn.setText("停止录制")
        self.rec_status.setText("录制中...")
        self.rec_status.setStyleSheet("color: #dc3545; font-size: 13px;")
        self.save_btn.setEnabled(False)

    def _stop_recording(self):
        self._recording = False
        self._rec_timer.stop()
        self.rec_btn.setText("开始录制")
        n = len(self._record_buffer)
        duration = self._record_buffer[-1]["timestamp"] if n > 0 else 0
        self.rec_status.setText(f"已录制 {n} 帧 ({duration:.1f}s)")
        self.rec_status.setStyleSheet("color: #28a745; font-size: 13px;")
        self.save_btn.setEnabled(n > 0)

    def _record_frame(self):
        t = time.time() - self._record_start_time
        angles = self.hand_state.angles.tolist()
        # 采集电机遥测数据 (电流、速度、温度等)
        telemetry = {}
        main_win = self.window()
        if hasattr(main_win, 'get_page'):
            conn_page = main_win.get_page("连接管理")
            if conn_page and hasattr(conn_page, 'get_motor_service'):
                svc = conn_page.get_motor_service()
                if svc and svc.is_connected:
                    telemetry = svc.get_latest_telemetry()
        self._record_buffer.append({
            "timestamp": t,
            "angles": angles,
            "telemetry": telemetry,
        })
        self.rec_status.setText(f"录制中... {t:.1f}s ({len(self._record_buffer)} 帧)")
        self._update_preview(self._record_buffer[-10:])

    def _save_recording(self):
        if not self._record_buffer:
            return

        path, _ = QFileDialog.getSaveFileName(
            self, "保存录制数据", "recording.csv",
            "CSV 文件 (*.csv);;JSON 文件 (*.json)"
        )
        if not path:
            return

        if path.endswith(".csv"):
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                # 表头: timestamp + 角度列 + 遥测列(每个电机: 电流/速度/温度/电压)
                tel_headers = []
                motor_ids = sorted(self.profile.motor_id_to_joint_index)
                for mid in motor_ids:
                    tel_headers += [
                        f"M{mid}_current_mA",
                        f"M{mid}_velocity_dps",
                        f"M{mid}_temperature_C",
                        f"M{mid}_bus_voltage_V",
                    ]
                writer.writerow(["timestamp"] + self.profile.joint_names + tel_headers)
                for frame in self._record_buffer:
                    tel = frame.get("telemetry", {})
                    tel_values = []
                    for mid in motor_ids:
                        md = tel.get(mid, {})
                        tel_values += [
                            f"{md.get('current', 0.0):.1f}",
                            f"{md.get('velocity', 0.0):.2f}",
                            f"{md.get('temperature', 0)}",
                            f"{md.get('bus_voltage', 0.0):.3f}",
                        ]
                    writer.writerow(
                        [f"{frame['timestamp']:.4f}"]
                        + [f"{a:.2f}" for a in frame["angles"]]
                        + tel_values
                    )
        else:
            import json
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self._record_buffer, f, indent=2)

        self.rec_status.setText(f"已保存至 {Path(path).name}")

    def _load_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "加载录制文件", "",
            "CSV 文件 (*.csv);;JSON 文件 (*.json);;所有文件 (*)"
        )
        if not path:
            return

        self._play_data = []
        try:
            if path.endswith(".csv"):
                with open(path, "r", encoding="utf-8") as f:
                    reader = csv.reader(f)
                    next(reader)  # skip header
                    for row in reader:
                        self._play_data.append({
                            "timestamp": float(row[0]),
                            "angles": [float(x) for x in row[1:self.profile.num_joints + 1]],
                        })
            else:
                import json
                with open(path, "r", encoding="utf-8") as f:
                    self._play_data = json.load(f)

            n = len(self._play_data)
            duration = self._play_data[-1]["timestamp"] if n > 0 else 0
            self.file_label.setText(f"{Path(path).name} ({n} 帧, {duration:.1f}s)")
            self.play_btn.setEnabled(n > 0)
            self.stop_btn.setEnabled(n > 0)
            self.progress.setRange(0, max(1, n - 1))
            self.progress.setValue(0)
        except Exception as e:
            self.file_label.setText(f"加载失败: {e}")

    def _toggle_playback(self):
        if self._playing:
            self._pause_playback()
        else:
            self._start_playback()

    def _start_playback(self):
        if not self._play_data:
            return
        self._playing = True
        speed = self.speed_spin.value()
        self._play_timer.start(max(10, int(50 / speed)))
        self.play_btn.setText("暂停")

    def _pause_playback(self):
        self._playing = False
        self._play_timer.stop()
        self.play_btn.setText("继续")

    def _stop_playback(self):
        self._playing = False
        self._play_timer.stop()
        self._play_index = 0
        self.progress.setValue(0)
        self.play_btn.setText("播放")
        self.play_time_label.setText("0.0s / 0.0s")

    def _play_frame(self):
        if self._play_index >= len(self._play_data):
            self._stop_playback()
            return

        frame = self._play_data[self._play_index]
        self.hand_state.set_all_angles(frame["angles"])
        self.progress.setValue(self._play_index)

        total_time = self._play_data[-1]["timestamp"]
        self.play_time_label.setText(
            f"{frame['timestamp']:.1f}s / {total_time:.1f}s"
        )
        self._play_index += 1

    def _update_preview(self, frames: list[dict]):
        self.table.setRowCount(len(frames))
        for r, frame in enumerate(frames):
            self.table.setItem(r, 0, QTableWidgetItem(f"{frame['timestamp']:.2f}"))
            angles = frame["angles"]
            for c_idx, j_idx in enumerate(self._preview_joint_indices):
                if j_idx < len(angles):
                    self.table.setItem(r, c_idx + 1, QTableWidgetItem(f"{angles[j_idx]:.1f}"))
            self.table.setItem(r, len(self._preview_joint_indices) + 1, QTableWidgetItem("..."))

    def cleanup(self):
        self._rec_timer.stop()
        self._play_timer.stop()
