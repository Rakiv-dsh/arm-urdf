#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""joint_slider -- PyQt5 replacement for joint_state_publisher_gui.

Publishes sensor_msgs/JointState on /joint_states so that robot_state_publisher
turns the slider values into TF.  Used because ros-jazzy-joint-state-publisher-gui
is not installed in this WSL image.
"""

import socket
import struct
import sys
import time

STOP_SOCK = None
STOP_IDS = [21, 22, 23, 24, 25]
CAN_EFF = 0x80000000


def can_stop(iface="can0", ids=None, repeat=3):
    """直接发停机帧（0x04），不依赖任何 ROS 节点"""
    global STOP_SOCK
    ids = ids or STOP_IDS
    if STOP_SOCK is None:
        STOP_SOCK = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        STOP_SOCK.bind((iface,))
        STOP_SOCK.settimeout(0.05)
    n = 0
    for _ in range(repeat):
        for mid in ids:
            STOP_SOCK.send(struct.pack("=IB3x8s", (0x04 << 24) | (0xFD << 8) | mid | CAN_EFF,
                                       8, b"\x00" * 8))
            n += 1
        time.sleep(0.02)
    return n

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LIMIT_DEG = 180.0  # +/- 3.14 rad


class JointSliderNode(Node):
    def __init__(self):
        super().__init__("arm_joint_slider")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        from std_msgs.msg import Bool
        self.enable_pub = self.create_publisher(Bool, "/arm/enable", 10)
        self.positions = [0.0] * len(JOINTS)
        self.get_logger().info("publishing /joint_states for: " + ", ".join(JOINTS))

    def set_enable(self, on):
        m = Bool()
        m.data = bool(on)
        self.enable_pub.publish(m)

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [float(p) for p in self.positions]
        self.pub.publish(msg)


class Window(QWidget):
    def __init__(self, node: JointSliderNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Arm joint sliders  (/joint_states)")
        self.setMinimumWidth(520)

        root = QVBoxLayout(self)
        grid = QGridLayout()

        self.sliders = []
        self.value_labels = []

        for row, name in enumerate(JOINTS):
            grid.addWidget(QLabel(name), row, 0)

            sl = QSlider(Qt.Horizontal)
            sl.setRange(-int(LIMIT_DEG * 100), int(LIMIT_DEG * 100))
            sl.setValue(0)
            sl.setTickInterval(4500)
            sl.setTickPosition(QSlider.TicksBelow)
            grid.addWidget(sl, row, 1)

            lab = QLabel("  0.0\u00b0")
            lab.setMinimumWidth(72)
            lab.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(lab, row, 2)

            sl.valueChanged.connect(
                lambda v, i=row, l=lab: self.on_slider(i, v, l)
            )
            self.sliders.append(sl)
            self.value_labels.append(lab)

        root.addLayout(grid)

        self.stop_btn = QPushButton("STOP  (E-STOP)")
        self.stop_btn.setMinimumHeight(64)
        self.stop_btn.setStyleSheet(
            "QPushButton{background:#c62828;color:white;border:3px solid #7f0000;"
            "border-radius:10px;font-size:22px;font-weight:bold;}"
            "QPushButton:pressed{background:#8e0000;}"
            "QPushButton:hover{background:#e53935;}")
        self.stop_btn.clicked.connect(self.emergency_stop)
        root.addWidget(self.stop_btn)

        btns = QHBoxLayout()
        b0 = QPushButton("All zero")
        b0.clicked.connect(self.reset)
        btns.addWidget(b0)
        b_home = QPushButton("Home pose")
        b_home.clicked.connect(self.home)
        btns.addWidget(b_home)
        btns.addStretch(1)
        root.addLayout(btns)

        tip = QLabel("Values are published to /joint_states at 30 Hz.")
        tip.setStyleSheet("color: #777;")
        root.addWidget(tip)

        self.stamp_timer = QTimer(self)
        self.stamp_timer.timeout.connect(self.node.publish)
        self.stamp_timer.start(33)

    # ---- callbacks -------------------------------------------------
    def emergency_stop(self):
        """停机：CAN 直接停 + 通知伺服节点失能"""
        try:
            n = can_stop()
            msg = "已发 %d 帧停机\n%s" % (n, time.strftime("%H:%M:%S"))
        except OSError as e:
            msg = "停机失败：%s" % e
        try:
            self.node.set_enable(False)
        except Exception:
            pass
        self.stop_btn.setText("STOPPED " + time.strftime("%H:%M:%S"))

    def on_slider(self, idx, value, label):
        # slider unit is 0.01 degree -> radian
        deg = value / 100.0
        rad = deg * 0.017453292519943295
        self.node.positions[idx] = rad
        label.setText("{:+.1f}\u00b0".format(deg))

    def reset(self):
        for sl in self.sliders:
            sl.setValue(0)

    def home(self):
        # a mildly folded pose so the arm obviously articulates
        pose_deg = [0, 30, -60, 0, 45, 0]
        for sl, d in zip(self.sliders, pose_deg):
            sl.setValue(int(d * 100))


def main():
    rclpy.init()
    node = JointSliderNode()

    app = QApplication(sys.argv)
    win = Window(node)
    win.show()
    # 固定尺寸停到屏幕右下角，别压住 rviz 的菜单栏
    scr = QApplication.primaryScreen().availableGeometry()
    w, h = 580, 340
    win.setGeometry(
        scr.x() + scr.width() - w - 24,
        scr.y() + scr.height() - h - 24,
        w,
        h,
    )
    print(
        "slider window geometry:", win.geometry().getRect(),
        "screen:", scr.getRect(), flush=True,
    )

    def tick():
        # rclpy 接管了 SIGINT/SIGTERM，只把 context 标记为 invalid；
        # Qt 事件循环不会因此退出，所以这里主动轮询并关闭窗口。
        if not rclpy.ok():
            app.quit()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            app.quit()

    spin = QTimer()
    spin.timeout.connect(tick)
    spin.start(10)

    code = app.exec_()
    try:
        node.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
