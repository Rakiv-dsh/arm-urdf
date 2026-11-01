#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""joint_slider -- PyQt5 replacement for joint_state_publisher_gui.

Publishes sensor_msgs/JointState on /joint_states so that robot_state_publisher
turns the slider values into TF.  Used because ros-jazzy-joint-state-publisher-gui
is not installed in this WSL image.
"""

import os
import socket
import struct
import sys
import time

STOP_SOCK = {}          # 每条总线一个 socket
STOP_IDS = [21, 22, 23, 24, 25]
CAN_EFF = 0x80000000


def _all_can_ifaces():
    """系统里所有 can 接口（USB 重插后编号会变，不能写死 can0）"""
    try:
        import glob
        return sorted(os.path.basename(x) for x in glob.glob("/sys/class/net/can*"))
    except Exception:
        return ["can0"]


def can_stop(iface="auto", ids=None, repeat=3):
    """在所有 can 接口上发停机帧（0x04）。返回发出的帧数。"""
    ids = ids or STOP_IDS
    ifaces = _all_can_ifaces() if iface in (None, "", "auto") else [iface]
    sent = 0
    for name in ifaces:
        try:
            s = STOP_SOCK.get(name)
            if s is None:
                s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                s.bind((name,))
                s.settimeout(0.05)
                STOP_SOCK[name] = s
            for _ in range(repeat):
                for mid in ids:
                    try:
                        s.send(struct.pack("=IB3x8s", (0x04 << 24) | (0xFD << 8) | mid | CAN_EFF,
                                           8, b"\x00" * 8))
                        sent += 1
                    except OSError:
                        break
                time.sleep(0.015)
        except OSError:
            STOP_SOCK.pop(name, None)
    return sent


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

from PyQt5.QtCore import Qt, QObject, QEvent, QTimer
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

try:
    from ament_index_python.packages import get_package_share_directory
    CFG_LIMITS = os.path.join(get_package_share_directory("arm_urdf"), "config", "joint_limits.yaml")
except Exception:
    CFG_LIMITS = ""

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LIMIT_DEG = 180.0  # +/- 3.14 rad


# ---------------- 直接驱动电机（面板自带，不依赖 C++ 伺服节点） ----------------
import glob


class MotorDriver:
    """读 joint_limits.yaml 的 can_id/dir/gear/offset/限位，用 CSP 直接驱动电机。

    ENABLE = 使能五台并停在当前位置；拖动滑块 = 斜坡跟随；STOP = 全总线停机帧。
    """

    def __init__(self, yaml_path):
        self.yaml_path = yaml_path
        self.motors = []          # [{name, can_id, dir, gear, offset, lo, hi}]
        self.socks = {}
        self.on = False
        self.targets = {}
        self.max_speed = 0.08     # rad/s
        self.limit_cur = 2.0      # A
        self._load_driver_cfg()
        self.last_t = time.monotonic()
        self.last_err = ""
        self.iface = None
        self._load()

    def _load_driver_cfg(self):
        """读取 driver_cfg.json（同目录或包的 config 下），让速度/限流可调不必改代码"""
        import json
        here = os.path.dirname(os.path.abspath(__file__))
        cands = [
            os.path.join(here, "driver_cfg.json"),
            os.path.join(os.path.dirname(os.path.dirname(here)), "share", "arm_urdf", "config", "driver_cfg.json"),
        ]
        if CFG_LIMITS:
            cands.append(os.path.join(os.path.dirname(CFG_LIMITS), "driver_cfg.json"))
        for f in cands:
            try:
                c = json.load(open(f, encoding="utf-8"))
                self.max_speed = float(c.get("max_speed_rad_s", self.max_speed))
                self.limit_cur = float(c.get("limit_cur", self.limit_cur))
                self.cfg_path = f
                return
            except Exception:
                continue
        self.cfg_path = None

    def _load(self):
        try:
            import yaml
            cfg = yaml.safe_load(open(self.yaml_path, encoding="utf-8"))
        except Exception as e:
            self.last_err = "读配置失败: %s" % e
            return
        for j in cfg.get("joints", []):
            m = j.get("motor")
            if not m:
                continue
            self.motors.append(dict(
                name=j["name"], can_id=int(m["can_id"]),
                dir=float(m.get("dir", 1.0)), gear=float(m.get("gear", 1.0)),
                offset=float(m.get("offset", 0.0)),
                lo=float(j["lower"]), hi=float(j["upper"])))

    # ---- CAN 基础 ----
    @staticmethod
    def ifaces():
        return sorted(os.path.basename(p) for p in glob.glob("/sys/class/net/can*"))

    def _sock(self, name):
        s = self.socks.get(name)
        if s is None:
            s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            s.bind((name,))
            s.settimeout(0.02)
            self.socks[name] = s
        return s

    def _send(self, name, cid, data):
        self._sock(name).send(struct.pack("=IB3x8s", cid | CAN_EFF, 8, bytes(data)))

    def _param_f(self, name, mid, idx, val):
        # PAD2: 索引放 data[0..1]，value 必须放 data[4..7]，中间两个字节留 0
        self._send(name, (0x12 << 24) | (0xFD << 8) | mid,
                   struct.pack("<H", idx) + b"\x00\x00" + struct.pack("<f", float(val)))

    def _param_b(self, name, mid, idx, val):
        # PAD2: uint8 参数同样是 data[4]
        self._send(name, (0x12 << 24) | (0xFD << 8) | mid,
                   struct.pack("<H", idx) + b"\x00\x00" + bytes([int(val), 0, 0, 0]))

    def read_mech(self, name, mid, idx=0x7019):
        try:
            s = self._sock(name)
            s.send(struct.pack("=IB3x8s", (0x11 << 24) | (0xFD << 8) | mid | CAN_EFF, 8,
                               struct.pack("<H", idx) + b"\x00" * 6))
            t_end = time.monotonic() + 0.2
            while time.monotonic() < t_end:
                try:
                    fr = s.recv(16)
                except socket.timeout:
                    continue
                if len(fr) < 16:
                    continue
                cid, dlc = struct.unpack("=IB", fr[:5])
                c = cid & 0x1FFFFFFF
                if ((c >> 24) & 0x1F) == 0x11 and ((c >> 8) & 0xFF) == mid:
                    d = fr[8:8 + min(dlc, 8)]
                    if (d[0] | (d[1] << 8)) == idx:
                        return struct.unpack("<f", d[4:8])[0]
        except OSError:
            pass
        return None

    def read_f32(self, name, mid, idx, timeout=0.2):
        try:
            s_ = self._sock(name)
            s_.send(struct.pack("=IB3x8s", (0x11 << 24) | (0xFD << 8) | mid | CAN_EFF, 8,
                                struct.pack("<H", idx) + b"\x00" * 6))
            t_end = time.monotonic() + timeout
            while time.monotonic() < t_end:
                try:
                    fr = s_.recv(16)
                except socket.timeout:
                    continue
                if len(fr) < 16:
                    continue
                cid, dlc = struct.unpack("=IB", fr[:5])
                c = cid & 0x1FFFFFFF
                if ((c >> 24) & 0x1F) == 0x11 and ((c >> 8) & 0xFF) == mid:
                    d = fr[8:8 + min(dlc, 8)]
                    if (d[0] | (d[1] << 8)) == idx:
                        return struct.unpack("<f", d[4:8])[0]
        except OSError:
            pass
        return None

    def _stop_all(self):
        sent = 0
        for name in self.ifaces():
            try:
                for _ in range(2):
                    for m in self.motors:
                        self._send(name, (0x04 << 24) | (0xFD << 8) | m["can_id"], b"\x00" * 8)
                        sent += 1
                    time.sleep(0.01)
            except OSError:
                self.socks.pop(name, None)
        return sent

    # ---- 启动 / 停止 ----
    def start(self):
        if self.on:
            return True, "already on"
        if not self.motors:
            return False, "没有可驱动关节"
        ok_iface = None
        for name in self.ifaces():
            for m in self.motors:
                if self.read_mech(name, m["can_id"]) is not None:
                    ok_iface = name
                    break
            if ok_iface:
                break
        if not ok_iface:
            self.last_err = "没有 can 接口应答（电机没上电？）"
            return False, self.last_err
        for m in self.motors:
            mid = m["can_id"]
            try:
                cur = self.read_mech(ok_iface, mid)
                if cur is None:
                    continue
                self._param_f(ok_iface, mid, 0x2019, self.limit_cur)   # 限流
                time.sleep(0.02)
                self._send(ok_iface, (0x04 << 24) | (0xFD << 8) | mid,
                           bytes([1, 0, 0, 0, 0, 0, 0, 0]))            # 停机 + 清故障
                time.sleep(0.03)
                self._param_b(ok_iface, mid, 0x7005, 5)                # 切 CSP
                time.sleep(0.02)
                self.read_f32(ok_iface, mid, 0x7005, timeout=0.05)     # 读回（和库一致）
                self._send(ok_iface, (0x03 << 24) | (0xFD << 8) | mid, b"\x00" * 8)   # 使能
                time.sleep(0.03)
                self._param_f(ok_iface, mid, 0x7017, self.max_speed)   # 限速
                self.targets[m["name"]] = cur
            except OSError as e:
                self.last_err = "CAN 发送失败: %s" % e
                return False, self.last_err
        self.iface = ok_iface
        self.on = True
        self.last_t = time.monotonic()
        time.sleep(0.25)
        cur = self.read_f32(ok_iface, self.motors[0]["can_id"], 0x701A, timeout=0.15)
        return True, "enabled on %s, iq=%s" % (ok_iface, "%.3f" % cur if cur is not None else "?")

    def stop(self):
        n = self._stop_all()
        self.on = False
        return n

    # ---- 每个周期跟随滑块 ----
    def tick(self, q_map):
        if not self.on:
            return
        now = time.monotonic()
        dt = max(0.0, min(0.1, now - self.last_t))
        self.last_t = now
        step = self.max_speed * dt
        iface = self.iface
        if iface is None:
            for name in self.ifaces():
                if self.read_mech(name, self.motors[0]["can_id"]) is not None:
                    self.iface = name
                    break
            iface = self.iface
        if iface is None:
            return
        for m in self.motors:
            q = q_map.get(m["name"])
            if q is None:
                continue
            q = max(m["lo"], min(m["hi"], q))
            want = m["offset"] + m["dir"] * m["gear"] * q
            cur = self.targets.get(m["name"], want)
            d = want - cur
            d = max(-step, min(step, d))
            self.targets[m["name"]] = cur + d
            try:
                self._param_f(iface, m["can_id"], 0x7016, self.targets[m["name"]])
            except OSError:
                return



class KeyFilter(QObject):
    """把方向键等按键从控件手里抢回来，统一交给窗口处理。"""

    def __init__(self, win):
        super().__init__()
        self.win = win

    def eventFilter(self, obj, ev):
        if ev.type() == QEvent.KeyPress and self.win.handle_key(ev):
            return True
        return False


class JointSliderNode(Node):
    def __init__(self):
        super().__init__("arm_joint_slider")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        from std_msgs.msg import Bool
        self.enable_pub = self.create_publisher(Bool, "/arm/enable", 10)
        self.positions = [0.0] * len(JOINTS)
        self.get_logger().info("publishing /joint_states for: " + ", ".join(JOINTS))

    def set_enable(self, on, times=3):
        """ENABLE_BTN_V2：连发几次，避免单次丢了没反应"""
        m = Bool()
        m.data = bool(on)
        for _ in range(times):
            self.enable_pub.publish(m)
            time.sleep(0.05)

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
        self.driver = MotorDriver(CFG_LIMITS)
        self.setWindowTitle("Arm joint sliders  (/joint_states)")
        self.setMinimumWidth(520)

        root = QVBoxLayout(self)
        grid = QGridLayout()

        self.sliders = []
        self.value_labels = []

        for row, name in enumerate(JOINTS):
            grid.addWidget(QLabel(name), row, 0)

            sl = QSlider(Qt.Horizontal)
            sl.setFocusPolicy(Qt.NoFocus)
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

        self.enable_btn = QPushButton("ENABLE  (motors ON)")
        self.enable_btn.setMinimumHeight(52)
        self.enable_btn.setFocusPolicy(Qt.NoFocus)
        self.enable_btn.setStyleSheet(
            "QPushButton{background:#2e7d32;color:white;border:3px solid #1b5e20;"
            "border-radius:10px;font-size:18px;font-weight:bold;}"
            "QPushButton:pressed{background:#1b5e20;}"
            "QPushButton:hover{background:#43a047;}")
        self.enable_btn.clicked.connect(self.do_enable)
        root.addWidget(self.enable_btn)

        self.stop_btn = QPushButton("STOP  (E-STOP)")
        self.stop_btn.setMinimumHeight(64)
        self.stop_btn.setStyleSheet(
            "QPushButton{background:#c62828;color:white;border:3px solid #7f0000;"
            "border-radius:10px;font-size:22px;font-weight:bold;}"
            "QPushButton:pressed{background:#8e0000;}"
            "QPushButton:hover{background:#e53935;}")
        self.stop_btn.setFocusPolicy(Qt.NoFocus)
        self.stop_btn.clicked.connect(self.emergency_stop)
        root.addWidget(self.stop_btn)

        btns = QHBoxLayout()
        b0 = QPushButton("All zero")
        b0.setFocusPolicy(Qt.NoFocus)
        b0.clicked.connect(self.reset)
        btns.addWidget(b0)
        b_home = QPushButton("Home pose")
        b_home.setFocusPolicy(Qt.NoFocus)
        b_home.clicked.connect(self.home)
        btns.addWidget(b_home)
        btns.addStretch(1)
        root.addLayout(btns)

        self.selected = 0
        self.step_deg = 1.0
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()

        self.kinfo = QLabel()
        self.kinfo.setStyleSheet("color:#1a237e;font-weight:bold;")
        root.addWidget(self.kinfo)

        tip = QLabel("ARROWS = jog selected joint   LEFT/RIGHT = pick joint   "
                     "[ ] = step   Z = zero joint   A = all zero   S = STOP")
        tip.setStyleSheet("color:#777;")
        tip.setWordWrap(True)
        root.addWidget(tip)
        self.refresh_hint()

        self.drive_timer = QTimer(self)
        self.drive_timer.timeout.connect(self.on_drive)
        self.drive_timer.start(33)

        self.stamp_timer = QTimer(self)
        self.stamp_timer.timeout.connect(self.node.publish)
        self.stamp_timer.start(33)

    # ---- callbacks -------------------------------------------------
    def on_drive(self):
        if self.driver.on:
            q_map = {name: self.node.positions[i] for i, name in enumerate(JOINTS)}
            self.driver.tick(q_map)

    def do_enable(self):
        """使能：面板自己直接通过 CAN 使能五台电机（不依赖 C++ 伺服节点）"""
        ok, msg = self.driver.start()
        if ok:
            self.enable_btn.setText("ENABLED %s  %s" % (time.strftime("%H:%M:%S"), msg))
        else:
            self.enable_btn.setText("ENABLE failed: " + str(msg)[:40])

    def emergency_stop(self):
        """停机：CAN 直接停（全总线）+ 通知伺服节点失能（重复 3 次）"""
        frames = -1
        try:
            frames = self.driver.stop()
        except Exception:
            frames = -1
        try:
            self.node.set_enable(False, times=3)
        except Exception:
            pass
        t = time.strftime("%H:%M:%S")
        if frames >= 0:
            self.stop_btn.setText("STOPPED %s  (%d frames + disable)" % (t, frames))
        else:
            self.stop_btn.setText("STOPPED %s  (disable sent, CAN failed)" % t)

    def refresh_hint(self):
        if not hasattr(self, "kinfo"):
            return
        for i, lab in enumerate(self.value_labels):
            lab.setStyleSheet("font-weight:bold;color:#c62828;" if i == self.selected else "")
        self.kinfo.setText("selected: %s     step: %.2f deg" % (JOINTS[self.selected], self.step_deg))

    def select(self, i):
        self.selected = max(0, min(len(JOINTS) - 1, i))
        self.refresh_hint()

    def nudge(self, direction, mult=1):
        sl = self.sliders[self.selected]
        d = int(round(self.step_deg * 100)) * mult * direction
        v = max(sl.minimum(), min(sl.maximum(), sl.value() + d))
        sl.setValue(v)
        self.refresh_hint()

    def handle_key(self, ev):
        K = ev.key()
        if Qt.Key_1 <= K <= Qt.Key_6:
            self.select(K - Qt.Key_1)
        elif K in (Qt.Key_Up,):
            self.nudge(+1)
        elif K in (Qt.Key_Down,):
            self.nudge(-1)
        elif K == Qt.Key_PageUp:
            self.nudge(+1, 10)
        elif K == Qt.Key_PageDown:
            self.nudge(-1, 10)
        elif K == Qt.Key_Left:
            self.select(self.selected - 1)
        elif K == Qt.Key_Right:
            self.select(self.selected + 1)
        elif K == Qt.Key_BracketLeft:
            for st in (5.0, 1.0, 0.5, 0.1):
                if st < self.step_deg - 1e-9:
                    self.step_deg = st
                    break
            else:
                self.step_deg = 5.0
            self.refresh_hint()
        elif K == Qt.Key_BracketRight:
            for st in (0.1, 0.5, 1.0, 5.0):
                if st > self.step_deg + 1e-9:
                    self.step_deg = st
                    break
            else:
                self.step_deg = 0.1
            self.refresh_hint()
        elif K == Qt.Key_Z:
            self.sliders[self.selected].setValue(0)
            self.refresh_hint()
        elif K == Qt.Key_A:
            self.reset()
        elif K == Qt.Key_S:
            self.emergency_stop()
        else:
            return False
        return True

    def keyPressEvent(self, ev):
        if not self.handle_key(ev):
            super().keyPressEvent(ev)

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
    _filter = KeyFilter(win)
    app.installEventFilter(_filter)
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
