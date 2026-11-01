#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""estop_button.py —— 大红停机按钮

按下后直接向 can0 上的 21~25 号电机发「停机」帧（0x04），不依赖任何 ROS 节点，
所以伺服节点卡死、崩溃、或者根本没启动的时候也照样能停。

用法：
    python3 estop_button.py                 # 真按
    python3 estop_button.py --dry-run       # 只打印，不真发（测试界面）
    python3 estop_button.py --ids 21,22,23,24,25 --iface can0

快捷键：空格 / 回车 / Esc 都能触发停机。
"""
import argparse
import os
import socket
import struct
import sys
import time

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (QApplication, QLabel, QPushButton, QVBoxLayout, QWidget)

CAN_EFF_FLAG = 0x80000000
MASTER = 0xFD
T_STOP = 0x04


class Stopper:
    def __init__(self, iface, ids, dry_run=False, repeat=3):
        self.iface = iface  # "auto" = 所有 can 接口都发
        self.ids = ids
        self.dry_run = dry_run
        self.repeat = repeat
        self.sock = None

    @staticmethod
    def all_can_ifaces():
        import glob
        return sorted(os.path.basename(p) for p in glob.glob("/sys/class/net/can*")) or ["can0"]

    def _socks(self):
        ifaces = self.all_can_ifaces() if self.iface in (None, "", "auto") else [self.iface]
        out = []
        for iface in ifaces:
            try:
                s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                s.bind((iface,))
                s.settimeout(0.05)
                out.append((iface, s))
            except OSError:
                pass
        return out

    def stop(self):
        """返回 (成功台数, 说明)"""
        if self.dry_run:
            return len(self.ids), "dry-run：没有真的发"
        socks = self._socks()
        if not socks:
            return 0, "没有可用的 can 接口"
        ok = 0
        for _ in range(self.repeat):
            for iface, s in socks:
                for mid in self.ids:
                    try:
                        s.send(struct.pack("=IB3x8s",
                                           (T_STOP << 24) | (MASTER << 8) | mid | CAN_EFF_FLAG,
                                           8, b"\x00" * 8))
                        ok += 1
                    except OSError:
                        pass
            time.sleep(0.02)
        return ok, "sent %d frames on %s" % (ok, ",".join(i for i, _ in socks))


class StopWindow(QWidget):
    def __init__(self, stopper):
        super().__init__()
        self.stopper = stopper
        self.setWindowTitle("EMERGENCY STOP  (el05)")
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)
        self.setMinimumSize(340, 240)

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)

        btn = QPushButton("STOP")
        btn.setMinimumHeight(140)
        btn.setStyleSheet(
            "QPushButton{background:#c62828;color:white;border:4px solid #7f0000;"
            "border-radius:14px;font-size:30px;font-weight:bold;}"
            "QPushButton:pressed{background:#8e0000;}"
            "QPushButton:hover{background:#e53935;}")
        btn.clicked.connect(self.on_stop)
        root.addWidget(btn)

        self.status = QLabel("就绪（空格 / 回车 / Esc 也能停）")
        self.status.setWordWrap(True)
        self.status.setAlignment(Qt.AlignCenter)
        f = QFont(); f.setPointSize(10); self.status.setFont(f)
        root.addWidget(self.status)

        self.hint = QLabel("接口 %s   电机 %s" % (stopper.iface, ",".join(str(i) for i in stopper.ids)))
        self.hint.setAlignment(Qt.AlignCenter)
        self.hint.setStyleSheet("color:#666;font-size:11px;")
        root.addWidget(self.hint)

    def keyPressEvent(self, ev):
        if ev.key() in (Qt.Key_Space, Qt.Key_Return, Qt.Key_Enter, Qt.Key_Escape):
            self.on_stop()
        else:
            super().keyPressEvent(ev)

    def on_stop(self):
        n, msg = self.stopper.stop()
        t = time.strftime("%H:%M:%S")
        if n > 0:
            self.status.setText("STOPPED %s\n%s" % (t, msg))
            self.status.setStyleSheet("color:#1b5e20;font-weight:bold;")
        else:
            self.status.setText("FAILED %s\n%s" % (t, msg))
            self.status.setStyleSheet("color:#b71c1c;font-weight:bold;")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--ids", default="21,22,23,24,25")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",") if x.strip()]

    app = QApplication(sys.argv)
    w = StopWindow(Stopper(a.iface, ids, dry_run=a.dry_run))
    w.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
