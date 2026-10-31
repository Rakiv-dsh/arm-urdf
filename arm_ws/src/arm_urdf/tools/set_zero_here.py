#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""set_zero_here.py —— 把「电机此刻的真实位置」定义为关节角 0 点。

做法：读每台电机的 0x7019（负载端机械角），写进 config/joint_limits.yaml 的 offset，
然后调用 sync_limits.py 同步到控制器参数。**可逆**：随时再跑一次就以新位置为零。

（另一种做法是给电机发「置零」命令 0x06，会把零位写进电机内部、断电不丢，
  但那是永久改动，要谨慎；本脚本只改 ROS 侧偏置。）

用法：
    python3 tools/set_zero_here.py             # 读当前姿态并写入
    python3 tools/set_zero_here.py --iface can0 --dry   # 只打印，不写
"""
import argparse
import os
import re
import socket
import struct
import subprocess
import sys
import time

import yaml

EFF = 0x80000000
HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(os.path.dirname(HERE), "config", "joint_limits.yaml")


def read_mechpos(iface, mid, master=0xFD, timeout=0.25):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(timeout)
    s.send(struct.pack("=IB3x8s", (0x11 << 24) | (master << 8) | mid | EFF, 8,
                       struct.pack("<H", 0x7019) + b"\x00" * 6))
    t_end = time.monotonic() + timeout
    try:
        while time.monotonic() < t_end:
            fr = s.recv(16)
            if len(fr) < 16:
                continue
            cid, dlc = struct.unpack("=IB", fr[:5])
            if not cid & EFF:
                continue
            c = cid & 0x1FFFFFFF
            if ((c >> 24) & 0x1F) != 0x11 or ((c >> 8) & 0xFF) != mid:
                continue
            d = fr[8:8 + min(dlc, 8)]
            if (d[0] | (d[1] << 8)) == 0x7019:
                return struct.unpack("<f", d[4:8])[0]
    except socket.timeout:
        return None
    finally:
        s.close()
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--cfg", default=CFG)
    ap.add_argument("--dry", action="store_true")
    a = ap.parse_args()

    cfg = yaml.safe_load(open(a.cfg, encoding="utf-8"))
    text = open(a.cfg, encoding="utf-8").read()

    print("%-9s %-7s %-12s %s" % ("joint", "电机", "当前机械角", "写入的 offset"))
    got = {}
    for j in cfg["joints"]:
        m = j.get("motor")
        if not m:
            continue
        v = read_mechpos(a.iface, int(m["can_id"]))
        if v is None:
            print("  %s  电机 %d 读取失败，放弃（不改任何东西）" % (j["name"], m["can_id"]))
            return 1
        got[int(m["can_id"])] = v
        print("  %-9s %-7d %-12.4f %.4f" % (j["name"], m["can_id"], v, v))

    if a.dry:
        print("\n[--dry] 没有写入")
        return 0

    for cid, off in got.items():
        text = re.sub(r"(can_id: %d, dir: [-\d.]+, gear: [\d.]+, offset: )[-\d.]+" % cid,
                      lambda mm: mm.group(1) + ("%.4f" % off), text)
    open(a.cfg, "w", encoding="utf-8").write(text)
    print("\n已写入 %s" % a.cfg)

    sync = os.path.join(HERE, "sync_limits.py")
    if os.path.exists(sync):
        print("--- 同步到 URDF 与控制器参数 ---")
        subprocess.run([sys.executable, sync], check=False)
    print("\n完成：现在「关节角 0」= 刚才那个姿态。滑块归零不会再让机械臂动。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
