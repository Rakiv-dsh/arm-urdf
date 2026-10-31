#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zero_and_hold.py —— 把当前姿态写成电机内部零位，随后立刻原地保持（把「松手窗口」压到最短）。

流程（全程约 1~2 秒电机会是松的，所以要有人扶住机械臂）：
  1. 读五台的机械角（打印，作为记录）
  2. 逐台发通信类型 6（设置机械零位）
  3. 立刻逐台使能 + 切 CSP + 目标 0（因为零位刚写过，0 就是当前位置）
  4. 持续保持一段时间，打印复核结果

用法：
    python3 tools/zero_and_hold.py                # 写零位并保持 8 秒
    python3 tools/zero_and_hold.py --hold 20
    python3 tools/zero_and_hold.py --dry
"""
import argparse
import os
import socket
import struct
import sys
import time

import yaml

EFF = 0x80000000
HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(os.path.dirname(HERE), "config", "joint_limits.yaml")
IDX_MECHPOS, IDX_LOCREF, IDX_RUNMODE, IDX_LIMSPD, IDX_LIMCUR = 0x7019, 0x7016, 0x7005, 0x7017, 0x2019


def sock(iface, timeout=0.3):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(timeout)
    return s


def send(s, cid, data=b"\x00" * 8):
    s.send(struct.pack("=IB3x8s", cid | EFF, 8, bytes(data)))


def drain(s, ms=40):
    t_end = time.monotonic() + ms / 1000.0
    while time.monotonic() < t_end:
        try:
            s.recv(16)
        except socket.timeout:
            return


def read_mechpos(s, mid, timeout=0.3):
    send(s, (0x11 << 24) | (0xFD << 8) | mid, struct.pack("<H", IDX_MECHPOS) + b"\x00" * 6)
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        try:
            fr = s.recv(16)
        except socket.timeout:
            return None
        if len(fr) < 16:
            continue
        cid, dlc = struct.unpack("=IB", fr[:5])
        if not cid & EFF:
            continue
        c = cid & 0x1FFFFFFF
        if ((c >> 24) & 0x1F) == 0x11 and ((c >> 8) & 0xFF) == mid:
            d = fr[8:8 + min(dlc, 8)]
            if (d[0] | (d[1] << 8)) == IDX_MECHPOS:
                return struct.unpack("<f", d[4:8])[0]
    return None


def write_param(s, mid, idx, val):
    send(s, (0x12 << 24) | (0xFD << 8) | mid, struct.pack("<H", idx) + struct.pack("<f", val))
    drain(s, 30)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--hold", type=float, default=8.0)
    ap.add_argument("--current", type=float, default=6.0)
    ap.add_argument("--speed", type=float, default=0.1)
    ap.add_argument("--dry", action="store_true")
    a = ap.parse_args()

    cfg = yaml.safe_load(open(CFG, encoding="utf-8"))
    ids = [int(j["motor"]["can_id"]) for j in cfg["joints"] if j.get("motor")]

    s = sock(a.iface)
    print("=== 写零前 ===")
    for mid in ids:
        print("  电机 %-3d 机械角 %s" % (mid, ("%.4f" % (read_mechpos(s, mid) or 0.0))))
    if a.dry:
        print("[--dry] 什么都不做")
        return 0

    print("=== 写零位 ===")
    for mid in ids:
        send(s, (0x04 << 24) | (0xFD << 8) | mid)          # 先失能
        drain(s, 30)
        send(s, (0x06 << 24) | (0xFD << 8) | mid, bytes([1] + [0] * 7))
        drain(s, 40)

    print("=== 立刻原地保持（目标 0）===")
    for mid in ids:
        if a.current > 0:
            write_param(s, mid, IDX_LIMCUR, a.current)
        send(s, (0x03 << 24) | (0xFD << 8) | mid)          # 使能
        drain(s, 30)
        write_mode(s, mid, 5)              # CSP
        write_param(s, mid, IDX_LIMSPD, a.speed)
        write_param(s, mid, IDX_LOCREF, 0.0)

    t_end = time.monotonic() + a.hold
    while time.monotonic() < t_end:
        for mid in ids:
            write_param(s, mid, IDX_LOCREF, 0.0)
        time.sleep(0.05)

    print("=== 复核（应全部 ≈ 0，且在保持）===")
    ok = True
    for mid in ids:
        v = read_mechpos(s, mid)
        good = v is not None and abs(v) < 0.02
        ok = ok and good
        print("  电机 %-3d 机械角 %s  %s" % (mid, ("%.4f" % v) if v is not None else "超时",
                                          "✓" if good else "✗"))
    s.close()
    print("\n零点已写进电机（断电不丢）。接着把 offset 全改 0 就完成了一一对应。")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
