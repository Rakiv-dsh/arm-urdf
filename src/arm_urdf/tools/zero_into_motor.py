#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zero_into_motor.py —— 把「当前姿态」写成电机内部的机械零位（断电不丢）。

原理：给电机发通信类型 6（设置机械零位），data[0]=1。
写完后 0x7019（负载端机械角）在当前位置读 ~0，并且这个零位存在电机里，重新上电依然有效。

⚠ 前提：先把机械臂摆到你想当作零位的姿态，**并且断电前就摆好**，因为这个零位是物理姿态定的。
   （写零位本身不需要断电，但摆姿势时电机应该是松的，所以通常先失能再摆。）

用法：
    python3 tools/zero_into_motor.py            # 读当前角度 → 写零位 → 复核
    python3 tools/zero_into_motor.py --dry      # 只看，不写
    python3 tools/zero_into_motor.py --ids 21   # 只处理某几台
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
IDX_MECHPOS = 0x7019


def sock(iface, timeout=0.3):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(timeout)
    return s


def send(s, cid, data=b"\x00" * 8):
    s.send(struct.pack("=IB3x8s", cid | EFF, 8, bytes(data)))


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


def set_zero(s, mid, timeout=0.3):
    send(s, (0x06 << 24) | (0xFD << 8) | mid, bytes([1] + [0] * 7))
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        try:
            fr = s.recv(16)
        except socket.timeout:
            return
        if len(fr) >= 16:
            cid, dlc = struct.unpack("=IB", fr[:5])
            if ((cid & 0x1FFFFFFF) >> 24) & 0x1F == 0x02:
                return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--ids", default="", help="逗号分隔，默认配置里的全部")
    ap.add_argument("--dry", action="store_true")
    a = ap.parse_args()

    cfg = yaml.safe_load(open(CFG, encoding="utf-8"))
    pairs = []
    for j in cfg["joints"]:
        m = j.get("motor")
        if m:
            pairs.append((j["name"], int(m["can_id"])))
    if a.ids:
        want = {int(x) for x in a.ids.split(",") if x.strip()}
        pairs = [(n, i) for n, i in pairs if i in want]

    s = sock(a.iface)
    print("=== 写零位前 ===")
    before = {}
    for name, mid in pairs:
        v = read_mechpos(s, mid)
        before[mid] = v
        print("  %-9s 电机 %-3d 机械角 %s" % (name, mid, ("%.4f" % v) if v is not None else "读取超时"))
    if any(v is None for v in before.values()):
        print("!! 有电机读不到，先别写零位")
        return 1
    if a.dry:
        print("\n[--dry] 没有写任何东西")
        return 0

    print("\n=== 写零位（通信类型 6，data[0]=1）===")
    for name, mid in pairs:
        set_zero(s, mid)
        print("  %-9s 电机 %-3d 已发送" % (name, mid))
        time.sleep(0.05)

    time.sleep(0.4)
    print("\n=== 复核（应全部 ≈ 0）===")
    ok = True
    for name, mid in pairs:
        v = read_mechpos(s, mid)
        good = v is not None and abs(v) < 0.02
        ok = ok and good
        print("  %-9s 电机 %-3d 机械角 %s   %s" % (
            name, mid, ("%.4f" % v) if v is not None else "读取超时",
            "✓" if good else "✗"))
    s.close()

    if ok:
        print("\n五台零位都写进电机了（断电不丢）。")
        print("下一步：把 joint_limits.yaml 里的 offset 全改成 0，再跑 sync_limits.py，")
        print("这样「关节角」就等于「电机读数」，仿真和真机一一对应。")
    else:
        print("\n有电机零位没写成，先别改 offset。")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
