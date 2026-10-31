#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zero_safe.py —— 在不松开机械臂的前提下，逐台把当前姿态写成电机内部零位。

为什么能不断电：先给每台下发「目标=当前位置」，再发「设置机械零位」。
如果电机把目标当成机械坐标里的值，零位一改目标就会指向别处、电机会跑；
为了兜住这种情况，先把速度限制压到 0.02 rad/s，并在写完后立刻回读：
位置必须仍然是 0（±0.02），否则说明它开始跑了，马上恢复。

用法：
    python3 tools/zero_safe.py            # 逐台处理并回读验证
    python3 tools/zero_safe.py --dry
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
IDX_MECHPOS, IDX_LOCREF, IDX_RUNMODE, IDX_LIMSPD = 0x7019, 0x7016, 0x7005, 0x7017


def sock(iface, timeout=0.25):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(timeout)
    return s


def send(s, cid, data=b"\x00" * 8):
    s.send(struct.pack("=IB3x8s", cid | EFF, 8, bytes(data)))


def drain(s, ms=30):
    t = time.monotonic() + ms / 1000.0
    while time.monotonic() < t:
        try:
            s.recv(16)
        except socket.timeout:
            return


def rd(s, mid, idx=IDX_MECHPOS, timeout=0.25):
    send(s, (0x11 << 24) | (0xFD << 8) | mid, struct.pack("<H", idx) + b"\x00" * 6)
    t = time.monotonic() + timeout
    while time.monotonic() < t:
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
            if (d[0] | (d[1] << 8)) == idx:
                return struct.unpack("<f", d[4:8])[0]
    return None


def write_mode(s, mid, mode):
    send(s, (0x12 << 24) | (0xFD << 8) | mid, struct.pack("<H", 0x7005) + bytes([int(mode), 0, 0, 0]))
    drain(s, 30)


def wr(s, mid, idx, val):
    send(s, (0x12 << 24) | (0xFD << 8) | mid, struct.pack("<H", idx) + struct.pack("<f", val))
    drain(s, 30)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--ids", default="")
    ap.add_argument("--dry", action="store_true")
    a = ap.parse_args()

    cfg = yaml.safe_load(open(CFG, encoding="utf-8"))
    ids = [int(j["motor"]["can_id"]) for j in cfg["joints"] if j.get("motor")]
    if a.ids:
        want = {int(x) for x in a.ids.split(",") if x.strip()}
        ids = [i for i in ids if i in want]

    s = sock(a.iface)
    print("=== 逐台：先把目标钉在当前位置 → 写零位 → 回读 ===")
    bad = []
    for mid in ids:
        p0 = rd(s, mid)
        if p0 is None:
            print("  电机 %-3d 读不到，跳过" % mid)
            bad.append(mid)
            continue
        print("  电机 %-3d 当前 %.4f rad" % (mid, p0))
        if a.dry:
            continue
        # 1) 慢速 + 目标=当前位置（就算零位改了目标，跑起来也只有 0.02 rad/s）
        wr(s, mid, IDX_LIMSPD, 0.02)
        write_mode(s, mid, 5)
        send(s, (0x03 << 24) | (0xFD << 8) | mid)
        drain(s, 30)
        for _ in range(5):
            wr(s, mid, IDX_LOCREF, p0)
            time.sleep(0.02)
        # 2) 写零位
        send(s, (0x06 << 24) | (0xFD << 8) | mid, bytes([1] + [0] * 7))
        drain(s, 60)
        # 3) 回读两次
        time.sleep(0.4)
        v1 = rd(s, mid)
        time.sleep(1.2)
        v2 = rd(s, mid)
        ok = v1 is not None and v2 is not None and abs(v1) < 0.02 and abs(v2) < 0.02
        print("     写零后: %.4f / %.4f   %s" % (
            v1 if v1 is not None else float('nan'),
            v2 if v2 is not None else float('nan'),
            "✓ 没动" if ok else "✗ 跑了或在动"))
        if not ok:
            bad.append(mid)
            # 兜底：把目标拉回 0（新零位）并保持
            for _ in range(20):
                wr(s, mid, IDX_LOCREF, 0.0)
                time.sleep(0.02)
    s.close()
    print()
    if bad:
        print("有问题的电机：%s —— 先别改 offset" % bad)
        return 2
    print("五台零位都写进电机了（断电不丢），而且全程没有松开机械臂。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
