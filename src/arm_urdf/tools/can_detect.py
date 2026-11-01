#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""can_detect.py —— 自动找出电机接在哪条 CAN 总线上

USB 重插后适配器顺序会变（有时是 can0，有时是 can2），这个脚本挨个试，
哪个接口能应答 21~25 就用哪个。

用法：
    python3 can_detect.py            # 打印接口名，例如 can2
    python3 can_detect.py --quiet    # 只打印名字（给脚本用）
退出码：0 = 找到，1 = 没找到
"""
import argparse
import socket
import struct
import sys
import time

EFF = 0x80000000
PROBE_ID = 21
IDX = 0x7019


def probe(iface, mid=PROBE_ID, timeout=0.12):
    try:
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((iface,))
        s.settimeout(0.02)
    except OSError:
        return False
    try:
        s.send(struct.pack("=IB3x8s", (0x11 << 24) | (0xFD << 8) | mid | EFF, 8,
                           struct.pack("<H", IDX) + b"\x00" * 6))
        t_end = time.monotonic() + timeout
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
                return True
        return False
    except OSError:
        return False
    finally:
        s.close()


def detect(candidates=None):
    if candidates is None:
        candidates = ["can%d" % i for i in range(8)]
    for iface in candidates:
        if probe(iface):
            return iface
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--list", default="")
    a = ap.parse_args()
    cands = [x for x in a.list.split(",") if x] or None
    iface = detect(cands)
    if iface is None:
        if not a.quiet:
            print("没找到：所有候选接口都不应答", file=sys.stderr)
        return 1
    print(iface)
    return 0


if __name__ == "__main__":
    sys.exit(main())
