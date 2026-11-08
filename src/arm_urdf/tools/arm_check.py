#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_check.py —— 上电后的体检（全程只读，不发任何运动/使能指令）

检查内容：
  1. 哪条 canX 上挂着电机（自动探测）
  2. 21~25 五台是否都在线
  3. 每台的母线电压、位置、电流、运行模式
  4. 监听 0.5 s 内的故障帧（0x15），解码故障位
  5. 给出结论：能不能开始实验

用法：
    python3 arm_check.py
    python3 arm_check.py --ids 21,22,23,24,25 --iface auto
"""
import argparse
import glob
import os
import socket
import struct
import sys
import time

EFF = 0x80000000
MASTER = 0xFD
PARAM_MODE = 0x7005
PARAM_POS = 0x7019
PARAM_IQ = 0x701A
PARAM_VEL = 0x701B
PARAM_VBUS = 0x701C

# RobStride 故障位（type 0x15，32 位小端）
FAULT_BITS = {
    0: "过流", 1: "过压", 2: "欠压", 3: "过温", 4: "电机过温",
    5: "磁编码器故障", 6: "过载", 7: "未标定", 8: "硬件故障", 9: "校准参数错误",
}


class Bus:
    def __init__(self, iface):
        self.s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.bind((iface,))
        self.s.settimeout(0.03)

    def read(self, mid, idx, timeout=0.12):
        try:
            self.s.send(struct.pack("=IB3x8s", (0x11 << 24) | (MASTER << 8) | mid | EFF, 8,
                                    struct.pack("<H", idx) + b"\x00" * 6))
            t_end = time.monotonic() + timeout
            while time.monotonic() < t_end:
                try:
                    fr = self.s.recv(16)
                except socket.timeout:
                    break
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

    def read_mode(self, mid, timeout=0.12):
        try:
            self.s.send(struct.pack("=IB3x8s", (0x11 << 24) | (MASTER << 8) | mid | EFF, 8,
                                    struct.pack("<H", PARAM_MODE) + b"\x00" * 6))
            t_end = time.monotonic() + timeout
            while time.monotonic() < t_end:
                try:
                    fr = self.s.recv(16)
                except socket.timeout:
                    break
                if len(fr) < 16:
                    continue
                cid, dlc = struct.unpack("=IB", fr[:5])
                c = cid & 0x1FFFFFFF
                if ((c >> 24) & 0x1F) == 0x11 and ((c >> 8) & 0xFF) == mid:
                    d = fr[8:8 + min(dlc, 8)]
                    if (d[0] | (d[1] << 8)) == PARAM_MODE:
                        return d[4]
        except OSError:
            pass
        return None

    def listen_faults(self, ms=500):
        out = {}
        t_end = time.monotonic() + ms / 1000.0
        while time.monotonic() < t_end:
            try:
                fr = self.s.recv(16)
            except socket.timeout:
                continue
            if len(fr) < 16:
                continue
            cid, dlc = struct.unpack("=IB", fr[:5])
            c = cid & 0x1FFFFFFF
            if ((c >> 24) & 0x1F) == 0x15:
                mid = (c >> 8) & 0xFF
                d = fr[8:8 + min(dlc, 8)]
                val = int.from_bytes(d[:4], "little")
                out[mid] = val
        return out


def decode(val):
    if not val:
        return "无故障"
    names = [name for bit, name in FAULT_BITS.items() if val >> bit & 1]
    return "、".join(names) if names else "未知位 0x%08X" % val


def probe_iface(ids):
    hits = []
    for name in sorted(os.path.basename(p) for p in glob.glob("/sys/class/net/can*")):
        try:
            bus = Bus(name)
        except OSError:
            continue
        n = sum(1 for mid in ids if bus.read(mid, PARAM_POS, 0.08) is not None)
        if n:
            hits.append((name, n, bus))
    hits.sort(key=lambda x: -x[1])
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default="21,22,23,24,25")
    ap.add_argument("--iface", default="auto")
    ap.add_argument("--fault-listen", type=float, default=0.5)
    a = ap.parse_args()

    ids = [int(x) for x in a.ids.split(",") if x.strip()]
    print("=" * 64)
    print("机械臂体检（只读，不会让电机动）")
    print("=" * 64)

    if a.iface == "auto":
        hits = probe_iface(ids)
        if not hits:
            print("\n❌ WSL 里没有任何 can 接口在应答电机")
            print("   1) USB-CAN 插好了吗？")
            print("   2) Windows 上跑过 usbipd attach 吗？")
            print("   3) 电机电源开了吗？")
            return 2
        for name, n, _ in hits:
            print("   接口 %s：%d/%d 台应答" % (name, n, len(ids)))
        iface = hits[0][0]
        bus = hits[0][2]
    else:
        iface = a.iface
        bus = Bus(iface)
    print("\n使用接口：%s\n" % iface)

    print("%-8s %-10s %-12s %-10s %-8s %s" % ("电机", "状态", "位置(rad)", "电流(A)", "母线(V)", "模式"))
    print("-" * 64)
    online = 0
    vbus_all = []
    bad_mode = []
    for mid in ids:
        pos = bus.read(mid, PARAM_POS)
        if pos is None:
            print("%-8d %-10s" % (mid, "离线"))
            continue
        online += 1
        iq = bus.read(mid, PARAM_IQ, 0.06)
        vel = bus.read(mid, PARAM_VEL, 0.06)
        vbus = bus.read(mid, PARAM_VBUS, 0.06)
        mode = bus.read_mode(mid, 0.06)
        if vbus is not None:
            vbus_all.append(vbus)
        if mode is not None and mode != 5:
            bad_mode.append((mid, mode))
        print("%-8d %-10s %-12.4f %-10s %-8s %s" % (
            mid, "在线", pos,
            "%.2f" % iq if iq is not None else "?",
            "%.1f" % vbus if vbus is not None else "?",
            str(mode) if mode is not None else "?"))

    print("\n监听故障帧 %.1f s ..." % a.fault_listen)
    faults = bus.listen_faults(int(a.fault_listen * 1000))
    if faults:
        for mid, val in sorted(faults.items()):
            print("   电机 %d：%s（0x%08X）" % (mid, decode(val), val))
    else:
        print("   没有收到故障帧")

    print("\n" + "=" * 64)
    print("结论")
    print("=" * 64)
    print("  在线：%d/%d 台" % (online, len(ids)))
    if vbus_all:
        print("  母线：%.1f ~ %.1f V" % (min(vbus_all), max(vbus_all)))
        if min(vbus_all) < 40:
            print("  ⚠ 有电机母线低于 40 V，加速时容易触发欠压保护")
        if max(vbus_all) > 55:
            print("  ⚠ 母线偏高，确认供电电压")
    if bad_mode:
        for mid, mode in bad_mode:
            print("  ⚠ 电机 %d 运行模式 = %s（不是 5/CSP），运动前需要切换" % (mid, mode))
    if faults:
        print("  ⚠ 存在故障帧，先排除故障再运动")
    ok = online == len(ids) and not faults and all(v >= 40 for v in vbus_all) if vbus_all else False
    if ok:
        print("\n  ✅ 可以开始实验：python3 sine_test.py --ids %s --amp-deg 3 --freq 0.2 --cycles 10 --yes-move"
              % ",".join(str(i) for i in ids))
        return 0
    print("\n  ⛔ 先解决上面的问题，再跑实验")
    return 1


if __name__ == "__main__":
    sys.exit(main())
