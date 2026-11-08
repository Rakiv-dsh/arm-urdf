#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sine_test.py —— 关节正弦摆动实验（跟踪性能测量）

每台电机按 q(t) = A·sin(2πft) 摆动，同时记录：
  指令角 / 实际角（0x7019）/ 速度（0x701B）/ 电流（0x701A）/ 母线电压（0x701C）

事后统计：幅值比、相位滞后、RMS 跟随误差、峰值电流、最低母线电压。

用法（默认只打印计划，不动电机）：
    python3 sine_test.py --ids 21,22,23,24,25 --amp-deg 5 --freq 0.25 --cycles 20
要真的动，必须显式加 --yes-move：
    python3 sine_test.py --ids 21 --amp-deg 3 --freq 0.2 --cycles 10 --yes-move
"""
import argparse
import csv
import glob
import math
import os
import socket
import struct
import sys
import time

EFF = 0x80000000
MASTER = 0xFD

PARAM_MODE = 0x7005      # 运行模式（5 = CSP）
PARAM_LOC = 0x7016       # 目标位置
PARAM_SPD = 0x7017       # 限速
PARAM_POS = 0x7019       # 机械位置
PARAM_IQ = 0x701A        # 电流
PARAM_VEL = 0x701B       # 速度
PARAM_VBUS = 0x701C      # 母线电压
PARAM_CUR = 0x2019       # 限流


class Bus:
    def __init__(self, iface):
        self.iface = iface
        self.s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.bind((iface,))
        self.s.settimeout(0.004)

    def send(self, mid, ctype, data):
        self.s.send(struct.pack("=IB3x8s", (ctype << 24) | (MASTER << 8) | mid | EFF, 8, bytes(data)))

    def param_f(self, mid, idx, val):
        self.send(mid, 0x12, struct.pack("<H", idx) + b"\x00\x00" + struct.pack("<f", float(val)))

    def param_b(self, mid, idx, val):
        self.send(mid, 0x12, struct.pack("<H", idx) + b"\x00\x00" + bytes([int(val), 0, 0, 0]))

    def read(self, mid, idx, timeout=0.02):
        self.send(mid, 0x11, struct.pack("<H", idx) + b"\x00" * 6)
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
        return None

    def enable(self, mid):
        self.send(mid, 0x03, b"\x00" * 8)

    def stop(self, mid, clear=False):
        self.send(mid, 0x04, bytes([1 if clear else 0, 0, 0, 0, 0, 0, 0, 0]))


def find_iface(ids):
    for name in sorted(os.path.basename(p) for p in glob.glob("/sys/class/net/can*")):
        try:
            bus = Bus(name)
        except OSError:
            continue
        if bus.read(ids[0], PARAM_POS, 0.05) is not None:
            return name
        bus.s.close()
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default="21,22,23,24,25")
    ap.add_argument("--iface", default="auto")
    ap.add_argument("--amp-deg", type=float, default=5.0, help="单边摆幅（度）")
    ap.add_argument("--freq", type=float, default=0.25, help="频率 Hz")
    ap.add_argument("--cycles", type=float, default=10.0, help="摆动周期数")
    ap.add_argument("--mode", choices=["per", "all"], default="per",
                    help="per=逐台单独摆，all=同时摆")
    ap.add_argument("--limit-cur", type=float, default=2.0, help="限流 A")
    ap.add_argument("--max-speed", type=float, default=1.0, help="限速 rad/s")
    ap.add_argument("--ramp", type=float, default=1.0, help="包络爬升时间 s")
    ap.add_argument("--vbus-min", type=float, default=40.0, help="母线低于此值立即停机")
    ap.add_argument("--rate", type=float, default=100.0, help="指令刷新率 Hz")
    ap.add_argument("--sample", type=float, default=20.0, help="遥测采样率 Hz")
    ap.add_argument("--out", default="sine_test.csv")
    ap.add_argument("--yes-move", action="store_true", help="真的驱动电机")
    a = ap.parse_args()

    ids = [int(x) for x in a.ids.split(",") if x.strip()]
    amp = math.radians(a.amp_deg)
    period = 1.0 / a.freq
    total = period * a.cycles + a.ramp * 2
    print("[计划] 电机 %s  摆幅 ±%.2f°  频率 %.2f Hz  周期 %.0f  总时长 %.1f s"
          % (ids, a.amp_deg, a.freq, a.cycles, total))
    print("[安全] 限流 %.1f A  限速 %.2f rad/s  母线 < %.0f V 立即停" % (a.limit_cur, a.max_speed, a.vbus_min))
    for mid in ids:
        v = max(a.max_speed, 2 * math.pi * a.freq * amp * 1.5)
        print("       电机 %d 需要的最小限速 %.3f rad/s" % (mid, 2 * math.pi * a.freq * amp))
    if not a.yes_move:
        print("\n[未执行] 这是预演。确认参数后加 --yes-move 才会真动。")
        return 0

    iface = a.iface
    if iface == "auto":
        iface = find_iface(ids)
        if iface is None:
            print("[失败] 没有 can 接口应答（电机没上电或 USB-CAN 没 attach）")
            return 2
    print("[总线]", iface)
    bus = Bus(iface)

    # 使能：限流 -> 清故障 -> CSP -> 读模式 -> 使能 -> 限速
    for mid in ids:
        bus.param_f(mid, PARAM_CUR, a.limit_cur)
        time.sleep(0.02)
        bus.stop(mid, clear=True)
        time.sleep(0.03)
        bus.param_b(mid, PARAM_MODE, 5)
        time.sleep(0.02)
        bus.read(mid, PARAM_MODE, 0.05)
        bus.enable(mid)
        time.sleep(0.03)
        bus.param_f(mid, PARAM_SPD, a.max_speed)
        time.sleep(0.02)

    rows = []
    stopped_reason = ""
    try:
        if a.mode == "per":
            groups = [[mid] for mid in ids]
        else:
            groups = [ids]

        for group in groups:
            print("[摆动] 电机 %s" % group)
            base = {mid: bus.read(mid, PARAM_POS) for mid in group}
            for mid, v in base.items():
                if v is None:
                    print("  [跳过] %d 无反馈" % mid)
            base = {k: v for k, v in base.items() if v is not None}
            if not base:
                continue
            for mid in base:
                bus.param_f(mid, PARAM_SPD, a.max_speed)
            t0 = time.monotonic()
            nxt_cmd = t0
            nxt_smp = t0
            while True:
                now = time.monotonic()
                t = now - t0
                if t > total:
                    break
                # 包络：前 ramp 从 0 升到 1，后 ramp 回落
                if t < a.ramp:
                    env = t / a.ramp
                elif t > total - a.ramp:
                    env = max(0.0, (total - t) / a.ramp)
                else:
                    env = 1.0
                q = amp * env * math.sin(2 * math.pi * a.freq * (t - a.ramp))
                if now >= nxt_cmd:
                    for mid in base:
                        bus.param_f(mid, PARAM_LOC, base[mid] + q)
                    nxt_cmd += 1.0 / a.rate
                if now >= nxt_smp:
                    nxt_smp += 1.0 / a.sample
                    rec = {"t": round(t, 4), "cmd": round(q, 5)}
                    for mid in base:
                        rec["pos_%d" % mid] = bus.read(mid, PARAM_POS, 0.006)
                        rec["iq_%d" % mid] = bus.read(mid, PARAM_IQ, 0.004)
                        rec["vel_%d" % mid] = bus.read(mid, PARAM_VEL, 0.004)
                    vbus = bus.read(base and list(base)[0], PARAM_VBUS, 0.01)
                    rec["vbus"] = vbus
                    rows.append(rec)
                    if vbus is not None and vbus < a.vbus_min:
                        stopped_reason = "母线电压 %.1f V 低于 %.0f V" % (vbus, a.vbus_min)
                        break
            # 收尾：回到起始位置
            for mid in base:
                bus.param_f(mid, PARAM_LOC, base[mid])
            time.sleep(0.5)
            if stopped_reason:
                break
    finally:
        for mid in ids:
            bus.stop(mid)
        print("[已停机]")

    # 写 CSV
    if rows:
        keys = list(rows[0].keys())
        for r in rows:
            for k in r:
                if k not in keys:
                    keys.append(k)
        with open(a.out, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in rows:
                w.writerow(r)
        print("[记录] %d 行 -> %s" % (len(rows), a.out))

    # 统计
    print("\n=== 结果 ===")
    if stopped_reason:
        print("!! 提前停机：%s" % stopped_reason)
    import statistics
    for mid in ids:
        pk = "pos_%d" % mid
        vals = [(r["t"], r["cmd"], r[pk]) for r in rows if r.get(pk) is not None]
        if len(vals) < 20:
            continue
        # 找稳态段（去掉前后各 20%）
        n = len(vals)
        seg = vals[int(n * 0.2):int(n * 0.8)]
        cmds = [abs(v[1]) for v in seg]
        poss = [abs(v[2] - seg[0][2] + 0.0) for v in seg]
        # 幅值：用峰峰值
        cmd_pp = max(v[1] for v in seg) - min(v[1] for v in seg)
        pos_pp = max(v[2] for v in seg) - min(v[2] for v in seg)
        err = statistics.mean(abs(v[1] - (v[2] - seg[0][2])) for v in seg)
        iq = [abs(r["iq_%d" % mid]) for r in rows if r.get("iq_%d" % mid) is not None]
        print("电机 %d: 指令峰峰 %.3f° 实际峰峰 %.3f° 幅值比 %.2f  平均误差 %.3f°  峰值电流 %.2f A"
              % (mid, math.degrees(cmd_pp), math.degrees(pos_pp),
                 (pos_pp / cmd_pp) if cmd_pp > 1e-9 else 0,
                 math.degrees(err), max(iq) if iq else 0))
    vb = [r["vbus"] for r in rows if r.get("vbus") is not None]
    if vb:
        print("母线电压：最低 %.1f V，最高 %.1f V" % (min(vb), max(vb)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
