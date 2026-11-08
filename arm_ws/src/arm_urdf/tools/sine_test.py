#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sine_test.py —— 关节正弦摆动 / 扫频实验

两条用法：

1) 定点摆动（只看跟随精度）
   python3 sine_test.py --ids 21 --amp-deg 3 --freq 0.2 --cycles 10

2) 扫频（额外得到带宽）
   python3 sine_test.py --ids 21 --amp-deg 3 --sweep --freq-start 0.1 --freq-end 2.0 --freq-steps 8

默认只打印计划；真正驱动电机必须加 --yes-move。

记录：指令 / 实际位置 / 速度 / 电流 / 母线电压（CSV）
统计：幅值比、相位滞后、RMS 误差、峰值电流、最低母线电压
"""
import argparse
import csv
import glob
import math
import os
import socket
import statistics
import struct
import sys
import time

EFF = 0x80000000
MASTER = 0xFD
P_MODE, P_LOC, P_SPD = 0x7005, 0x7016, 0x7017
P_POS, P_IQ, P_VEL, P_VBUS, P_CUR = 0x7019, 0x701A, 0x701B, 0x701C, 0x2019


class Bus:
    def __init__(self, iface):
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
        try:
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
        except OSError:
            pass
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
        if bus.read(ids[0], P_POS, 0.08) is not None:
            return name, bus
        bus.s.close()
    return None, None


def fit_sine(ts, ys, freq, t0):
    """在 ts 上拟合 y = A·sin(ω(t-t0)) + B·cos(ω(t-t0)) + C，返回 (幅值, 相位滞后°, RMS)"""
    w = 2 * math.pi * freq
    n = len(ts)
    if n < 8:
        return None
    s = [math.sin(w * (t - t0)) for t in ts]
    c = [math.cos(w * (t - t0)) for t in ts]
    one = [1.0] * n
    # 正规方程 3x3
    def dot(a, b):
        return sum(x * y for x, y in zip(a, b))
    M = [[dot(s, s), dot(s, c), dot(s, one)],
         [dot(c, s), dot(c, c), dot(c, one)],
         [dot(one, s), dot(one, c), n]]
    v = [dot(s, ys), dot(c, ys), sum(ys)]
    # 高斯消元
    for i in range(3):
        p = max(range(i, 3), key=lambda r: abs(M[r][i]))
        M[i], M[p] = M[p], M[i]
        v[i], v[p] = v[p], v[i]
        if abs(M[i][i]) < 1e-12:
            return None
        for r in range(i + 1, 3):
            f = M[r][i] / M[i][i]
            for k in range(i, 3):
                M[r][k] -= f * M[i][k]
            v[r] -= f * v[i]
    x = [0.0, 0.0, 0.0]
    for i in (2, 1, 0):
        acc = v[i] - sum(M[i][k] * x[k] for k in range(i + 1, 3))
        x[i] = acc / M[i][i]
    A, B, C = x
    amp = math.hypot(A, B)
    lag_deg = math.degrees(math.atan2(B, A))
    pred = [A * si + B * ci + C for si, ci in zip(s, c)]
    rms = math.sqrt(sum((y - pr) ** 2 for y, pr in zip(ys, pred)) / n)
    return amp, lag_deg, rms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default="21,22,23,24,25")
    ap.add_argument("--iface", default="auto")
    ap.add_argument("--amp-deg", type=float, default=5.0)
    ap.add_argument("--freq", type=float, default=0.25)
    ap.add_argument("--cycles", type=float, default=10.0)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--freq-start", type=float, default=0.1)
    ap.add_argument("--freq-end", type=float, default=2.0)
    ap.add_argument("--freq-steps", type=int, default=8)
    ap.add_argument("--cycles-per-step", type=float, default=4.0)
    ap.add_argument("--mode", choices=["per", "all"], default="per")
    ap.add_argument("--limit-cur", type=float, default=2.0)
    ap.add_argument("--max-speed", type=float, default=2.0)
    ap.add_argument("--ramp", type=float, default=1.0)
    ap.add_argument("--vbus-min", type=float, default=40.0)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--sample", type=float, default=25.0)
    ap.add_argument("--out", default="sine_test.csv")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--yes-move", action="store_true")
    a = ap.parse_args()

    ids = [int(x) for x in a.ids.split(",") if x.strip()]
    amp = math.radians(a.amp_deg)

    if a.sweep:
        freqs = []
        n = max(2, a.freq_steps)
        for i in range(n):
            f = a.freq_start + (a.freq_end - a.freq_start) * i / (n - 1)
            freqs.append(round(f, 3))
        segs = [(f, a.cycles_per_step) for f in freqs]
    else:
        segs = [(a.freq, a.cycles)]

    total = 0.0
    for f, cyc in segs:
        total += cyc / f + a.ramp * 2

    print("[计划] 电机 %s  摆幅 ±%.2f°  %s" % (ids, a.amp_deg,
          ("扫频 %.2f→%.2f Hz，%d 档" % (a.freq_start, a.freq_end, len(segs))) if a.sweep
          else "定点 %.2f Hz × %g 周期" % (segs[0][0], segs[0][1])))
    print("[时长] 每台约 %.1f s，逐台 %d 组共 %.1f s" % (total, len(ids) if a.mode == "per" else 1,
          total * (len(ids) if a.mode == "per" else 1)))
    print("[安全] 限流 %.1f A  限速 %.2f rad/s  母线 < %.0f V 立即停" % (a.limit_cur, a.max_speed, a.vbus_min))
    for f, _ in segs:
        need = 2 * math.pi * f * amp
        flag = "✓" if need <= a.max_speed * 0.8 else "⚠ 限速偏小"
        print("       %.2f Hz 需要峰速 %.3f rad/s %s" % (f, need, flag))

    if not a.yes_move:
        print("\n[未执行] 预演。确认参数后加 --yes-move 才真动。")
        return 0

    iface, bus = (a.iface, None)
    if a.iface == "auto":
        iface, bus = find_iface(ids)
        if iface is None:
            print("[失败] 没有 can 接口应答（电机没上电 / USB-CAN 没 attach）")
            return 2
    if bus is None:
        bus = Bus(iface)
    print("[总线] %s" % iface)

    for mid in ids:
        bus.param_f(mid, P_CUR, a.limit_cur); time.sleep(0.02)
        bus.stop(mid, clear=True); time.sleep(0.03)
        bus.param_b(mid, P_MODE, 5); time.sleep(0.02)
        bus.read(mid, P_MODE, 0.05)
        bus.enable(mid); time.sleep(0.03)
        bus.param_f(mid, P_SPD, a.max_speed); time.sleep(0.02)

    rows = []
    stopped = ""
    try:
        groups = [[m] for m in ids] if a.mode == "per" else [ids]
        for group in groups:
            base = {}
            for mid in group:
                v = bus.read(mid, P_POS)
                if v is None:
                    print("  [跳过] 电机 %d 无反馈" % mid)
                else:
                    base[mid] = v
            if not base:
                continue
            print("[摆动] 电机 %s" % list(base))
            for mid in base:
                bus.param_f(mid, P_SPD, a.max_speed)

            for f, cyc in segs:
                dur = cyc / f + 2 * a.ramp
                t_seg0 = time.monotonic()
                nxt_cmd = t_seg0
                nxt_smp = t_seg0
                print("   %.2f Hz ..." % f, end="", flush=True)
                while True:
                    now = time.monotonic()
                    t = now - t_seg0
                    if t > dur:
                        break
                    if t < a.ramp:
                        env = t / a.ramp
                    elif t > dur - a.ramp:
                        env = max(0.0, (dur - t) / a.ramp)
                    else:
                        env = 1.0
                    q = amp * env * math.sin(2 * math.pi * f * max(0.0, t - a.ramp))
                    if now >= nxt_cmd:
                        for mid in base:
                            bus.param_f(mid, P_LOC, base[mid] + q)
                        nxt_cmd += 1.0 / a.rate
                    if now >= nxt_smp:
                        nxt_smp += 1.0 / a.sample
                        rec = {"freq": f, "t": round(t, 4), "cmd": round(q, 6)}
                        for mid in base:
                            rec["pos_%d" % mid] = bus.read(mid, P_POS, 0.006)
                            rec["iq_%d" % mid] = bus.read(mid, P_IQ, 0.004)
                            rec["vel_%d" % mid] = bus.read(mid, P_VEL, 0.004)
                        vbus = bus.read(list(base)[0], P_VBUS, 0.01)
                        rec["vbus"] = vbus
                        # 记录本段起始时间，便于后续分段拟合
                        rec["seg0"] = round(t_seg0, 4)
                        rows.append(rec)
                        if vbus is not None and vbus < a.vbus_min:
                            stopped = "母线 %.1f V 低于 %.0f V" % (vbus, a.vbus_min)
                            break
                print(" 完成")
                for mid in base:
                    bus.param_f(mid, P_LOC, base[mid])
                time.sleep(0.3)
                if stopped:
                    break
            if stopped:
                break
    finally:
        for mid in ids:
            bus.stop(mid)
        print("[已停机]")

    if rows:
        keys = list(rows[0].keys())
        for r in rows:
            for k in r:
                if k not in keys:
                    keys.append(k)
        with open(a.out, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            for r in rows:
                w.writerow(r)
        print("[记录] %d 行 -> %s" % (len(rows), a.out))

    print("\n=== 结果 ===")
    if stopped:
        print("!! 提前停机：%s" % stopped)
    hdr = "%-6s %-6s %-9s %-11s %-9s %-8s" % ("电机", "频率", "幅值比", "相位滞后", "RMS误差", "峰值电流")
    print(hdr)
    print("-" * len(hdr))
    for mid in ids:
        pk = "pos_%d" % mid
        for f, _ in segs:
            seg = [r for r in rows if r.get(pk) is not None and abs(r["freq"] - f) < 1e-9]
            if len(seg) < 30:
                continue
            t0 = seg[0]["seg0"]
            # 取后半段（去掉 ramp）
            half = seg[len(seg) // 2:]
            ts = [r["t"] for r in half]
            ys = [r[pk] for r in half]
            cmd = [r["cmd"] for r in half]
            fr = fit_sine(ts, ys, f, t0)
            fc = fit_sine(ts, cmd, f, t0)
            iq = [abs(r["iq_%d" % mid]) for r in half if r.get("iq_%d" % mid) is not None]
            if fr and fc:
                gain = fr[0] / fc[0] if fc[0] > 1e-9 else 0.0
                print("%-6d %-6.2f %-9.2f %-11.1f %-9.3f %-8.2f" % (
                    mid, f, gain, -fr[1], math.degrees(fr[2]), max(iq) if iq else 0.0))
    vb = [r["vbus"] for r in rows if r.get("vbus") is not None]
    if vb:
        print("\n母线电压：最低 %.1f V，最高 %.1f V" % (min(vb), max(vb)))

    if a.plot:
        try:
            import sine_plot
            sys.argv = ["sine_plot", a.out, a.out.replace(".csv", ".png")]
            sine_plot.main()
        except Exception as exc:
            print("[画图失败] %s" % exc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
