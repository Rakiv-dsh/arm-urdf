#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""record_range.py —— 拖滑块时实时记录每根轴实际走到的范围。

不驱动任何东西，只读电机位置（0x7019），统计 min/max。
按 Ctrl+C 结束，或者在别的终端看 /tmp/range.json。

用法：
  python3 tools/record_range.py            # 5 台，默认 21..25
  python3 tools/record_range.py --rate 5 --out /tmp/range.json
"""
import argparse, json, math, os, signal, socket, struct, sys, time

CAN_EFF_FLAG = 0x80000000
MASTER = 0xFD
IDX_MECH = 0x7019


def sock(iface="can0"):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(0.03)
    return s


def read_mech(s, mid, idx=IDX_MECH, timeout=0.05):
    s.send(struct.pack("=IB3x8s", (0x11 << 24) | (MASTER << 8) | mid | CAN_EFF_FLAG, 8,
                       struct.pack("<H", idx) + b"\x00" * 6))
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        try:
            fr = s.recv(16)
        except socket.timeout:
            return None
        if len(fr) < 16:
            continue
        cid, dlc = struct.unpack("=IB", fr[:5])
        if not cid & CAN_EFF_FLAG:
            continue
        c = cid & 0x1FFFFFFF
        if ((c >> 24) & 0x1F) == 0x11 and ((c >> 8) & 0xFF) == mid:
            d = fr[8:8 + min(dlc, 8)]
            if (d[0] | (d[1] << 8)) == idx:
                return struct.unpack("<f", d[4:8])[0]
    return None


def wrap(v):
    """折到 (-pi, pi]，连续关节转整圈也不让范围虚胖。"""
    if v is None:
        return None
    while v > math.pi:
        v -= 2 * math.pi
    while v <= -math.pi:
        v += 2 * math.pi
    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--ids", default="21,22,23,24,25")
    ap.add_argument("--rate", type=float, default=4.0, help="每秒采样轮数")
    ap.add_argument("--out", default="/tmp/range.json")
    a = ap.parse_args()

    ids = [int(x) for x in a.ids.split(",")]
    names = ["joint%d" % (i + 1) for i in range(len(ids))]
    lo = {n: float("inf") for n in names}
    hi = {n: float("-inf") for n in names}
    cur = {n: None for n in names}
    stop = {"v": False}

    def on_sig(*_):
        stop["v"] = True
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    s = sock(a.iface)
    print("开始记录（%.1f Hz）。拖滑块吧，Ctrl+C 结束。" % a.rate, flush=True)
    last_print = 0.0
    while not stop["v"]:
        for mid, n in zip(ids, names):
            v = read_mech(s, mid)
            if v is not None and math.isfinite(v):
                v = wrap(v)
                cur[n] = v
                lo[n] = min(lo[n], v)
                hi[n] = max(hi[n], v)
        now = time.monotonic()
        if now - last_print > 2.0:
            last_print = now
            line = "  ".join("%s %+0.3f [%+0.3f,%+0.3f]" % (n, cur[n] or 0.0, lo[n] if lo[n] != float("inf") else 0.0,
                                                            hi[n] if hi[n] != float("-inf") else 0.0)
                             for n in names)
            print(line, flush=True)
            data = {n: {"now": cur[n], "min": lo[n] if lo[n] != float("inf") else None,
                        "max": hi[n] if hi[n] != float("-inf") else None} for n in names}
            try:
                json.dump({"time": time.time(), "range": data}, open(a.out, "w"), indent=1)
            except OSError:
                pass
        time.sleep(max(0.0, 1.0 / a.rate - 0.05))

    data = {n: {"now": cur[n], "min": lo[n] if lo[n] != float("inf") else None,
                "max": hi[n] if hi[n] != float("-inf") else None} for n in names}
    try:
        json.dump({"time": time.time(), "range": data}, open(a.out, "w"), indent=1)
    except OSError:
        pass
    print("\n结束。最终范围：")
    for n in names:
        print("  %-7s min %+0.4f  max %+0.4f  (跨度 %.1f°)" % (
            n, lo[n] if lo[n] != float("inf") else 0, hi[n] if hi[n] != float("-inf") else 0,
            math.degrees((hi[n] if hi[n] != float("-inf") else 0) - (lo[n] if lo[n] != float("inf") else 0))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
