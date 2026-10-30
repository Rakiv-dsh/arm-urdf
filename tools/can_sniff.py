#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""can_sniff.py —— 裸监听 + 不限 ID 的试探，用来判断 CAN 上到底有没有东西在说话。

第一阶段（被动）：不发任何帧，只把 5 路上的所有帧打出来 3 秒
第二阶段（试探）：对一批候选 ID 发「取设备ID(0x00)」和「读参数 0x7019」，**不过滤回包**，
                  任何回来的帧都打印出来（并按已知格式解读一次）
用法：python3 can_sniff.py              # can0~can4
      python3 can_sniff.py --ifaces can0
      python3 can_sniff.py --seconds 2 --ids 1,2,3,4,5,127
"""
import argparse, select, socket, struct, sys, time

EFF = 0x80000000


def open_if(iface):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.setblocking(False)
    return s


def fmt(cand_id, dlc, data):
    eff = bool(cand_id & EFF)
    cid = cand_id & 0x1FFFFFFF
    hexs = " ".join("%02X" % b for b in data[:dlc])
    tag = "EFF" if eff else "SFF"
    return "%-4s  0x%08X  [%d] %-24s  %s" % (tag, cid, dlc, hexs, decode(cid, data, dlc, eff))


def decode(cid, data, dlc, eff):
    if not eff:
        return "(标准帧)"
    ctype = (cid >> 24) & 0x1F
    mid = (cid >> 8) & 0xFF
    host = cid & 0xFF
    if ctype == 0x02 and dlc >= 8:
        pos = ((data[0] << 8 | data[1]) / 32767.0 - 1) * 4 * 3.14159265
        vel = ((data[2] << 8 | data[3]) / 32767.0 - 1) * 50
        tq = ((data[4] << 8 | data[5]) / 32767.0 - 1) * 17
        tmp = (data[6] << 8 | data[7]) * 0.1
        return "反馈 type2 motor=%d host=%d pos=%.4f vel=%.2f tq=%.2f T=%.1fC" % (mid, host, pos, vel, tq, tmp)
    return "type=0x%02X dev=%d host=%d" % (ctype, mid, host)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ifaces", default="can0,can1,can2,can3,can4")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--ids", default="1,2,3,4,5,127")
    a = ap.parse_args()
    ifaces = [x for x in a.ifaces.split(",") if x.strip()]
    socks = {}
    for i in ifaces:
        try:
            socks[i] = open_if(i)
        except OSError as e:
            print("!! %s 打不开: %s" % (i, e))
    print("== 第一阶段：被动监听 %.1f s（不发帧）==" % a.seconds)
    t_end = time.monotonic() + a.seconds
    total = 0
    while time.monotonic() < t_end:
        r, _, _ = select.select(list(socks.values()), [], [], 0.2)
        for s in r:
            name = [k for k, v in socks.items() if v is s][0]
            while True:
                try:
                    fr = s.recv(16)
                except BlockingIOError:
                    break
                if len(fr) < 16:
                    continue
                cid, dlc = struct.unpack("=IB", fr[:5])
                print("  %-6s %s" % (name, fmt(cid, dlc, fr[8:8 + dlc])))
                total += 1
    print("  被动阶段收到 %d 帧" % total)

    print("== 第二阶段：按候选 ID 发探测帧，打印所有回包 ==")
    ids = [int(x) for x in a.ids.split(",") if x.strip()]
    for iface, s in socks.items():
        for mid in ids:
            for ctype, label in ((0x00, "取ID"), (0x11, "读0x7019")):
                cand = (ctype << 24) | (0xFD << 8) | mid | EFF
                payload = struct.pack("<H", 0x7019) + b"\x00" * 6 if ctype == 0x11 else b"\x00" * 8
                s.send(struct.pack("=IB3x8s", cand, 8, payload))
                t1 = time.monotonic() + 0.12
                got = False
                while time.monotonic() < t1:
                    r, _, _ = select.select([s], [], [], 0.03)
                    if not r:
                        continue
                    fr = s.recv(16)
                    if len(fr) < 16:
                        continue
                    cid, dlc = struct.unpack("=IB", fr[:5])
                    print("  %-6s <- %s   (应答 %s/ID%d)" % (iface, fmt(cid, dlc, fr[8:8 + dlc]), label, mid))
                    got = True
                if not got:
                    print("  %-6s -> 发 %-8s ID %-3d 无任何回包" % (iface, label, mid))


if __name__ == "__main__":
    main()
