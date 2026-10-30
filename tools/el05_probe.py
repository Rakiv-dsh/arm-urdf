#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""el05_probe.py —— 只读扫描：确认 CAN 上哪些 EL05 在应答（不使能、不动电机）

用法：
    sudo ip link set can0 up type can bitrate 1000000
    python3 tools/el05_probe.py --iface can0            # 扫 ID 1~5
    python3 tools/el05_probe.py --iface can0 --ids 1,2  # 只扫指定 ID

它给每个 ID 发两帧「读参数」请求（0x7019 机械角、0x701C 母线电压），等 150 ms 回包。
回包里能读到位置/电压 = 电机在线且 CAN 通；读不到 = 没电、没接线、ID 不对或波特率不对。
注意：这里不发使能帧，电机不会动。
"""
import argparse, math, socket, struct, sys, time

EFF = 0x80000000
T_GETPARAM = 0x11
IDX_MECHPOS, IDX_VBUS = 0x7019, 0x701C


def make_socket(iface):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        s.bind((iface,))
    except OSError as e:
        sys.exit("绑定 %s 失败：%s（先确认接口存在且 up）" % (iface, e))
    s.settimeout(0.15)
    return s


def send_get(s, master, mid, index):
    cand_id = (T_GETPARAM << 24) | (master << 8) | mid | EFF
    data = struct.pack("<H", index) + b"\x00" * 6
    fr = struct.pack("=IB3x8s", cand_id, 8, data)
    for attempt in range(3):
        try:
            s.send(fr)
            return
        except OSError:
            if attempt == 2:
                return
            time.sleep(0.01 + 0.02 * attempt)


def drain(s, mid, index, timeout):
    """等一帧 comm_type=0x11 且 index 匹配的回包，返回 float；超时返回 None。"""
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        try:
            frame = s.recv(16)
        except socket.timeout:
            return None
        if len(frame) < 16:
            continue
        cand_id, dlc = struct.unpack("=IB", frame[:5])
        if not cand_id & EFF:
            continue
        cid = cand_id & 0x1FFFFFFF
        ctype = (cid >> 24) & 0x1F
        got_mid = (cid >> 8) & 0xFF
        if got_mid != mid:
            continue
        d = frame[8:8 + min(dlc, 8)]
        if ctype == T_GETPARAM:
            idx = d[0] | (d[1] << 8)
            if idx == index:
                return struct.unpack("<f", d[4:8])[0]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--master", type=lambda x: int(x, 0), default=0xFD)
    ap.add_argument("--ids", default="1,2,3,4,5")
    ap.add_argument("--timeout", type=float, default=0.15)
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",") if x.strip()]
    s = make_socket(a.iface)
    print("== 扫描 %s（主机ID 0x%02X），只读、不使能 ==" % (a.iface, a.master))
    online = []
    for mid in ids:
        send_get(s, a.master, mid, IDX_MECHPOS)
        pos = drain(s, mid, IDX_MECHPOS, a.timeout)
        send_get(s, a.master, mid, IDX_VBUS)
        vbus = drain(s, mid, IDX_VBUS, a.timeout)
        if pos is None and vbus is None:
            print("  ID %d : 无应答  <-- 断电/未接线/CAN ID 不对/波特率不对" % mid)
        else:
            online.append(mid)
            deg = math.degrees(pos) if pos is not None else float("nan")
            print("  ID %d : 在线   机械角 = %s rad (%s deg)   母线 = %s V" % (
                mid,
                ("%.4f" % pos) if pos is not None else "读取超时",
                ("%.1f" % deg) if pos is not None else "-",
                ("%.1f" % vbus) if vbus is not None else "读取超时"))
    print("== 在线电机：%s（共 %d 个） ==" % (online if online else "无", len(online)))
    if not online:
        print("排查顺序：① 电机是否上电（15~60V）② CANH/CANL 是否接反 ③ 终端电阻 120Ω "
              "④ 波特率是否 1 Mbps ⑤ 电机 CAN ID 是否 1~5（出厂可能是 127）")
    return 0 if online else 1


if __name__ == "__main__":
    sys.exit(main())
