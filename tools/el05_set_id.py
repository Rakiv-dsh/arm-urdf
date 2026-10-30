#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""el05_set_id.py —— 改 EL05 的 CAN ID（等价于上位机里的"设置设备 ID"），不需要 Windows 工具。

协议（私有协议，29 位扩展帧，来自 rs_motor_ros2 的 motor_cfg.cpp）：
  停止（先失能）  0x04<<24 | master<<8 | old_id
  设置 CAN ID     0x07<<24 | new_id<<16 | master<<8 | old_id     data 全 0
  读参数          0x11<<24 | master<<8 | id                      data[0:2]=索引

用法：
  python3 el05_set_id.py --iface can0 --list                  # 只读：扫 1~127 看谁在线
  python3 el05_set_id.py --iface can0 --from 127 --to 1       # 把 127 改成 1
  python3 el05_set_id.py --iface can0 --from 127 --to 1 --dry # 只看会不会动，不改

注意：改 ID 时**总线上只接这一台电机**，否则两台同 ID 会一起应答，改完分不清。
"""
import argparse, socket, struct, sys, time

EFF = 0x80000000
T_GETID, T_STOP, T_SETID, T_GETPARAM = 0x00, 0x04, 0x07, 0x11
IDX_MECHPOS, IDX_VBUS = 0x7019, 0x701C


def sock_open(iface):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        s.bind((iface,))
    except OSError as e:
        sys.exit("绑定 %s 失败：%s" % (iface, e))
    s.settimeout(0.05)
    return s


def send(s, cand_id, data):
    # 死总线（没人 ACK）时驱动队列会顶满，send 抛 ENOBUFS；稍等重试一次即可
    fr = struct.pack("=IB3x8s", cand_id | EFF, 8, bytes(data))
    for attempt in range(3):
        try:
            s.send(fr)
            return True
        except OSError as e:
            if attempt == 2:
                return False
            time.sleep(0.01 + 0.02 * attempt)


def wait_reply(s, mid, index=None, timeout=0.15):
    """等 motor=mid 的回包；index 给了就只认 type=0x11 且索引匹配的回包。"""
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        try:
            fr = s.recv(16)
        except socket.timeout:
            continue
        if len(fr) < 16:
            continue
        cid, dlc = struct.unpack("=IB", fr[:5])
        if not cid & EFF:
            continue
        c = cid & 0x1FFFFFFF
        if ((c >> 8) & 0xFF) != mid:
            continue
        ctype = (c >> 24) & 0x1F
        d = fr[8:8 + min(dlc, 8)]
        if index is None:
            return (ctype, d)
        if ctype == T_GETPARAM and (d[0] | (d[1] << 8)) == index:
            return struct.unpack("<f", d[4:8])[0]
    return None


def probe(s, master, mid, index=IDX_MECHPOS, timeout=0.15):
    send(s, (T_GETPARAM << 24) | (master << 8) | mid, struct.pack("<H", index) + b"\x00" * 6)
    return wait_reply(s, mid, index, timeout)


def count_nodes(s, master, mid, rounds=5, timeout=0.2):
    """数一数总线上到底有几台电机在用同一个 ID 应答（靠各自的位置值区分）。"""
    seen = set()
    for _ in range(rounds):
        while True:
            try:
                s.recv(16)
            except socket.timeout:
                break
            except BlockingIOError:
                break
        send(s, (T_GETPARAM << 24) | (master << 8) | mid, struct.pack("<H", IDX_MECHPOS) + b"\x00" * 6)
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            try:
                fr = s.recv(16)
            except socket.timeout:
                continue
            if len(fr) < 16:
                continue
            cid, dlc = struct.unpack("=IB", fr[:5])
            if not cid & EFF:
                continue
            c = cid & 0x1FFFFFFF
            if ((c >> 24) & 0x1F) != T_GETPARAM or ((c >> 8) & 0xFF) != mid:
                continue
            d = fr[8:8 + min(dlc, 8)]
            if (d[0] | (d[1] << 8)) == IDX_MECHPOS:
                seen.add(round(struct.unpack("<f", d[4:8])[0], 3))
    return len(seen), sorted(seen)


def scan(s, master, lo=1, hi=127, timeout=0.06):
    found = []
    for mid in range(lo, hi + 1):
        v = probe(s, master, mid, IDX_VBUS, timeout)
        if v is not None:
            pos = probe(s, master, mid, IDX_MECHPOS, timeout)
            found.append((mid, pos, v))
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--master", type=lambda x: int(x, 0), default=0xFD)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--from", dest="old", type=int, default=None, help="当前 CAN ID（例如 127）")
    ap.add_argument("--to", dest="new", type=int, default=None, help="目标 CAN ID（1~5）")
    ap.add_argument("--dry", action="store_true", help="只检查，不发改 ID 帧")
    ap.add_argument("--force", action="store_true", help="明知总线上有多台同 ID 也照改（危险）")
    a = ap.parse_args()

    s = sock_open(a.iface)
    print("== %s（主机ID 0x%02X）==" % (a.iface, a.master))

    if a.list or a.old is None:
        t0 = time.monotonic()
        found = scan(s, a.master)
        print("扫描 1~127 用时 %.1f s，在线 %d 台：" % (time.monotonic() - t0, len(found)))
        for mid, pos, vbus in found:
            print("   ID %-3d 机械角=%s rad  母线=%s V" % (
                mid, ("%.4f" % pos) if pos is not None else "?", ("%.1f" % vbus) if vbus is not None else "?"))
        if not found:
            print("   （一台都没有：电机没上电 / 线没接好 / 波特率不是 1 Mbps）")
        return 0 if found else 1

    if a.new is None:
        sys.exit("要改 ID 就同时给 --from 和 --to")

    v = probe(s, a.master, a.old, IDX_VBUS)
    if v is None:
        sys.exit("ID %d 没有应答：先确认这台电机是通的（可以先 --list 扫一遍）" % a.old)
    print("目标电机 ID %d：在线，母线 %.1f V" % (a.old, v))

    others = [m for m, _, _ in scan(s, a.master) if m != a.old]
    if others:
        print("!! 总线上还有其它电机 %s —— 改 ID 时建议只留这一台，否则可能一起被改" % others)

    n, pos_list = count_nodes(s, a.master, a.old)
    if n > 1 and not a.force:
        print("!! 总线上有 %d 台电机都在用 ID %d 应答（位置 %s）" % (n, a.old, pos_list))
        print("   这种情况下发改 ID 帧会把这 %d 台一起改掉，分不清谁是谁。" % n)
        print("   先隔离成：一次只通电一台 / 只接一台 CAN，再来改；确实要一起改就加 --force。")
        return 3
    print("同 ID 应答节点数：%d %s" % (n, pos_list))
    print("步骤：失能(0x04) → 设新 ID(0x07: %d → %d)" % (a.old, a.new))
    if a.dry:
        print("--dry：到这里停止，没有发任何帧")
        return 0

    send(s, (T_STOP << 24) | (a.master << 8) | a.old, b"\x00" * 8)
    wait_reply(s, a.old, None, 0.2)
    time.sleep(0.05)

    send(s, (T_SETID << 24) | ((a.new & 0xFF) << 16) | (a.master << 8) | a.old, b"\x00" * 8)
    time.sleep(0.2)

    v_new = probe(s, a.master, a.new, IDX_VBUS, 0.3)
    v_old = probe(s, a.master, a.old, IDX_VBUS, 0.15)
    print("验证：新 ID %d %s ；旧 ID %d %s" % (
        a.new,
        ("有应答（母线 %.1f V）✔" % v_new) if v_new is not None else "无应答 ✘",
        a.old,
        "仍在应答 ✘（可能没改成功）" if v_old is not None else "已静默 ✔"))
    return 0 if v_new is not None else 2


if __name__ == "__main__":
    sys.exit(main())
