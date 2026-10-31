#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calibrate.py —— 零位与限位的实机标定工具（EL05 + 本仓库的 joint_limits.yaml）

子命令：
  show       只读：打印每台的 机械角 / 关节角 / 电流 / 母线，不做任何动作
  zero       把当前姿态设为关节角 0 点（写 offset，可逆）
  jog        让某个关节走一小步并保持几秒，然后松开（安全点动）
  record     把当前关节角记成该轴的 lower 或 upper（带安全余量），并同步到 URDF/控制器

用法示例：
  python3 calibrate.py show
  python3 calibrate.py jog    --joint joint2 --step 0.05 --hold 2
  python3 calibrate.py record --joint joint2 --which lower --margin 0.05
  python3 calibrate.py zero

安全：jog/record 都按 joint_limits.yaml 里的 current 限位夹一次，且默认电流限幅 6 A、速度 0.08 rad/s。
"""
import argparse
import os
import re
import socket
import struct
import subprocess
import sys
import time

import yaml

EFF = 0x80000000
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
CFG = os.path.join(PKG, "config", "joint_limits.yaml")
IDX_MECHPOS, IDX_IQF, IDX_VBUS = 0x7019, 0x701A, 0x701C


# ---------- CAN 基础 ----------
def _sock(iface, timeout=0.25):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(timeout)
    return s


def send(s, cid, data=b"\x00" * 8):
    s.send(struct.pack("=IB3x8s", cid | EFF, 8, bytes(data)))


def recv_frame(s, t_end, want=None):
    """收取到 t_end；want=(mid, index) 时只返回匹配的读数。"""
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
        if want is None:
            return (cid, dlc, fr[8:8 + dlc])
        mid, idx = want
        if ((c >> 24) & 0x1F) == 0x11 and ((c >> 8) & 0xFF) == mid:
            d = fr[8:8 + min(dlc, 8)]
            if (d[0] | (d[1] << 8)) == idx:
                return struct.unpack("<f", d[4:8])[0]
    return None


def read_param(s, mid, idx, timeout=0.25):
    send(s, (0x11 << 24) | (0xFD << 8) | mid, struct.pack("<H", idx) + b"\x00" * 6)
    return recv_frame(s, time.monotonic() + timeout, (mid, idx))


def write_param(s, mid, idx, value, timeout=0.25):
    send(s, (0x12 << 24) | (0xFD << 8) | mid, struct.pack("<H", idx) + struct.pack("<f", value))
    recv_frame(s, time.monotonic() + timeout)


# ---------- 配置 ----------
def load_cfg(path=CFG):
    return yaml.safe_load(open(path, encoding="utf-8"))


def joint_cfg(cfg, name):
    for j in cfg["joints"]:
        if j["name"] == name:
            return j
    raise SystemExit("配置里没有关节 %s" % name)


def to_joint(m, mech):
    return (mech - m["offset"]) / (m["dir"] * m["gear"])


def to_mech(m, q):
    return m["offset"] + m["dir"] * m["gear"] * q


# ---------- 子命令 ----------
def cmd_show(a):
    cfg = load_cfg()
    s = _sock(a.iface)
    print("%-9s %-6s %-11s %-11s %-9s %s" % ("joint", "电机", "机械角", "关节角", "电流A", "母线V"))
    for j in cfg["joints"]:
        m = j.get("motor")
        if not m:
            continue
        mid = int(m["can_id"])
        mech = read_param(s, mid, IDX_MECHPOS)
        iq = read_param(s, mid, IDX_IQF)
        vbus = read_param(s, mid, IDX_VBUS)
        if mech is None:
            print("  %-9s %-6d 读取超时" % (j["name"], mid))
            continue
        q = to_joint(m, mech)
        lim = "[%.3f, %.3f]" % (j["lower"], j["upper"])
        flag = "" if j["lower"] <= q <= j["upper"] else "  ← 超出限位"
        print("  %-9s %-6d %-11.4f %-11.4f %-9.2f %.1f   限位%s%s" % (
            j["name"], mid, mech, q,
            iq if iq is not None else float("nan"),
            vbus if vbus is not None else float("nan"), lim, flag))
    s.close()


def cmd_zero(a):
    sync = os.path.join(HERE, "set_zero_here.py")
    return subprocess.call([sys.executable, sync])


def cmd_jog(a):
    cfg = load_cfg()
    j = joint_cfg(cfg, a.joint)
    m = j.get("motor")
    if not m:
        raise SystemExit("%s 没有电机" % a.joint)
    mid = int(m["can_id"])
    s = _sock(a.iface)
    mech = read_param(s, mid, IDX_MECHPOS)
    if mech is None:
        raise SystemExit("读不到 %s 的位置，先确认电机在线" % a.joint)
    q_now = to_joint(m, mech)
    q_new = q_now + a.step
    if not a.ignore_limits and not (j["lower"] - 1e-6 <= q_new <= j["upper"] + 1e-6):
        raise SystemExit("目标 %.3f 超出限位 [%.3f, %.3f]；确认要越过就加 --ignore-limits"
                         % (q_new, j["lower"], j["upper"]))
    print(" %s: 当前关节角 %.4f -> 目标 %.4f（机械角 %.4f -> %.4f）" % (
        a.joint, q_now, q_new, mech, to_mech(m, q_new)))
    # 使能 + 限速/限流 + 切 CSP
    if a.current > 0:
        write_param(s, mid, 0x2019, a.current)
    send(s, (0x03 << 24) | (0xFD << 8) | mid)          # 使能
    recv_frame(s, time.monotonic() + 0.2)
    write_param(s, mid, 0x7005, 5.0)                    # CSP
    write_param(s, mid, 0x7017, 0.08)                   # 限速 rad/s
    target = to_mech(m, q_new)
    t_end = time.monotonic() + a.hold
    while time.monotonic() < t_end:
        write_param(s, mid, 0x7016, target, timeout=0.05)
        time.sleep(0.02)
    mech2 = read_param(s, mid, IDX_MECHPOS)
    send(s, (0x04 << 24) | (0xFD << 8) | mid)           # 失能
    recv_frame(s, time.monotonic() + 0.2)
    s.close()
    if mech2 is not None:
        print(" 走完：关节角 %.4f（目标 %.4f，误差 %.4f）" % (
            to_joint(m, mech2), q_new, to_joint(m, mech2) - q_new))


def cmd_record(a):
    cfg = load_cfg()
    j = joint_cfg(cfg, a.joint)
    m = j.get("motor")
    if not m:
        raise SystemExit("%s 没有电机" % a.joint)
    s = _sock(a.iface)
    mech = read_param(s, int(m["can_id"]), IDX_MECHPOS)
    s.close()
    if mech is None:
        raise SystemExit("读不到位置")
    q = to_joint(m, mech)
    margin = abs(a.margin)
    if a.which == "lower":
        val = q + margin       # 下限往里收
        old = j["lower"]
        j["lower"] = round(val, 4)
    else:
        val = q - margin
        old = j["upper"]
        j["upper"] = round(val, 4)
    print("%s: 当前关节角 %.4f，%s 由 %.4f 改成 %.4f（留 %.3f 余量）" % (
        a.joint, q, a.which, old, j["upper"] if a.which == "upper" else j["lower"], margin))
    if a.dry:
        print("[--dry] 没有写文件")
        return 0
    text = open(CFG, encoding="utf-8").read()
    # 只替换该关节块里的 lower/upper
    pat = re.compile(r"(- name: %s\b.*?)(\n  - name: |\Z)" % re.escape(a.joint), re.S)
    mm = pat.search(text)
    if not mm:
        raise SystemExit("配置里找不到 %s 的块" % a.joint)
    block = mm.group(1)
    nb = re.sub(r"lower: [-\d.]+", "lower: %.4f" % j["lower"], block)
    nb = re.sub(r"upper: [-\d.]+", "upper: %.4f" % j["upper"], nb)
    text = text[:mm.start(1)] + nb + text[mm.end(1):]
    open(CFG, "w", encoding="utf-8").write(text)
    print("已写入 %s" % CFG)
    return subprocess.call([sys.executable, os.path.join(HERE, "sync_limits.py")])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("show")

    sub.add_parser("zero")

    p = sub.add_parser("jog")
    p.add_argument("--joint", required=True)
    p.add_argument("--step", type=float, default=0.05, help="关节角增量 rad")
    p.add_argument("--hold", type=float, default=2.0, help="保持时间 s")
    p.add_argument("--current", type=float, default=6.0, help="电流限幅 A")
    p.add_argument("--ignore-limits", action="store_true")

    p = sub.add_parser("record")
    p.add_argument("--joint", required=True)
    p.add_argument("--which", choices=["lower", "upper"], required=True)
    p.add_argument("--margin", type=float, default=0.05, help="安全余量 rad")
    p.add_argument("--dry", action="store_true")

    a = ap.parse_args()
    return {"show": cmd_show, "zero": cmd_zero, "jog": cmd_jog, "record": cmd_record}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
