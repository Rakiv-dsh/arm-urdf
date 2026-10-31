#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sync_limits.py —— 一份配置，同时管 URDF 限位和真机限位。

数据源：config/joint_limits.yaml
产出：
  1. URDF（urdf/arm_urdf.urdf）里每个 joint 的 <limit lower upper effort velocity>
  2. 控制器参数（../el05_serial_ws/src/robstride_ros_sample/config/five_dof_controller.yaml）
     的 motor_ids / mech_offset / joint_direction / lower_limit / upper_limit

用法：
  python3 sync_limits.py                 # 按配置改写两个文件，并打印对照表
  python3 sync_limits.py --check         # 只打印对照表，不改文件
  python3 sync_limits.py --urdf X --ctrl Y --cfg Z
"""
import argparse
import math
import os
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
DEFAULT_CFG = os.path.join(PKG, "config", "joint_limits.yaml")
DEFAULT_URDF = os.path.join(PKG, "urdf", "arm_urdf.urdf")
def _find_ctrl():
    cands = [
        os.path.expanduser("~/el05_serial_ws/src/robstride_ros_sample/config/five_dof_controller.yaml"),
        "/home/raki/el05_serial_ws/src/robstride_ros_sample/config/five_dof_controller.yaml",
        os.path.join(PKG, "..", "..", "el05_serial_ws", "src",
                     "robstride_ros_sample", "config", "five_dof_controller.yaml"),
    ]
    for c in cands:
        if os.path.exists(c):
            return os.path.abspath(c)
    return cands[0]


DEFAULT_CTRL = _find_ctrl()
MECH_LIMIT = 4 * math.pi  # 电机 0x7019 机械角的量程上限（±4π）


def deg(x):
    return "%.1f°" % math.degrees(x)


def update_urdf(path, joints, dry):
    text = open(path, encoding="utf-8").read()
    report = []
    for j in joints:
        name = j["name"]
        pat = re.compile(
            r'(<joint\b[^>]*\bname="%s"[^>]*>)(.*?)(</joint>)' % re.escape(name), re.S
        )
        m = pat.search(text)
        if not m:
            report.append((name, "URDF 里找不到这个 joint", False))
            continue
        body = m.group(2)
        limit = '<limit lower="%.4f" upper="%.4f" effort="%.1f" velocity="%.1f"/>' % (
            j["lower"], j["upper"], j.get("effort", 6.0), j.get("velocity", 10.0)
        )
        if re.search(r"<limit\b[^>]*/>", body, re.S):
            body_new = re.sub(r"<limit\b[^>]*/>", limit, body, count=1, flags=re.S)
        else:
            body_new = body.rstrip() + "\n    " + limit + "\n  "
        text = text[: m.start(2)] + body_new + text[m.end(2):]
        report.append((name, "已写入", True))
    if not dry:
        open(path, "w", encoding="utf-8").write(text)
    return report


def read_ctrl_yaml(path):
    txt = open(path, encoding="utf-8").read()
    data = yaml.safe_load(txt)
    node = list(data.keys())[0]
    return txt, node, data[node]["ros__parameters"]


def fmt_list(vals, fmt):
    return "[" + ", ".join(fmt % v for v in vals) + "]"


def update_ctrl_yaml(path, joints, dry):
    txt, node, params = read_ctrl_yaml(path)
    motors = [j["motor"] for j in joints if j.get("motor")]
    wanted = {
        "motor_ids": fmt_list([m["can_id"] for m in motors], "%d"),
        "mech_offset": fmt_list([m["offset"] for m in motors], "%.6f"),
        "joint_direction": fmt_list([m["dir"] for m in motors], "%.1f"),
        "gear_ratio": fmt_list([m["gear"] for m in motors], "%.3f"),
        "lower_limit": fmt_list([j["lower"] for j in joints if j.get("motor")], "%.4f"),
        "upper_limit": fmt_list([j["upper"] for j in joints if j.get("motor")], "%.4f"),
    }
    out = []
    seen = set()
    for line in txt.splitlines():
        m = re.match(r"^(\s*)([a-z_]+):(\s*)(.*)$", line)
        if m and m.group(2) in wanted:
            key = m.group(2)
            out.append("%s%s:%s%s" % (m.group(1), key, m.group(3) or " ", wanted[key]))
            seen.add(key)
        else:
            out.append(line)
    for key, val in wanted.items():
        if key not in seen:
            out.append("    %s: %s" % (key, val))
    if not dry:
        open(path, "w", encoding="utf-8").write("\n".join(out) + "\n")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", default=DEFAULT_CFG)
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--ctrl", default=DEFAULT_CTRL)
    ap.add_argument("--check", action="store_true", help="只打印，不改文件")
    a = ap.parse_args()

    cfg = yaml.safe_load(open(a.cfg, encoding="utf-8"))
    joints = cfg["joints"]

    urdf_report = update_urdf(a.urdf, joints, a.check)
    update_ctrl_yaml(a.ctrl, joints, a.check)

    print("=" * 108)
    print("%-9s %-22s %-6s %-5s %-9s %-24s %s" %
          ("joint", "仿真限位 (URDF)", "电机", "方向", "偏置", "电机侧 mechPos 范围", "检查"))
    print("-" * 108)
    ok = True
    for j, (name, msg, found) in zip(joints, urdf_report):
        m = j.get("motor")
        if not found:
            print("%-9s %-22s %s" % (name, "—", msg))
            ok = False
            continue
        if not m:
            print("%-9s %-22s %-6s" % (name, "%s .. %s" % (deg(j["lower"]), deg(j["upper"])), "无"))
            continue
        lo_m = m["offset"] + m["dir"] * m["gear"] * j["lower"]
        hi_m = m["offset"] + m["dir"] * m["gear"] * j["upper"]
        lo_m, hi_m = min(lo_m, hi_m), max(lo_m, hi_m)
        warn = []
        if hi_m > MECH_LIMIT or lo_m < -MECH_LIMIT:
            warn.append("超出电机 ±%.0f°" % math.degrees(MECH_LIMIT))
            ok = False
        if hi_m - lo_m > 2 * MECH_LIMIT:
            warn.append("行程超过一圈")
            ok = False
        print("%-9s %-22s %-6d %-5s %-9.3f %-24s %s" % (
            name,
            "%s .. %s" % (deg(j["lower"]), deg(j["upper"])),
            m["can_id"], m["dir"],
            m["offset"],
            "%s .. %s" % (deg(lo_m), deg(hi_m)),
            "、".join(warn) if warn else "OK",
        ))
    print("=" * 108)
    print("配置：%s" % a.cfg)
    print("URDF：%s" % a.urdf)
    print("控制器：%s" % a.ctrl)
    if a.check:
        print("\n[--check] 只打印，没有写任何文件")
    else:
        print("\n已写入上面两个文件。URDF 在 install 里是符号链接，改完直接生效（不用重新 colcon build）。")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())