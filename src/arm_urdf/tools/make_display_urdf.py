#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_display_urdf.py —— 生成一份"只用于看/拖滑块"的 URDF：把所有关节限位放宽。

真机控制仍用 arm_urdf.urdf（限位来自 joint_limits.yaml，是真的）；
这份 arm_display.urdf 只给 RViz + 滑块用，让模型能拖到 ±180° 这种范围。

用法：python3 tools/make_display_urdf.py [--limit-deg 180]
"""
import argparse, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(PKG, "urdf", "arm_urdf.urdf"))
    ap.add_argument("--dst", default=os.path.join(PKG, "urdf", "arm_display.urdf"))
    ap.add_argument("--limit-deg", type=float, default=180.0)
    a = ap.parse_args()

    lim = a.limit_deg * 3.141592653589793 / 180.0
    text = open(a.src, encoding="utf-8").read()

    def repl(m):
        tag = m.group(0)
        tag = re.sub(r'lower="[-\d.]+"', 'lower="%.4f"' % (-lim), tag)
        tag = re.sub(r'upper="[-\d.]+"', 'upper="%.4f"' % lim, tag)
        return tag

    new, n = re.subn(r"<limit\b[^>]*/>", repl, text)
    open(a.dst, "w", encoding="utf-8").write(new)
    print("已生成 %s（%d 个关节限位放宽到 ±%.0f°）" % (os.path.basename(a.dst), n, a.limit_deg))
    return 0


if __name__ == "__main__":
    sys.exit(main())
