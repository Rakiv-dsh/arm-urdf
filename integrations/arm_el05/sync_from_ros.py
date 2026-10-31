#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sync_from_ros.py —— 从 ROS 侧的 joint_limits.yaml 生成上位机包的 manifest/calibration。

数据源唯一：arm_ws/src/arm_urdf/config/joint_limits.yaml
用法：python3 sync_from_ros.py [--cfg 路径]
"""
import argparse
import json
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CFG = os.path.normpath(os.path.join(
    HERE, "..", "..", "arm_ws", "src", "arm_urdf", "config", "joint_limits.yaml"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", default=DEFAULT_CFG)
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.cfg, encoding="utf-8"))
    joints = [j for j in cfg["joints"] if j.get("motor")]

    manifest_path = os.path.join(HERE, "manifest.json")
    calib_path = os.path.join(HERE, "calibration.json")
    manifest = json.load(open(manifest_path, encoding="utf-8"))
    calib = json.load(open(calib_path, encoding="utf-8"))

    for j, item in zip(joints, calib["joints"]):
        m = j["motor"]
        item["motor_id"] = int(m["can_id"])
        item["min_angle"] = round(math.degrees(j["lower"]), 2)
        item["max_angle"] = round(math.degrees(j["upper"]), 2)
        item["default_angle"] = 0.0
        item["direction"] = float(m["dir"])
        item["zero_offset_rad"] = float(m["offset"])

    fingers = manifest["fingers"][0]["joints"]
    for j, item in zip(joints, fingers):
        m = j["motor"]
        item["motor_id"] = int(m["can_id"])
        item["min_angle"] = round(math.degrees(j["lower"]), 2)
        item["max_angle"] = round(math.degrees(j["upper"]), 2)
        item["default_angle"] = 0.0

    json.dump(manifest, open(manifest_path, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
    json.dump(calib, open(calib_path, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
    print("已刷新 %s 与 %s（%d 个关节）" % (
        os.path.basename(manifest_path), os.path.basename(calib_path), len(joints)))
    for j, item in zip(joints, fingers):
        print("  %-8s motor %-3d  %7.2f° .. %7.2f°" % (
            item["name"], item["motor_id"], item["min_angle"], item["max_angle"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
