#!/usr/bin/env bash
# demo_virtual.sh —— 没有硬件时的一条龙验证
#   vcan 建 can0  →  虚拟 EL05 ×5  →  five_dof_arm_controller  →  /arm/joint_states
# 用法（需要 root 建 vcan）：
#   sudo bash tools/demo_virtual.sh          # 默认 8 秒正弦
#   sudo bash tools/demo_virtual.sh 15       # 自定义时长
set -u
DUR="${1:-8}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="$WS/tools/el05_vcan_sim.py"
CSV=/tmp/el05_demo_trace.csv
LOG=/tmp/el05_demo_ctrl.log

cleanup() {
  pkill -f five_dof_arm_controller 2>/dev/null
  pkill -f el05_vcan_sim.py 2>/dev/null
  ip link set can0 down 2>/dev/null; ip link del can0 2>/dev/null
}
trap cleanup EXIT

as_user() { if [ "$(id -u)" = 0 ] && [ -n "${SUDO_USER:-}" ]; then sudo -u "$SUDO_USER" bash -lc "$1"; else bash -lc "$1"; fi; }

modprobe vcan 2>/dev/null
if ip link show can0 >/dev/null 2>&1; then
  if ip -d link show can0 | grep -q vcan; then ip link set can0 down; ip link del can0; else
    echo "!! can0 已存在且不是 vcan（可能是真硬件），本脚本不动它，退出。"; exit 1; fi
fi
ip link add dev can0 type vcan && ip link set can0 up || { echo "!! 建 can0(vcan) 失败"; exit 1; }
echo "[1/4] vcan can0 就绪"

python3 "$SIM" --iface can0 --quiet --csv "$CSV" >/dev/null 2>&1 &
sleep 0.5
echo "[2/4] 虚拟 EL05 ×5 已上线（CAN ID 1~5，主机 0xFD）"

as_user "source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash; exec ros2 run rs_motor_ros2 five_dof_arm_controller --ros-args --params-file $WS/src/robstride_ros_sample/config/five_dof_controller.yaml -p sine_amplitude:=0.30 -p sine_frequency:=0.1" >"$LOG" 2>&1 &
sleep 4
echo "[3/4] 控制器已启动，发 /arm/enable=true、/arm/sine_enable=true"
as_user "source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash;
  ros2 topic pub --once /arm/enable std_msgs/msg/Bool '{data: true}';
  ros2 topic pub --once /arm/sine_enable std_msgs/msg/Bool '{data: true}'" >/dev/null 2>&1

echo "[4/4] 采集 /arm/joint_states $DUR s ..."
as_user "source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash; timeout $DUR ros2 topic echo /arm/joint_states" > /tmp/el05_demo_js.log 2>&1
cleanup

python3 - "$LOG" <<'PY'
import re, sys
js = open("/tmp/el05_demo_js.log", encoding="utf-8", errors="replace").read()
fr = []
for b in js.split("---"):
    m = re.search(r"position:\s*\n((?:\s*-\s*[-0-9.eE]+\n?)+)", b)
    if m:
        v = [float(x) for x in re.findall(r"-\s*([-0-9.eE]+)", m.group(1))]
        if len(v) == 5: fr.append(v)
print("==== 结果 ====")
print("joint_states 帧数 :", len(fr))
if fr:
    rng = [max(f[i] for f in fr) - min(f[i] for f in fr) for i in range(5)]
    print("各关节摆幅(rad)  :", ", ".join("%.3f" % r for r in rng))
    print("首帧             :", ", ".join("%+.3f" % x for x in fr[0]))
    print("末帧             :", ", ".join("%+.3f" % x for x in fr[-1]))
log = open(sys.argv[1], encoding="utf-8", errors="replace").read()
err = [l for l in log.splitlines() if "ERROR" in l or "Safety" in l]
print("控制器报错       :", err[:3] if err else "无")
PY
echo "轨迹 CSV: $CSV"
