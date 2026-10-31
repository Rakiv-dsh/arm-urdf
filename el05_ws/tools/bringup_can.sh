#!/usr/bin/env bash
# bringup_can.sh —— WSL 内：加载 gs_usb，把出现的 canX 全部 1 Mbps up，并逐路只读探测电机
set -u
for m in can can_raw can_dev can_bcm gs_usb; do modprobe "$m" 2>/dev/null; done
echo "[1/3] gs_usb 模块数: $(lsmod | grep -c gs_usb)"

IFS=$'\n' read -r -d '' -a ifs < <(ip -br link show type can 2>/dev/null | awk '{print $1}' && printf '\0')
if [ ${#ifs[@]} -eq 0 ]; then
  echo "!! 没有 CAN 接口。检查："
  echo "   Windows: usbipd list   （STATE 应为 Attached）"
  echo "   WSL    : lsusb | grep -i 1d50 ; dmesg | tail -20"
  exit 1
fi
echo "[2/3] 找到 CAN 接口: ${ifs[*]}"
for i in "${ifs[@]}"; do
  ip link set "$i" down 2>/dev/null
  ip link set "$i" type can bitrate 1000000 2>/dev/null || echo "   ($i 设 bitrate 失败)"
  ip link set "$i" up 2>/dev/null
  ip link set "$i" txqueuelen 100 2>/dev/null
done
ip -details link show type can | grep -E "bitrate" | sed 's/^/   /'

echo "[3/3] 逐路只读探测 CAN ID 1~5（不发使能帧；没接电机的路会全无应答）"
PY=$(dirname "$0")/el05_probe.py
for i in "${ifs[@]}"; do
  echo "----- $i -----"
  python3 "$PY" --iface "$i" --timeout 0.12 2>&1 | sed 's/^/  /'
done
