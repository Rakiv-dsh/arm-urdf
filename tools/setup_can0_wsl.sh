#!/usr/bin/env bash
# setup_can0_wsl.sh —— 在 WSL 里把 USB-CANHUB 拉成 SocketCAN can0（1 Mbps）
# 前提：usbipd-win 已把 USB 设备 attach 进 WSL，内核里 gs_usb 已加载。
set -u
IFACE="${1:-can0}"
for m in can can_raw can_dev can_bcm gs_usb slcan vcan; do modprobe "$m" 2>/dev/null; done
echo "[1/4] 已加载 CAN 内核模块: $(lsmod | grep -E '^(gs_usb|can|can_raw|slcan|vcan)' | awk '{print $1}' | tr '\n' ' ')"
echo "[2/4] 当前 CAN 接口:"; ip -br link show type can 2>/dev/null || echo "  （还没有 CAN 接口——USB 设备没进来）"
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "!! 没有 $IFACE。先看是不是 USB 没 attach："
  echo "   Windows 侧（管理员）：usbipd list  然后  usbipd attach --wsl --busid <BUSID>"
  echo "   WSL 侧：lsusb 应能看到 candleLight/gs_usb 设备，dmesg | tail 看 gs_usb 是否绑定"
  exit 1
fi
ip link set "$IFACE" down 2>/dev/null
ip link set "$IFACE" type can bitrate 1000000
ip link set "$IFACE" up
ip link set "$IFACE" txqueuelen 100
echo "[3/4] 已 up:"; ip -details link show "$IFACE" | head -4
echo "[4/4] 核对：用只读探针确认电机应答 → python3 tools/el05_probe.py --iface $IFACE"
