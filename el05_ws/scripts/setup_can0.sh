#!/usr/bin/env bash
set -euo pipefail

CAN_IFACE=can0
CAN_BITRATE=1000000

sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo ip link set "$CAN_IFACE" down || true
sudo ip link set "$CAN_IFACE" type can bitrate "$CAN_BITRATE"
sudo ip link set "$CAN_IFACE" up
sudo ip link set "$CAN_IFACE" txqueuelen 100
ip -details link show "$CAN_IFACE"
