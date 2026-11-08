#!/bin/bash
# arm_up.sh —— 一键起 CAN 桥（后台运行，日志在 /tmp/arm_bridge.log）
#   ./arm_up.sh             真机
#   ./arm_up.sh --dry-run   假数据（调界面用）
#   ./arm_up.sh --port 5600 --watchdog 3
set -u
PORT=5599
WATCHDOG=2.0
DRY=""
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY="--dry-run" ;;
    --port) PORT="$2"; shift ;;
    --watchdog) WATCHDOG="$2"; shift ;;
    *) echo "未知参数 $1"; exit 2 ;;
  esac
  shift
done

DIR=$(cd "$(dirname "$0")" && pwd)
LOG=/tmp/arm_bridge.log

if ss -ltn 2>/dev/null | grep -q ":$PORT "; then
  echo "[arm_up] 端口 $PORT 已被占用，先跑 ./arm_down.sh"
  exit 1
fi

cd "$DIR"
nohup python3 can_bridge.py --port "$PORT" --watchdog "$WATCHDOG" $DRY > "$LOG" 2>&1 &
echo "[arm_up] 桥已启动 pid=$! 端口=$PORT 日志=$LOG"

for i in $(seq 1 20); do
  sleep 0.3
  if ss -ltn 2>/dev/null | grep -q ":$PORT "; then
    echo "[arm_up] 就绪"
    tail -3 "$LOG"
    exit 0
  fi
done
echo "[arm_up] 启动超时，看日志：$LOG"
tail -20 "$LOG"
exit 1
