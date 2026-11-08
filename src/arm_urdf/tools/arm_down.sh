#!/bin/bash
# arm_down.sh —— 停机 + 关桥（先发停机帧，再杀进程）
set -u
DIR=$(cd "$(dirname "$0")" && pwd)

echo "[arm_down] 广播停机帧 ..."
python3 - <<'EOS'
import glob, os, socket, struct, time
EFF = 0x80000000
def ifaces():
    return sorted(os.path.basename(p) for p in glob.glob("/sys/class/net/can*"))
sent = 0
for name in ifaces():
    try:
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((name,))
        for _ in range(2):
            for mid in range(21, 29):
                s.send(struct.pack("=IB3x8s", (0x04 << 24) | (0xFD << 8) | mid | EFF, 8, b"\x00" * 8))
                sent += 1
            time.sleep(0.02)
        s.close()
    except OSError:
        pass
print("  发出 %d 帧（%d 条总线）" % (sent, len(ifaces())))
EOS

echo "[arm_down] 关闭桥 ..."
pkill -f "can_bridge.py" 2>/dev/null && echo "  已关闭" || echo "  没有在跑"
