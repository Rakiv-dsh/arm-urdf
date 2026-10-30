#!/usr/bin/env python3
"""el05_vcan_sim.py —— 虚拟 RobStride EL05 电机（跑在 SocketCAN 虚拟总线上，无需硬件）

用途：在没有 USB-CANHUB / 电机的情况下，把 ROS 侧「使能 → 设模式 → 下发位置 → 收反馈」
整条链路跑通。它绑一个 CAN 接口（默认 can0，可先建 vcan），对主机发来的每一帧按
EL05 协议回一帧，并按一段简单的一阶跟随动力学更新「电机」位置。

协议要点（来自 rs_motor_ros2 的 motor_cfg.cpp）：
  主机 -> 电机  id = (comm_type << 24) | (master_id << 8) | motor_id      低字节 = 电机号
  电机 -> 主机  id = (comm_type << 24) | (err<<16) | (motor_id << 8) | 0xFD  低字节 = 主机号
  comm_type: 0x00 取ID / 0x01 运控 / 0x02 反馈 / 0x03 使能 / 0x04 停止
             0x06 置零 / 0x07 改ID / 0x11 读参数 / 0x12 写参数 / 0x15 故障
  反馈数据（大端 4×u16）：位置、速度、扭矩（均为 (x/32767-1)*满量程）、温度(x*0.1℃)
  读参数回包（comm_type 0x11）：data[0:2]=索引(小端), data[4:8]=float(小端)
"""
import argparse, math, os, signal, socket, struct, sys, time

CAN_EFF_FLAG = 0x80000000
T_GET_ID, T_MOTION, T_FEEDBACK, T_ENABLE, T_STOP = 0x00, 0x01, 0x02, 0x03, 0x04
T_SETZERO, T_SETID, T_GETPARAM, T_SETPARAM, T_FAULT = 0x06, 0x07, 0x11, 0x12, 0x15

# EL05 按 ROBSTRIDE_00 量程（motor_cfg.h 的 ACTUATOR_OPERATION_MAPPING[0]）
POS_MAX, VEL_MAX, TQ_MAX, KP_MAX, KD_MAX = 4 * math.pi, 50.0, 17.0, 500.0, 5.0
IDX_RUN_MODE, IDX_IQ_REF, IDX_SPD_REF, IDX_LOC_REF, IDX_LIMIT_SPD = 0x7005, 0x7006, 0x700A, 0x7016, 0x7017
IDX_MECHPOS, IDX_IQF, IDX_MECHVEL, IDX_VBUS, IDX_ROTATION = 0x7019, 0x701A, 0x701B, 0x701C, 0x701D


def u2f(raw, lo, hi):
    # 真机：u16 = (x - lo) / (hi - lo) * 65535
    return raw / 65535.0 * (hi - lo) + lo


def f2u(x, lo, hi):
    v = (x - lo) / (hi - lo) * 65535.0
    return max(0, min(65535, int(round(v))))


class Motor:
    def __init__(self, mid, pos0=0.0):
        self.id = mid
        self.pos = pos0          # 机械角 rad（负载端）
        self.vel = 0.0
        self.torque = 0.0
        self.temp = 28.0
        self.target = pos0
        self.engaged = False
        self.mode = 0            # 0 运控 1 PP 2 速度 3 电流 4 零点 5 CSP
        self.limit_spd = 0.1     # rad/s，0x7017
        self.iq = 0.0
        self.vbus = 48.0
        self.frames = 0

    def step(self, dt):
        if not self.engaged:
            self.vel = 0.0
            self.torque = 0.0
            return
        vmax = max(self.limit_spd, 0.0)
        if self.mode == 2:                       # 速度模式：目标就是速度
            self.vel = self.target
            self.pos += self.vel * dt
        else:
            err = self.target - self.pos
            step = max(-vmax * dt, min(vmax * dt, err))
            self.pos += step
            self.vel = step / dt if dt > 0 else 0.0
        self.pos = max(-POS_MAX, min(POS_MAX, self.pos))
        self.torque = 0.0


class Sim:
    def __init__(self, iface, master, motors, quiet=False):
        self.master, self.motors, self.quiet = master, {m.id: m for m in motors}, quiet
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.settimeout(0.002)
        self.iface = iface

    def send(self, cand_id, data):
        fr = struct.pack("=IB3x8s", cand_id | CAN_EFF_FLAG, len(data), bytes(data))
        self.sock.send(fr)

    def reply_feedback(self, m):
        data = struct.pack(">HHHH",
                           f2u(m.pos, -POS_MAX, POS_MAX),
                           f2u(m.vel, -VEL_MAX, VEL_MAX),
                           f2u(m.torque, -TQ_MAX, TQ_MAX),
                           int(m.temp * 10))
        self.send((T_FEEDBACK << 24) | (m.id << 8) | (self.master & 0xFF), data)

    def reply_param(self, m, index):
        if index == IDX_RUN_MODE:
            val = bytes([m.mode, 0, 0, 0])
        elif index == IDX_MECHPOS:
            val = struct.pack("<f", m.pos)
        elif index == IDX_IQF:
            val = struct.pack("<f", m.iq)
        elif index == IDX_MECHVEL:
            val = struct.pack("<f", m.vel)
        elif index == IDX_VBUS:
            val = struct.pack("<f", m.vbus)
        elif index == IDX_LIMIT_SPD:
            val = struct.pack("<f", m.limit_spd)
        elif index == IDX_LOC_REF:
            val = struct.pack("<f", m.target)
        elif index == IDX_ROTATION:
            val = struct.pack("<h", int(m.pos / (2 * math.pi)))
        else:
            val = struct.pack("<f", 0.0)
        data = struct.pack("<H", index & 0xFFFF) + b"\x00\x00" + val
        data = (data + b"\x00" * 8)[:8]
        self.send((T_GETPARAM << 24) | (m.id << 8) | (self.master & 0xFF), data)

    def handle(self, cand_id, data):
        if not cand_id & CAN_EFF_FLAG:
            return
        cid = cand_id & 0x1FFFFFFF
        if (cid & 0xFF) == (self.master & 0xFF):     # 自己发出的回包，忽略
            return
        mid = cid & 0xFF
        ctype = (cid >> 24) & 0x1F
        m = self.motors.get(mid)
        if m is None:
            return
        m.frames += 1
        d = bytes(data)
        if ctype == T_ENABLE:
            m.engaged = True
        elif ctype == T_STOP:
            m.engaged = False
            m.vel = 0.0
        elif ctype == T_MOTION:
            pos_u = (d[0] << 8) | d[1]
            vel_u = (d[2] << 8) | d[3]
            tq_u = (cid >> 8) & 0xFFFF
            m.target = u2f(pos_u, -POS_MAX, POS_MAX)
            m.limit_spd = max(u2f(vel_u, -VEL_MAX, VEL_MAX), 0.001)
            m.torque = u2f(tq_u, -TQ_MAX, TQ_MAX)
        elif ctype == T_SETID:            # 改 ID：新 ID 在扩展帧 bit16~23
            new_id = (cid >> 16) & 0xFF
            if new_id and new_id != m.id:
                del self.motors[m.id]
                m.id = new_id
                self.motors[new_id] = m
            self.reply_feedback(m)
            return
        elif ctype == T_SETZERO:
            m.pos = 0.0
            m.target = 0.0
        elif ctype == T_SETPARAM:
            idx = d[0] | (d[1] << 8)
            if idx == IDX_RUN_MODE:
                m.mode = d[4]
            elif idx == IDX_LOC_REF:
                m.target = struct.unpack("<f", d[4:8])[0]
                if m.mode not in (1, 5):
                    m.mode = 5
            elif idx == IDX_LIMIT_SPD:
                m.limit_spd = struct.unpack("<f", d[4:8])[0]
            elif idx == IDX_SPD_REF:
                m.target = struct.unpack("<f", d[4:8])[0]
            elif idx == IDX_IQ_REF:
                m.iq = struct.unpack("<f", d[4:8])[0]
        elif ctype == T_GETPARAM:
            idx = d[0] | (d[1] << 8)
            if not self.quiet:
                sys.stderr.write("[sim] get param 0x%04X motor %d\n" % (idx, mid))
                sys.stderr.flush()
            self.reply_param(m, idx)
            return
        # 其它命令（使能/停止/写参数/运控）一律回一帧状态反馈，与真机一致
        self.reply_feedback(m)

    def run(self, duration=None, csv_path=None):
        csv_fh = open(csv_path, "w", encoding="utf-8") if csv_path else None
        if csv_fh:
            csv_fh.write("t,motor_id,engaged,mode,target,pos,vel,limit_spd\n")
        def _bye(signum, frame):
            raise SystemExit(0)
        signal.signal(signal.SIGTERM, _bye)
        t0 = last = time.monotonic()
        if not self.quiet:
            sys.stderr.write("[sim] %s 上虚拟电机已就位：%s（主机ID 0x%02X）\n"
                             % (self.iface, sorted(self.motors), self.master))
            sys.stderr.flush()
        while True:
            now = time.monotonic()
            if duration and now - t0 > duration:
                break
            try:
                frame = self.sock.recv(16)
            except socket.timeout:
                frame = None
            dt = min(now - last, 0.05)
            last = now
            for m in self.motors.values():
                m.step(dt)
                if csv_fh:
                    csv_fh.write("%.4f,%d,%d,%d,%.6f,%.6f,%.6f,%.6f\n" % (
                        now - t0, m.id, 1 if m.engaged else 0, m.mode,
                        m.target, m.pos, m.vel, m.limit_spd))
            if frame and len(frame) >= 16:
                cand_id, dlc = struct.unpack("=IB", frame[:5])
                self.handle(cand_id, frame[8:8 + min(dlc, 8)])
            if csv_fh and int((now - t0) * 50) != int((last - t0) * 50):
                csv_fh.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--master", type=lambda s: int(s, 0), default=0xFD)
    ap.add_argument("--ids", default="1,2,3,4,5")
    ap.add_argument("--pos", default="", help="逗号分隔的初始位置(rad)，默认 0,0.26,0,4.24,0")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--csv", default=None, help="把每个电机的位置/目标写成 csv")
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",") if x.strip()]
    pos0 = [float(x) for x in a.pos.split(",")] if a.pos else [0.0, 0.265762, 0.0, 4.244908, 0.0]
    motors = [Motor(i, pos0[k] if k < len(pos0) else 0.0) for k, i in enumerate(ids)]
    sim = Sim(a.iface, a.master, motors, a.quiet)
    try:
        sim.run(a.duration, a.csv)
    except KeyboardInterrupt:
        pass
    finally:
        if not a.quiet:
            sys.stderr.write("[sim] 收到帧数：%s\n" % {m.id: m.frames for m in motors})
            sys.stderr.write("[sim] 结束时位置：%s\n" % {m.id: round(m.pos, 4) for m in motors})


if __name__ == "__main__":
    main()