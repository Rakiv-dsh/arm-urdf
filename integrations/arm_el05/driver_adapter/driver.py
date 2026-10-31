"""EL05 机械臂的驱动适配器 —— 按 hand-host（dexhand.hand.v1）的契约实现。

上位机侧需要的接口（app/hands/README 里的契约）：
    connect(port, baudrate, protocol=None) -> bool
    disconnect()
    enable_all(enable=True)
    set_motor_angle(motor_id, angle_deg, speed=0.5)
    set_angles({motor_id: angle_deg})
    set_all_joint_angles([angle_deg, ...])
    home_all_motors(target_angles=None, speed=0.3)
    get_latest_telemetry() -> dict[int, dict]
    get_status(motor_id) -> dict
    cleanup()
可选：
    set_zero_point(motor_id)
    set_max_current(motor_id, value_q15)

本适配器把「角度」统一在**度**上（和 manifest 里的 min/max_angle 一致），
内部换算成弧度后走 EL05 私有协议（29 位扩展帧，主机 ID 0xFD）。

硬件前提：
    * Linux / WSL 下的 SocketCAN 接口（如 can0），1 Mbps，已 ip link up
    * 电机的 CAN ID 在 manifest 的 motor_id 里声明（当前 21~25）
    * 零点已经写进电机（0x7019 在零位读 0），所以不需要额外的 offset；
      如需要，calibration.json 里的 direction 用来翻转正方向。
"""

from __future__ import annotations

import math
import socket
import struct
import threading
import time
from typing import Dict, Optional

CAN_EFF_FLAG = 0x80000000

# 通信类型（私有协议）
T_MOTION = 0x01
T_FEEDBACK = 0x02
T_ENABLE = 0x03
T_STOP = 0x04
T_SET_ZERO = 0x06
T_SET_ID = 0x07
T_GET_PARAM = 0x11
T_SET_PARAM = 0x12
T_FAULT = 0x15

# 参数索引
IDX_RUN_MODE = 0x7005
IDX_IQ_REF = 0x7006
IDX_SPD_REF = 0x700A
IDX_LOC_REF = 0x7016
IDX_LIMIT_SPD = 0x7017
IDX_MECHPOS = 0x7019
IDX_IQF = 0x701A
IDX_MECHVEL = 0x701B
IDX_VBUS = 0x701C
IDX_LIMIT_CUR = 0x2019      # 功能码（掉电丢失）
IDX_FAULT_CODE = 0x3022

MODE_CSP = 5

POS_MAX_RAD, VEL_MAX, TQ_MAX = 4 * math.pi, 50.0, 17.0
FAULT_BITS = {
    0: "过温", 1: "驱动芯片故障", 2: "欠压", 3: "过压",
    4: "B相过流", 5: "C相过流", 7: "编码器未标定", 8: "硬件识别",
    9: "位置初始化", 14: "堵转过载", 16: "A相过流",
}


def _fmt_fault(v: int) -> str:
    if not v:
        return ""
    return "、".join(name for bit, name in FAULT_BITS.items() if v >> bit & 1) or hex(v)


def _u2f(raw: int, lo: float, hi: float) -> float:
    return (raw / 32767.0 - 1.0) * (hi - lo)


class HandDriver:
    """EL05 × N 的驱动适配器（N 由 manifest 的 motor_id 决定）。"""

    def __init__(self, profile):
        self.profile = profile
        self.config = dict(getattr(profile, "motor_adapter_config", {}) or {})
        self.iface = str(self.config.get("can_interface", "can0"))
        self.master_id = int(self.config.get("master_id", 0xFD))
        self.limit_current_a = float(self.config.get("limit_current_a", 6.0))
        self.max_speed = float(self.config.get("max_speed_rad_s", 0.05))
        self.poll_period_s = max(0.05, float(self.config.get("telemetry_period_ms", 150)) / 1000.0)

        # motor_id -> 关节定义
        self.joints = {}
        for finger in profile.finger_configs:
            for j in finger.joints:
                self.joints[int(j.motor_id)] = j
        self.motor_ids = tuple(sorted(self.joints))

        # direction / 零位（来自 calibration.json，缺省不翻转、零位 0）
        self.direction = {mid: 1.0 for mid in self.motor_ids}
        self.zero_offset_rad = {mid: 0.0 for mid in self.motor_ids}
        self._load_calibration()

        self._sock: Optional[socket.socket] = None
        self.connected = False
        self.enabled = False
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._telemetry: Dict[int, dict] = {}
        self._fault: Dict[int, int] = {}

    # ---------- 校准文件 ----------
    def _load_calibration(self):
        path = getattr(self.profile, "calibration_path", None)
        if not path:
            return
        try:
            import json
            from pathlib import Path
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except Exception:
            return
        for item in data.get("joints", []):
            mid = int(item.get("motor_id", -1))
            if mid in self.direction:
                self.direction[mid] = float(item.get("direction", 1.0)) or 1.0
                self.zero_offset_rad[mid] = float(item.get("zero_offset_rad", 0.0))

    # ---------- CAN 基础 ----------
    def _open(self) -> bool:
        try:
            s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            s.bind((self.iface,))
            s.settimeout(0.002)
        except OSError as exc:
            raise RuntimeError("打不开 CAN 接口 %s：%s" % (self.iface, exc))
        self._sock = s
        return True

    def _send(self, cid: int, data: bytes = b"\x00" * 8) -> bool:
        if self._sock is None:
            return False
        frame = struct.pack("=IB3x8s", cid | CAN_EFF_FLAG, 8, bytes(data)[:8].ljust(8, b"\x00"))
        for _ in range(3):
            try:
                self._sock.send(frame)
                return True
            except OSError:
                time.sleep(0.01)
        return False

    def _read_param(self, mid: int, idx: int, timeout: float = 0.05) -> Optional[float]:
        """读一个 float 参数（阻塞，只在 poll 线程里用）。"""
        if self._sock is None:
            return None
        self._send((T_GET_PARAM << 24) | (self.master_id << 8) | mid, struct.pack("<H", idx) + b"\x00" * 6)
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            try:
                fr = self._sock.recv(16)
            except socket.timeout:
                continue
            if len(fr) < 16:
                continue
            cid, dlc = struct.unpack("=IB", fr[:5])
            if not cid & CAN_EFF_FLAG:
                continue
            c = cid & 0x1FFFFFFF
            typ = (c >> 24) & 0x1F
            who = (c >> 8) & 0xFF
            d = fr[8:8 + min(dlc, 8)]
            if typ == T_FAULT:
                with self._lock:
                    self._fault[who] = struct.unpack("<I", d[0:4])[0]
                continue
            if typ == T_GET_PARAM and who == mid and (d[0] | (d[1] << 8)) == idx:
                return struct.unpack("<f", d[4:8])[0]
        return None

    def _write_mode(self, mid: int, mode: int) -> bool:
        """0x7005 是 uint8：值放在 data[4]，不能用 float 写（float 5.0 的 data[4] 是 0）."""
        return self._send((T_SET_PARAM << 24) | (self.master_id << 8) | mid,
                          struct.pack("<H", IDX_RUN_MODE) + bytes([int(mode), 0, 0, 0]))

    def _write_param(self, mid: int, idx: int, value: float) -> bool:
        return self._send((T_SET_PARAM << 24) | (self.master_id << 8) | mid,
                          struct.pack("<H", idx) + struct.pack("<f", value))

    # ---------- 角度换算 ----------
    def _deg_to_target(self, mid: int, angle_deg: float) -> float:
        return self.direction[mid] * math.radians(float(angle_deg)) + self.zero_offset_rad[mid]

    def _target_to_deg(self, mid: int, mech_rad: float) -> float:
        return math.degrees((mech_rad - self.zero_offset_rad[mid]) * self.direction[mid])

    def _clamp_deg(self, mid: int, angle_deg: float) -> float:
        j = self.joints[mid]
        return max(float(j.min_angle), min(float(j.max_angle), float(angle_deg)))

    # ---------- 契约：连接 ----------
    def connect(self, port: str, baudrate: int, protocol: str | None = None) -> bool:
        self.disconnect()
        if port:
            self.iface = port
        self._open()
        self.connected = True
        if self.limit_current_a > 0:
            for mid in self.motor_ids:
                self._write_param(mid, IDX_LIMIT_CUR, self.limit_current_a)
        self._stop.clear()
        self._thread = threading.Thread(target=self._poll_loop, name="el05-poll", daemon=True)
        self._thread.start()
        return True

    def disconnect(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        self.connected = False
        self.enabled = False

    def cleanup(self) -> None:
        try:
            self.enable_all(False)
        finally:
            self.disconnect()

    # ---------- 契约：使能 ----------
    def enable_all(self, enable: bool = True) -> None:
        if not self.connected:
            return
        if enable:
            for mid in self.motor_ids:
                now = self._read_param(mid, IDX_MECHPOS, timeout=0.08)
                hold = now if now is not None else 0.0
                self._write_mode(mid, MODE_CSP)
                self._write_param(mid, IDX_LIMIT_SPD, self.max_speed)
                self._write_param(mid, IDX_LOC_REF, hold)   # 先钉在当前位置，不乱动
                self._send((T_ENABLE << 24) | (self.master_id << 8) | mid)
        else:
            for mid in self.motor_ids:
                self._send((T_STOP << 24) | (self.master_id << 8) | mid)
        self.enabled = bool(enable)

    # ---------- 契约：下发角度 ----------
    def set_motor_angle(self, motor_id: int, angle_deg: float, speed: float = 0.5) -> None:
        mid = int(motor_id)
        if mid not in self.joints:
            return
        target = self._deg_to_target(mid, self._clamp_deg(mid, angle_deg))
        self._write_param(mid, IDX_LIMIT_SPD, self.max_speed)
        self._write_param(mid, IDX_LOC_REF, target)
        with self._lock:
            t = self._telemetry.get(mid)
            if t is not None:
                t["position"] = float(angle_deg)
                t["timestamp"] = time.time()

    def set_angles(self, angles_dict: dict[int, float]) -> None:
        for mid, deg in angles_dict.items():
            self.set_motor_angle(int(mid), float(deg))

    def set_all_joint_angles(self, angles_list: list[float]) -> None:
        if len(angles_list) != self.profile.num_joints:
            raise ValueError("需要 %d 个角度，收到 %d" % (self.profile.num_joints, len(angles_list)))
        mapping = self.profile.joint_index_to_motor_id
        for idx in range(self.profile.num_joints):
            self.set_motor_angle(mapping[idx], float(angles_list[idx]))

    def home_all_motors(self, target_angles=None, speed: float = 0.3) -> None:
        if target_angles is None:
            targets = [float(self.joints[mid].default_angle) for mid in self.motor_ids]
        else:
            targets = [float(a) for a in target_angles]
        for mid, deg in zip(self.motor_ids, targets):
            self.set_motor_angle(mid, deg, speed)

    # ---------- 契约：遥测 ----------
    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            try:
                snapshot = {}
                for mid in self.motor_ids:
                    mech = self._read_param(mid, IDX_MECHPOS)
                    iq = self._read_param(mid, IDX_IQF, timeout=0.03) if mech is not None else None
                    vbus = self._read_param(mid, IDX_VBUS, timeout=0.03) if mech is not None else None
                    with self._lock:
                        fault = self._fault.get(mid, 0)
                    online = mech is not None
                    snapshot[mid] = {
                        "position": self._target_to_deg(mid, mech) if online else
                                    self._telemetry.get(mid, {}).get("position", 0.0),
                        "velocity": 0.0,
                        "current": float(iq) if iq is not None else 0.0,
                        "torque": 0.0,
                        "temperature": 0.0,
                        "bus_voltage": float(vbus) if vbus is not None else 0.0,
                        "online": online,
                        "error_code": int(fault),
                        "error_text": _fmt_fault(fault),
                        "fsm_state": 1 if self.enabled else 0,
                        "timestamp": time.time(),
                    }
                with self._lock:
                    self._telemetry = snapshot
            except Exception:
                pass
            self._stop.wait(self.poll_period_s)

    def get_latest_telemetry(self) -> dict[int, dict]:
        with self._lock:
            if self._telemetry:
                return {mid: dict(v) for mid, v in self._telemetry.items()}
        return {mid: {"position": 0.0, "velocity": 0.0, "current": 0.0, "torque": 0.0,
                      "temperature": 0.0, "bus_voltage": 0.0, "online": False,
                      "error_code": 0, "error_text": "", "fsm_state": 1 if self.enabled else 0,
                      "timestamp": time.time()}
                for mid in self.motor_ids}

    def get_status(self, motor_id: int) -> dict:
        return self.get_latest_telemetry().get(int(motor_id), {})

    # ---------- 可选能力 ----------
    def set_zero_point(self, motor_id: int) -> None:
        """把当前姿态写成该电机的机械零位（写进电机，断电不丢）。"""
        mid = int(motor_id)
        if mid not in self.joints:
            return
        self._send((T_SET_ZERO << 24) | (self.master_id << 8) | mid, bytes([1] + [0] * 7))
        time.sleep(0.05)

    def set_max_current(self, motor_id: int, value_q15: int) -> None:
        """q15 → 安培（0~11 A），写 0x2019。"""
        amps = max(0.0, min(11.0, float(value_q15) / 32767.0 * 11.0))
        self._write_param(int(motor_id), IDX_LIMIT_CUR, amps)


def create_driver(profile):
    return HandDriver(profile)