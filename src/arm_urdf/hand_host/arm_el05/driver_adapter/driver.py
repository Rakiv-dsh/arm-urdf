# -*- coding: utf-8 -*-
"""arm_el05 驱动适配器 —— 通过 TCP 桥接到 WSL 里的 CAN 电机

WSL 里先跑：
    python3 ~/ros2_arm_ws/src/arm_urdf/tools/can_bridge.py --port 5599

驱动只是客户端：发角度（度）、收发使能、收遥测。
"""
from __future__ import annotations

import json
import socket
import threading
import time

DEFAULT_ENDPOINT = "127.0.0.1:5599"


class HandDriver:
    """实现 dexhand 的 DexHandMotorAdapterV1 契约。"""

    def __init__(self, profile=None):
        self.profile = profile
        self.motor_ids: list[int] = []
        self.joint_names: list[str] = []
        self.min_angle = {}
        self.max_angle = {}
        self.default_angle = {}
        if profile is not None:
            for f in getattr(profile, "finger_configs", []):
                for j in f.joints:
                    self.motor_ids.append(int(j.motor_id))
                    self.joint_names.append(j.name)
                    self.min_angle[int(j.motor_id)] = float(j.min_angle)
                    self.max_angle[int(j.motor_id)] = float(j.max_angle)
                    self.default_angle[int(j.motor_id)] = float(j.default_angle)
        cfg = getattr(profile, "motor_adapter_config", {}) or {}
        self.endpoint = str(cfg.get("bridge_endpoint", DEFAULT_ENDPOINT))
        self._lock = threading.Lock()
        self._sock: socket.socket | None = None
        self._reader: threading.Thread | None = None
        self._alive = False
        self._telemetry: dict[int, dict] = {}
        self._enabled = False
        self._angles = [0.0] * max(1, len(self.motor_ids))

    # ---------------- 连接 ----------------
    def connect(self, port: str = "", baudrate: int = 0, protocol: str | None = None) -> bool:
        host, _, p = (port or self.endpoint).partition(":")
        host = host or "127.0.0.1"
        p = p or "5599"
        try:
            s = socket.create_connection((host, int(p)), timeout=3.0)
        except OSError as e:
            print("[arm_el05] bridge connect failed:", e)
            return False
        s.settimeout(0.2)
        self._sock = s
        self._alive = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._cmd({"cmd": "status"})
        return True

    def disconnect(self) -> None:
        self._alive = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._enabled = False

    def cleanup(self) -> None:
        self.disconnect()

    # ---------------- 内部 ----------------
    def _cmd(self, payload: dict) -> None:
        if self._sock is None:
            return
        try:
            self._sock.sendall((json.dumps(payload) + "\n").encode())
        except OSError:
            self._alive = False

    def _read_loop(self) -> None:
        buf = b""
        while self._alive and self._sock is not None:
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line.decode())
                except Exception:
                    continue
                tel = msg.get("telemetry")
                if tel:
                    with self._lock:
                        for k, v in tel.items():
                            try:
                                self._telemetry[int(k)] = v
                            except (TypeError, ValueError):
                                pass
                if "enabled" in msg:
                    self._enabled = bool(msg["enabled"])

    # ---------------- 契约方法 ----------------
    def enable_all(self, enable: bool = True) -> None:
        if enable:
            self._cmd({"cmd": "enable", "on": True})
        else:
            self._cmd({"cmd": "enable", "on": False})
        self._enabled = bool(enable)

    def set_motor_angle(self, motor_id: int, angle_deg: float, speed: float = 0.5) -> None:
        self.set_angles({int(motor_id): float(angle_deg)})

    def set_angles(self, angles_dict: dict[int, float]) -> None:
        with self._lock:
            for i, mid in enumerate(self.motor_ids):
                if mid in angles_dict:
                    lo = self.min_angle.get(mid, -180.0)
                    hi = self.max_angle.get(mid, 180.0)
                    self._angles[i] = max(lo, min(hi, float(angles_dict[mid])))
            q = list(self._angles)
        self._cmd({"cmd": "angles", "q": q})

    def set_all_joint_angles(self, angles_list: list[float]) -> None:
        n = min(len(angles_list), len(self.motor_ids))
        self.set_angles({self.motor_ids[i]: float(angles_list[i]) for i in range(n)})

    def home_all_motors(self, target_angles: dict[int, float] | None = None, speed: float = 0.3) -> None:
        if target_angles:
            self.set_angles(target_angles)
        else:
            self.set_angles({mid: self.default_angle.get(mid, 0.0) for mid in self.motor_ids})

    def get_latest_telemetry(self) -> dict[int, dict]:
        with self._lock:
            return {k: dict(v) for k, v in self._telemetry.items()}

    def get_status(self, motor_id: int) -> dict:
        with self._lock:
            return dict(self._telemetry.get(int(motor_id), {}))

    # ---------------- 可选能力 ----------------
    def find_motor_limits(self, motor_id: int, **_kwargs):
        return None

    def set_max_current(self, motor_id: int, value_q15: int) -> None:
        return None


def create_driver(profile=None):
    return HandDriver(profile)
