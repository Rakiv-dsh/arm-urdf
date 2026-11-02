#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""can_bridge.py —— 把 WSL 里的 CAN 电机能力暴露成一个 TCP 服务

Windows 上的 hand-host 上位机通过 localhost:5599 连进来：
  发 {"cmd":"enable","on":true}      使能/失能
  发 {"cmd":"angles","q":[...deg]}   设目标角度（度，长度 = 关节数）
  发 {"cmd":"stop"}                  停机（全总线广播）
  发 {"cmd":"ping"}                  心跳
  收 {"ok":true,"telemetry":{...},"angles":[...]}   每 100ms 推一次

角度约定：关节角（度）= (mechPos - offset) / dir，与 joint_limits.yaml 一致。

用法：
  python3 can_bridge.py --port 5599            # 真机
  python3 can_bridge.py --port 5599 --dry-run  # 不发 CAN，只回假数据（调界面用）
"""
import argparse
import glob
import json
import math
import os
import selectors
import socket
import struct
import time

import yaml

EFF = 0x80000000
CFG = "/home/raki/ros2_arm_ws/src/arm_urdf/config/joint_limits.yaml"


class Arm:
    def __init__(self, dry=False, limit_cur=6.0, max_speed=0.5):
        cfg = yaml.safe_load(open(CFG, encoding="utf-8"))
        self.joints = []
        for j in cfg["joints"]:
            m = j.get("motor")
            if not m:
                continue
            self.joints.append(dict(
                name=j["name"], mid=int(m["can_id"]), off=float(m.get("offset", 0.0)),
                dir=float(m.get("dir", 1.0)), gear=float(m.get("gear", 1.0)),
                lo=float(j["lower"]), hi=float(j["upper"])))
        self.dry = dry
        self.limit_cur = limit_cur
        self.max_speed = max_speed
        self.on = False
        self.socks = {}
        self.iface = None
        self.targets = [0.0] * len(self.joints)
        self.telemetry = {}
        self.last_t = time.monotonic()
        self._q = [0.0] * len(self.joints)

    # ---------- CAN ----------
    @staticmethod
    def ifaces():
        return sorted(os.path.basename(p) for p in glob.glob("/sys/class/net/can*"))

    def _sock(self, name):
        s = self.socks.get(name)
        if s is None:
            s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            s.bind((name,))
            s.settimeout(0.02)
            self.socks[name] = s
        return s

    def _send(self, name, cid, data):
        self._sock(name).send(struct.pack("=IB3x8s", cid | EFF, 8, bytes(data)))

    def _pf(self, name, mid, idx, val):
        self._send(name, (0x12 << 24) | (0xFD << 8) | mid,
                   struct.pack("<H", idx) + b"\x00\x00" + struct.pack("<f", float(val)))

    def _pb(self, name, mid, idx, val):
        self._send(name, (0x12 << 24) | (0xFD << 8) | mid,
                   struct.pack("<H", idx) + b"\x00\x00" + bytes([int(val), 0, 0, 0]))

    def read_f32(self, name, mid, idx, timeout=0.05):
        try:
            s = self._sock(name)
            s.send(struct.pack("=IB3x8s", (0x11 << 24) | (0xFD << 8) | mid | EFF, 8,
                               struct.pack("<H", idx) + b"\x00" * 6))
            t_end = time.monotonic() + timeout
            while time.monotonic() < t_end:
                try:
                    fr = s.recv(16)
                except socket.timeout:
                    return None
                if len(fr) < 16:
                    continue
                cid, dlc = struct.unpack("=IB", fr[:5])
                c = cid & 0x1FFFFFFF
                if ((c >> 24) & 0x1F) == 0x11 and ((c >> 8) & 0xFF) == mid:
                    d = fr[8:8 + min(dlc, 8)]
                    if (d[0] | (d[1] << 8)) == idx:
                        return struct.unpack("<f", d[4:8])[0]
        except OSError:
            return None
        return None

    def detect(self):
        for name in self.ifaces():
            if self.read_f32(name, self.joints[0]["mid"], 0x7019, 0.15) is not None:
                self.iface = name
                return name
        return None

    def stop_all(self):
        sent = 0
        for name in self.ifaces():
            try:
                for _ in range(2):
                    for j in self.joints:
                        self._send(name, (0x04 << 24) | (0xFD << 8) | j["mid"], b"\x00" * 8)
                        sent += 1
                    time.sleep(0.01)
            except OSError:
                pass
        self.on = False
        return sent

    def enable(self, on):
        if self.dry:
            self.on = bool(on)
            return True, "dry-run"
        if not on:
            self.stop_all()
            return True, "disabled"
        iface = self.detect()
        if iface is None:
            return False, "no can interface answering"
        for j in self.joints:
            mid = j["mid"]
            cur = self.read_f32(iface, mid, 0x7019, 0.2)
            if cur is None:
                continue
            self._pf(iface, mid, 0x2019, self.limit_cur)
            time.sleep(0.02)
            self._send(iface, (0x04 << 24) | (0xFD << 8) | mid, bytes([1, 0, 0, 0, 0, 0, 0, 0]))
            time.sleep(0.03)
            self._pb(iface, mid, 0x7005, 5)
            time.sleep(0.02)
            self.read_f32(iface, mid, 0x7005, 0.05)
            self._send(iface, (0x03 << 24) | (0xFD << 8) | mid, b"\x00" * 8)
            time.sleep(0.03)
            self._pf(iface, mid, 0x7017, self.max_speed)
            self.targets[self.joints.index(j)] = cur
        self.on = True
        self.last_t = time.monotonic()
        return True, "enabled on %s" % iface

    # ---------- 周期 ----------
    def tick(self):
        now = time.monotonic()
        dt = max(0.0, min(0.1, now - self.last_t))
        self.last_t = now
        if self.dry:
            for i, j in enumerate(self.joints):
                want = j["off"] + j["dir"] * j["gear"] * math.radians(self._q[i])
                self.targets[i] += max(-self.max_speed * dt, min(self.max_speed * dt, want - self.targets[i]))
                pos = self.targets[i]
                self.telemetry[j["mid"]] = dict(position=math.degrees((pos - j["off"]) / (j["dir"] * j["gear"])),
                                                velocity=0.0, current=0.0, temperature=30.0,
                                                online=True, error_code=0, fsm_state=5,
                                                timestamp=time.time())
            return
        if not self.on or self.iface is None:
            return
        step = self.max_speed * dt
        for i, j in enumerate(self.joints):
            want = j["off"] + j["dir"] * j["gear"] * math.radians(self._q[i])
            self.targets[i] += max(-step, min(step, want - self.targets[i]))
            try:
                self._pf(self.iface, j["mid"], 0x7016, self.targets[i])
            except OSError:
                self.iface = None
                return

    def poll_telemetry(self):
        if self.dry or not self.on or self.iface is None:
            return
        for i, j in enumerate(self.joints):
            pos = self.read_f32(self.iface, j["mid"], 0x7019, 0.05)
            iq = self.read_f32(self.iface, j["mid"], 0x701A, 0.02)
            v = self.read_f32(self.iface, j["mid"], 0x701B, 0.02)
            q_deg = math.degrees((pos - j["off"]) / (j["dir"] * j["gear"])) if pos is not None else 0.0
            self.telemetry[j["mid"]] = dict(position=q_deg, velocity=v or 0.0, current=iq or 0.0,
                                            temperature=0.0, online=pos is not None, error_code=0,
                                            fsm_state=5, timestamp=time.time())

    def set_angles(self, q_deg):
        n = min(len(q_deg), len(self.joints))
        for i in range(n):
            j = self.joints[i]
            self._q[i] = max(math.degrees(j["lo"]), min(math.degrees(j["hi"]), float(q_deg[i])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5599)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--limit-cur", type=float, default=6.0)
    ap.add_argument("--max-speed", type=float, default=0.5)
    a = ap.parse_args()

    arm = Arm(dry=a.dry_run, limit_cur=a.limit_cur, max_speed=a.max_speed)
    print("[bridge] joints:", [j["name"] for j in arm.joints], " dry-run:", a.dry_run, flush=True)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", a.port))
    srv.listen(4)
    srv.setblocking(False)
    sel = selectors.DefaultSelector()
    sel.register(srv, selectors.EVENT_READ, "srv")
    clients = {}
    print("[bridge] listening on %d" % a.port, flush=True)

    last_ctrl = 0.0
    last_tel = 0.0
    while True:
        for key, _ in sel.select(timeout=0.02):
            if key.data == "srv":
                conn, addr = srv.accept()
                conn.setblocking(False)
                sel.register(conn, selectors.EVENT_READ, "cli")
                clients[conn] = b""
                print("[bridge] client", addr, flush=True)
            else:
                conn = key.fileobj
                try:
                    data = conn.recv(65536)
                except OSError:
                    data = b""
                if not data:
                    sel.unregister(conn)
                    clients.pop(conn, None)
                    print("[bridge] client gone", flush=True)
                    continue
                clients[conn] += data
                while b"\n" in clients[conn]:
                    line, clients[conn] = clients[conn].split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        msg = json.loads(line.decode())
                    except Exception:
                        continue
                    cmd = msg.get("cmd")
                    if cmd == "ping":
                        reply = dict(ok=True, pong=True)
                    elif cmd == "enable":
                        ok, info = arm.enable(bool(msg.get("on", True)))
                        reply = dict(ok=ok, info=info, enabled=arm.on)
                    elif cmd == "stop":
                        n = arm.stop_all()
                        reply = dict(ok=True, frames=n, enabled=False)
                    elif cmd == "angles":
                        arm.set_angles(msg.get("q") or [])
                        reply = dict(ok=True, q=arm._q, enabled=arm.on)
                    elif cmd == "status":
                        reply = dict(ok=True, enabled=arm.on, iface=arm.iface,
                                     joints=[j["name"] for j in arm.joints])
                    else:
                        reply = dict(ok=False, error="unknown cmd")
                    try:
                        conn.sendall((json.dumps(reply) + "\n").encode())
                    except OSError:
                        pass

        now = time.monotonic()
        if now - last_ctrl >= 0.033:
            last_ctrl = now
            arm.tick()
        if now - last_tel >= 0.1:
            last_tel = now
            arm.poll_telemetry()
            if clients:
                payload = json.dumps(dict(ok=True, telemetry={str(k): v for k, v in arm.telemetry.items()},
                                          enabled=arm.on)).encode() + b"\n"
                for conn in list(clients):
                    try:
                        conn.sendall(payload)
                    except OSError:
                        sel.unregister(conn); clients.pop(conn, None)


if __name__ == "__main__":
    main()
