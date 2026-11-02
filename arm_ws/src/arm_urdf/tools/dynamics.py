# -*- coding: utf-8 -*-
"""真机 arm_real 的刚体动力学仿真（不驱动真机）

- 从 URDF 读质量 / 质心 / 惯量
- 用雅可比法算质量矩阵 M(q)、重力项 G(q)、科氏项 C(q,dq)dq
- 分析：全空间的重力力矩分布（能不能自己撑住）
- 仿真：PD + 重力补偿跟踪一条 2 秒轨迹，RK4 积分，输出力矩曲线

输出：kin_real/dynamics_report.txt、dynamics_summary.json、dyn_*.png
"""
import json, math, os
import numpy as np
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "kin_real", "arm_real.urdf")
OUT = os.path.join(HERE, "kin_real")
os.makedirs(OUT, exist_ok=True)

G = 9.81

def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r); cp, sp = math.cos(p), math.sin(p); cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]); Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return Rz @ Ry @ Rx

def T_of(el):
    T = np.eye(4); o = el.find("origin") if el is not None else None
    if o is None: return T
    T[:3,:3] = rpy_to_R(*[float(v) for v in o.get("rpy","0 0 0").split()])
    T[:3,3] = [float(v) for v in o.get("xyz","0 0 0").split()]
    return T

def axis_of(el):
    a = el.find("axis")
    v = np.array([float(x) for x in (a.get("xyz") if a is not None else "0 0 1").split()])
    return v / np.linalg.norm(v)

root = ET.parse(URDF).getroot()
links = {}
for l in root.findall("link"):
    i = l.find("inertial")
    if i is None:
        links[l.get("name")] = None
        continue
    m = float(i.find("mass").get("value"))
    o = i.find("origin")
    com = np.array([float(v) for v in (o.get("xyz") if o is not None else "0 0 0").split()])
    Rc = rpy_to_R(*[float(v) for v in (o.get("rpy") if o is not None else "0 0 0").split()])
    it = i.find("inertia")
    I = np.array([[float(it.get("ixx")), float(it.get("ixy", 0)), float(it.get("ixz", 0))],
                  [float(it.get("ixy", 0)), float(it.get("iyy")), float(it.get("iyz", 0))],
                  [float(it.get("ixz", 0)), float(it.get("iyz", 0)), float(it.get("izz"))]])
    links[l.get("name")] = dict(mass=m, com=com, R=Rc, I=I)

joints = []
for j in root.findall("joint"):
    joints.append(dict(name=j.get("name"), parent=j.find("parent").get("link"),
                       child=j.find("child").get("link"), T=T_of(j), axis=axis_of(j)))
chain = {j["parent"]: j for j in joints}
NJ = len(joints)
NACT = 5                      # 只算 joint1~5
TIP = joints[-1]["child"]

def rotm(ax, ang):
    a = ax / np.linalg.norm(ax)
    K = np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3) + math.sin(ang)*K + (1-math.cos(ang))*(K@K)

def fk(q):
    """返回 (世界原点列表, 世界轴列表, 每个 link 的世界变换)"""
    world = {"base_link": np.eye(4)}
    pos, axes, children = [], [], []
    link = "base_link"; i = 0
    while link in chain:
        j = chain[link]
        Tj = world[link] @ j["T"]
        R = np.eye(4); R[:3,:3] = rotm(j["axis"], q[i] if i < len(q) else 0.0)
        world[j["child"]] = Tj @ R
        pos.append(Tj[:3,3]); axes.append(Tj[:3,:3] @ j["axis"]); children.append(j["child"])
        link = j["child"]; i += 1
    return np.array(pos), np.array(axes), world, children

def mass_matrix(q):
    pos, axes, world, children = fk(q)
    M = np.zeros((NACT, NACT))
    for idx, link in enumerate(children):
        L = links.get(link)
        if L is None: continue
        # 该 link 的质心在世界系
        Tc = world[link] @ np.block([[L["R"], L["com"].reshape(3,1)],[np.zeros((1,3)),1]])
        pc = Tc[:3,3]; Rc = Tc[:3,:3]
        Jv = np.zeros((3, NACT)); Jw = np.zeros((3, NACT))
        for k in range(NACT):
            if k > idx: break
            Jv[:,k] = np.cross(axes[k], pc - pos[k])
            Jw[:,k] = axes[k]
        Iw = Rc @ L["I"] @ Rc.T
        M += L["mass"] * (Jv.T @ Jv) + (Jw.T @ Iw @ Jw)
    return M

def gravity(q):
    """重力力矩 G = dU/dq，U = sum m g z"""
    pos, axes, world, children = fk(q)
    Gq = np.zeros(NACT)
    for idx, link in enumerate(children):
        L = links.get(link)
        if L is None: continue
        Tc = world[link] @ np.block([[L["R"], L["com"].reshape(3,1)],[np.zeros((1,3)),1]])
        pc = Tc[:3,3]
        for k in range(NACT):
            if k > idx: break
            Jv = np.cross(axes[k], pc - pos[k])
            Gq[k] += L["mass"] * G * Jv[2]
    return Gq

def dM_dq(q, h=1e-4):
    dM = []
    for k in range(NACT):
        qp = np.array(q, float); qm = np.array(q, float)
        qp[k] += h; qm[k] -= h
        dM.append((mass_matrix(qp) - mass_matrix(qm)) / (2*h))
    return dM

def coriolis(q, dq):
    dM = dM_dq(q)
    n = NACT
    c = np.zeros(n)
    for i in range(n):
        for j in range(n):
            for k in range(n):
                c[i] += 0.5 * (dM[k][i,j] + dM[j][i,k] - dM[i][j,k]) * dq[j] * dq[k]
    return c

rep = []
def say(s=""):
    rep.append(s); print(s.encode("ascii","replace").decode())

# ---------- 基本量 ----------
ms = [links[n]["mass"] for n in links if links[n]]
say("="*76); say("arm_real 刚体动力学"); say("="*76)
say("连杆质量：")
for n in links:
    if links[n]:
        L = links[n]
        say("  %-9s m=%.4f kg   com=[%+.4f %+.4f %+.4f]" % (n, L["mass"], *L["com"]))
say("  总质量 %.4f kg" % sum(ms))

q0 = np.zeros(NACT)
M0 = mass_matrix(q0); G0 = gravity(q0)
say(""); say("零位质量矩阵 M(0)：")
for row in M0:
    say("  [%+.5f %+.5f %+.5f %+.5f %+.5f]" % tuple(row))
say("")
say("零位重力力矩 G(0) = [%+.4f %+.4f %+.4f %+.4f %+.4f] N·m" % tuple(G0))
say("  → 静止保持这一姿态，每个关节至少要出这么多力矩")

# ---------- 全空间重力力矩 ----------
rng = np.random.default_rng(0)
N = 4000
Q = rng.uniform(-math.pi, math.pi, size=(N, NACT))
Gm = np.array([gravity(q) for q in Q])
say(""); say("全空间重力力矩统计（%d 个随机位形）" % N)
for k in range(NACT):
    say("  joint%d: 最大 %+.3f N·m，均值 %.3f N·m" % (k+1, np.abs(Gm[:,k]).max(), np.abs(Gm[:,k]).mean()))
say("  电机额定峰值 ≈ 6 N·m（T-N 曲线低速段），所以重力不是瓶颈")

# ---------- 仿真：PD + 重力补偿跟踪 ----------
dt = 0.001       # 1 ms（计算力矩控制后闭环只有 Kp/Kd，稳定）
T_end = 2.0
steps = int(T_end / dt)
q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
q_goal  = np.array([1.2, 0.6, -0.5, 0.4, 0.7])
Kp = np.array([25, 25, 25, 25, 25], float)     # 闭环刚度（rad/s^2 per rad）
Kd = np.array([10, 10, 10, 10, 10], float)     # 闭环阻尼
TAU_MAX = 6.0

def traj(t):
    s = 0.5 - 0.5*math.cos(math.pi * min(1.0, t/T_end))
    return q_start + (q_goal - q_start) * s

def plant_tau(q, dq, qd, dqd):
    """计算力矩控制：先算期望加速度，再补上惯性/科氏/重力"""
    M = mass_matrix(q)
    Gq = gravity(q)
    C = coriolis(q, dq) if np.linalg.norm(dq) > 1e-6 else np.zeros(NACT)
    qdd_des = Kp*(qd - q) + Kd*(dqd - dq)
    tau = M @ qdd_des + C + Gq
    return np.clip(tau, -TAU_MAX, TAU_MAX), M, Gq, C

def deriv(q, dq, qd, dqd):
    tau, M, Gq, C = plant_tau(q, dq, qd, dqd)
    dq2 = np.linalg.solve(M, tau - C - Gq)
    return dq, dq2, tau

q = q_start.copy(); dq = np.zeros(NACT)
log_t, log_q, log_tau, log_err = [], [], [], []
for i in range(steps):
    t = i*dt
    qd = traj(t); dqd = (traj(t+dt) - traj(t))/dt
    k1 = deriv(q, dq, qd, dqd)
    k2 = deriv(q + 0.5*dt*k1[0], dq + 0.5*dt*k1[1], qd, dqd)
    k3 = deriv(q + 0.5*dt*k2[0], dq + 0.5*dt*k2[1], qd, dqd)
    k4 = deriv(q + dt*k3[0], dq + dt*k3[1], qd, dqd)
    q = q + dt/6*(k1[0] + 2*k2[0] + 2*k3[0] + k4[0])
    dq = dq + dt/6*(k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
    log_t.append(t); log_q.append(q.copy()); log_tau.append(k1[2]); log_err.append(np.linalg.norm(qd - q))

log_t = np.array(log_t); log_q = np.array(log_q); log_tau = np.array(log_tau); log_err = np.array(log_err)
say(""); say("仿真：计算力矩控制（τ = M(Kp e + Kd ė) + C + G），跟踪 2 秒轨迹（%d 步，dt=1ms）" % steps)
say("  关节力矩上限 %.1f N·m（与电机峰值一致）" % TAU_MAX)
say("  末态跟踪误差 %.5f rad" % log_err[-1])
say("  最大跟踪误差 %.5f rad（出现在中间）" % log_err.max())
say("  峰值力矩：%s N·m" % np.round(np.abs(log_tau).max(axis=0), 3).tolist())
say("  峰值速度：%s rad/s" % np.round(np.abs(np.diff(log_q, axis=0)).max(axis=0)/dt, 2).tolist())

json.dump(dict(total_mass=float(sum(ms)), G0=G0.tolist(),
               max_gravity=float(np.abs(Gm).max()), tau_peak=np.abs(log_tau).max(axis=0).tolist(),
               track_err_max=float(log_err.max()), track_err_end=float(log_err[-1])),
          open(os.path.join(OUT, "dynamics_summary.json"), "w"), indent=1)
np.savez(os.path.join(OUT, "dyn_sim.npz"), t=log_t, q=log_q, tau=log_tau, err=log_err,
         Gm=Gm, q_goal=q_goal)
open(os.path.join(OUT, "dynamics_report.txt"), "w", encoding="utf-8").write("\n".join(rep))
print("\nSaved:", OUT)
