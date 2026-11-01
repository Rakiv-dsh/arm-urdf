# -*- coding: utf-8 -*-
"""机械臂正/逆运动学分析 v2

两种限位各自分析：
  A. 满行程（所有关节 ±180°）：看机构本身的能力
  B. 当前限位（joint2 -0.40..0.88，joint4 0..0.27）：看现在实际能用多少
输出：kin/report.txt（UTF-8）、kin/workspace.png、kin/cond.png
"""
import math, os, json
import numpy as np
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
# 默认读本包的 URDF；也可以用环境变量指定
URDF = os.environ.get("ARM_URDF", os.path.join(PKG, "urdf", "arm_urdf.urdf"))
OUT = os.environ.get("KIN_OUT", os.path.join(PKG, "docs"))
os.makedirs(OUT, exist_ok=True)

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
    return v/np.linalg.norm(v)

root = ET.parse(URDF).getroot()
joints = []
for j in root.findall("joint"):
    lim = j.find("limit")
    joints.append(dict(name=j.get("name"), parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                       T=T_of(j), axis=axis_of(j),
                       lower=float(lim.get("lower","-3.14")) if lim is not None else -3.14,
                       upper=float(lim.get("upper","3.14")) if lim is not None else 3.14))
chain = {j["parent"]: j for j in joints}
N = len(joints); TIP = joints[-1]["child"]; NAMES = [j["name"] for j in joints]
LO_CUR = np.array([j["lower"] for j in joints]); HI_CUR = np.array([j["upper"] for j in joints])
LO_FULL = -np.pi*np.ones(N); HI_FULL = np.pi*np.ones(N)

def rotm(axis, ang):
    a = axis/np.linalg.norm(axis)
    K = np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3)+math.sin(ang)*K+(1-math.cos(ang))*(K@K)

def fk_all(q):
    world = {"base_link": np.eye(4)}; axes_w=[]; pts_w=[]; link="base_link"
    while link in chain:
        j = chain[link]; Tj = world[link] @ j["T"]
        R = np.eye(4); R[:3,:3] = rotm(j["axis"], float(q[len(axes_w)]))
        world[j["child"]] = Tj @ R
        axes_w.append(Tj[:3,:3] @ j["axis"]); pts_w.append(Tj[:3,3]); link = j["child"]
    return world, np.array(axes_w), np.array(pts_w)

def fk(q):
    return fk_all(q)[0][TIP]

def jacobian(q, eps=1e-6):
    T0 = fk(q); p0 = T0[:3,3]; J = np.zeros((6,N))
    for i in range(N):
        dq = np.array(q,float); dq[i] += eps
        T1 = fk(dq)
        J[:3,i] = (T1[:3,3]-p0)/eps
        dR = T1[:3,:3] @ T0[:3,:3].T
        J[3:,i] = np.array([dR[2,1]-dR[1,2], dR[0,2]-dR[2,0], dR[1,0]-dR[0,1]])/(2*eps)
    return J

rep = []
def say(s=""):
    rep.append(s); print(s.encode("ascii","replace").decode())

rng = np.random.default_rng(0)
w0, axes0, pts0 = fk_all(np.zeros(N))
say("="*72); say("运动链（base_link -> %s），共 %d 个关节" % (TIP, N)); say("="*72)
say("%-8s %-26s %-28s %s" % ("关节","轴（世界系）","关节原点（世界系 m）","当前限位"))
for i,j in enumerate(joints):
    ax, pt = axes0[i], pts0[i]
    say("%-8s [%+.3f %+.3f %+.3f]        [%+.4f %+.4f %+.4f]    [%+.2f, %+.2f]" %
        (j["name"], ax[0],ax[1],ax[2], pt[0],pt[1],pt[2], j["lower"], j["upper"]))

T0 = fk(np.zeros(N))
say(""); say("零位末端：位置 [%.4f %.4f %.4f] m" % tuple(T0[:3,3]))
say("零位末端：姿态 R = %s" % np.round(T0[:3,:3],3).tolist())

def sample(LO, HI, n):
    return rng.uniform(LO, HI, size=(n,N))

def positions(Q, chunk=8000):
    out = np.zeros((len(Q),3))
    for s in range(0,len(Q),chunk):
        for k in range(s, min(s+chunk,len(Q))):
            out[k] = fk(Q[k])[:3,3]
    return out

def ik(Tt, q0, LO, HI, iters=300, tol=1e-4, lam=1e-2):
    q = np.clip(np.array(q0,float), LO, HI)
    tp = Tt[:3,3]; e = np.ones(6)
    for _ in range(iters):
        T = fk(q)
        ep = tp - T[:3,3]
        dR = Tt[:3,:3] @ T[:3,:3].T
        eo = np.array([dR[2,1]-dR[1,2], dR[0,2]-dR[2,0], dR[1,0]-dR[0,1]])*0.5
        e = np.concatenate([ep, eo])
        if np.linalg.norm(e) < tol: break
        J = jacobian(q)
        w = np.array([1,1,1,0.15,0.15,0.15])
        Jw = J*w[:,None]; ew = e*w
        dq = Jw.T @ np.linalg.solve(Jw@Jw.T + lam**2*np.eye(6), ew)
        q = np.clip(q+dq, LO, HI)
    return q, float(np.linalg.norm(e))

scenarios = [("A. 满行程（全部 ±180°）", LO_FULL, HI_FULL),
             ("B. 当前限位", LO_CUR, HI_CUR)]
summary = {}
for title, LO, HI in scenarios:
    say(""); say("="*72); say(title); say("="*72)
    Q = sample(LO, HI, 6000)
    P = positions(Q)
    r = np.linalg.norm(P, axis=1)
    say("  末端到原点距离：%.3f ~ %.3f m（平均 %.3f）" % (r.min(), r.max(), r.mean()))
    say("  x %.3f ~ %.3f   y %.3f ~ %.3f   z %.3f ~ %.3f"
        % (P[:,0].min(), P[:,0].max(), P[:,1].min(), P[:,1].max(), P[:,2].min(), P[:,2].max()))
    idx = rng.choice(len(Q), 300, replace=False)
    cs = []
    for i in idx:
        s = np.linalg.svd(jacobian(Q[i]), compute_uv=False)
        cs.append(s[0]/max(s[-1],1e-9))
    cs = np.array(cs)
    say("  雅可比条件数：中位 %.0f，90%% 分位 %.0f，最大 %.0f" % (np.median(cs), np.percentile(cs,90), cs.max()))
    say("  接近奇异（条件数>200）的位形占比：%.1f%%" % (100*np.mean(cs>200)))
    ok=0; errs=[]
    for t in range(40):
        qt = rng.uniform(LO,HI); Tt = fk(qt)
        best=None
        for s in range(3):
            q0 = qt+rng.normal(0,0.1,N) if s==0 else rng.uniform(LO,HI)
            q,e = ik(Tt,q0,LO,HI)
            if best is None or e<best[1]: best=(q,e)
        errs.append(best[1])
        if best[1] < 0.01: ok+=1
    errs=np.array(errs)
    say("  逆解：40 个随机可达位姿，成功 %d（%.0f%%），位置误差中位 %.4f m"
        % (ok, 100*ok/40.0, np.median(errs)))
    summary[title] = dict(reach_min=float(r.min()), reach_max=float(r.max()),
                          cond_median=float(np.median(cs)), cond_p90=float(np.percentile(cs,90)),
                          ik_pct=100.0*ok/40.0, ik_err=float(np.median(errs)))
    np.save(os.path.join(OUT, "ws_%s.npy" % ("full" if "满" in title else "cur")), P[::10])

qt = np.array([0.6,0.5,0.4,0.3,0.2,0.0])
Tt = fk(qt)
q_sol, e = ik(Tt, np.zeros(N), LO_FULL, HI_FULL)
say(""); say("例子：目标 q = %s" % np.round(qt,3).tolist())
say("      解出 q = %s   误差 %.5f m" % (np.round(q_sol,3).tolist(), e))
say("      两次正解末端位置差 %.6f m" % np.linalg.norm(fk(qt)[:3,3]-fk(q_sol)[:3,3]))

open(os.path.join(OUT,"report.txt"), "w", encoding="utf-8").write("\n".join(rep))
json.dump(summary, open(os.path.join(OUT,"summary.json"),"w"), ensure_ascii=False, indent=1)
print("\nSaved:", os.path.join(OUT,"report.txt"))
