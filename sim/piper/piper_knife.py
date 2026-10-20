"""Left Piper smooths the pressed butter with the web demo's table knife (assets/scene.xml, PR #13).
MuJoCo only, NOT Isaac Sim.

Knife = the knife of assets/scene.xml: rubber handle 9 x 2 x 1.6 cm (40 g), steel blade 14 x 2.8 x 0.4 cm (20 g).
Grip: rigid child body of left_link6 between the closed fingertips (fingers at 10 mm = handle half-width), i.e. a
weld-equivalent; no slip simulation. Finger meshes do not collide in these scenes, only the knife does.
"""
import numpy as np, mujoco
import xml.etree.ElementTree as ET
import piper_scene as P

KNIFE_Z = 0.131                                   # handle centre on the fingertip line (link6 frame)
HANDLE = dict(pos=(-0.025, 0, KNIFE_Z), size=(0.045, 0.01, 0.008), mass=0.04)
BLADE = dict(pos=(0.09, 0, KNIFE_Z), size=(0.07, 0.014, 0.002), mass=0.02)
TILT_DEG = 15.0                                   # roll about the blade axis; world +x edge up (see plan)
NSEG = 14                                         # collision segments, each a welded child body (max 50 contacts per body-flex pair)
GRIP_OPEN = 0.010                                 # each finger 10 mm out: closed on the 2 cm handle
FRICTION = "0.2 0.005 0.0001"


def blade_quat(tilt_deg):
    a = np.radians(tilt_deg); return (np.cos(a / 2), np.sin(a / 2), 0.0, 0.0)


# X2 table knife (sim/x2/knife.py: handle 10 x 2 x 1.5 cm, blade 10 x 2 cm x 1.5 mm, 40 g) for the cheese stack:
# the 4 mm web-demo blade is twice as thick as a cheese slice and cannot enter a 2 mm seam.
X2_HANDLE = dict(pos=(-0.02, 0, KNIFE_Z), size=(0.05, 0.0075, 0.01), mass=0.016)
X2_BLADE = dict(pos=(0.08, 0, KNIFE_Z), size=(0.05, 0.01, 0.00075), mass=0.024)


def attach_knife(r, side="left", tilt_deg=TILT_DEG, segments=True, kind="web"):
    """kind='web': knife of assets/scene.xml; 'x2': the sim/x2 knife."""
    l6 = P.find_body(r, f"{side}_link6")
    HANDLE, BLADE = (globals()["HANDLE"], globals()["BLADE"]) if kind == "web" else (X2_HANDLE, X2_BLADE)
    k = ET.SubElement(l6, "body", name="knife", pos="0 0 0")
    ET.SubElement(k, "geom", name="knife_handle", type="box", pos=" ".join(map(str, HANDLE["pos"])),
                  size=" ".join(map(str, HANDLE["size"])), mass=str(HANDLE["mass"]), material="rubber",
                  friction="1.4 0.01 0.001", contype="0", conaffinity="0")
    q = " ".join(f"{x:.6f}" for x in blade_quat(tilt_deg))
    ET.SubElement(k, "geom", name="knife_blade", type="box", pos=" ".join(map(str, BLADE["pos"])), quat=q,
                  size=" ".join(map(str, BLADE["size"])), mass=str(BLADE["mass"]), rgba="0.8 0.82 0.85 1",
                  contype="0" if segments else "1", conaffinity="0" if segments else "1", friction=FRICTION,
                  priority="1")
    if segments:
        L = BLADE["size"][0]
        for i in range(NSEG):
            x = BLADE["pos"][0] - L + L / NSEG * (2 * i + 1)
            ET.SubElement(ET.SubElement(k, "body", name=f"knife_seg{i}"), "geom", name=f"knife_seg{i}", type="box", pos=f"{x} 0 {KNIFE_Z}", quat=q,
                          size=f"{L / NSEG} {BLADE['size'][1]} {BLADE['size'][2]}", mass="0", group="3",
                          rgba="0.8 0.8 0.8 0", friction=FRICTION, priority="1")


def build(rest, offscreen=(1280, 720)):
    r, wb = P.base_root(offscreen, "2.0 0.005 0.0001")   # table 2.0: stands in for butter adhesion (as sim/x2)
    P.fingers_collide(r, "left", False); P.fingers_collide(r, "right", False)
    attach_knife(r)
    P.add_butter(wb, rest, friction="0.2 0.005 0.0001")
    return P.compile_root(r)


def blade_low(m, d):
    z = []
    for i in range(NSEG):
        g = m.geom(f"knife_seg{i}").id; R = d.geom_xmat[g].reshape(3, 3)
        z.append(d.geom_xpos[g][2] - np.abs(R[2]) @ m.geom_size[g])
    return float(min(z))


PASS_HEIGHTS = (0.009, 0.007, 0.0055, 0.0045)   # lowest blade point above the table, as sim/x2
STROKE = (-0.035, 0.02)
ABOVE = 0.03
SCHEDULE = [("approach", 0.6, 0)]
for _k in range(len(PASS_HEIGHTS)):
    SCHEDULE += [("descend", 0.5, _k), ("stroke", 1.5, _k), ("lift", 0.4, _k), ("return", 0.6, _k)]
SCHEDULE.append(("rest", 0.8, len(PASS_HEIGHTS) - 1))
XAX = (0, -1, 0)                                  # blade points from the hand towards -y over the butter


def plan(m, center):
    cx, cy = center; qa = [m.joint(j).qposadr[0] for j in P.arm("left")]; dd = mujoco.MjData(m); errs = []

    def solve(x, h, q):
        tgt = np.array([cx + x, cy, h + 0.004])
        for _ in range(3):
            q, e, _ = P.ik(m, "left", tgt, q, offset=BLADE["pos"], xax=XAX)
            dd.qpos[:] = q; mujoco.mj_kinematics(m, dd); tgt[2] += h - blade_low(m, dd)
        errs.append(e); return q[qa].copy(), q
    q, e, _ = P.ik_restarts(m, "left", (cx + STROKE[0], cy, ABOVE), offset=BLADE["pos"], xax=XAX)
    xs = np.linspace(*STROKE, 9); W = {}
    for k, h in enumerate(PASS_HEIGHTS):
        W[k, "a0"], q = solve(STROKE[0], ABOVE, q); W[k, "a1"], _ = solve(STROKE[1], ABOVE, q)
        path = []
        for x in xs:
            p, q = solve(x, h, q); path.append(p)
        W[k, "s"] = np.array(path); q = q.copy(); q[qa] = W[k, "a0"]
    seg, T = P.segmenter(SCHEDULE)

    def target(t):
        i, (name, dur, k), u = seg(t); s = P.smooth(u); mix = lambda a, b: (1 - s) * a + s * b
        if name in ("approach", "rest"): pa = W[k, "a0"]
        elif name == "descend": pa = mix(W[k, "a0"], W[k, "s"][0])
        elif name == "stroke": pa = P.interp(W[k, "s"], s)
        elif name == "lift": pa = mix(W[k, "s"][-1], W[k, "a1"])
        else: pa = mix(W[k, "a1"], W[min(k + 1, len(PASS_HEIGHTS) - 1), "a0"])
        qt = np.zeros(m.nq); qt[qa] = pa; return qt
    return target, seg, T, max(errs)


def simulate(rest, plastic=True, callback=None, dt_cb=1 / 30):
    m = build(rest); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    c = (rest[:, :2].min(0) + rest[:, :2].max(0)) / 2
    target, seg, T, ik_err = plan(m, c); nrob = m.nq - 3 * len(rest)
    d.qpos[:nrob] = target(0)[:nrob]; mujoco.mj_forward(m, d); cur = P.butter_verts(m, d)
    res = dict(initial=P.butter_metrics(cur), ik_max_err=ik_err, finite=True, yields=0, passes=[], min_blade_height=1.0)
    t = 0.0; nxt = 0.0; last = 0
    while t < T - 1e-9:
        i, sg, u = seg(t)
        if i != last:
            if SCHEDULE[last][0] == "stroke":
                res["passes"].append(P.butter_metrics(P.butter_verts(m, d)))
                if plastic:
                    m, d, cur = P.rebuild_with_yield(m, d, cur, build); res["yields"] += 1
            last = i
        P.set_ctrl(m, d, target(t), {"left": GRIP_OPEN}); mujoco.mj_step(m, d); t = d.time
        if not np.isfinite(d.qpos).all(): res["finite"] = False; break
        if sg[0] == "stroke": res["min_blade_height"] = min(res["min_blade_height"], blade_low(m, d))
        if callback and t >= nxt: callback(m, d, t, f"{sg[0]} {sg[2] + 1}/{len(PASS_HEIGHTS)}"); nxt += dt_cb
    res["final"] = P.butter_metrics(P.butter_verts(m, d))
    return res
