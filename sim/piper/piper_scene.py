"""Dual Piper (assets/scene.xml) in MuJoCo -- base scene, IK and the butter-press experiment.
MuJoCo only, NOT Isaac Sim. Reuses the butter material/mesh/plasticity parameters of sim/x2 so results compare 1:1.

Coordinates: world frame of assets/scene.xml; table top at z = 0; Piper bases at (-0.3, +-0.22).
Piper hand frame = <side>_link6: z = approach axis, fingers slide along y (each 0..35 mm -> max ~70 mm opening),
closed fingertips occupy x +-12.7 mm, y +-7.7 mm, z 0.126..0.136 m.
"""
import os, sys
import numpy as np, mujoco
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
SCENE = os.path.join(REPO, "assets", "scene.xml")
sys.path.insert(0, os.path.join(REPO, "sim", "x2"))
import butter_scene as XB  # noqa: E402  (material constants + tet topology only)

REMOVE = ("bread", "butter", "knife", "jar", "plate", "board")
TABLE_TOP = 0.0
TIP_Z = 0.136                       # closed fingertip bottom, link6 frame
TIP_PROXY = dict(pos=(0, 0, 0.131), size=(0.0127, 0.0077, 0.005))
HOME = [0, 1.05, -1.15, 0, 0.55, 0]
BUTTER, RADIUS, YIELD = XB.BUTTER, XB.RADIUS, XB.YIELD
BUTTER_XY = (0.0, 0.18)
TILES_C = (0.02, 0.19)     # centre of the 16 x 16 cm tile patch (butter scenes; other scenes pass tiles_c)
# 3 x 3 raster identical to sim/x2
PRESS_OFFSETS = XB.PRESS_OFFSETS
PRESS_GAP = 0.006
KV_RATIO = 0.0
TIMESTEP = 0.001
ARMATURE = 0.01
PROXY_SPLIT = (6, 4)   # contacts per body-flex pair are capped at 50 (element-based flex collision)
TILES = (16, 0.01)
SAFE_VMIN = 0.1       # None: plain X2 rule (always YIELD); a number: cap the yield so no tet gets thinner


def arm(side): return [f"{side}_joint{i}" for i in range(1, 7)]


def find_body(r, name):
    return next(b for b in r.iter("body") if b.get("name") == name)


def base_root(offscreen=(1280, 720), table_friction="1 0.005 0.0001", tiles_c=None):
    r = ET.parse(SCENE).getroot()
    r.find("compiler").set("meshdir", os.path.join(REPO, "assets", "meshes"))
    for e in list(r):
        if e.tag == "keyframe": r.remove(e)
    r.find("size").attrib.pop("nkey", None)
    r.find("option").attrib.update(timestep=str(TIMESTEP), integrator="discrete", solver="CG", iterations="60",
                                   tolerance="1e-8")
    for a in ("cone", "impratio"): r.find("option").attrib.pop(a, None)  # pyramidal cone: as sim/x2 (flex)
    ET.SubElement(ET.SubElement(r, "visual"), "global", offwidth=str(offscreen[0]), offheight=str(offscreen[1]))
    wb = r.find("worldbody")
    for b in list(wb):
        if b.tag == "body" and b.get("name", "").startswith(REMOVE): wb.remove(b)
    for b in wb.iter("body"):  # robot meshes: visual only (self-contacts stalled the CG solver and shook the flex)
        if b.get("name", "").startswith(("left_link", "right_link")):
            for g in b.findall("geom"): g.set("contype", "0"); g.set("conaffinity", "0")
    for g in wb.findall("geom"):
        if g.get("type") == "mesh": g.set("contype", "0"); g.set("conaffinity", "0")
    for a in r.find("actuator"):  # web scene has no velocity feedback; add kv = KV_RATIO*kp (stable flex contacts)
        kp = float(a.get("gainprm")); a.set("biasprm", f"0 {-kp} {-KV_RATIO * kp}")
    t = next(g for g in wb.iter("geom") if g.get("name") == "table")
    t.attrib.update(friction=table_friction, solref="0.002 1", solimp="0.95 0.99 0.0005", contype="2", conaffinity="2")
    # flex objects touch the table through 1 cm tiles, each its own static body (max 50 contacts per body-flex pair)
    n, a = TILES; tc = tiles_c or TILES_C
    for i in range(n):
        for j in range(n):
            ET.SubElement(ET.SubElement(wb, "body", name=f"tile{i}_{j}"), "geom", name=f"tile{i}_{j}", type="box", size=f"{a / 2} {a / 2} 0.005",
                          pos=f"{tc[0] + (i - n / 2 + 0.5) * a} {tc[1] + (j - n / 2 + 0.5) * a} -0.005",
                          friction=table_friction, solref="0.002 1", solimp="0.95 0.99 0.0005", contype="1",
                          conaffinity="1", rgba="0 0 0 0", group="3")
    for j in r.iter("joint"):  # reflected motor inertia, as the X2 scene (0.01): without it the light Piper links +
        if j.get("name", "").startswith(("left_joint", "right_joint")): j.set("armature", str(ARMATURE))  # stiff gains + flex shake
    return r, wb


def fingers_collide(r, side, on):
    for b in (f"{side}_link7", f"{side}_link8"):
        for g in find_body(r, b).findall("geom"):
            g.set("contype", "1" if on else "0"); g.set("conaffinity", "1" if on else "0")


def compile_root(r):
    return mujoco.MjModel.from_xml_string(ET.tostring(r, encoding="unicode"))


def ik(m, side, target, q, offset=(0, 0, TIP_Z), zax=(0, 0, -1), xax=None, iters=400, w=0.3):
    """Damped least squares: point `offset` (link6 frame) to target, link6 z -> zax, link6 x -> xax."""
    b = m.body(f"{side}_link6").id; names = arm(side)
    dofs = [m.joint(j).dofadr[0] for j in names]; qa = [m.joint(j).qposadr[0] for j in names]
    rng = np.array([m.jnt_range[m.joint(j).id] for j in names]); dd = mujoco.MjData(m); q = q.copy()
    for _ in range(iters):
        dd.qpos[:] = q; mujoco.mj_kinematics(m, dd); mujoco.mj_comPos(m, dd)
        R = dd.xmat[b].reshape(3, 3); p = dd.xpos[b] + R @ np.asarray(offset)
        jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv)); mujoco.mj_jac(m, dd, jp, jr, p, b)
        er = np.cross(R[:, 2], zax) + (np.cross(R[:, 0], xax) if xax is not None else 0)
        J = np.vstack([jp[:, dofs], w * jr[:, dofs]]); e = np.hstack([np.asarray(target) - p, w * er])
        q[qa] = np.clip(q[qa] + 0.5 * J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e), rng[:, 0], rng[:, 1])
    dd.qpos[:] = q; mujoco.mj_kinematics(m, dd)
    R = dd.xmat[b].reshape(3, 3); p = dd.xpos[b] + R @ np.asarray(offset)
    ang = float(np.degrees(np.arccos(np.clip(R[:, 2] @ np.asarray(zax), -1, 1))))
    return q, float(np.linalg.norm(np.asarray(target) - p)), ang


def ik_restarts(m, side, target, n=12, seed=0, **kw):
    rs = np.random.RandomState(seed); qa = [m.joint(j).qposadr[0] for j in arm(side)]
    rng = np.array([m.jnt_range[m.joint(j).id] for j in arm(side)]); best = None
    for k in range(n):
        q0 = np.zeros(m.nq); q0[qa] = HOME if k == 0 else rs.uniform(rng[:, 0], rng[:, 1])
        q, e, a = ik(m, side, target, q0, iters=600, **kw)
        if best is None or e + 0.001 * a < best[1] + 0.001 * best[2]: best = (q, e, a)
    return best


def set_ctrl(m, d, q, grip=None):
    """q: full qpos target for arm joints; grip: {side: finger opening per finger (0 = closed, 0.035 = open)}."""
    for i in range(m.nu):
        j = m.actuator_trnid[i, 0]; n = m.joint(j).name
        if n.endswith(("joint7", "joint8")):
            side = n.split("_")[0]; g = (grip or {}).get(side, 0.0)
            d.ctrl[i] = g if n.endswith("7") else -g
        else:
            d.ctrl[i] = q[m.jnt_qposadr[j]]


def smooth(u): return 0.5 - 0.5 * np.cos(np.pi * np.clip(u, 0, 1))


# ---------------------------------------------------------------- butter
def add_butter(wb, rest=None, friction="1.0 0.005 0.0001"):
    sx, sy, sz = BUTTER["size"]; nx, ny, nz = BUTTER["count"]
    mass = BUTTER["density"] * sx * sy * sz
    common = dict(name="butter", dim="3", mass=f"{mass:.5f}", radius=str(RADIUS), rgba="1 0.88 0.35 1")
    if rest is None:
        fc = ET.SubElement(wb, "flexcomp", type="grid", count=f"{nx} {ny} {nz}",
                           spacing=f"{sx/(nx-1)} {sy/(ny-1)} {sz/(nz-1)}",
                           pos=f"{BUTTER_XY[0]} {BUTTER_XY[1]} {TABLE_TOP + sz/2 + RADIUS}", **common)
    else:
        fc = ET.SubElement(wb, "flexcomp", type="direct", point=" ".join(f"{v:.6f}" for v in np.asarray(rest).ravel()),
                           element=" ".join(map(str, XB._ELEMENTS)), **common)
    ET.SubElement(fc, "edge", equality="false")
    ET.SubElement(fc, "elasticity", young=str(BUTTER["young"]), poisson=str(BUTTER["poisson"]),
                  damping=str(BUTTER["damping"]))
    ET.SubElement(fc, "contact", selfcollide="none", condim="3", solref="0.002 1", solimp="0.95 0.99 0.0005",
                  friction=friction)


def add_tip_proxy(l6):
    """Closed-fingertip envelope, split 6 x 4 into separate (welded child) bodies: MuJoCo keeps at most 50
    contacts per body-flex pair."""
    (px, py, pz), (sx, sy, sz) = TIP_PROXY["pos"], TIP_PROXY["size"]
    nx, ny = PROXY_SPLIT
    for i in range(nx):
        for j in range(ny):
            ET.SubElement(ET.SubElement(l6, "body", name=f"left_tip_proxy{i}_{j}"), "geom", name=f"left_tip_proxy{i}_{j}", type="box",
                          pos=f"{px - sx + sx / nx * (2 * i + 1)} {py - sy + sy / ny * (2 * j + 1)} {pz}",
                          size=f"{sx / nx + 0.0005} {sy / ny + 0.0005} {sz}", rgba="0.2 0.8 1 0", group="3", friction="0.3 0.005 0.0001")


def build_press(rest=None, offscreen=(1280, 720)):
    r, wb = base_root(offscreen, "0.8 0.005 0.0001")
    fingers_collide(r, "left", False); fingers_collide(r, "right", False)
    l6 = find_body(r, "left_link6")
    add_tip_proxy(l6)
    add_butter(wb, rest)
    return compile_root(r)


def butter_verts(m, d, f=0):
    a = m.flex_vertadr[f]; n = m.flex_vertnum[f]
    return d.flexvert_xpos[a:a + n].copy()


def butter_metrics(v):
    nz = BUTTER["count"][2]; top = v.reshape(-1, nz, 3)[:, -1, 2] - TABLE_TOP - RADIUS
    return dict(height=float(np.ptp(v[:, 2])), max_height=float(v[:, 2].max() - TABLE_TOP - RADIUS),
                top_std=float(top.std()), top_mean=float(top.mean()),
                footprint=float(np.ptp(v[:, 0]) * np.ptp(v[:, 1])), zmin=float(v[:, 2].min()))


_TETS = np.array(XB._ELEMENTS).reshape(-1, 4)


def tet_volumes(x):
    a, b, c, e = (x[_TETS[:, i]] for i in range(4))
    return np.einsum("ij,ij->i", np.cross(b - a, c - a), e - a) / 6


def safe_yield(rest0, v, vmin=SAFE_VMIN):
    """Largest yield <= YIELD whose new rest shape keeps every tet >= vmin x its volume in the initial block
    (Piper's broad fingertips flatten the block much more evenly than the X2 tips; fully yielded, the rest shape
    gets near-degenerate tets and the flex explodes after ~6 presses)."""
    global _V0
    if vmin is None: return YIELD
    if _V0 is None:
        m = build_press(); d = mujoco.MjData(m); mujoco.mj_forward(m, d); _V0 = np.abs(tet_volumes(butter_verts(m, d)))
    v0 = _V0
    for a in np.linspace(YIELD, 0, 17):
        vol = tet_volumes(rest0 + a * (v - rest0)) * np.sign(tet_volumes(rest0))
        if (vol >= vmin * v0).all(): return float(a)
    return 0.0


_V0 = None


def rebuild_with_yield(m, d, rest0, builder):
    """Plasticity approximation (as sim/x2): rest_new = rest0 + YIELD*(current - rest0), state carried over."""
    v = butter_verts(m, d); a = safe_yield(rest0, v); rest_new = rest0 + a * (v - rest0)
    rebuild_with_yield.__dict__.setdefault("alphas", []).append(a)
    m2 = builder(rest_new); d2 = mujoco.MjData(m2); nrob = m.nq - 3 * len(v)
    d2.qpos[:nrob] = d.qpos[:nrob]; d2.qvel[:nrob] = d.qvel[:nrob]; d2.qpos[nrob:] = 0
    mujoco.mj_forward(m2, d2); base = butter_verts(m2, d2)
    d2.qpos[nrob:] = (v - base).ravel(); d2.qvel[nrob:] = d.qvel[nrob:]
    d2.ctrl[:] = d.ctrl; d2.time = d.time; mujoco.mj_forward(m2, d2)
    return m2, d2, rest_new


def press_schedule():
    segs = [("approach", 0.6, 0, 0.0, 0.0)]
    for k in range(len(PRESS_OFFSETS)):
        segs += [("descend", 0.8, k, 0.0, 1.0), ("hold", 0.3, k, 1.0, 1.0), ("lift", 0.5, k, 1.0, 0.0)]
    return segs + [("rest", 1.0, len(PRESS_OFFSETS) - 1, 0.0, 0.0)]


PRESS_SCHEDULE = press_schedule()


def segmenter(schedule):
    T0 = np.cumsum([0] + [s[1] for s in schedule])
    def seg(t):
        i = int(np.clip(np.searchsorted(T0, t, side="right") - 1, 0, len(schedule) - 1))
        return i, schedule[i], float(np.clip((t - T0[i]) / schedule[i][1], 0, 1))
    return seg, float(T0[-1])


def interp(path, s):
    x = s * (len(path) - 1); i = min(int(x), len(path) - 2); f = x - i
    return (1 - f) * path[i] + f * path[i + 1]


def plan_press(m):
    """Press raster around the butter centre; each press is re-planned (IK) on the *measured* centre when it starts
    (the butter creeps a few mm per press; a fixed raster then walks off the block)."""
    qa = [m.joint(j).qposadr[0] for j in arm("left")]
    zs = np.linspace(0.05, PRESS_GAP, 11); errs = [0.0]; paths = {}
    q0, e, _ = ik_restarts(m, "left", (BUTTER_XY[0], BUTTER_XY[1], zs[0]), xax=(1, 0, 0)); state = dict(q=q0)

    def make(k, c):
        q = state["q"].copy(); dx, dy = PRESS_OFFSETS[k]; path = []
        for z in zs:
            q, e, _ = ik(m, "left", (c[0] + dx, c[1] + dy, z), q, xax=(1, 0, 0)); errs.append(e); path.append(q[qa].copy())
        paths[k] = np.array(path); state["q"] = q.copy(); state["q"][qa] = path[0]
    make(0, BUTTER_XY)
    seg, T = segmenter(PRESS_SCHEDULE)

    def target(t, center=None):
        i, (name, dur, k, s0, s1), u = seg(t); sm = s0 + (s1 - s0) * smooth(u)
        if name == "lift" and u > 0.6 and k + 1 < len(PRESS_OFFSETS) and k + 1 not in paths: make(k + 1, center)
        if name == "lift":  # straight up first (no sideways drag of the stuck block), then over to the next spot
            pa = interp(paths[k], 1 - smooth(u / 0.6))
            if k + 1 < len(PRESS_OFFSETS) and u > 0.6:
                a = smooth((u - 0.6) / 0.4); pa = (1 - a) * paths[k][0] + a * paths[k + 1][0]
        else:
            pa = interp(paths[k], sm)
        qt = np.zeros(m.nq); qt[qa] = pa; return qt
    target.errs = errs
    return target, seg, T, 0.0


def simulate_press(plastic=True, callback=None, dt_cb=1 / 30):
    global _V0
    m = build_press(); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    rest = butter_verts(m, d); m0 = butter_metrics(rest); _V0 = np.abs(tet_volumes(rest)); rebuild_with_yield.alphas = []
    target, seg, T, ik_err = plan_press(m); nrob = m.nq - 3 * len(rest)
    d.qpos[:nrob] = target(0)[:nrob]; mujoco.mj_forward(m, d)
    res = dict(initial=m0, min_top=m0["top_mean"], finite=True, yields=0, pressed=[], ik_max_err=ik_err)
    t = 0.0; nxt = 0.0; last = 0
    while t < T - 1e-9:
        i, sg, u = seg(t)
        if i != last:
            if PRESS_SCHEDULE[last][0] == "hold":
                res["pressed"].append(butter_metrics(butter_verts(m, d)))
                if plastic:
                    m, d, rest = rebuild_with_yield(m, d, rest, lambda r: build_press(r)); res["yields"] += 1
            last = i
        v = butter_verts(m, d)
        set_ctrl(m, d, target(t, (v[:, :2].min(0) + v[:, :2].max(0)) / 2)); mujoco.mj_step(m, d); t = d.time
        if not np.isfinite(d.qpos).all(): res["finite"] = False; break
        res["min_top"] = min(res["min_top"], butter_metrics(butter_verts(m, d))["top_mean"])
        if callback and t >= nxt:
            callback(m, d, t, sg[0] if sg[0] in ("approach", "rest") else f"{sg[0]} {sg[2] + 1}/9"); nxt += dt_cb
    v = butter_verts(m, d); res["ik_max_err"] = max(target.errs); res["yield_alphas"] = list(rebuild_with_yield.alphas)
    res["final"] = butter_metrics(v); res["final_verts"] = v; res["drift_mm"] = 1000 * float(np.linalg.norm(
        (v[:, :2].min(0) + v[:, :2].max(0)) / 2 - np.array(BUTTER_XY)))
    return res
