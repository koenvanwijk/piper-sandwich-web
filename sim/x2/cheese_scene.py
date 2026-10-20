"""MuJoCo scene: X2 separates the top slice from a stack of 10 cheese slices with the table knife (left hand)
and picks it with the right OmniPicker. MuJoCo only, NOT Isaac Sim.

Modelling (approximations, see docs/X2.md):
  - 10 slices, 8 x 8 cm x 2 mm, each a 2D flex shell (flexcomp grid 6x6, elastic2d="both", thickness 2 mm,
    E = 0.5 MPa, nu = 0.4), density ~1100 kg/m^3 -> 14 g per slice.
  - slice-slice friction 0.8 stands in for 'light adhesion' (MuJoCo flex contacts have no attractive force).
  - knife: same rigid table knife (40 g), rigidly fixed in the closed LEFT gripper (weld-equivalent); in this
    scene the real 1.5 mm blade collides, with contact priority so knife-cheese friction is 0.2.
  - right-gripper grasp: no finger-cheese contact; when the closed gripper is at the slice edge an equality
    'connect' between gripper_r_center_link and the 3 nearest top-slice vertices is switched on (weld-like grasp).
  - cutting/tearing cannot be simulated (flex has no fracture).
"""
import os, time
import numpy as np, mujoco
import xml.etree.ElementTree as ET
import butter_scene as B
import knife as K

N, T, SIZE, COUNT = 10, 0.002, 0.08, 6
STOP = True
CHEESE = dict(young=5e5, poisson=0.4, density=1100.0, friction=0.8, damping=0.002)
STACK_XY = (0.27, 0.02)
TABLE_TOP = -0.018
ARM_R = [j.replace("left", "right") for j in B.ARM_L]
LEVEL = np.radians(K.TILT_DEG)  # wrist turned so the (15 deg mounted) blade is level


def slice_mass():
    return CHEESE["density"] * SIZE * SIZE * T


def build(offscreen=(1280, 720)):
    if "xml_thin" not in B._ROBOT_CACHE:
        B._ROBOT_CACHE.setdefault("dir", __import__("tempfile").mkdtemp())
        B._ROBOT_CACHE["xml_thin"] = B._robot_xml(B._ROBOT_CACHE["dir"], knife="thin")
    r = ET.parse(B._ROBOT_CACHE["xml_thin"]).getroot()
    r.find("compiler").set("meshdir", B.X2)
    opt = r.find("option")
    if opt is None: opt = ET.SubElement(r, "option")
    opt.attrib.update(timestep="0.001", integrator="discrete", solver="CG", iterations="60", tolerance="1e-8")
    ET.SubElement(ET.SubElement(r, "visual"), "global", offwidth=str(offscreen[0]), offheight=str(offscreen[1]))
    wb = r.find("worldbody")
    ET.SubElement(wb, "light", pos="0 -1 3", dir="0 0.3 -1", diffuse="0.8 0.8 0.8", castshadow="false")
    ET.SubElement(wb, "light", pos="1.5 1.5 2", dir="-1 -1 -1", diffuse="0.4 0.4 0.4", castshadow="false")
    ET.SubElement(wb, "geom", name="floor", type="plane", size="3 3 0.1", pos="0 0 -0.68", rgba="0.35 0.4 0.45 1",
                  contype="0", conaffinity="0")
    ET.SubElement(wb, "geom", name="table", type="box", size="0.12 0.12 0.01",
                  pos=f"{STACK_XY[0]} {STACK_XY[1]} {TABLE_TOP - 0.01}", rgba="0.55 0.4 0.3 1",
                  contype="1", conaffinity="1", friction="1.0 0.005 0.0001", solref="0.002 1", solimp="0.95 0.99 0.0005")
    # cheese holder: low stop on the far (-y) side, up to just below the top slice, keeps the lower 9 slices
    # from being pushed off by the blade (a fixture; without it the whole stack slides, see docs/X2.md)
    if STOP:
        h = INTERFACE_Z - 0.0005 - TABLE_TOP
        ET.SubElement(wb, "geom", name="stop", type="box", size=f"{SIZE / 2 + 0.01} 0.003 {h / 2}",
                      pos=f"{STACK_XY[0]} {STACK_XY[1] - SIZE / 2 - T / 2 - 0.003} {TABLE_TOP + h / 2}",
                      rgba="0.45 0.3 0.2 1", contype="1", conaffinity="1", friction="1.0 0.005 0.0001")
    sp = SIZE / (COUNT - 1)
    for i in range(N):
        fc = ET.SubElement(wb, "flexcomp", name=f"cheese{i}", type="grid", dim="2", count=f"{COUNT} {COUNT} 1",
                           spacing=f"{sp} {sp} {sp}", pos=f"{STACK_XY[0]} {STACK_XY[1]} {TABLE_TOP + T / 2 + i * T}",
                           mass=f"{slice_mass():.5f}", radius=f"{T / 2}",
                           rgba="1 0.85 0.3 1" if i % 2 else "0.98 0.78 0.25 1")
        ET.SubElement(fc, "edge", equality="false")
        ET.SubElement(fc, "elasticity", young=str(CHEESE["young"]), poisson=str(CHEESE["poisson"]),
                      thickness=str(T), damping=str(CHEESE["damping"]), elastic2d="both")
        ET.SubElement(fc, "contact", selfcollide="none", condim="3", friction=f"{CHEESE['friction']} 0.005 0.0001",
                      solref="0.003 1", solimp="0.95 0.99 0.0005")
    eq = ET.SubElement(r, "equality")
    for k in range(3):
        ET.SubElement(eq, "connect", name=f"grasp{k}", body1="gripper_r_center_link", body2=f"cheese{N - 1}_0",
                      anchor="0 0 0", active="false")
    act = r.find("actuator")
    if act is None: act = ET.SubElement(r, "actuator")
    for j in r.iter("joint"):
        n = j.get("name")
        if not n or j.get("type") not in (None, "hinge"): continue
        stiff = n.startswith(B.LOCKED_PREFIX)
        kp = "5000" if stiff else ("2000" if n in B.ARM_L or n in ARM_R else "300")
        ET.SubElement(act, "position", name=f"a_{n}", joint=n, kp=kp, kv="50" if stiff else "20")
    return mujoco.MjModel.from_xml_string(ET.tostring(r, encoding="unicode"))


def slice_verts(m, d, i):
    a, n = m.flex_vertadr[i], m.flex_vertnum[i]
    return d.flexvert_xpos[a:a + n].copy()


def ik(m, arm, body, offset, target, zax, xax, q, iters=400, w=0.3):
    b = m.body(body).id; dofs = [m.joint(j).dofadr[0] for j in arm]; qa = [m.joint(j).qposadr[0] for j in arm]
    rng = np.array([m.jnt_range[m.joint(j).id] for j in arm]); dd = mujoco.MjData(m); q = q.copy()
    for _ in range(iters):
        dd.qpos[:] = q; mujoco.mj_kinematics(m, dd); mujoco.mj_comPos(m, dd)
        R = dd.xmat[b].reshape(3, 3); p = dd.xpos[b] + R @ np.asarray(offset)
        jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv)); mujoco.mj_jac(m, dd, jp, jr, p, b)
        er = np.cross(R[:, 2], zax) + (np.cross(R[:, 0], xax) if xax is not None else 0)
        J = np.vstack([jp[:, dofs], w * jr[:, dofs]]); e = np.hstack([target - p, w * er])
        q[qa] = np.clip(q[qa] + 0.5 * J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e), rng[:, 0], rng[:, 1])
    return q, float(np.linalg.norm(target - p))


# timeline (s)
SCHEDULE = (("settle", 0.6), ("knife_down", 0.6), ("insert", 2.0), ("knife_lift", 0.6), ("reach", 1.2),
            ("grasp", 0.2), ("pick", 1.2), ("hold", 0.8))
T_TOTAL = sum(s[1] for s in SCHEDULE)
INTERFACE_Z = TABLE_TOP + (N - 1) * T            # between slice N-2 and the top slice N-1
BLADE_Z = INTERFACE_Z - K.BLADE["size"][2]       # blade top face at the interface: rides over slice N-2
INSERT_Y = (0.12, 0.03)                           # blade centre y: tip just outside the stack -> under the slice
BLADE_X = STACK_XY[0] + 0.01
LIFT = 0.006
GRASP_XY = (STACK_XY[0] - 0.035, STACK_XY[1] - 0.035)  # near corner of the top slice (not over the blade)
# TCP (gripper_r_center_link) target above the slice corner; the closed fingertips reach ~2 cm below the TCP
GRASP_Z = INTERFACE_Z + LIFT + 0.014
PICK_UP = 0.06


def plan(m):
    zl = np.array([-np.sin(LEVEL), 0, -np.cos(LEVEL)])
    qL = np.zeros(m.nq); qa = [m.joint(j).qposadr[0] for j in B.ARM_L]; qL[qa] = [-0.5, 0.2, 0, -1.6, 0, 0, 0]
    qR = np.zeros(m.nq); qb = [m.joint(j).qposadr[0] for j in ARM_R]
    errs = []
    def L(y, z, q):
        q, e = ik(m, B.ARM_L, "L_omnipicker_base_link", K.BLADE["pos"], np.array([BLADE_X, y, z]), zl, (0, 1, 0), q)
        errs.append(e); return q
    wL = {}
    q = L(INSERT_Y[0], BLADE_Z + 0.03, qL); wL["above"] = q[qa].copy()
    q = L(INSERT_Y[0], BLADE_Z, q); wL["start"] = q[qa].copy()
    ys = np.linspace(*INSERT_Y, 6); wL["insert"] = []
    for y in ys:
        q = L(y, BLADE_Z, q); wL["insert"].append(q[qa].copy())
    q = L(INSERT_Y[1], BLADE_Z + LIFT, q); wL["lifted"] = q[qa].copy()
    # right arm: random-restart IK for the first waypoint, then continuation
    rngR = np.array([m.jnt_range[m.joint(j).id] for j in ARM_R]); rs = np.random.RandomState(0)
    tgt0 = np.array([GRASP_XY[0], GRASP_XY[1], GRASP_Z]); best = (9, None)
    for _ in range(16):  # random restarts: the right wrist is close to its limits here
        q0 = np.zeros(m.nq); q0[qb] = rs.uniform(rngR[:, 0] * 0.6, rngR[:, 1] * 0.6)
        q1, e = ik(m, ARM_R, "R_omnipicker_base_link", (0, 0, 0.13), tgt0, np.array([0, 0, -1.0]), None, q0, 500, 0.03)
        if e < best[0]: best = (e, q1)
    def R(p, q):
        q, e = ik(m, ARM_R, "R_omnipicker_base_link", (0, 0, 0.13), np.asarray(p), np.array([0, 0, -1.0]), None, q,
                  500, 0.03)
        errs.append(e); return q
    errs.append(best[0]); wR = {"grasp": best[1][qb].copy()}
    q = R((GRASP_XY[0], GRASP_XY[1], GRASP_Z + 0.05), best[1]); wR["above"] = q[qb].copy()
    q = R((GRASP_XY[0], GRASP_XY[1] - 0.02, GRASP_Z + PICK_UP), best[1]); wR["up"] = q[qb].copy()
    T0 = np.cumsum([0] + [s[1] for s in SCHEDULE])

    def seg(t):
        i = int(np.clip(np.searchsorted(T0, t, side="right") - 1, 0, len(SCHEDULE) - 1))
        return SCHEDULE[i][0], float(np.clip((t - T0[i]) / SCHEDULE[i][1], 0, 1))

    def target(t):
        name, u = seg(t); s = 0.5 - 0.5 * np.cos(np.pi * u); qt = np.zeros(m.nq)
        mix = lambda a, b: (1 - s) * a + s * b
        if name == "settle": l = wL["above"]
        elif name == "knife_down": l = mix(wL["above"], wL["start"])
        elif name == "insert":
            x = s * (len(ys) - 1); i = min(int(x), len(ys) - 2); f = x - i
            l = (1 - f) * wL["insert"][i] + f * wL["insert"][i + 1]
        elif name == "knife_lift": l = mix(wL["insert"][-1], wL["lifted"])
        else: l = wL["lifted"]
        if name in ("settle", "knife_down", "insert", "knife_lift"): r = wR["above"]
        elif name == "reach": r = mix(wR["above"], wR["grasp"])
        elif name == "grasp": r = wR["grasp"]
        elif name == "pick": r = mix(wR["grasp"], wR["up"])
        else: r = wR["up"]
        qt[qa] = l; qt[qb] = r
        if name in ("reach", "grasp", "pick", "hold"):  # right claw closes while reaching (0 = closed)
            pass
        return qt
    return target, seg, max(errs)


def activate_grasp(m, d):
    """Switch on 'connect' constraints between the right gripper centre and the 3 nearest top-slice vertices."""
    b1 = m.body("gripper_r_center_link").id; p1 = d.xpos[b1]; R1 = d.xmat[b1].reshape(3, 3)
    v = slice_verts(m, d, N - 1); idx = np.argsort(np.linalg.norm(v - p1, axis=1))[:3]
    a = m.flex_vertadr[N - 1]
    for k, vi in enumerate(idx):
        e = m.equality(f"grasp{k}").id
        m.eq_obj2id[e] = m.flex_vertbodyid[a + vi]
        m.eq_data[e, 0:3] = R1.T @ (v[vi] - p1); m.eq_data[e, 3:6] = 0
        d.eq_active[e] = 1
    return float(np.linalg.norm(v[idx[0]] - p1))


def simulate(callback=None, dt_cb=1 / 30):
    m = build(); d = mujoco.MjData(m)
    target, seg, ik_err = plan(m)
    nflexq = sum(3 * m.flex_vertnum[i] for i in range(N)); nrob = m.nq - nflexq
    d.qpos[:nrob] = target(0)[:nrob]
    claw = m.joint("right_claw_joint").qposadr[0]
    mujoco.mj_forward(m, d)
    v0 = [slice_verts(m, d, i) for i in range(N)]
    locked = [j for j in range(m.njnt) if m.joint(j).name.startswith(B.LOCKED_PREFIX)]
    res = dict(ik_max_err=ik_err, finite=True, max_locked_drift=0.0, grasp_dist=None)
    nxt = 0.0; t = 0.0; grasped = False; wall = time.time(); settle_ref = None
    while t < T_TOTAL - 1e-9:
        name, u = seg(t)
        qt = target(t)
        if name in ("reach", "grasp", "pick", "hold"): qt[claw] = 0.0
        else: qt[claw] = -1.0  # open
        if name == "knife_down" and settle_ref is None:
            settle_ref = [slice_verts(m, d, i) for i in range(N)]
        if name == "grasp" and not grasped:
            res["grasp_dist"] = activate_grasp(m, d); grasped = True
        B.ctrl_from_q(m, d, qt); mujoco.mj_step(m, d); t = d.time
        if not np.isfinite(d.qpos).all(): res["finite"] = False; break
        res["max_locked_drift"] = max(res["max_locked_drift"], max(abs(d.qpos[m.jnt_qposadr[j]]) for j in locked))
        if callback and t >= nxt: callback(m, d, t, name); nxt += dt_cb
    res["wall_per_sim_s"] = (time.time() - wall) / max(t, 1e-9)
    ref = settle_ref or v0
    vs = [slice_verts(m, d, i) for i in range(N)]
    rise = [float(v[:, 2].mean() - r[:, 2].mean()) for v, r in zip(vs, ref)]
    shift = [float(np.linalg.norm(v[:, :2].mean(0) - r[:, :2].mean(0))) for v, r in zip(vs, ref)]
    res.update(rise=rise, shift=shift, n_lifted=int(sum(r > 0.02 for r in rise)),
               top_lifted=bool(rise[-1] > 0.02), max_shift_rest_mm=1000 * max(shift[:-1]),
               deflection_mm=1000 * float(np.ptp(vs[-1][:, 2])), settle_sag_mm=1000 * max(
                   float(np.ptp(r[:, 2])) for r in ref[:-1]))
    return res
