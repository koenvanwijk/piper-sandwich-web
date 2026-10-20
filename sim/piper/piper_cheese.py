"""Dual Piper: the left arm slides a table knife between the top cheese slice and the stack, the right gripper
pinches the lifted slice at its corner. Same cheese model and knife as sim/x2/cheese_scene.py. MuJoCo, NOT Isaac Sim.

Piper-specific:
  - knife = the Piper (web-demo) knife, blade now 1.5 mm like the X2 knife (rubber handle, 60 g);
    mounted level in the left hand (weld-equivalent).
  - right gripper: parallel jaw, max ~70 mm opening. It approaches the slice corner from the side (jaw axis ~vertical),
    pitched 30 deg down, one finger above, one under the 6 mm-lifted corner, and closes. The pinch is modelled as 'connect' constraints
    between the fingertip centre and the 3 nearest slice vertices (switched on when the jaw closes); finger meshes
    do not collide with the flex.
"""
import os, sys, time
import numpy as np, mujoco
import xml.etree.ElementTree as ET
import piper_scene as P
import piper_knife as K
import cheese_scene as XC  # sim/x2 (on sys.path via piper_scene)

N, T, SIZE, COUNT, CHEESE = XC.N, XC.T, XC.SIZE, XC.COUNT, XC.CHEESE
STACK_XY = (0.06, 0.0)
INTERFACE_Z = P.TABLE_TOP + (N - 1) * T
BLADE = K.BLADE
BLADE_Z = INTERFACE_Z - BLADE["size"][2]
INSERT_Y = (STACK_XY[1] + 0.10, STACK_XY[1] + 0.01)   # blade centre y: tip outside the stack -> under the slice
BLADE_X = STACK_XY[0] + 0.01
LIFT = 0.006
DIAG = np.array([1.0, 1.0, 0.0]) / np.sqrt(2)         # right hand approaches the (-x,-y) corner along +x+y
CORNER = np.array([STACK_XY[0] - SIZE / 2, STACK_XY[1] - SIZE / 2])
GRASP_IN = 0.012                                      # fingertip centre 12 mm inside the corner
JAW_OPEN, JAW_CLOSED = 0.03, 0.001                    # per finger (Piper max 0.035)
PICK_UP = 0.06
PITCH_DEG = 30.0   # hand pitched 30 deg down: a fully horizontal approach at 2.4 cm height is out of reach (wrist)
SCHEDULE = (("settle", 0.6), ("knife_down", 0.6), ("insert", 2.0), ("knife_lift", 0.6), ("reach", 1.4),
            ("grasp", 0.3), ("pick", 1.2), ("hold", 0.8))
TILES_C = STACK_XY
SAG = []


def build(offscreen=(1280, 720)):
    r, wb = P.base_root(offscreen, "1.0 0.005 0.0001", tiles_c=TILES_C)
    for s in ("left", "right"): P.fingers_collide(r, s, False)
    K.attach_knife(r, tilt_deg=0.0, segments=False, kind="web")
    h = INTERFACE_Z - 0.0005 - P.TABLE_TOP  # holder stop on the far (-y) side, as sim/x2
    ET.SubElement(wb, "geom", name="stop", type="box", size=f"{SIZE / 2 + 0.01} 0.003 {h / 2}",
                  pos=f"{STACK_XY[0]} {STACK_XY[1] - SIZE / 2 - T / 2 - 0.003} {P.TABLE_TOP + h / 2}",
                  rgba="0.45 0.3 0.2 1", friction="1.0 0.005 0.0001")
    sp = SIZE / (COUNT - 1)
    for i in range(N):
        fc = ET.SubElement(wb, "flexcomp", name=f"cheese{i}", type="grid", dim="2", count=f"{COUNT} {COUNT} 1",
                           spacing=f"{sp} {sp} {sp}", pos=f"{STACK_XY[0]} {STACK_XY[1]} {P.TABLE_TOP + T / 2 + i * T}",
                           mass=f"{XC.slice_mass():.5f}", radius=f"{T / 2}",
                           rgba="1 0.85 0.3 1" if i % 2 else "0.98 0.78 0.25 1")
        ET.SubElement(fc, "edge", equality="false")
        ET.SubElement(fc, "elasticity", young=str(CHEESE["young"]), poisson=str(CHEESE["poisson"]),
                      thickness=str(T), damping=str(CHEESE["damping"]), elastic2d="both")
        ET.SubElement(fc, "contact", selfcollide="none", condim="3", friction=f"{CHEESE['friction']} 0.005 0.0001",
                      solref="0.003 1", solimp="0.95 0.99 0.0005")
    ET.SubElement(P.find_body(r, "right_link6"), "site", name="right_pinch", pos=f"0 0 {P.TIP_Z - 0.005}", size="0.003",
                  group="4")
    eq = ET.SubElement(r, "equality")
    for k in range(3):
        ET.SubElement(eq, "connect", name=f"grasp{k}", body1="right_link6", body2=f"cheese{N - 1}_0", anchor="0 0 0",
                      active="false")
    return P.compile_root(r)


slice_verts = XC.slice_verts


def plan(m):
    qa = [m.joint(j).qposadr[0] for j in P.arm("left")]; qb = [m.joint(j).qposadr[0] for j in P.arm("right")]
    errs = []; XAX = (0, -1, 0)

    dz = [0.0]

    def L(y, z, q):
        q, e, _ = P.ik(m, "left", (BLADE_X, y, z + dz[0]), q, offset=BLADE["pos"], xax=XAX); errs.append(e); return q
    q0, e, _ = P.ik_restarts(m, "left", (BLADE_X, INSERT_Y[0], BLADE_Z + 0.03), offset=BLADE["pos"], xax=XAX)
    # static sag of the arm under gravity (the Piper position servos are not stiff enough for a 2 mm seam):
    # hold the start pose (knife outside the stack), measure the blade height error, re-aim all left waypoints
    q = L(INSERT_Y[0], BLADE_Z, q0); d = mujoco.MjData(m); nrob = m.nq - sum(3 * m.flex_vertnum[i] for i in range(N))
    d.qpos[:nrob] = q[:nrob]; mujoco.mj_forward(m, d); g = m.geom("knife_blade").id
    for _ in range(400): P.set_ctrl(m, d, q, {"left": K.GRIP_OPEN, "right": JAW_OPEN}); mujoco.mj_step(m, d)
    dz[0] = BLADE_Z - d.geom_xpos[g][2]; SAG.append(dz[0])
    q = q0.copy(); q, e, _ = P.ik(m, "left", (BLADE_X, INSERT_Y[0], BLADE_Z + 0.03 + dz[0]), q, offset=BLADE["pos"], xax=XAX)
    wL = {"above": q[qa].copy()}
    q = L(INSERT_Y[0], BLADE_Z, q); wL["start"] = q[qa].copy()
    ys = np.linspace(*INSERT_Y, 6); wL["insert"] = []
    for y in ys: q = L(y, BLADE_Z, q); wL["insert"].append(q[qa].copy())
    q = L(INSERT_Y[1], BLADE_Z + LIFT, q); wL["lifted"] = q[qa].copy()
    # right: horizontal approach, link6 z along DIAG, jaw (link6 y) vertical -> link6 x = y x z
    a = np.radians(PITCH_DEG); zR = np.array([np.cos(a) * DIAG[0], np.cos(a) * DIAG[1], -np.sin(a)])
    xR = np.cross([0, 0, 1.0], DIAG); off = (0, 0, P.TIP_Z - 0.005)
    g = np.array([*(CORNER + GRASP_IN * DIAG[:2]), INTERFACE_Z + LIFT + T / 2])
    def R(p, q):
        q, e, _ = P.ik(m, "right", p, q, offset=off, zax=zR, xax=xR); errs.append(e); return q
    q, e, _ = P.ik_restarts(m, "right", g - 0.05 * DIAG, offset=off, zax=zR, xax=xR, n=16)
    wR = {"above": q[qb].copy()}
    q = R(g, q); wR["grasp"] = q[qb].copy()
    q = R(g + np.array([0, 0, PICK_UP]) - 0.02 * DIAG, q); wR["up"] = q[qb].copy()
    T0 = np.cumsum([0] + [s[1] for s in SCHEDULE])

    def seg(t):
        i = int(np.clip(np.searchsorted(T0, t, side="right") - 1, 0, len(SCHEDULE) - 1))
        return SCHEDULE[i][0], float(np.clip((t - T0[i]) / SCHEDULE[i][1], 0, 1))

    def target(t):
        name, u = seg(t); s = P.smooth(u); qt = np.zeros(m.nq); mix = lambda a, b: (1 - s) * a + s * b
        l = {"settle": wL["above"], "knife_down": mix(wL["above"], wL["start"]),
             "knife_lift": mix(wL["insert"][-1], wL["lifted"])}.get(name, wL["lifted"])
        if name == "insert": l = P.interp(np.array(wL["insert"]), s)
        r_ = {"reach": mix(wR["above"], wR["grasp"]), "grasp": wR["grasp"], "pick": mix(wR["grasp"], wR["up"]),
              "hold": wR["up"]}.get(name, wR["above"])
        qt[qa] = l; qt[qb] = r_; return qt
    return target, seg, float(T0[-1]), max(errs)


def activate_grasp(m, d):
    s = m.site("right_pinch").id; p1 = d.site_xpos[s]; b1 = m.body("right_link6").id
    R1 = d.xmat[b1].reshape(3, 3); x1 = d.xpos[b1]
    v = slice_verts(m, d, N - 1); idx = np.argsort(np.linalg.norm(v - p1, axis=1))[:3]; a = m.flex_vertadr[N - 1]
    for k, vi in enumerate(idx):
        e = m.equality(f"grasp{k}").id
        m.eq_obj2id[e] = m.flex_vertbodyid[a + vi]; m.eq_data[e, 0:3] = R1.T @ (v[vi] - x1); m.eq_data[e, 3:6] = 0
        d.eq_active[e] = 1
    return float(np.linalg.norm(v[idx[0]] - p1))


def simulate(callback=None, dt_cb=1 / 30):
    m = build(); d = mujoco.MjData(m)
    target, seg, T_TOTAL, ik_err = plan(m)
    nrob = m.nq - sum(3 * m.flex_vertnum[i] for i in range(N))
    d.qpos[:nrob] = target(0)[:nrob]; mujoco.mj_forward(m, d)
    res = dict(ik_max_err=ik_err, finite=True, grasp_dist=None, sag_correction_mm=1000 * SAG[-1])
    t = 0.0; nxt = 0.0; grasped = False; wall = time.time(); ref = None; res["phase_shift_mm"] = {}; prev = None
    while t < T_TOTAL - 1e-9:
        name, u = seg(t)
        if name != prev and ref is not None:  # shift of slices 8/9 (below the top) at each phase start
            res["phase_shift_mm"][name] = [round(1000 * float(np.linalg.norm(slice_verts(m, d, i)[:, :2].mean(0)
                                                                              - ref[i][:, :2].mean(0))), 1) for i in (N - 3, N - 2)]
        prev = name
        if name == "knife_down" and ref is None: ref = [slice_verts(m, d, i) for i in range(N)]
        if name == "grasp" and u > 0.5 and not grasped: res["grasp_dist"] = activate_grasp(m, d); grasped = True
        jaw = JAW_CLOSED if (name in ("pick", "hold") or (name == "grasp" and u > 0.5)) else JAW_OPEN
        P.set_ctrl(m, d, target(t), {"left": K.GRIP_OPEN, "right": jaw}); mujoco.mj_step(m, d); t = d.time
        if not np.isfinite(d.qpos).all(): res["finite"] = False; break
        if callback and t >= nxt: callback(m, d, t, name); nxt += dt_cb
    res["wall_per_sim_s"] = (time.time() - wall) / max(t, 1e-9)
    vs = [slice_verts(m, d, i) for i in range(N)]
    rise = [float(v[:, 2].mean() - r[:, 2].mean()) for v, r in zip(vs, ref)]
    shift = [float(np.linalg.norm(v[:, :2].mean(0) - r[:, :2].mean(0))) for v, r in zip(vs, ref)]
    res.update(rise=rise, shift=shift, n_lifted=int(sum(r > 0.02 for r in rise)), top_lifted=bool(rise[-1] > 0.02),
               max_shift_rest_mm=1000 * max(shift[:-1]), deflection_mm=1000 * float(np.ptp(vs[-1][:, 2])))
    return res
