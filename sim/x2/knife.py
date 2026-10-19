"""Table knife (rigid) held in the closed LEFT OmniPicker, and a butter smoothing routine. MuJoCo only, NOT Isaac Sim.

Grip: the knife is a rigid child body of L_omnipicker_base_link, i.e. equivalent to a weld that is active from
t=0 with the gripper already closed (no grasp/slip simulation). Hand/finger collisions are off in this scene;
only the knife touches butter and table.
"""
import numpy as np, mujoco
import butter_scene as B

# Knife geometry in the L_omnipicker_base_link frame (base z points down at the work pose; base x ~ world y,
# base y ~ world x). Handle 10 x 2 x 1.5 cm between the fingertips, blade 10 x 2 cm x 1.5 mm towards -x.
HANDLE = dict(pos=(0.0, 0.0, 0.150), size=(0.05, 0.0075, 0.01), mass=0.016)
TILT_DEG = 15.0                       # blade cross-section tilted, leading (+world x) edge up
BLADE = dict(pos=(-0.10, 0.0, 0.159), size=(0.05, 0.01, 0.00075), mass=0.024)
_a = np.radians(-TILT_DEG)
BLADE_QUAT = (np.cos(_a / 2), np.sin(_a / 2), 0.0, 0.0)
# Collision proxy for the 1.5 mm blade: 6 mm thick box sharing the blade's top face (butter vertices tunnel
# through a 1.5 mm box). The visible blade stays 1.5 mm.
PROXY_T = 0.003
_off = PROXY_T - BLADE["size"][2]  # proxy centre shifted down (blade-local -z = base +z after tilt)
PROXY_POS = tuple(np.array(BLADE["pos"]) + _off * np.array([0, -np.sin(_a), np.cos(_a)]))
# lowest proxy point (trailing edge) below the *blade centre*, along base z (= world -z)
BLADE_LOW = BLADE["size"][1] * np.sin(np.radians(TILT_DEG)) + (2 * PROXY_T - BLADE["size"][2]) * np.cos(np.radians(TILT_DEG))
KNIFE_FRICTION = [0.2, 0.005, 0.0001]  # greasy steel on butter


def attach_knife(hb, thin=False):
    """thin=True: blade collision = real 1.5 mm blade (cheese scene); else 6 mm proxy (butter scene)."""
    k = hb.add_body(name="knife", pos=[0, 0, 0])
    k.add_geom(name="knife_handle", type=mujoco.mjtGeom.mjGEOM_BOX, pos=HANDLE["pos"], size=HANDLE["size"],
               mass=HANDLE["mass"], rgba=[0.15, 0.1, 0.08, 1], contype=1, conaffinity=1, friction=KNIFE_FRICTION)
    k.add_geom(name="knife_blade", type=mujoco.mjtGeom.mjGEOM_BOX, pos=BLADE["pos"], quat=list(BLADE_QUAT),
               size=BLADE["size"], mass=BLADE["mass"], rgba=[0.8, 0.82, 0.85, 1], contype=1 if thin else 0,
               conaffinity=1 if thin else 0, friction=KNIFE_FRICTION, priority=1)
    if thin:
        return
    # split into 5 segments: MuJoCo keeps at most 50 contacts per geom-flex pair
    n = 5; L = BLADE["size"][0]
    for i in range(n):
        pos = np.array(PROXY_POS); pos[0] += -L + L / n * (2 * i + 1)
        k.add_geom(name=f"knife_blade_proxy{i}", type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(pos), quat=list(BLADE_QUAT),
                   size=[L / n, BLADE["size"][1], PROXY_T], mass=0.0, rgba=[0.8, 0.8, 0.8, 0], group=3,
                   contype=1, conaffinity=1, friction=KNIFE_FRICTION)


# smoothing passes: blade trailing-edge height above the table (m); stroke along world x over the butter
PASS_HEIGHTS = (0.009, 0.007, 0.0055, 0.0045)
STROKE = (-0.035, 0.02)


def _schedule():
    segs = [("approach", 0.6, 0)]
    for k in range(len(PASS_HEIGHTS)):
        segs += [("descend", 0.5, k), ("stroke", 1.5, k), ("lift", 0.4, k), ("return", 0.6, k)]
    segs.append(("rest", 0.8, len(PASS_HEIGHTS) - 1))
    return segs


SCHEDULE = _schedule()
T_TOTAL = sum(s[1] for s in SCHEDULE)


def plan(m, center=None):
    """center: butter (x, y) centre; the butter slides a bit during the presses, so pass the measured centre."""
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    cx, cy = center if center is not None else B.BUTTER_XY
    qa = [m.joint(j).qposadr[0] for j in B.ARM_L]
    q = d.qpos.copy(); q[qa] = [-0.5, 0.2, 0, -1.6, 0, 0, 0]
    dd = mujoco.MjData(m); y = B.BUTTER_XY[1]; cache = {}  # blade (10 cm) spans the butter in y; keep the hand where IK is reliable; dd = mujoco.MjData(m)

    def solve(x, z, q):
        tgt = np.array([cx + x, y, B.TABLE_TOP + B.RADIUS + z + BLADE_LOW])
        for _ in range(2):  # correct for the exact lowest proxy point (geometry + IK orientation residual)
            q, err = B.arm_ik(m, d, tgt, q, offset=BLADE["pos"], x_axis=(0, 1, 0), iters=400)
            dd.qpos[:] = q; mujoco.mj_kinematics(m, dd)
            tgt = tgt + np.array([0, 0, (B.TABLE_TOP + B.RADIUS + z) - _low(m, dd)])
        return q, err
    # pre-solve a grid of (x, z) waypoints with continuation
    xs = np.linspace(STROKE[0], STROKE[1], 9); zs_above = 0.03
    errs = []
    q0 = q.copy()
    for k, h in enumerate(PASS_HEIGHTS):
        q = q0.copy()
        for z in (zs_above, h):
            for x in (xs if z == h else xs[[0, -1]]):
                q, e = solve(x, z, q)
                if z == zs_above and x == xs[0]: q0 = q.copy()
                cache[(k, round(x, 4), z)] = q[qa].copy(); errs.append((e, k, x, z))
    max_err = max(errs)[0]
    print("ik worst", max(errs)) if max(errs)[0] > 2e-3 else None

    def arm(k, x, z):
        # interpolate in x between solved waypoints
        if z != zs_above:
            i = np.clip(np.searchsorted(xs, x) - 1, 0, len(xs) - 2); f = (x - xs[i]) / (xs[i + 1] - xs[i])
            return (1 - f) * cache[(k, round(xs[i], 4), z)] + f * cache[(k, round(xs[i + 1], 4), z)]
        f = (x - xs[0]) / (xs[-1] - xs[0])
        return (1 - f) * cache[(k, round(xs[0], 4), z)] + f * cache[(k, round(xs[-1], 4), z)]

    def segment(t):
        t0 = 0.0
        for i, sg in enumerate(SCHEDULE):
            if t < t0 + sg[1] or i == len(SCHEDULE) - 1:
                return i, sg, min(max((t - t0) / sg[1], 0), 1)
            t0 += sg[1]

    def target(t):
        i, (name, dur, k), u = segment(t); sm = 0.5 - 0.5 * np.cos(np.pi * u); h = PASS_HEIGHTS[k]
        x0, x1 = STROKE
        if name in ("approach", "rest"): pa = arm(k, x0 if name == "approach" else x0, zs_above)
        elif name == "descend": pa = (1 - sm) * arm(k, x0, zs_above) + sm * arm(k, x0, h)
        elif name == "stroke": pa = arm(k, x0 + (x1 - x0) * sm, h)
        elif name == "lift": pa = (1 - sm) * arm(k, x1, h) + sm * arm(k, x1, zs_above)
        else:
            kn = min(k + 1, len(PASS_HEIGHTS) - 1)
            pa = (1 - sm) * arm(k, x1, zs_above) + sm * arm(kn, x0, zs_above)
        qt = np.zeros(m.nq); qt[qa] = pa
        return qt
    return target, segment, max_err


def _low(m, d):
    return blade_lowest_z(m, d) - B.RADIUS * 0  # proxy lowest point (world z)


def blade_lowest_z(m, d):
    g = m.geom("knife_blade_proxy2").id; R = d.geom_xmat[g].reshape(3, 3); s = m.geom_size[g]
    return float(d.geom_xpos[g][2] - np.abs(R[2]) @ s)


def simulate(rest, plastic=True, callback=None, dt_cb=1 / 30):
    """Smooth butter that starts with rest shape `rest` (e.g. after the 9 presses)."""
    m = B.build(rest=rest, knife=True); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    c = rest[:, :2].min(0) / 2 + rest[:, :2].max(0) / 2
    target, segment, max_err = plan(m, center=c)
    nb = 3 * len(rest); nrob = m.nq - nb
    locked = [j for j in range(m.njnt) if m.joint(j).name.startswith(B.LOCKED_PREFIX)]
    d.qpos[:nrob] = target(0)[:nrob]; mujoco.mj_forward(m, d)
    cur_rest = B.butter_verts(m, d)
    res = dict(initial=B.butter_metrics(cur_rest), ik_max_err=max_err, center=c.tolist(), finite=True, yields=0, passes=[],
               max_locked_drift=0.0, min_blade_height=1.0)
    t = 0.0; nxt = 0.0; last = 0
    while t < T_TOTAL - 1e-9:
        i, sg, u = segment(t)
        if i != last:
            if SCHEDULE[last][0] == "stroke":
                res["passes"].append(B.butter_metrics(B.butter_verts(m, d)))
                if plastic:
                    m, d, cur_rest = B.rebuild_with_yield(m, d, cur_rest, knife=True); res["yields"] += 1
            last = i
        B.ctrl_from_q(m, d, target(t)); mujoco.mj_step(m, d); t = d.time
        if not np.isfinite(d.qpos).all(): res["finite"] = False; break
        if sg[0] == "stroke": res["min_blade_height"] = min(res["min_blade_height"], blade_lowest_z(m, d) - B.TABLE_TOP)
        res["max_locked_drift"] = max(res["max_locked_drift"], max(abs(d.qpos[m.jnt_qposadr[j]]) for j in locked))
        if callback and t >= nxt:
            callback(m, d, t, f"{sg[0]} {sg[2] + 1}/{len(PASS_HEIGHTS)}"); nxt += dt_cb
    res["final"] = B.butter_metrics(B.butter_verts(m, d))
    return res
