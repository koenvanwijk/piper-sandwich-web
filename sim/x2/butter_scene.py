"""MuJoCo scene: AgiBot X2 (fixed base) presses a soft butter block (flexcomp, dim=3) on a table.

MuJoCo only, NOT Isaac Sim. Used by sim/x2/tests/test_x2_butter.py and sim/x2/render_butter_press.py.
Butter model (see docs/X2.md):
  - flexcomp grid 9x9x5 vertices (5 mm spacing, 2 mm collision radius), 4 x 4 x 2 cm, density 911 kg/m^3 -> mass 29 g
  - linear elasticity (StVK) young=2e4 Pa, poisson=0.45, damping=0.01 s (Rayleigh-style)
  - MuJoCo has no native plasticity: optional 'plastic yield' rebuilds the flex with a new rest shape
    rest_new = rest + YIELD * (current - rest) after the press (see apply_plastic_yield()).
"""
import os, tempfile
import numpy as np, mujoco, yaml, json
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.abspath(__file__))  # sim/x2
X2 = os.path.join(ROOT, "assets", "X2")
ARM_L = ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
         "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint"]
RADIUS = 0.002                    # flex vertex collision radius
BUTTER = dict(size=(0.03, 0.03, 0.015), count=(7, 7, 4), density=911.0, young=2e4, poisson=0.45, damping=0.01)
BUTTER_XY = (0.285, 0.15)          # in pelvis frame, under the closed left OmniPicker fingertips
PRESS_GAP = 0.003                 # fingertips stop 3 mm above the table (butter is 15 mm thick)
TABLE_TOP = -0.0733 - PRESS_GAP - RADIUS  # m, pelvis frame (pelvis at 0, feet ~ -0.67); -0.0733 = fingertip z at PRESS_Z
APPROACH_Z, PRESS_Z = 0.0, -0.05  # gripper-centre target heights (IK, gripper pointing down)
YIELD = 0.8                       # plastic fraction kept after release (approximation)
LOCKED_PREFIX = ("left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle", "waist_", "head_")
HAND_PREFIX = ("L_omnipicker", "L_hand", "gripper_l_center")


def _robot_xml(tmpdir, knife=False):
    s = mujoco.MjSpec.from_file(os.path.join(X2, "X2_omnipicker_fixed_dual.urdf"))
    s.compiler.meshdir = X2; s.compiler.fusestatic = False
    s.compiler.discardvisual = False
    for b in s.bodies:
        hand = b.name.startswith(HAND_PREFIX)
        for g in b.geoms:
            hand_col = hand and (g.contype or g.conaffinity) and not knife  # visual geoms never collide
            g.contype = 1 if hand_col else 0; g.conaffinity = 1 if hand_col else 0
            if hand: g.friction = [1.0, 0.005, 0.0001]
    for j in s.joints:
        j.armature = 0.01; j.damping = [2.0, 0, 0]; j.actfrclimited = mujoco.mjtLimited.mjLIMITED_FALSE
    hb = s.body("L_omnipicker_base_link")
    hb.add_geom(name="L_fingertip_proxy", type=mujoco.mjtGeom.mjGEOM_BOX, pos=TIP_PROXY["pos"], size=TIP_PROXY["size"],
                contype=0 if knife else 1, conaffinity=0 if knife else 1, friction=[1.0, 0.005, 0.0001],
                rgba=[0.2, 0.8, 1.0, 0.0], group=3)
    if knife:
        import knife as K
        K.attach_knife(hb, thin=(knife == "thin"))
    s.compile()
    p = os.path.join(tmpdir, f"x2_{knife}.xml"); open(p, "w").write(s.to_xml())
    return p


_ROBOT_CACHE = {}


def build(rest=None, offscreen=(1280, 720), knife=False):
    """Return MjModel. rest: optional (N,3) butter rest vertex positions (world). knife: table knife in left hand."""
    key = "xml_knife" if knife else "xml"
    if key not in _ROBOT_CACHE:
        _ROBOT_CACHE.setdefault("dir", tempfile.mkdtemp()); _ROBOT_CACHE[key] = _robot_xml(_ROBOT_CACHE["dir"], knife)
    t = ET.parse(_ROBOT_CACHE[key]); r = t.getroot()
    r.find("compiler").set("meshdir", X2)
    opt = r.find("option")
    if opt is None: opt = ET.SubElement(r, "option")
    opt.attrib.update(timestep="0.001", integrator="discrete", solver="CG", iterations="60", tolerance="1e-8")
    vis = ET.SubElement(r, "visual"); ET.SubElement(vis, "global", offwidth=str(offscreen[0]), offheight=str(offscreen[1]))
    wb = r.find("worldbody")
    ET.SubElement(wb, "light", pos="0 -1 3", dir="0 0.3 -1", diffuse="0.8 0.8 0.8", castshadow="false")
    ET.SubElement(wb, "light", pos="1.5 1.5 2", dir="-1 -1 -1", diffuse="0.4 0.4 0.4", castshadow="false")
    ET.SubElement(wb, "geom", name="floor", type="plane", size="3 3 0.1", pos="0 0 -0.68", rgba="0.35 0.4 0.45 1",
                  contype="0", conaffinity="0")
    ET.SubElement(wb, "geom", name="table", type="box", size="0.15 0.15 0.01",
                  pos=f"{BUTTER_XY[0]} {BUTTER_XY[1]} {TABLE_TOP - 0.01}", rgba="0.55 0.4 0.3 1",
                  contype="1", conaffinity="1", friction="2.0 0.005 0.0001" if knife else "0.8 0.005 0.0001",
                  solref="0.002 1", solimp="0.95 0.99 0.0005")
    sx, sy, sz = BUTTER["size"]; nx, ny, nz = BUTTER["count"]
    mass = BUTTER["density"] * sx * sy * sz
    common = dict(name="butter", dim="3", mass=f"{mass:.5f}", radius=str(RADIUS), rgba="1 0.88 0.35 1")
    if rest is None:
        fc = ET.SubElement(wb, "flexcomp", type="grid", count=f"{nx} {ny} {nz}",
                           spacing=f"{sx/(nx-1)} {sy/(ny-1)} {sz/(nz-1)}",
                           pos=f"{BUTTER_XY[0]} {BUTTER_XY[1]} {TABLE_TOP + sz/2 + RADIUS}", **common)
    else:
        fc = ET.SubElement(wb, "flexcomp", type="direct", point=" ".join(f"{v:.6f}" for v in np.asarray(rest).ravel()),
                           element=" ".join(map(str, _ELEMENTS)), **common)
    ET.SubElement(fc, "edge", equality="false")
    ET.SubElement(fc, "elasticity", young=str(BUTTER["young"]), poisson=str(BUTTER["poisson"]),
                  damping=str(BUTTER["damping"]))
    ET.SubElement(fc, "contact", selfcollide="none", condim="3", solref="0.002 1", solimp="0.95 0.99 0.0005",
                  friction="0.2 0.005 0.0001" if knife else "1.0 0.005 0.0001")
    # knife scene: MuJoCo takes the max friction of a pair, so butter=0.2 (greasy blade), table=2.0 (butter sticks
    # to the board; stands in for adhesion, which MuJoCo flex contacts do not model)
    act = r.find("actuator")
    if act is None: act = ET.SubElement(r, "actuator")
    for j in r.iter("joint"):
        n = j.get("name")
        if not n or j.get("type") not in (None, "hinge"): continue
        stiff = n.startswith(LOCKED_PREFIX)
        ET.SubElement(act, "position", name=f"a_{n}", joint=n, kp="5000" if stiff else ("2000" if n in ARM_L else "300"),
                      kv="50" if stiff else "20")
    m = mujoco.MjModel.from_xml_string(ET.tostring(r, encoding="unicode"))
    return m


def _grid_elements():
    nx, ny, nz = BUTTER["count"]; idx = lambda i, j, k: (i * ny + j) * nz + k
    el = []
    for i in range(nx - 1):
        for j in range(ny - 1):
            for k in range(nz - 1):
                c = [idx(i + a, j + b, k + cc) for a in (0, 1) for b in (0, 1) for cc in (0, 1)]
                # 6-tet (Kuhn) split of each hex: same topology family as flexcomp grid
                for t in ((0, 1, 3, 7), (0, 3, 2, 7), (0, 2, 6, 7), (0, 6, 4, 7), (0, 4, 5, 7), (0, 5, 1, 7)):
                    el += [c[q] for q in t]
    return el


_ELEMENTS = _grid_elements()


def butter_verts(m, d):
    f = 0  # single flex in this scene
    a = m.flex_vertadr[f]; n = m.flex_vertnum[f]
    return d.flexvert_xpos[a:a + n].copy()


def butter_metrics(v):
    """height = max-min vertex z; top_mean = mean height of the top vertex layer above the table (thickness);
    footprint = xy bounding-box area."""
    top = v.reshape(-1, BUTTER["count"][2], 3)[:, -1, 2] - TABLE_TOP - RADIUS
    return dict(height=float(v[:, 2].max() - v[:, 2].min()), max_height=float(v[:, 2].max() - TABLE_TOP - RADIUS),
                top_std=float(top.std()),
                top_mean=float(v.reshape(-1, BUTTER["count"][2], 3)[:, -1, 2].mean() - TABLE_TOP - RADIUS),
                footprint=float(np.ptp(v[:, 0]) * np.ptp(v[:, 1])), zmin=float(v[:, 2].min()))


def arm_ik(m, d, target, q_init, iters=600, down_weight=0.3, offset=(0, 0, 0.13), x_axis=None):
    """Damped least-squares IK: left gripper centre to target, gripper z-axis pointing down."""
    b = m.body("L_omnipicker_base_link").id
    dofs = [m.joint(j).dofadr[0] for j in ARM_L]; qa = [m.joint(j).qposadr[0] for j in ARM_L]
    rng = np.array([m.jnt_range[m.joint(j).id] for j in ARM_L])
    q = q_init.copy(); dd = mujoco.MjData(m)
    for _ in range(iters):
        dd.qpos[:] = q; mujoco.mj_kinematics(m, dd); mujoco.mj_comPos(m, dd)
        R = dd.xmat[b].reshape(3, 3); p = dd.xpos[b] + R @ np.asarray(offset)
        jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv)); mujoco.mj_jac(m, dd, jp, jr, p, b)
        J = np.vstack([jp[:, dofs], down_weight * jr[:, dofs]])
        er = np.cross(R[:, 2], [0, 0, -1])
        if x_axis is not None: er = er + np.cross(R[:, 0], np.asarray(x_axis, float))
        e = np.hstack([target - p, down_weight * er])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
        q[qa] = np.clip(q[qa] + 0.5 * dq, rng[:, 0], rng[:, 1])
    return q, float(np.linalg.norm(target - p))


def hand_lowest_z(m, d):
    """Lowest point of the left-hand collision geoms (via geom AABB)."""
    z = []
    for g in range(m.ngeom):
        if m.geom_contype[g] and m.geom_bodyid[g] and m.body(m.geom_bodyid[g]).name.startswith(HAND_PREFIX):
            R = d.geom_xmat[g].reshape(3, 3); mid = m.geom_dataid[g]
            if mid >= 0:
                a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
                z.append(float((m.mesh_vert[a:a + n] @ R.T + d.geom_xpos[g])[:, 2].min()))
            else:
                c = d.geom_xpos[g] + R @ m.geom_aabb[g, :3]; z.append(c[2] - np.abs(R[2]) @ m.geom_aabb[g, 3:])
    return min(z)


def ctrl_from_q(m, d, q):
    for i in range(m.nu): d.ctrl[i] = q[m.jnt_qposadr[m.actuator_trnid[i, 0]]]


# closed fingertips touch an area of only ~0.7 x 2.3 cm (x by y), so the butter is pressed in a 3 x 3 raster
PRESS_OFFSETS = tuple((dx, dy) for dy in (-0.008, 0.0, 0.008) for dx in (-0.009, 0.0, 0.009))
# Collision proxy: flat box that envelopes the two closed fingertips (narrow3/wide3 meshes are thin, and
# flex vertices slip between them). Frame: L_omnipicker_base_link; matches the measured fingertip envelope.
TIP_PROXY = dict(pos=(0.0, 0.0, 0.1493), size=(0.0115, 0.0035, 0.004))


def _schedule():
    segs = [("approach", 0.6, None, 0.0, 0.0)]
    for k in range(len(PRESS_OFFSETS)):
        segs += [("descend", 0.8, k, 0.0, 1.0), ("hold", 0.3, k, 1.0, 1.0), ("lift", 0.5, k, 1.0, 0.0)]
    segs.append(("rest", 1.0, len(PRESS_OFFSETS) - 1, 0.0, 0.0))
    return segs


SCHEDULE = _schedule()
T_TOTAL = sum(sg[1] for sg in SCHEDULE)


def plan(m):
    """IK waypoints (above -> press) per press offset; returns (target(t) -> qpos, segment(t))."""
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    qa = [m.joint(j).qposadr[0] for j in ARM_L]
    q = d.qpos.copy(); q[qa] = [-0.5, 0.2, 0, -1.6, 0, 0, 0]
    zs = np.linspace(APPROACH_Z, PRESS_Z, 11); paths = []
    for dx, dy in PRESS_OFFSETS:
        path = []
        for z in zs:
            q, err = arm_ik(m, d, np.array([BUTTER_XY[0] + dx, BUTTER_XY[1] + dy, z]), q)
            assert err < 4e-3, (dx, z, err)
            path.append(q[qa].copy())
        paths.append(np.array(path)); q = q.copy(); q[qa] = path[0]
    base = np.zeros(m.nq)

    def segment(t):
        t0 = 0.0
        for i, sg in enumerate(SCHEDULE):
            if t < t0 + sg[1] or i == len(SCHEDULE) - 1:
                return i, sg, min(max((t - t0) / sg[1], 0), 1)
            t0 += sg[1]

    def target(t):
        i, (name, dur, k, s0, s1), u = segment(t)
        k = 0 if k is None else k
        sm = s0 + (s1 - s0) * (0.5 - 0.5 * np.cos(np.pi * u))
        if name == "lift" and k + 1 < len(paths):  # move over to the next press line while lifting
            a = 0.5 - 0.5 * np.cos(np.pi * u)
            pa = (1 - a) * _interp(paths[k], sm) + a * _interp(paths[k + 1], sm)
        else:
            pa = _interp(paths[k], sm)
        qt = base.copy(); qt[qa] = pa
        return qt
    return target, segment


def _interp(path, s):
    x = s * (len(path) - 1); i = min(int(x), len(path) - 2); f = x - i
    return (1 - f) * path[i] + f * path[i + 1]


def rebuild_with_yield(m, d, rest0, knife=False):
    """Plasticity approximation: new rest shape = rest0 + YIELD*(current - rest0); state transferred."""
    v = butter_verts(m, d); rest_new = rest0 + YIELD * (v - rest0)
    m2 = build(rest=rest_new, knife=knife); d2 = mujoco.MjData(m2)
    nrob = m.nq - 3 * len(v)
    assert m2.nq == m.nq and m2.nv == m.nv
    d2.qpos[:nrob] = d.qpos[:nrob]; d2.qvel[:nrob] = d.qvel[:nrob]
    d2.qpos[nrob:] = 0; mujoco.mj_kinematics(m2, d2)
    base = butter_verts(m2, d2) if d2.flexvert_xpos.any() else rest_new
    mujoco.mj_forward(m2, d2); base = butter_verts(m2, d2)
    d2.qpos[nrob:] = (v - base).ravel(); d2.qvel[nrob:] = d.qvel[nrob:]
    d2.ctrl[:] = d.ctrl; d2.time = d.time; mujoco.mj_forward(m2, d2)
    return m2, d2, rest_new


def simulate(plastic=True, callback=None, dt_cb=1 / 30):
    """Run the press sequence. Returns metrics dict. callback(m, d, t, phase_label) every dt_cb s."""
    m = build(); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    rest = butter_verts(m, d); m0 = butter_metrics(rest)
    target, segment = plan(m)
    nrob = m.nq - 3 * len(rest)
    locked = [j for j in range(m.njnt) if m.joint(j).name.startswith(LOCKED_PREFIX)]
    d.qpos[:nrob] = target(0)[:nrob]; mujoco.mj_forward(m, d)
    res = dict(initial=m0, min_top=m0["top_mean"], max_footprint=m0["footprint"], max_locked_drift=0.0,
               finite=True, yields=0, pressed=[])
    nxt = 0.0; t = 0.0; last_seg = 0
    while t < T_TOTAL - 1e-9:
        i, sg, u = segment(t)
        if i != last_seg:
            prev = SCHEDULE[last_seg]
            if prev[0] == "hold":  # end of a press
                res["pressed"].append(butter_metrics(butter_verts(m, d)))
                if plastic:
                    m, d, rest = rebuild_with_yield(m, d, rest); res["yields"] += 1
            last_seg = i
        ctrl_from_q(m, d, target(t)); mujoco.mj_step(m, d); t = d.time
        if not np.isfinite(d.qpos).all(): res["finite"] = False; break
        mt = butter_metrics(butter_verts(m, d))
        res["min_top"] = min(res["min_top"], mt["top_mean"]); res["max_footprint"] = max(res["max_footprint"], mt["footprint"])
        res["max_locked_drift"] = max(res["max_locked_drift"], max(abs(d.qpos[m.jnt_qposadr[j]]) for j in locked))
        if callback and t >= nxt:
            k = sg[2]; label = sg[0] if k is None else f"{sg[0]} {k + 1}/{len(PRESS_OFFSETS)}"
            callback(m, d, t, label); nxt += dt_cb
    res["final"] = butter_metrics(butter_verts(m, d)); res["final_verts"] = butter_verts(m, d)
    return res
