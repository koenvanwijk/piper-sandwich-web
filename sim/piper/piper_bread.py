"""Dual Piper picks up a slice of bread (9 x 9 x 2.6 cm, 30 g, rigid, as the web demo's bread) in 3 ways.
MuJoCo only, NOT Isaac Sim. Bread lies centred on a 6 cm wide cutting board, so it overhangs 1.5 cm on both y
sides: the Piper jaw opens only ~70 mm, so a top-down grip across the 9 cm slice is impossible.

  rand     : right gripper, pitched 30 deg down, pinches the -y overhang (finger above + finger under)
  bimanual : both grippers pinch opposite overhangs (+y left, -y right) and lift together
  spatel   : the left arm slides the web-demo knife (flat) under the bread from +y and lifts it like a spatula;
             the right gripper's closed fingertips rest on top of the far edge (support/stop)
The bread underside is modelled with a 5 mm crust step (bottom box as wide as the board top minus 5 mm): a rigid box lying flat on a rigid
board leaves no gap for a 4 mm blade. Grips are real finger/blade contacts (friction), no constraints.
"""
import numpy as np, mujoco
import xml.etree.ElementTree as ET
import piper_scene as P
import piper_knife as K

BREAD_XY = (-0.02, 0.0)   # closer to the bases: pitched pinch poses stay inside the workspace
PITCH_DEG = 30.0   # right arm cannot reach flatter approaches at bread height
GRIP_IN = 0.01
BREAD = dict(size=(0.045, 0.045, 0.013), mass=0.03, friction="1.2 0.02 0.001")
BOARD = dict(size=(0.06, 0.025, 0.02))   # 5 cm wide, 4 cm high bread stand: 2 cm overhang + finger clearance
STEP = dict(h=0.005, inset=0.01)
BOARD_TOP = 2 * BOARD["size"][2]
BREAD_Z = BOARD_TOP + STEP["h"] + BREAD["size"][2]          # bread (main box) centre
PITCH = np.radians(PITCH_DEG)
OPEN, CLOSED = 0.022, 0.0
LIFT = 0.05
VARIANTS = ("rand", "bimanual", "spatel")
BEST = "bimanual"
SCHEDULE = (("settle", 0.5), ("reach", 1.2), ("place", 0.8), ("close", 0.6), ("lift", 1.5), ("hold", 1.0))
SPATEL_SCHEDULE = (("settle", 0.5), ("reach", 1.2), ("down", 0.6), ("insert", 2.0), ("lift", 1.5), ("hold", 1.0))


def build(variant, offscreen=(1280, 720)):
    r, wb = P.base_root(offscreen, "1 0.005 0.0001", tiles_c=BREAD_XY)
    for s in ("left", "right"): P.fingers_collide(r, s, variant != "spatel" or s == "right")
    if variant == "spatel":
        K.attach_knife(r, tilt_deg=0.0, segments=False, kind="web")
        for g in r.iter("geom"):
            if g.get("name") == "knife_blade": g.attrib.update(contype="1", conaffinity="1", friction="0.6 0.005 0.0001")
    ET.SubElement(wb, "geom", name="board", type="box", size=" ".join(map(str, BOARD["size"])),
                  pos=f"{BREAD_XY[0]} {BREAD_XY[1]} {BOARD['size'][2]}", rgba="0.55 0.36 0.2 1", friction="1.0 0.005 0.0001")
    b = ET.SubElement(wb, "body", name="bread", pos=f"{BREAD_XY[0]} {BREAD_XY[1]} {BREAD_Z}")
    ET.SubElement(b, "freejoint")
    sx, sy, sz = BREAD["size"]
    ET.SubElement(b, "geom", name="bread", type="box", size=f"{sx} {sy} {sz}", mass=str(BREAD["mass"] * 0.9),
                  condim="4", friction=BREAD["friction"], rgba="0.9 0.78 0.52 1")
    ET.SubElement(b, "geom", name="bread_bottom", type="box", size=f"{sx - STEP['inset']} {BOARD['size'][1] - 0.005} {STEP['h'] / 2}",
                  pos=f"0 0 {-sz - STEP['h'] / 2}", mass=str(BREAD["mass"] * 0.1), condim="4", friction=BREAD["friction"],
                  rgba="0.8 0.62 0.35 1")
    for s in ("left", "right"):
        ET.SubElement(P.find_body(r, f"{s}_link6"), "site", name=f"{s}_pinch", pos=f"0 0 {P.TIP_Z - 0.006}", size="0.003", group="4")
    return P.compile_root(r)


def _pinch_pose(side):
    """Pitched approach towards the bread from its +y (left) or -y (right) side; jaw axis ~vertical."""
    sgn = -1.0 if side == "left" else 1.0                  # approach direction along y
    zax = np.array([0, sgn * np.cos(PITCH), -np.sin(PITCH)]); xax = np.cross([0, 0, 1.0], [0, sgn, 0])
    edge_y = BREAD_XY[1] - sgn * BREAD["size"][1]
    grasp = np.array([BREAD_XY[0], edge_y + sgn * GRIP_IN, BREAD_Z])
    return zax, xax, grasp, np.array([0, sgn, 0])


def plan(m, variant):
    arms = {"rand": ["right"], "bimanual": ["left", "right"], "spatel": ["left", "right"]}[variant]
    off = (0, 0, P.TIP_Z - 0.006); W = {}; errs = []
    for side in arms:
        qa = [m.joint(j).qposadr[0] for j in P.arm(side)]
        if variant == "spatel" and side == "left":
            o = K.BLADE["pos"]; xax = (0, -1, 0); bz = BOARD_TOP + K.BLADE["size"][2] + 0.0005
            ys = (BREAD_XY[1] + 0.16, BREAD_XY[1] + 0.13, BREAD_XY[1] + 0.092)  # blade centre: tip outside -> tip 2.5 cm under
            pts = dict(above=(BREAD_XY[0], ys[0], bz + 0.04), start=(BREAD_XY[0], ys[1], bz),
                       end=(BREAD_XY[0], ys[2], bz), up=(BREAD_XY[0], ys[2], bz + 0.04))
            q, e, _ = P.ik_restarts(m, side, pts["above"], offset=o, xax=xax); W[side] = {}
            for k in ("above", "start", "end", "up"):
                q, e, _ = P.ik(m, side, pts[k], q, offset=o, xax=xax); errs.append(e); W[side][k] = q[qa].copy()
            continue
        if variant == "spatel":   # right: closed fingertips resting on the far top edge
            top = BREAD_Z + BREAD["size"][2]; p = np.array([BREAD_XY[0], BREAD_XY[1] - 0.03, top - 0.001])
            pts = dict(above=p + [0, 0, 0.04], place=p, up=p + [0, 0, 0.04]); zax, xax = (0, 0, -1), (1, 0, 0)
            o = (0, 0, P.TIP_Z)
        else:
            zax, xax, g, ap = _pinch_pose(side)
            pts = dict(above=g - 0.04 * ap, place=g, up=g + [0, 0, LIFT]); o = off
        qp, e, _ = P.ik_restarts(m, side, pts["place"], offset=o, zax=zax, xax=xax, n=24); W[side] = {}
        for k in ("place", "above", "up"):
            q, e, _ = P.ik(m, side, pts[k], qp, offset=o, zax=zax, xax=xax); errs.append(e); W[side][k] = q[qa].copy()
    sched = SPATEL_SCHEDULE if variant == "spatel" else SCHEDULE
    T0 = np.cumsum([0] + [s[1] for s in sched])

    def seg(t):
        i = int(np.clip(np.searchsorted(T0, t, side="right") - 1, 0, len(sched) - 1))
        return sched[i][0], float(np.clip((t - T0[i]) / sched[i][1], 0, 1))

    def target(t):
        name, u = seg(t); s = P.smooth(u); qt = np.zeros(m.nq); grip = {}
        for side, w in W.items():
            qa = [m.joint(j).qposadr[0] for j in P.arm(side)]; mix = lambda a, b: (1 - s) * w[a] + s * w[b]
            if variant == "spatel" and side == "left":
                qt[qa] = {"settle": w["above"], "reach": w["above"], "down": mix("above", "start"),
                          "insert": mix("start", "end"), "lift": mix("end", "up"), "hold": w["up"]}[name]
                grip[side] = 0.01
            elif variant == "spatel":
                qt[qa] = {"settle": w["above"], "reach": mix("above", "place"), "lift": mix("place", "up"),
                          "hold": w["up"]}.get(name, w["place"])
                grip[side] = 0.0
            else:
                qt[qa] = {"settle": w["above"], "reach": mix("above", "place"), "lift": mix("place", "up"),
                          "hold": w["up"]}.get(name, w["place"])
                grip[side] = OPEN if name in ("settle", "reach", "place") else (
                    OPEN + (CLOSED - OPEN) * s if name == "close" else CLOSED)
        return qt, grip
    return target, seg, float(T0[-1]), max(errs), arms


def _tilt(d, bid):
    return float(np.degrees(np.arccos(np.clip(d.xmat[bid].reshape(3, 3)[2, 2], -1, 1))))


def info(m, d):
    b = m.body("bread").id
    return f"boterham: hoogte +{1000*(d.xpos[b][2]-BREAD_Z):5.1f} mm | kanteling {_tilt(d, b):4.1f} graden"


def simulate(variant, callback=None, dt_cb=1 / 30):
    m = build(variant); d = mujoco.MjData(m); target, seg, T, ik_err, arms = plan(m, variant)
    qt, g = target(0); d.qpos[:m.nq - 7] = qt[:m.nq - 7]; mujoco.mj_forward(m, d)
    b = m.body("bread").id; ref_site = "right_pinch" if variant != "spatel" or True else None
    rel0 = None; max_tilt = 0.0; t = 0.0; nxt = 0.0
    while t < T - 1e-9:
        name, u = seg(t); qt, g = target(t); P.set_ctrl(m, d, qt, g); mujoco.mj_step(m, d); t = d.time
        if not np.isfinite(d.qpos).all(): return dict(variant=variant, finite=False)
        if name == "lift" and rel0 is None:
            ref = d.site_xpos[m.site("right_pinch").id] if variant != "spatel" else d.geom_xpos[m.geom("knife_blade").id]
            rel0 = d.xpos[b] - ref
        if name in ("lift", "hold"): max_tilt = max(max_tilt, _tilt(d, b))
        if callback and t >= nxt: callback(m, d, t, name); nxt += dt_cb
    ref = d.site_xpos[m.site("right_pinch").id] if variant != "spatel" else d.geom_xpos[m.geom("knife_blade").id]
    h = float(d.xpos[b][2] - BREAD_Z)
    return dict(variant=variant, finite=True, ik_max_err=ik_err, height_mm=1000 * h, lifted=bool(h > 0.02),
                slip_mm=1000 * float(np.linalg.norm(d.xpos[b] - ref - rel0)), tilt_deg=_tilt(d, b), max_tilt_deg=max_tilt)
