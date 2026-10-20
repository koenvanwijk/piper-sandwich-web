"""Offline (no GPU / no Isaac Sim) checks for the AgiBot X2 MuJoCo model.
Run: sim/x2/fetch_x2_assets.sh && python -m pytest sim/x2 -q
"""
import json, os, itertools
import numpy as np, pytest, yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))  # sim/x2
X2 = os.path.join(ROOT, "assets", "X2")
URDF = os.path.join(X2, "X2_omnipicker_fixed_dual.urdf")
RC = json.load(open(os.path.join(X2, "X2_omnipicker_fixed_dual.json")))
ARMS = RC["robot"]["active_arm_joints"]
needs_assets = pytest.mark.skipif(not os.path.isdir(os.path.join(X2, "meshes")), reason="run fetch_x2_assets.sh")


@pytest.fixture(scope="module")
def robot():
    import yourdfpy
    return yourdfpy.URDF.load(URDF, mesh_dir=X2, build_scene_graph=True, load_meshes=False)


def test_urdf_loads(robot):
    assert robot.base_link == "pelvis"
    for s in ARMS:
        assert len(ARMS[s]) == 7
        for j in ARMS[s]:
            assert j in robot.joint_map
    for l in ("gripper_l_center_link", "gripper_r_center_link"):
        assert l in robot.link_map


@needs_assets
def test_mesh_paths_exist(robot):
    missing = [g.geometry.mesh.filename for l in robot.robot.links for g in l.visuals + l.collisions
               if g.geometry.mesh is not None and not os.path.isfile(os.path.join(X2, g.geometry.mesh.filename))]
    assert not missing


def test_config_joints_and_limits(robot):
    r = RC["robot"]
    names = list(json.load(open(os.path.join(X2, "joint_index.json"))))
    for n, v in zip(names, r["init_joint_position"]):
        j = robot.joint_map[n]
        if v is not None and j.limit is not None:
            assert j.limit.lower - 1e-6 <= v <= j.limit.upper + 1e-6, n
    for n in r["lock_joints"] + RC["gripper"]["finger_names"]["left"] + RC["gripper"]["finger_names"]["right"]:
        assert n in robot.joint_map, n


def _cfg(robot, **over):
    q = {n: 0.0 for n in robot.actuated_joint_names}
    for s in ARMS:
        q.update(dict(zip(ARMS[s], yaml.safe_load(open(os.path.join(X2, f"X2_omnipicker_fixed_{s}.yaml")))["default_q"])))
    q.update(over); return q


def test_arm_sweep_within_limits_changes_ee(robot):
    """Each arm joint swept lower->upper moves the gripper centre and stays finite."""
    for s, ee in (("left", "gripper_l_center_link"), ("right", "gripper_r_center_link")):
        for j in ARMS[s]:
            lim = robot.joint_map[j].limit
            ps = []
            for v in np.linspace(lim.lower, lim.upper, 5):
                robot.update_cfg(_cfg(robot, **{j: v}))
                ps.append(robot.get_transform(ee, "pelvis")[:3, 3])
            ps = np.array(ps); assert np.isfinite(ps).all()
            assert np.ptp(ps, axis=0).max() > 1e-3 or j.endswith("wrist_roll_joint"), j


def test_gripper_open_close(robot):
    """Claw joint moves fingers apart; mimic joint follows with multiplier -1."""
    for side, claw, mimic, a, b in (("L", "left_claw_joint", "L_hand_wide1_joint", "L_hand_narrow1_Link", "L_hand_wide1_Link"),
                                    ("R", "right_claw_joint", "R_hand_wide1_joint", "R_hand_narrow1_Link", "R_hand_wide1_Link")):
        m = robot.joint_map[mimic].mimic
        assert m is not None and m.joint == claw and m.multiplier == -1.0
        lim = robot.joint_map[claw].limit
        d = []
        for v in (lim.lower, lim.upper):
            robot.update_cfg(_cfg(robot, **{claw: v}))
            d.append((robot.get_transform(a, "pelvis")[:3, :3], robot.get_transform(b, "pelvis")[:3, :3]))
        ang = lambda R: np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
        # both fingers rotate ~57 deg (1 rad) between open and closed
        assert ang(d[0][0].T @ d[1][0]) > 45 and ang(d[0][1].T @ d[1][1]) > 45


@needs_assets
def test_mujoco_fixed_base_locked_joints_no_drift():
    """Load in MuJoCo with fixed pelvis, hold legs/waist/head with stiff PD for 2 s: no drift, no NaN."""
    mujoco = pytest.importorskip("mujoco")
    import xml.etree.ElementTree as ET
    t = ET.parse(URDF); r = t.getroot()
    mj = r.find("mujoco")
    mj = ET.SubElement(r, "mujoco") if mj is None else mj
    c = mj.find("compiler")
    c = ET.SubElement(mj, "compiler") if c is None else c
    c.attrib.update(meshdir=".", discardvisual="true", fusestatic="false")
    tmp = os.path.join(X2, "_mj_tmp.urdf"); t.write(tmp)
    try:
        m = mujoco.MjModel.from_xml_path(tmp)
    finally:
        os.remove(tmp)
    d = mujoco.MjData(m)
    names = [m.joint(i).name for i in range(m.njnt)]
    q0 = np.zeros(m.nq)
    for s in ARMS:
        home = yaml.safe_load(open(os.path.join(X2, f"X2_omnipicker_fixed_{s}.yaml")))["default_q"]
        for n, v in zip(ARMS[s], home):
            q0[m.joint(n).qposadr[0]] = v
    d.qpos[:] = q0; mujoco.mj_forward(m, d)
    pelvis = m.body("pelvis").id
    for _ in range(2000):
        d.qfrc_applied[:] = 400 * (q0 - d.qpos) - 20 * d.qvel
        mujoco.mj_step(m, d)
    assert np.isfinite(d.qpos).all()
    locked = [n for n in RC["robot"]["lock_joints"] if n in names and not n.startswith(("L_hand", "R_hand"))]
    drift = max(abs(d.qpos[m.joint(n).qposadr[0]] - q0[m.joint(n).qposadr[0]]) for n in locked)
    assert drift < 0.05, drift
    assert np.allclose(d.xpos[pelvis], 0, atol=1e-9)  # fixed base never moves
