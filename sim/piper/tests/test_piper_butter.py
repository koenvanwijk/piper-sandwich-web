"""Dual Piper (assets/scene.xml) presses butter (flex). Fast checks always; full run with PIPER_SLOW=1 (~1 min)."""
import os
import numpy as np, pytest
mujoco = pytest.importorskip("mujoco")
import piper_scene as P

slow = pytest.mark.skipif(not os.environ.get("PIPER_SLOW"), reason="set PIPER_SLOW=1")


def test_web_scene_has_rubber_knife():  # PR #13 is in main: rubber handle
    xml = open(P.SCENE).read()
    assert 'name="knife_handle"' in xml and 'material="rubber"' in xml


def test_build_press():
    m = P.build_press()
    assert m.nflex == 1 and m.nu == 16
    assert m.jnt_range[m.joint("left_joint7").id][1] == pytest.approx(0.035)  # 2 x 35 mm = 70 mm opening


def test_press_ik():
    m = P.build_press(); target, seg, T, _ = P.plan_press(m)
    assert max(target.errs) < 2e-3 and T == pytest.approx(16.0)


@slow
def test_press_run():
    r = P.simulate_press()
    assert r["finite"] and r["yields"] == 9
    assert r["final"]["max_height"] < 0.8 * r["initial"]["max_height"]
    assert r["final"]["footprint"] > 1.3 * r["initial"]["footprint"]
