"""Piper cheese stack (10 x 2D flex). Full run with PIPER_SLOW=1 (~2 min)."""
import os
import pytest
mujoco = pytest.importorskip("mujoco")
import piper_cheese as C

slow = pytest.mark.skipif(not os.environ.get("PIPER_SLOW"), reason="set PIPER_SLOW=1")


def test_build():
    m = C.build(offscreen=(64, 64))
    assert m.nflex == C.N and m.neq == 3


def test_holder_rims_below_blade():
    m = C.build(offscreen=(64, 64)); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    blade_bottom = C.BLADE_Z - C.BLADE["size"][2]
    for g in ("rim_px", "rim_mx"):
        top = d.geom_xpos[m.geom(g).id][2] + m.geom_size[m.geom(g).id][2]
        assert top < blade_bottom - 0.0005  # mes en bovenste plak gaan er vrij overheen


def test_finger_proxy():
    m = C.build(offscreen=(64, 64))
    assert C.FINGER and m.geom("right_tip_proxy").contype == 1


def test_jaw_fits():  # Piper: 2 x 35 mm; the jaw opens 2 x 30 mm around a 2 mm slice
    assert 2 * C.JAW_OPEN <= 0.070 and 2 * C.JAW_OPEN > C.T + 0.02


@slow
def test_pick_top_slice():
    r = C.simulate()
    assert r["finite"] and r["top_lifted"] and r["n_lifted"] == 1
    assert r["ik_max_err"] < 5e-3
    assert max(r["shift"][:-2]) < 0.015  # plakken 1-8 met rand + vinger: <= ~11 mm
    assert r["shift"][-2] < 0.02         # plak 9 met drukvinger: ~13 mm (zonder 31 mm)
