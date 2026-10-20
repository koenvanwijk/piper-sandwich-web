"""Left Piper smooths butter with the web-demo knife (PR #13). Full run with PIPER_SLOW=1 (~1.5 min)."""
import os
import numpy as np, pytest
mujoco = pytest.importorskip("mujoco")
import piper_scene as P
import piper_knife as K

slow = pytest.mark.skipif(not os.environ.get("PIPER_SLOW"), reason="set PIPER_SLOW=1")


def _rest():
    m = P.build_press(); d = mujoco.MjData(m); mujoco.mj_forward(m, d); return P.butter_verts(m, d)


def test_knife_model():
    m = K.build(_rest())
    assert float(m.body_subtreemass[m.body("knife").id]) == pytest.approx(0.06, abs=1e-6)  # 40 g rubber handle + 20 g blade
    assert m.geom("knife_handle").matid >= 0


def test_knife_plan():
    m = K.build(_rest()); target, seg, T, err = K.plan(m, P.BUTTER_XY)
    assert err < 2e-3


@slow
def test_smoothing():
    p = P.simulate_press(); s = K.simulate(p["final_verts"])
    assert s["finite"] and s["yields"] == len(K.PASS_HEIGHTS)
    assert s["passes"][-1]["max_height"] < p["final"]["max_height"]
    assert s["final"]["footprint"] > p["final"]["footprint"]
