"""X2 smooths pressed butter with a table knife (MuJoCo flexcomp, offline, NOT Isaac Sim). ~1.5 min."""
import os, sys
import pytest

mujoco = pytest.importorskip("mujoco")
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))  # sim/x2
sys.path.insert(0, ROOT)
import butter_scene as B  # noqa: E402
import knife as K  # noqa: E402

pytestmark = pytest.mark.skipif(not os.path.isdir(os.path.join(B.X2, "meshes")), reason="run fetch_x2_assets.sh")


@pytest.fixture(scope="module")
def runs():
    press = B.simulate(plastic=True)
    smooth = K.simulate(press["final_verts"], plastic=True)
    return press, smooth


def test_knife_model():
    m = B.build(knife=True)
    mass = float(m.body_mass[m.body("knife").id])
    assert mass == pytest.approx(0.040, abs=1e-6)


def test_smoothing_stable(runs):
    _, s = runs
    assert s["finite"] and s["yields"] == len(K.PASS_HEIGHTS)
    assert s["ik_max_err"] < 0.01
    assert s["max_locked_drift"] < 0.01


def test_smoother_than_pressed(runs):
    p, s = runs
    a, b = p["final"], s["final"]
    assert b["max_height"] <= 0.9 * a["max_height"], (a, b)   # rim knocked down >= 10 %
    assert b["top_std"] <= 0.9 * a["top_std"], (a, b)         # top surface >= 10 % smoother
    assert b["top_mean"] <= a["top_mean"] + 1e-4, (a, b)
    assert b["footprint"] >= a["footprint"], (a, b)            # spread out, not scraped off
