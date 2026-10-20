"""X2 presses a soft butter block (MuJoCo flexcomp, dim=3) -- offline, no GPU. MuJoCo only, NOT Isaac Sim.
Run: sim/x2/fetch_x2_assets.sh && python -m pytest sim/x2/tests/test_x2_butter.py -q   (~1 min)
"""
import os, sys
import numpy as np, pytest

mujoco = pytest.importorskip("mujoco")
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))  # sim/x2
sys.path.insert(0, ROOT)
import butter_scene as B  # noqa: E402

pytestmark = pytest.mark.skipif(not os.path.isdir(os.path.join(B.X2, "meshes")), reason="run fetch_x2_assets.sh")


def test_mujoco_supports_flex():
    assert tuple(int(x) for x in mujoco.__version__.split(".")[:2]) >= (3, 0)
    m = B.build()
    assert m.nflex == 1 and m.flex_dim[0] == 3
    mass = float(m.body_mass[m.flex_vertbodyid[m.flex_vertadr[0]:m.flex_vertadr[0] + m.flex_vertnum[0]]].sum())
    sx, sy, sz = B.BUTTER["size"]
    assert mass == pytest.approx(911 * sx * sy * sz, rel=0.02)


@pytest.fixture(scope="module")
def press():
    return B.simulate(plastic=True)


def test_press_is_stable(press):
    assert press["finite"]
    assert press["yields"] == len(B.PRESS_OFFSETS)


def test_butter_gets_flatter(press):
    h0 = press["initial"]["top_mean"]
    assert press["min_top"] <= 0.70 * h0, press     # >= 30 % thinner under load
    assert press["final"]["top_mean"] <= 0.75 * h0, press  # stays flattened after release (plastic approx.)


def test_footprint_grows(press):
    assert press["final"]["footprint"] >= 1.3 * press["initial"]["footprint"], press


def test_legs_waist_head_stay_still(press):
    assert press["max_locked_drift"] < 0.01  # rad
