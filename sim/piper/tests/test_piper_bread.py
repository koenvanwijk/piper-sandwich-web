"""Piper picks up bread (rigid, ~3 s per variant)."""
import pytest
mujoco = pytest.importorskip("mujoco")
import piper_bread as B


@pytest.fixture(scope="module")
def runs():
    return {v: B.simulate(v) for v in B.VARIANTS}


def test_bread_wider_than_jaw():
    assert 2 * B.BREAD["size"][0] > 0.070  # 9 cm slice, 7 cm max opening: no top-down grip across


def test_all_finite(runs):
    assert all(r["finite"] for r in runs.values())


def test_best_variant_lifts(runs):
    r = runs[B.BEST]
    assert r["lifted"] and r["height_mm"] > 25 and r["slip_mm"] < 30
