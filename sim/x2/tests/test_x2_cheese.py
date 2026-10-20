"""Kaasstapel: 1 plak los, rest blijft liggen. Traag (~80 s); zet X2_SLOW=1."""
import os, sys
import pytest
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import cheese_scene as C


def test_slice_mass():
    assert abs(C.slice_mass() - 0.014) < 0.002


def test_build():
    m = C.build(offscreen=(64, 64))
    assert m.nflex == C.N


@pytest.mark.skipif(not os.environ.get("X2_SLOW"), reason="set X2_SLOW=1")
def test_pick_top_slice():
    r = C.simulate()
    assert r["finite"] and r["n_lifted"] == 1 and r["top_lifted"]
    assert r["max_shift_rest_mm"] < 5
    assert r["wall_per_sim_s"] > 0
