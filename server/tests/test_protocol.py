import pytest
from lerobot_robot_piper_sim.protocol import encode_frame, fnv1a, limits_by_name, parse_frame, state_names


def test_names_14():
    n = state_names()
    assert len(n) == 14 and n[0] == "left_joint1.pos" and n[6] == "left_gripper.pos" and n[7] == "right_joint1.pos" and n[-1] == "right_gripper.pos"
    assert set(limits_by_name()) == set(n)


def test_frame_roundtrip(jpg):
    j = jpg()
    assert parse_frame(encode_frame(7, 1, j)) == (7, 1, 0, j)
    with pytest.raises(ValueError):
        parse_frame(b"\x00" * 6)
    with pytest.raises(ValueError):
        parse_frame(encode_frame(1, 0, j)[:4] + b"\x00\x09\x00\x00" + j)   # cam 0, onbekend formaat 9


def test_fnv1a_matches_browser():
    # waarde uit de e2e-run: fnv1a(scene.xml) in src/rec-state.js == Python
    import pathlib
    xml = (pathlib.Path(__file__).resolve().parents[2] / "assets" / "scene.xml").read_text(encoding="utf-8")
    assert fnv1a(xml) == "e47a6641"
