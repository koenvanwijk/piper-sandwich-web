import json
from pathlib import Path

import numpy as np
import pytest

from lerobot_robot_piper_sim.assembler import Tick
from lerobot_robot_piper_sim.config import PiperSimRobotConfig
from lerobot_robot_piper_sim.protocol import state_names
from piper_sim_recorder import hub
from piper_sim_recorder.recorder import EpisodeRecorder

CFG = dict(camera_sizes={"front": (64, 48)}, allowed_origins=[])


def tick(seq, jpeg):
    s = [0.01 * seq] * 14
    return Tick(seq=seq, t_sim=seq / 30, state=s, action=[x + 0.001 for x in s], msg={}, frames={0: (seq, jpeg)})


@pytest.fixture
def ds_root(tmp_path, jpg):
    events = []
    rec = EpisodeRecorder(PiperSimRobotConfig(port=0, **CFG), tmp_path / "ds", "t/ds", "testtaak", on_event=lambda **e: events.append(e))
    # tick vóór start wordt genegeerd
    rec.on_tick(tick(0, jpg(64, 48)))
    rec.on_cmd({"cmd": "start", "task": "taak A"})
    for s in range(1, 13):
        rec.on_tick(tick(s, jpg(64, 48)))
    rec.on_cmd({"cmd": "stop", "success": True})
    rec.on_cmd({"cmd": "stop"})                                      # stop zonder episode: fout-event, geen crash
    rec.on_cmd({"cmd": "start"})
    for s in range(20, 25):
        rec.on_tick(tick(s, jpg(64, 48)))
    rec.on_cmd({"cmd": "discard"})
    rec.on_cmd({"cmd": "start", "task": "taak B"})
    for s in range(30, 40):
        rec.on_tick(tick(s, jpg(64, 48)))
    rec.on_sim_disconnect()                                          # verbinding weg -> onderbroken, weggegooid
    rec.close()
    return tmp_path / "ds", rec, events


def test_recorder_episodes(ds_root):
    root, rec, events = ds_root
    names = [e["event"] for e in events]
    assert names.count("episode_saved") == 1 and names.count("episode_discarded") == 2 and "error" in names
    assert rec.saved[0]["success"] is True and rec.saved[0]["task"] == "taak A" and rec.saved[0]["seq_gaps"] == 0
    assert len(rec.saved) == 1
    from lerobot.datasets import LeRobotDataset
    ds = LeRobotDataset("t/ds", root=root)
    assert ds.meta.total_episodes == 1 and len(ds) == rec.saved[0]["frames"] and len(ds) >= 10
    f = ds[0]
    assert tuple(f["observation.state"].shape) == (14,) and tuple(f["action"].shape) == (14,)
    assert tuple(f["observation.images.front"].shape) == (3, 48, 64)
    assert ds.meta.features["observation.state"]["names"] == state_names()


def test_dry_run_and_mock_push(ds_root, monkeypatch, tmp_path):
    root, _rec, _ = ds_root
    (root / "meta" / "piper_sim_info.json").write_text(json.dumps({"fps": 30, "proto": 1, "browser_scene_hash": "abc",
        "server_scene_hash": "abc", "scene_hash_matches": True, "server_checkout_git_sha": "deadbeef", "physics_timestep_s": 0.002,
        "substeps": 16, "mode": "teleop", "cameras": [{"name": "front", "w": 64, "h": 48}]}))
    # 1. dry-run: geen netwerk, geen token nodig
    monkeypatch.delenv("HF_TOKEN", raising=False)
    import huggingface_hub
    calls = []
    monkeypatch.setattr(huggingface_hub.HfApi, "whoami", lambda *a, **k: calls.append("whoami") or {"name": "x"})
    monkeypatch.setattr("lerobot.datasets.LeRobotDataset.push_to_hub", lambda *a, **k: calls.append(("push", k)))
    rep = hub.push(root, "koen/piper-test", dry_run=True)
    assert rep.dry_run and not rep.uploaded and rep.private and rep.episodes == 1 and not calls
    card = Path(rep.card_preview).read_text()
    assert "Gesimuleerde" in card and "deadbeef" in card and "abc" in card and "radialen" in card and "Beperkingen" in card
    # 2. ongeldige repo_id
    with pytest.raises(hub.PushError, match="gebruiker/naam"):
        hub.push(root, "geen-slash", dry_run=True)
    # 3. echte push zonder token faalt vóór elke netwerkaanroep
    with pytest.raises(hub.PushError, match="HF_TOKEN"):
        hub.push(root, "koen/piper-test")
    assert not calls
    # 4. mock-push: privé is default; token wordt niet in logs/rapport gezet
    monkeypatch.setenv("HF_TOKEN", "hf_SECRET_FOR_TEST")
    rep = hub.push(root, "koen/piper-test")
    assert rep.uploaded and rep.hf_user == "x" and rep.private
    kind, kw = calls[-1]
    assert kind == "push" and kw["private"] is True and "hf_SECRET" not in json.dumps(rep.__dict__) and "hf_SECRET" not in str(kw)
    rep = hub.push(root, "koen/piper-test", public=True)
    assert calls[-1][1]["private"] is False
    # 5. lege/ongeldige map
    with pytest.raises(hub.PushError):
        hub.push(tmp_path / "leeg", "koen/piper-test", dry_run=True)


def test_reset_cmd_discards_running_episode(tmp_path, jpg):
    """Fase 3: `reset` (Y-knop / scene-reset) bewaart een lopende episode nooit; zonder episode is het een no-op + event."""
    events = []
    rec = EpisodeRecorder(PiperSimRobotConfig(port=0, **CFG), tmp_path / "ds", "t/ds", "t", on_event=lambda **e: events.append(e))
    rec.on_cmd({"cmd": "reset"})                                     # IDLE: alleen scene_reset-event
    rec.on_cmd({"cmd": "start"})
    for s in range(0, 6):
        rec.on_tick(tick(s, jpg(64, 48)))
    rec.on_cmd({"cmd": "reset"})                                     # RECORDING: weggooien
    rec.on_cmd({"cmd": "start"})
    for s in range(10, 16):
        rec.on_tick(tick(s, jpg(64, 48)))
    rec.on_cmd({"cmd": "success", "value": True})
    rec.on_cmd({"cmd": "stop", "success": True})
    rec.on_cmd({"cmd": "status"})
    rec.close()
    names = [e["event"] for e in events]
    assert names.count("scene_reset") == 2 and names.count("episode_discarded") == 1 and names.count("episode_saved") == 1
    assert len(rec.saved) == 1 and rec.saved[0]["success"] is True and rec.saved[0]["episode_index"] == 0
    st = [e for e in events if e["event"] == "status"][-1]
    assert st["next_episode"] == 1 and st["episodes_saved"] == 1
