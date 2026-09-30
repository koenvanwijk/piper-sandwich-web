import asyncio
import json

import numpy as np
import pytest
from lerobot.utils.errors import DeviceNotConnectedError
from websockets.asyncio.client import connect

from lerobot_robot_piper_sim import PiperSimRobot, PiperSimRobotConfig
from lerobot_robot_piper_sim.protocol import encode_frame, state_names
from test_bridge import hello

TOKEN = "t-" + "x" * 20


def test_features_and_clamp(monkeypatch, jpg):
    monkeypatch.setenv("PIPER_WS_TOKEN", TOKEN)
    cfg = PiperSimRobotConfig(port=0, allowed_origins=["https://ok.example"], camera_sizes={"front": (64, 48)}, max_relative_target=0.1)
    r = PiperSimRobot(cfg)
    assert list(r.action_features) == state_names() == [k for k in r.observation_features if k != "front"]
    assert r.observation_features["front"] == (48, 64, 3)
    with pytest.raises(DeviceNotConnectedError):
        r.get_observation()
    r.connect()
    try:
        async def go():
            async with connect(f"ws://127.0.0.1:{r.bridge.port}/ws", origin="https://ok.example") as ws:
                await ws.send(json.dumps(hello()))
                await ws.recv()
                st = [0.0] * 14
                st[6] = 0.0
                await ws.send(encode_frame(0, 0, jpg(64, 48)))
                await ws.send(json.dumps({"type": "state", "seq": 0, "t_sim": 0, "state": st, "action": st}))
                await asyncio.sleep(0.4)
                obs = r.get_observation()
                assert obs["front"].shape == (48, 64, 3) and obs["front"].dtype == np.uint8
                assert obs["left_joint2.pos"] == 0.0
                a = {n: 0.0 for n in state_names()}
                a["left_joint2.pos"] = 5.0            # buiten limiet en te grote stap
                a["left_joint5.pos"] = 0.05           # binnen stap en limiet
                a["left_gripper.pos"] = 2.0
                out = r.send_action(a)
                assert out["left_joint2.pos"] == pytest.approx(0.1)            # max_relative_target
                assert out["left_joint5.pos"] == pytest.approx(0.05)
                assert out["left_gripper.pos"] == pytest.approx(0.1)
                assert r.clamped_actions == 1
                with pytest.raises(KeyError):
                    r.send_action({"bogus": 1.0})
        asyncio.run(go())
    finally:
        r.disconnect()
    assert not r.is_connected
