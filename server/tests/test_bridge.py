import asyncio
import json

import pytest
from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed, InvalidStatus

from lerobot_robot_piper_sim.bridge import SimBridge
from lerobot_robot_piper_sim.config import PiperSimRobotConfig
from lerobot_robot_piper_sim.protocol import encode_frame, state_names

TOKEN = "t-" + "x" * 20


def hello(**kw):
    h = {"type": "hello", "proto": 1, "fps": 30, "token": TOKEN, "state_names": state_names(), "action_names": state_names(),
         "cameras": [{"id": 0, "name": "front", "w": 64, "h": 48, "format": "jpeg"}], "app": "t", "scene_hash": "abc"}
    h.update(kw)
    return h


def state(seq, n=14):
    return {"type": "state", "seq": seq, "t_sim": seq / 30, "state": [0.1] * n, "action": [0.2] * n}


@pytest.fixture
def bridge(monkeypatch):
    monkeypatch.setenv("PIPER_WS_TOKEN", TOKEN)
    ticks, cmds = [], []
    cfg = PiperSimRobotConfig(port=0, allowed_origins=["https://ok.example"], camera_sizes={"front": (64, 48)})
    b = SimBridge(cfg, on_tick=ticks.append, on_cmd=cmds.append)
    b.start()
    b.ticks, b.cmds = ticks, cmds
    yield b
    b.stop()


def url(b):
    return f"ws://127.0.0.1:{b.port}/ws"


async def closed_code(u, origin, h):
    async with connect(u, origin=origin) as ws:
        await ws.send(json.dumps(h))
        with pytest.raises(ConnectionClosed):
            await asyncio.wait_for(ws.recv(), 5)
        return ws.close_code


def test_bad_token_4401(bridge):
    assert asyncio.run(closed_code(url(bridge), "https://ok.example", hello(token="fout"))) == 4401
    assert bridge.stats["rejected_auth"] == 1


def test_origin_rejected(bridge):
    async def go():
        with pytest.raises(InvalidStatus) as e:
            async with connect(url(bridge), origin="https://evil.example"):
                pass
        return e.value.response.status_code
    assert asyncio.run(go()) == 403
    async def go2():                                                          # geen Origin-header en niet toegestaan
        with pytest.raises(InvalidStatus) as e:
            async with connect(url(bridge)):
                pass
        return e.value.response.status_code
    assert asyncio.run(go2()) == 403


def test_wrong_path_404(bridge):
    async def go():
        with pytest.raises(InvalidStatus) as e:
            async with connect(f"ws://127.0.0.1:{bridge.port}/", origin="https://ok.example"):
                pass
        return e.value.response.status_code
    assert asyncio.run(go()) == 404


def test_config_mismatch_4400(bridge):
    bad = hello(cameras=[{"id": 0, "name": "front", "w": 99, "h": 48}])
    assert asyncio.run(closed_code(url(bridge), "https://ok.example", bad)) == 4400
    assert asyncio.run(closed_code(url(bridge), "https://ok.example", hello(state_names=["a"]))) == 4400


def test_flow_ticks_frames_cmd_ping(bridge, jpg):
    async def go():
        async with connect(url(bridge), origin="https://ok.example") as ws:
            await ws.send(json.dumps(hello()))
            w = json.loads(await ws.recv())
            assert w["type"] == "welcome" and w["fps"] == 30
            await ws.send(json.dumps({"type": "ping", "id": 5, "t_client_ms": 1.5}))
            assert json.loads(await ws.recv()) == {"type": "pong", "id": 5, "t_client_ms": 1.5}
            for s in range(20):
                await ws.send(encode_frame(s, 0, jpg(64, 48)))
                await ws.send(json.dumps(state(s)))
            await ws.send(json.dumps(state(30)))                              # gat (20..29)
            await ws.send(json.dumps(state(31, n=13)))                        # ongeldig: genegeerd
            await ws.send(json.dumps({"type": "cmd", "cmd": "start", "task": "x"}))
            await ws.send(b"\x00\x01")                                        # rommel-frame: genegeerd
            await asyncio.sleep(0.5)
    asyncio.run(go())
    assert bridge.stats["state"] == 21 and bridge.stats["seq_gaps"] == 1 and bridge.stats["bad_messages"] >= 2
    assert bridge.cmds == [{"cmd": "start", "task": "x"}]
    assert [t.seq for t in bridge.ticks][:5] == [0, 1, 2, 3, 4] and all(0 in t.frames for t in bridge.ticks)


def test_token_required(monkeypatch):
    monkeypatch.delenv("PIPER_WS_TOKEN", raising=False)
    with pytest.raises(RuntimeError, match="PIPER_WS_TOKEN"):
        SimBridge(PiperSimRobotConfig(port=0)).start()
    with pytest.raises(RuntimeError, match="loopback"):
        SimBridge(PiperSimRobotConfig(port=0, host="0.0.0.0", allow_no_auth=True)).start()
