#!/usr/bin/env python3
"""Testscript: stuurt cmd-berichten naar de server (stand-in voor de knoppen in de browser, fase 3).

  PIPER_WS_TOKEN=... python scripts/send_cmds.py --url ws://127.0.0.1:8765/ws \
      --script "start;wait=8;stop:success;start;wait=3;discard;status"

Stappen (gescheiden door ';'): start[:taak] | stop[:success|:fail] | discard | success | wait=SEC | status
Luistert naar `event`-berichten en print ze. Het token komt uit de omgeving (nooit een argument, nooit geprint).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys

from websockets.asyncio.client import connect


async def run(url: str, script: str, origin: str | None) -> int:
    token = os.environ.get("PIPER_WS_TOKEN", "")
    events: list[dict] = []
    async with connect(url, origin=origin, max_size=None) as ws:
        await ws.send(json.dumps({"type": "hello", "proto": 1, "role": "control", "token": token, "app": "send_cmds"}))
        welcome = json.loads(await asyncio.wait_for(ws.recv(), 10))
        assert welcome["type"] == "welcome", welcome

        async def reader():
            async for raw in ws:
                if isinstance(raw, str):
                    m = json.loads(raw)
                    if m.get("type") == "event":
                        events.append(m)
                        print("event:", json.dumps({k: v for k, v in m.items() if k != "type"}), flush=True)
        task = asyncio.create_task(reader())
        for step in filter(None, (s.strip() for s in script.split(";"))):
            name, _, arg = step.partition(":")
            if name.startswith("wait="):
                await asyncio.sleep(float(name[5:])); continue
            if name == "start":
                msg = {"cmd": "start", **({"task": arg} if arg else {})}
            elif name == "stop":
                msg = {"cmd": "stop", **({"success": arg == "success"} if arg else {})}
            elif name in ("discard", "success", "status"):
                msg = {"cmd": name}
            else:
                print("onbekende stap", step, file=sys.stderr); return 2
            await ws.send(json.dumps({"type": "cmd", **msg}))
            print("cmd:", msg, flush=True)
            await asyncio.sleep(0.3)
        # wacht tot opslaan klaar is (saving -> episode_saved)
        for _ in range(600):
            saving = sum(e["event"] == "saving" for e in events) > sum(e["event"] == "episode_saved" for e in events)
            if not saving:
                break
            await asyncio.sleep(0.5)
        await asyncio.sleep(0.5)
        task.cancel()
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="ws://127.0.0.1:8765/ws")
    ap.add_argument("--script", required=True)
    ap.add_argument("--origin", default=None)
    a = ap.parse_args()
    sys.exit(asyncio.run(run(a.url, a.script, a.origin)))
