"""WebSocket-server (websockets/asyncio) die het fase-1-protocol van piper-sandwich-web spreekt.

  browser -> server : hello | state | ping | pong | binaire JPEG-frames | cmd
  server  -> browser: welcome | pong | ping | event

Beveiliging (DESIGN §5): token uit env (PIPER_WS_TOKEN, constant-time vergelijking, nooit gelogd), Origin-check,
alleen luisteren op het opgegeven adres, maximale berichtgrootte, sluitcodes 4401 (auth) / 4403 (origin) /
4400 (protocol of configuratie komt niet overeen) die de browser-client als "niet herhalen" behandelt (4401/4403).
"""
from __future__ import annotations

import asyncio
import hmac
import json
import logging
import math
import os
import ssl
import threading
import time
from typing import Callable

from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed

from .assembler import Tick, TickAssembler
from .config import PiperSimRobotConfig
from .protocol import PROTO_VERSION, fnv1a, parse_frame, state_names

log = logging.getLogger("piper_sim.bridge")


class SimBridge:
    """Draait een asyncio-loop in een thread; callbacks worden vanuit die thread aangeroepen."""

    def __init__(self, cfg: PiperSimRobotConfig, on_tick: Callable[[Tick], None] | None = None,
                 on_cmd: Callable[[dict], None] | None = None, on_sim_connect: Callable[[dict], None] | None = None,
                 on_sim_disconnect: Callable[[], None] | None = None, scene_hash: str | None = None):
        self.cfg = cfg
        self.on_tick, self.on_cmd = on_tick, on_cmd
        self.on_sim_connect, self.on_sim_disconnect = on_sim_connect, on_sim_disconnect
        self.scene_hash = scene_hash                  # hash van assets/scene.xml van deze checkout (mag None)
        self.token = os.environ.get(cfg.token_env, "")
        self.loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()
        self._stop: asyncio.Event | None = None
        self._conns: set[ServerConnection] = set()
        self._sim: ServerConnection | None = None
        self.port = cfg.port
        self.error: BaseException | None = None
        self.hello: dict | None = None
        self.latest_state: dict | None = None
        self.latest_frames: dict[int, tuple[int, bytes]] = {}
        self.assembler: TickAssembler | None = None
        self.stats = {"connections": 0, "rejected_auth": 0, "rejected_origin": 0, "rejected_proto": 0,
                      "state": 0, "frames": 0, "bad_messages": 0, "seq_gaps": 0, "seq_repeats": 0}
        self._last_seq: int | None = None

    # ---------------------------------------------------------------- levenscyclus
    def start(self, timeout: float = 10.0) -> None:
        cfg = self.cfg
        if not self.token and not cfg.allow_no_auth:
            raise RuntimeError(f"omgevingsvariabele {cfg.token_env} is niet gezet (token is verplicht; "
                               "--allow-no-auth alleen op loopback)")
        if not self.token and cfg.host not in ("127.0.0.1", "localhost", "::1"):
            raise RuntimeError("zonder token mag alleen op loopback worden geluisterd")
        self._thread = threading.Thread(target=self._run, name="piper-sim-bridge", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout):
            raise RuntimeError("WebSocket-server startte niet op tijd")
        if self.error:
            raise RuntimeError(f"WebSocket-server kon niet starten: {self.error}")

    def stop(self) -> None:
        if self.loop and self._stop:
            self.loop.call_soon_threadsafe(self._stop.set)
        if self._thread:
            self._thread.join(timeout=10)

    def _run(self) -> None:
        try:
            asyncio.run(self._main())
        except BaseException as e:  # noqa: BLE001
            self.error = e
            self._ready.set()

    async def _main(self) -> None:
        self.loop = asyncio.get_running_loop()
        self._stop = asyncio.Event()
        cfg = self.cfg
        ssl_ctx = None
        if cfg.tls_cert and cfg.tls_key:
            ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_ctx.load_cert_chain(cfg.tls_cert, cfg.tls_key)
        origins = [o for o in cfg.allowed_origins] + ([None] if cfg.allow_missing_origin else [])
        async with serve(self._handler, cfg.host, cfg.port, origins=origins or None, ssl=ssl_ctx,
                         max_size=cfg.max_message_bytes, ping_interval=None, compression=None,
                         process_request=self._process_request) as server:
            self.port = server.sockets[0].getsockname()[1]
            log.info("luistert op %s:%s (%s), origins=%s, token=%s", cfg.host, self.port,
                     "wss" if ssl_ctx else "ws", origins, "vereist" if self.token else "UIT")
            self._ready.set()
            await self._stop.wait()
        for c in list(self._conns):
            await c.close(1001, "server stopt")

    def _process_request(self, connection, request):
        # alleen het pad /ws; anders 404 (geen open "catch-all")
        if request.path.split("?")[0] != "/ws":
            return connection.respond(404, "niet gevonden\n")
        return None

    # ---------------------------------------------------------------- thread-veilige helpers
    def broadcast_event(self, **event) -> None:
        if not self.loop:
            return
        msg = json.dumps({"type": "event", **event})
        for c in list(self._conns):
            self.loop.call_soon_threadsafe(lambda c=c: asyncio.ensure_future(self._safe_send(c, msg)))

    async def _safe_send(self, c: ServerConnection, msg: str) -> None:
        try:
            await c.send(msg)
        except ConnectionClosed:
            pass

    # ---------------------------------------------------------------- verbinding
    async def _handler(self, ws: ServerConnection) -> None:
        self.stats["connections"] += 1
        role = "sim"
        authed = False
        try:
            try:
                first = await asyncio.wait_for(ws.recv(), timeout=10)
            except asyncio.TimeoutError:
                await ws.close(4408, "hello time-out")
                return
            try:
                hello = json.loads(first) if isinstance(first, str) else None
            except json.JSONDecodeError:
                hello = None
            if not isinstance(hello, dict) or hello.get("type") != "hello":
                self.stats["rejected_proto"] += 1
                await ws.close(4400, "verwachtte hello")
                return
            supplied = str(hello.get("token", ""))
            if self.token and not hmac.compare_digest(supplied.encode(), self.token.encode()):
                self.stats["rejected_auth"] += 1
                await ws.close(4401, "auth mislukt")                # geen detail over het token
                return
            if hello.get("proto") != PROTO_VERSION:
                self.stats["rejected_proto"] += 1
                await ws.close(4400, f"proto {PROTO_VERSION} vereist")
                return
            role = hello.get("role", "sim")
            authed = True
            if role == "sim":
                problem = self._check_sim_hello(hello)
                if problem:
                    self.stats["rejected_proto"] += 1
                    log.warning("hello geweigerd: %s", problem)
                    await ws.close(4400, problem[:100])
                    return
                if self._sim is not None and self._sim is not ws:
                    await self._sim.close(4409, "vervangen door nieuwe verbinding")
                self._sim = ws
                self.hello = {k: v for k, v in hello.items() if k != "token"}
                self._last_seq = None
                self.assembler = TickAssembler(len(self.cfg.camera_sizes))
                if self.on_sim_connect:
                    self.on_sim_connect(self.hello)
            self._conns.add(ws)
            await ws.send(json.dumps({"type": "welcome", "proto": PROTO_VERSION, "role": role,
                                      "server_ms": int(time.time() * 1000), "fps": self.cfg.fps}))
            async for raw in ws:
                if isinstance(raw, (bytes, bytearray)):
                    if role == "sim":
                        self._on_frame(bytes(raw))
                    continue
                await self._on_text(ws, raw, role)
        except ConnectionClosed:
            pass
        except Exception:                                           # noqa: BLE001
            log.exception("fout in verbinding")
            try:
                await ws.close(1011, "interne fout")
            except Exception:                                       # noqa: BLE001
                pass
        finally:
            self._conns.discard(ws)
            if authed and role == "sim" and self._sim is ws:
                self._sim = None
                if self.assembler and self.on_tick:
                    for t in self.assembler.flush():
                        self.on_tick(t)
                if self.on_sim_disconnect:
                    self.on_sim_disconnect()

    def _check_sim_hello(self, h: dict) -> str | None:
        if h.get("state_names") != state_names() or h.get("action_names") != state_names():
            return "state_names/action_names komen niet overeen"
        if h.get("fps") != self.cfg.fps:
            return f"fps {h.get('fps')} != {self.cfg.fps}"
        cams = h.get("cameras") or []
        want = {n: wh for n, wh in self.cfg.camera_sizes.items()}
        got = {c.get("name"): (c.get("w"), c.get("h")) for c in cams}
        if list(got) != list(want) or any(tuple(got[n]) != tuple(want[n]) for n in want):
            return f"cameras komen niet overeen (server {want}, browser {got}); start de browser met ?cams=..&camsize=.."
        if [c.get("id") for c in cams] != list(range(len(cams))):
            return "camera-id's moeten 0..n-1 zijn"
        return None

    async def _on_text(self, ws: ServerConnection, raw: str, role: str) -> None:
        try:
            m = json.loads(raw)
            typ = m["type"]
        except (json.JSONDecodeError, KeyError, TypeError):
            self.stats["bad_messages"] += 1
            return
        if typ == "ping":
            await ws.send(json.dumps({"type": "pong", "id": m.get("id"), "t_client_ms": m.get("t_client_ms")}))
        elif typ == "pong":
            pass
        elif typ == "state" and role == "sim":
            self._on_state(m)
        elif typ == "cmd":
            if self.on_cmd:
                self.on_cmd({k: v for k, v in m.items() if k != "type"})
        else:
            self.stats["bad_messages"] += 1

    def _on_state(self, m: dict) -> None:
        st, ac, seq = m.get("state"), m.get("action"), m.get("seq")
        if (not isinstance(seq, int) or not isinstance(st, list) or not isinstance(ac, list) or len(st) != 14
                or len(ac) != 14 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in st + ac)):
            self.stats["bad_messages"] += 1
            return
        if self._last_seq is not None:
            if seq == self._last_seq:
                self.stats["seq_repeats"] += 1
                return
            if seq != self._last_seq + 1:
                self.stats["seq_gaps"] += 1
        self._last_seq = seq
        self.stats["state"] += 1
        self.latest_state = m
        if self.assembler:
            for t in self.assembler.add_state(m):
                if self.on_tick:
                    self.on_tick(t)

    def _on_frame(self, data: bytes) -> None:
        try:
            seq, cam, _fmt, jpeg = parse_frame(data)
        except ValueError:
            self.stats["bad_messages"] += 1
            return
        if jpeg[:2] != b"\xff\xd8":
            self.stats["bad_messages"] += 1
            return
        self.stats["frames"] += 1
        self.latest_frames[cam] = (seq, jpeg)
        if self.assembler:
            self.assembler.add_frame(seq, cam, jpeg)

    @property
    def sim_connected(self) -> bool:
        return self._sim is not None


def scene_hash_of(path: str) -> str | None:
    try:
        with open(path, encoding="utf-8") as f:
            return fnv1a(f.read())
    except OSError:
        return None
