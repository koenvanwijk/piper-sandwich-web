"""Eigen recorder-loop met de LeRobotDataset-API (DESIGN-vr-recording.md §3/§4; alleen LOKAAL opslaan).

Episode-statemachine:  IDLE --start--> RECORDING --stop--> SAVING --> IDLE
                                          \\--discard--> IDLE
cmd-berichten (van de browser of een testscript):
  {"cmd":"start", "task":"...", "id":"optioneel"}   begin episode (de eerstvolgende tick telt als frame 0)
  {"cmd":"stop",  "success": true|false}            episode bewaren (save_episode)
  {"cmd":"discard"}                                 episode weggooien (clear_episode_buffer)
  {"cmd":"success","value":true|false}              succes markeren van de lopende episode
  {"cmd":"status"}                                  status terug als event

Alle dataset-bewerkingen draaien in één worker-thread (video-encodering blokkeert anders de WebSocket-loop);
ticks en commando's worden in volgorde verwerkt, dus een episodegrens valt exact tussen twee ticks.
LeRobot v3 heeft geen succes-veld; dat gaat naar `meta/piper_sim_episodes.jsonl` (per episode, naast de standaardbestanden).
"""
from __future__ import annotations

import json
import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from lerobot.datasets import LeRobotDataset

from lerobot_robot_piper_sim.assembler import Tick
from lerobot_robot_piper_sim.config import PiperSimRobotConfig
from lerobot_robot_piper_sim.protocol import state_names
from lerobot_robot_piper_sim.robot import decode_jpeg

log = logging.getLogger("piper_sim.recorder")
SIDECAR = "meta/piper_sim_episodes.jsonl"


def build_features(cfg: PiperSimRobotConfig) -> dict:
    """LeRobot-features: observation.state / action (14 float32, met namen) + observation.images.<cam> (video)."""
    names = state_names()
    feats = {
        "observation.state": {"dtype": "float32", "shape": (len(names),), "names": names},
        "action": {"dtype": "float32", "shape": (len(names),), "names": names},
    }
    for cam, (w, h) in cfg.camera_sizes.items():
        feats[f"observation.images.{cam}"] = {"dtype": "video", "shape": (h, w, 3),
                                              "names": ["height", "width", "channels"]}
    return feats


@dataclass
class EpisodeStats:
    start_seq: int = -1
    last_seq: int = -1
    frames: int = 0
    gaps: int = 0
    skipped_no_frame: int = 0
    success: bool | None = None
    task: str = ""
    max_frame_lag: int = 0
    lag_sum: int = 0
    t_start: float = field(default_factory=time.time)
    meta: dict = field(default_factory=dict)


class EpisodeRecorder:
    def __init__(self, cfg: PiperSimRobotConfig, root: Path, repo_id: str, task: str,
                 bridge=None, vcodec: str | None = None, crf: int | None = None, on_event=None, sim_info: dict | None = None):
        self.cfg, self.root, self.repo_id, self.default_task = cfg, Path(root), repo_id, task
        self.bridge, self.on_event = bridge, on_event or (lambda **_: None)
        self.sim_info = sim_info or {}
        self.state = "IDLE"
        self.features = build_features(cfg)
        self.q: queue.Queue = queue.Queue()
        self.cur: EpisodeStats | None = None
        self.saved: list[dict] = []
        self.discarded = 0
        self.ds: LeRobotDataset | None = None
        self._vcodec, self._crf = vcodec, crf
        self._thread = threading.Thread(target=self._worker, name="piper-sim-recorder", daemon=True)
        self._closed = threading.Event()
        self.errors: list[str] = []
        self._open_dataset()
        self._thread.start()

    # ------------------------------------------------------------------ dataset
    def _open_dataset(self) -> None:
        kw = {}
        if self._vcodec or self._crf:
            from lerobot.configs.video import RGBEncoderConfig
            enc = {}
            if self._vcodec:
                enc["vcodec"] = self._vcodec
            if self._crf:
                enc["crf"] = self._crf
            kw["rgb_encoder"] = RGBEncoderConfig(**enc)
        if (self.root / "meta" / "info.json").exists():
            self.ds = LeRobotDataset.resume(self.repo_id, root=self.root)
            if self.ds.meta.features.keys() != self.features.keys():
                raise RuntimeError("bestaande dataset heeft andere features dan de huidige configuratie")
            log.info("dataset hervat: %s (%d episodes)", self.root, self.ds.meta.total_episodes)
        else:
            self.ds = LeRobotDataset.create(self.repo_id, fps=self.cfg.fps, features=self.features, root=self.root,
                                            robot_type="piper_sim", use_videos=True, image_writer_threads=4, **kw)
            log.info("dataset aangemaakt: %s", self.root)

    # ------------------------------------------------------------------ API (aangeroepen vanuit de bridge-thread)
    def on_tick(self, t: Tick) -> None:
        self.q.put(("tick", t))

    def on_cmd(self, m: dict) -> None:
        cmd = m.get("cmd")
        if cmd == "start":
            latest = self.bridge.latest_state["seq"] if self.bridge and self.bridge.latest_state else -1
            self.q.put(("start", {"start_seq": latest + 1, "task": m.get("task") or self.default_task, "id": m.get("id")}))
        elif cmd in ("stop", "discard"):
            if self.bridge and self.bridge.assembler:              # wachtende ticks nog in de huidige episode duwen
                for t in self.bridge.assembler.flush():
                    self.on_tick(t)
            self.q.put((cmd, {"success": m.get("success")}))
        elif cmd == "success":
            self.q.put(("success", {"value": bool(m.get("value", True))}))
        elif cmd == "reset":                                       # scene-reset in de browser: lopende episode NOOIT bewaren
            self.q.put(("reset", {}))
        elif cmd == "status":                                      # via de queue: komt na eerdere start/stop/... aan de beurt
            self.q.put(("status", {}))
        else:
            self.on_event(event="error", message=f"onbekend commando {cmd!r}")

    def on_sim_disconnect(self) -> None:
        self.q.put(("interrupted", {}))

    def status(self) -> dict:
        return {"state": self.state, "episodes_saved": len(self.saved), "next_episode": self.ds.meta.total_episodes, "discarded": self.discarded,
                "frames_in_episode": self.cur.frames if self.cur else 0, "errors": self.errors[-3:]}

    def close(self, timeout: float = 300) -> None:
        """Verwerk de rest van de queue, gooi een lopende (niet-gestopte) episode weg en finaliseer de dataset."""
        self.q.put(("close", {}))
        self._closed.wait(timeout)
        self._thread.join(timeout=5)

    # ------------------------------------------------------------------ worker
    def _worker(self) -> None:
        try:
            while True:
                kind, arg = self.q.get()
                if kind == "close":
                    if self.state == "RECORDING":
                        self._discard("server stopt tijdens episode")
                    break
                try:
                    getattr(self, "_h_" + kind)(arg)
                except Exception as e:                                 # noqa: BLE001
                    log.exception("fout bij %s", kind)
                    self.errors.append(f"{kind}: {e}")
                    self.on_event(event="error", message=f"{kind}: {e}")
                    if self.state == "RECORDING":
                        self._discard(f"fout: {e}")
        finally:
            try:
                self.ds.finalize()
                log.info("dataset gefinaliseerd: %d episodes", len(self.saved))
            except Exception:                                          # noqa: BLE001
                log.exception("finalize mislukt")
            self._closed.set()

    def _h_start(self, a: dict) -> None:
        if self.state != "IDLE":
            self.on_event(event="error", message=f"start genegeerd: toestand {self.state}")
            return
        self.cur = EpisodeStats(start_seq=a["start_seq"], task=a["task"], meta={"client_id": a.get("id")})
        self.state = "RECORDING"
        self.on_event(event="episode_started", start_seq=a["start_seq"], task=a["task"])

    def _h_tick(self, t: Tick) -> None:
        if self.state != "RECORDING" or self.cur is None or t.seq < self.cur.start_seq:
            return
        c = self.cur
        if c.last_seq >= 0 and t.seq != c.last_seq + 1:
            c.gaps += 1
        cams = list(self.cfg.camera_sizes)
        if any(i not in t.frames for i in range(len(cams))):
            c.skipped_no_frame += 1                    # nog geen beeld ontvangen voor alle camera's
            return
        frame = {"task": c.task,
                 "observation.state": np.asarray(t.state, dtype=np.float32),
                 "action": np.asarray(t.action, dtype=np.float32)}
        for i, name in enumerate(cams):
            fseq, jpeg = t.frames[i]
            img = decode_jpeg(jpeg)
            w, h = self.cfg.camera_sizes[name]
            if img.shape != (h, w, 3):
                raise ValueError(f"camera {name}: beeld {img.shape} != {(h, w, 3)}")
            frame[f"observation.images.{name}"] = img
            lag = t.seq - fseq
            c.max_frame_lag = max(c.max_frame_lag, lag); c.lag_sum += lag
        self.ds.add_frame(frame)
        if c.frames == 0:
            c.meta["first_seq"] = t.seq
        c.frames += 1
        c.last_seq = t.seq

    def _h_success(self, a: dict) -> None:
        if self.cur:
            self.cur.success = a["value"]

    def _h_stop(self, a: dict) -> None:
        if self.state != "RECORDING" or self.cur is None:
            self.on_event(event="error", message="stop genegeerd: geen lopende episode")
            return
        c = self.cur
        if a.get("success") is not None:
            c.success = bool(a["success"])
        if c.frames == 0:
            self._discard("geen frames (nog geen beeld ontvangen?)")
            return
        self.state = "SAVING"
        self.on_event(event="saving", frames=c.frames)
        ep_index = self.ds.meta.total_episodes
        t0 = time.time()
        self.ds.save_episode()
        rec = {"episode_index": ep_index, "task": c.task, "frames": c.frames, "duration_s": round(c.frames / self.cfg.fps, 3),
               "success": c.success, "first_seq": c.meta.get("first_seq"), "last_seq": c.last_seq, "seq_gaps": c.gaps,
               "frames_without_image_skipped": c.skipped_no_frame, "max_image_lag_ticks": c.max_frame_lag,
               "mean_image_lag_ticks": round(c.lag_sum / max(1, c.frames * len(self.cfg.camera_sizes)), 2),
               "client_id": c.meta.get("client_id"), "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
               "encode_s": round(time.time() - t0, 2)}
        if c.gaps:
            log.warning("episode %d bevat %d seq-gaten (timestamps zijn dan niet echt 1/30 s)", ep_index, c.gaps)
        side = self.root / SIDECAR
        side.parent.mkdir(parents=True, exist_ok=True)
        with open(side, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec) + "\n")
        self.saved.append(rec)
        self.cur, self.state = None, "IDLE"
        self.on_event(event="episode_saved", **rec)

    def _h_discard(self, a: dict) -> None:
        if self.state == "RECORDING":
            self._discard("discard-commando")
        else:
            self.on_event(event="error", message="discard genegeerd: geen lopende episode")

    def _h_status(self, a: dict) -> None:
        self.on_event(event="status", **self.status())

    def _h_reset(self, a: dict) -> None:
        if self.state == "RECORDING":
            self._discard("scene reset")
        self.on_event(event="scene_reset")

    def _h_interrupted(self, a: dict) -> None:
        if self.state == "RECORDING":
            self._discard("verbinding met de browser verbroken (episode onderbroken)")

    def _discard(self, why: str) -> None:
        try:
            self.ds.clear_episode_buffer()
        finally:
            self.discarded += 1
            self.cur, self.state = None, "IDLE"
            log.info("episode weggegooid: %s", why)
            self.on_event(event="episode_discarded", reason=why)
