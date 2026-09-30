"""Zet losse `state`-berichten en binaire JPEG-frames om in vaste ticks (DESIGN-vr-recording.md §1).

De browser stuurt de JPEG's asynchroon: het beeld van tick `s` komt vaak ná `state[s]`, en niet elke tick heeft
een beeld (de render is zwaarder dan de sim). We houden daarom een korte vertraging (`delay` ticks) aan en koppelen
aan tick `s` per camera het nieuwste beeld met `frame_seq <= s`. Zo is het beeld exact als het er is, en anders
het laatste eerdere beeld (met `lag` = aantal ticks oud, voor statistiek).
Geen netwerk/asyncio hier; puur datastructuur, dus direct te testen.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field


@dataclass
class Tick:
    seq: int
    t_sim: float
    state: list[float]
    action: list[float]
    msg: dict                                   # volledig state-bericht (tcp, ctrl, objects, ...)
    frames: dict[int, tuple[int, bytes]] = field(default_factory=dict)   # cam_id -> (frame_seq, jpeg)


class TickAssembler:
    def __init__(self, n_cams: int, delay: int = 6, keep_frames: int = 90):
        self.n_cams, self.delay = n_cams, delay
        self._states: deque[dict] = deque()
        self._frames: dict[int, deque[tuple[int, bytes]]] = {c: deque(maxlen=keep_frames) for c in range(n_cams)}
        self.last_emitted = -1

    def add_state(self, msg: dict) -> list[Tick]:
        self._states.append(msg)
        out: list[Tick] = []
        while self._states and self._states[0]["seq"] + self.delay <= msg["seq"]:
            out.append(self._make(self._states.popleft()))
        return out

    def add_frame(self, seq: int, cam: int, jpeg: bytes) -> None:
        if cam in self._frames:
            self._frames[cam].append((seq, jpeg))

    def flush(self) -> list[Tick]:
        out = [self._make(m) for m in self._states]
        self._states.clear()
        return out

    def _make(self, m: dict) -> Tick:
        t = Tick(seq=m["seq"], t_sim=m["t_sim"], state=m["state"], action=m["action"], msg=m)
        for cam, q in self._frames.items():
            best = None
            for fseq, jpeg in q:
                if fseq <= t.seq and (best is None or fseq >= best[0]):
                    best = (fseq, jpeg)
            if best is not None:
                t.frames[cam] = best
        self.last_emitted = t.seq
        return t
