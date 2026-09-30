"""PiperSimRobot: LeRobot `Robot`-plugin voor de dubbele Piper-sim in de Quest-browser (naar voorbeeld van
koenvanwijk/lerobot-robot-mujoco, maar met de browser als fysica-bron i.p.v. een lokale MuJoCo-instantie).

- `connect()` start de WebSocket-server (SimBridge); de browser (?rec=wss://..#token=..) verbindt zelf.
- `get_observation()` geeft de laatste toestand: 14 `*.pos`-waarden + per camera een RGB-array (H, W, 3).
- `send_action()` clamp de 14 waarden op de gewrichtslimieten (+ `max_relative_target` t.o.v. de laatste toestand)
  en bewaart ze in `last_action`. In de sim-opstelling wordt de actie in de browser bepaald (VR-teleop + IK); de
  clamp hier is de veiligheidslaag die later ook voor de echte arm (piper_sdk) gebruikt wordt. Niets gaat (nog)
  naar hardware.
"""
from __future__ import annotations

import io
import logging
import time
from functools import cached_property

import numpy as np
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.robots.robot import Robot

from .bridge import SimBridge, scene_hash_of
from .config import PiperSimRobotConfig
from .protocol import limits_by_name, state_names

log = logging.getLogger("piper_sim.robot")


def decode_jpeg(jpeg: bytes) -> np.ndarray:
    from PIL import Image
    with Image.open(io.BytesIO(jpeg)) as im:
        return np.asarray(im.convert("RGB"), dtype=np.uint8)


class PiperSimRobot(Robot):
    config_class = PiperSimRobotConfig
    name = "piper_sim"

    def __init__(self, config: PiperSimRobotConfig, bridge: SimBridge | None = None):
        super().__init__(config)
        self.config = config
        self.bridge = bridge
        self._owns_bridge = bridge is None
        self._connected = False
        self._limits = limits_by_name()
        self.last_action: dict[str, float] | None = None
        self.clamped_actions = 0

    # ----------------------------------------------------------- features
    @cached_property
    def _motor_ft(self) -> dict[str, type]:
        return {n: float for n in state_names()}

    @property
    def observation_features(self) -> dict:
        cams = {n: (h, w, 3) for n, (w, h) in self.config.camera_sizes.items()}
        return {**self._motor_ft, **cams}

    @property
    def action_features(self) -> dict:
        return dict(self._motor_ft)

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True                      # sim: altijd "gekalibreerd"

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    # ----------------------------------------------------------- verbinding
    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            raise DeviceAlreadyConnectedError(f"{self} is al verbonden")
        if self.bridge is None:
            self.bridge = SimBridge(self.config, scene_hash=scene_hash_of(
                str((__import__('pathlib').Path(__file__).resolve().parents[2] / "assets" / "scene.xml"))))
        if self._owns_bridge:
            self.bridge.start()
        self._connected = True
        if self.config.connect_timeout_s > 0:
            t0 = time.time()
            while not self.bridge.sim_connected and time.time() - t0 < self.config.connect_timeout_s:
                time.sleep(0.05)
            if not self.bridge.sim_connected:
                raise TimeoutError("browser verbond niet binnen connect_timeout_s")

    def disconnect(self) -> None:
        if not self._connected:
            raise DeviceNotConnectedError(f"{self} is niet verbonden")
        if self._owns_bridge and self.bridge:
            self.bridge.stop()
        self._connected = False

    # ----------------------------------------------------------- I/O
    def get_observation(self) -> dict:
        if not self._connected:
            raise DeviceNotConnectedError(f"{self} is niet verbonden")
        st = self.bridge.latest_state
        if st is None:
            raise RuntimeError("nog geen toestand ontvangen van de browser")
        obs: dict = {n: float(v) for n, v in zip(state_names(), st["state"])}
        for cam_id, (name, (w, h)) in enumerate(self.config.camera_sizes.items()):
            fr = self.bridge.latest_frames.get(cam_id)
            obs[name] = decode_jpeg(fr[1]) if fr else np.zeros((h, w, 3), np.uint8)
        return obs

    def send_action(self, action: dict) -> dict:
        if not self._connected:
            raise DeviceNotConnectedError(f"{self} is niet verbonden")
        names = state_names()
        unknown = set(action) - set(names)
        if unknown:
            raise KeyError(f"onbekende actie-sleutels: {sorted(unknown)}")
        present = None
        st = self.bridge.latest_state
        if st is not None and self.config.max_relative_target is not None:
            present = dict(zip(names, st["state"]))
        out: dict[str, float] = {}
        clamped = False
        for n in names:
            if n not in action:
                continue
            v = float(action[n])
            lo, hi = self._limits[n]
            c = min(hi, max(lo, v))
            if present is not None:
                d = self.config.max_relative_target
                c = min(present[n] + d, max(present[n] - d, c))
            clamped |= abs(c - v) > 1e-9
            out[n] = c
        self.clamped_actions += int(clamped)
        self.last_action = out
        return out
