from __future__ import annotations

import os
from dataclasses import dataclass, field

from lerobot.robots.config import RobotConfig


def _default_cams() -> dict[str, tuple[int, int]]:
    return {"front": (640, 480), "top": (640, 480)}      # naam -> (breedte, hoogte); zie ?cams=/?camsize= in de browser


@RobotConfig.register_subclass("piper_sim")
@dataclass(kw_only=True)
class PiperSimRobotConfig(RobotConfig):
    """Virtuele dubbele Piper (sim in de Quest-browser) als LeRobot-robot.

    De token komt NIET uit de config maar uit de omgevingsvariabele `PIPER_WS_TOKEN` (naam instelbaar).
    """
    host: str = "127.0.0.1"                 # interface waarop geluisterd wordt (nooit 0.0.0.0 tenzij bewust)
    port: int = 8765
    token_env: str = "PIPER_WS_TOKEN"
    allowed_origins: list[str] = field(default_factory=lambda: [
        o for o in os.environ.get("PIPER_WS_ORIGINS", "https://koenvanwijk.github.io").split(",") if o])
    allow_missing_origin: bool = False      # alleen voor lokale test-/CLI-clients
    allow_no_auth: bool = False             # alleen toegestaan op loopback (testen)
    tls_cert: str | None = None             # optioneel; normaal zet je Caddy/Tailscale voor de server
    tls_key: str | None = None
    fps: int = 30
    # naam -> (breedte, hoogte) van de browser-camera's (?cams=&camsize=). Bewust niet `cameras`: RobotConfig verwacht daar CameraConfig-objecten.
    camera_sizes: dict[str, tuple[int, int]] = field(default_factory=_default_cams)
    # safety: maximale verandering per stap (rad; gripper 0-1) voor send_action(); None = alleen limieten clampen
    max_relative_target: float | None = 0.35
    connect_timeout_s: float = 0.0          # >0: connect() wacht zo lang op de eerste browser-hello
    max_message_bytes: int = 4 * 1024 * 1024
