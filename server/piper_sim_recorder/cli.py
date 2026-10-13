"""`piper-sim-recorder serve|push` -- zie server/README.md."""
from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import sys
import threading
from pathlib import Path

from lerobot_robot_piper_sim.bridge import SimBridge, scene_hash_of
from lerobot_robot_piper_sim.config import PiperSimRobotConfig

from . import hub
from .recorder import EpisodeRecorder

log = logging.getLogger("piper_sim.cli")
REPO_DIR = Path(__file__).resolve().parents[2]          # checkout van piper-sandwich-web


def parse_cams(s: str) -> dict[str, tuple[int, int]]:
    out = {}
    for part in filter(None, (p.strip() for p in s.split(","))):
        name, _, size = part.partition(":")
        w, _, h = (size or "640x480").partition("x")
        out[name] = (int(w), int(h))
    return out


DEFAULT_PAGE = "https://koenvanwijk.github.io/piper-sandwich-web/"


def setup_url(page: str, ws_url: str, token: str, cams: dict) -> str:
    """record.html-link met alles in het #fragment (wordt niet naar servers gestuurd / niet gelogd)."""
    from urllib.parse import urlencode
    sizes = {f"{w}x{h}" for w, h in cams.values()}
    f = {"rec": ws_url}
    if token:
        f["token"] = token
    f["cams"] = ",".join(cams)
    if len(sizes) == 1:
        f["camsize"] = sizes.pop()
    return page.rstrip("/") + "/record.html#" + urlencode(f)


def print_setup_url(a, bridge, cfg) -> None:
    """Print (stdout, niet via logging) de kant-en-klare link + optioneel een QR (pip install qrcode)."""
    ws_url = a.public_url or f"{'wss' if cfg.tls_cert else 'ws'}://{'localhost' if cfg.host in ('127.0.0.1', '0.0.0.0', '::') else cfg.host}:{bridge.port}/ws"
    url = setup_url(a.page_url, ws_url, os.environ.get("PIPER_WS_TOKEN", ""), cfg.camera_sizes)
    print("\n=== Opname-sessie ===  (bevat het token: niet delen, niet in chat/issues plakken)", flush=True)
    if not a.public_url:
        print("  let op: geen --public-url opgegeven; vanaf de Quest is wss://<host-met-geldig-cert>/ws nodig", flush=True)
    print("  open op de Quest:  " + url, flush=True)
    try:
        import qrcode                                               # optioneel
        q = qrcode.QRCode(border=1); q.add_data(url); q.make(fit=True)
        q.print_ascii(invert=True)
    except ImportError:
        print("  (QR-code: pip install qrcode)", flush=True)
    print(flush=True)


def cmd_serve(a) -> int:
    cfg = PiperSimRobotConfig(host=a.host, port=a.port, fps=30, camera_sizes=parse_cams(a.cams),
                              allowed_origins=[o for o in a.origin], allow_missing_origin=a.allow_missing_origin,
                              allow_no_auth=a.allow_no_auth, tls_cert=a.tls_cert, tls_key=a.tls_key)
    repo_id = a.repo_id or os.environ.get("HF_REPO_ID") or "local/piper-sandwich-sim"
    root = Path(a.root).expanduser()
    scene_hash = scene_hash_of(str(REPO_DIR / "assets" / "scene.xml"))
    rec_holder: dict = {}

    def on_event(**ev):
        log.info("event: %s", json.dumps(ev, default=str))
        if bridge.loop:
            bridge.broadcast_event(**ev)

    bridge = SimBridge(cfg, scene_hash=scene_hash,
                       on_tick=lambda t: rec_holder["r"].on_tick(t),
                       on_cmd=lambda m: rec_holder["r"].on_cmd(m),
                       on_sim_connect=lambda h: (hub.write_sim_info(root, h, scene_hash, REPO_DIR),
                                                 log.info("browser verbonden: mode=%s scene_hash=%s", h.get("mode"), h.get("scene_hash"))),
                       on_sim_disconnect=lambda: rec_holder["r"].on_sim_disconnect())
    recorder = EpisodeRecorder(cfg, root, repo_id, a.task, bridge=bridge, vcodec=a.vcodec, crf=a.crf, on_event=on_event)
    rec_holder["r"] = recorder
    bridge.start()
    log.info("klaar: luistert op %s:%s, dataset %s (%s)", cfg.host, bridge.port, root, repo_id)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    if a.port_file:
        Path(a.port_file).write_text(str(bridge.port))
    if not a.no_print_url:
        print_setup_url(a, bridge, cfg)
    stop.wait()
    log.info("stoppen ...")
    bridge.stop()
    recorder.close()
    print(json.dumps({"episodes": recorder.saved, "discarded": recorder.discarded, "bridge": bridge.stats}, indent=2, default=str))
    if a.push or a.dry_run:
        try:
            rep = hub.push(root, repo_id, task=a.task, public=a.public, dry_run=a.dry_run or not a.push)
            print(json.dumps(rep.__dict__, indent=2))
        except hub.PushError as e:
            print(f"push mislukt: {e}", file=sys.stderr)
            return 3
    return 0


def cmd_push(a) -> int:
    try:
        rep = hub.push(Path(a.root).expanduser(), a.repo_id, task=a.task, public=a.public, dry_run=a.dry_run)
    except hub.PushError as e:
        print(f"push mislukt: {e}", file=sys.stderr)
        return 3
    print(json.dumps(rep.__dict__, indent=2))
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="piper-sim-recorder", description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("serve", help="WebSocket-server + recorder (lokale dataset)")
    s.add_argument("--root", required=True, help="map voor de LeRobot-dataset (wordt aangemaakt, of hervat)")
    s.add_argument("--task", default="Teleoperated in VR (piper-sandwich-web)")
    s.add_argument("--repo-id", default=None, help="'gebruiker/naam' (of env HF_REPO_ID); alleen echt nodig voor push")
    s.add_argument("--host", default="127.0.0.1", help="interface om op te luisteren (standaard alleen loopback)")
    s.add_argument("--port", type=int, default=8765)
    s.add_argument("--origin", action="append", default=None, help="toegestane Origin (herhaalbaar); standaard env PIPER_WS_ORIGINS of https://koenvanwijk.github.io")
    s.add_argument("--allow-missing-origin", action="store_true", help="accepteer clients zonder Origin-header (alleen testen)")
    s.add_argument("--allow-no-auth", action="store_true", help="geen token (alleen op loopback, alleen testen)")
    s.add_argument("--cams", default="front:640x480,top:640x480", help="camera's, moet overeenkomen met ?cams=&camsize= in de browser")
    s.add_argument("--vcodec", default=None, help="videocodec (standaard LeRobot: libsvtav1)")
    s.add_argument("--crf", type=int, default=None)
    s.add_argument("--tls-cert"); s.add_argument("--tls-key")
    s.add_argument("--public-url", help="wss-URL zoals de Quest de server bereikt, bv. wss://robot-pc.tailnet.ts.net/ws (voor de geprinte link)")
    s.add_argument("--page-url", default=DEFAULT_PAGE, help="basis-URL van de webapp (standaard GitHub Pages)")
    s.add_argument("--no-print-url", action="store_true", help="print de opname-link (met token) niet bij start")
    s.add_argument("--port-file", help="schrijf de gekozen poort hierheen (bij --port 0)")
    s.add_argument("--push", action="store_true", help="na afloop uploaden naar de Hub (standaard UIT; vereist HF_TOKEN)")
    s.add_argument("--dry-run", action="store_true", help="na afloop alleen valideren (geen upload)")
    s.add_argument("--public", action="store_true", help="maak de Hub-repo PUBLIEK (standaard privé)")
    s.set_defaults(fn=cmd_serve)
    u = sub.add_parser("push", help="bestaande lokale dataset valideren/uploaden")
    u.add_argument("--root", required=True); u.add_argument("--repo-id", default=None)
    u.add_argument("--task", default=None)
    u.add_argument("--dry-run", action="store_true", help="valideer + datasetkaart-voorbeeld, geen netwerk")
    u.add_argument("--public", action="store_true")
    u.set_defaults(fn=cmd_push)
    a = p.parse_args(argv)
    logging.basicConfig(force=True, level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    if a.cmd == "serve" and a.origin is None:
        a.origin = [o for o in os.environ.get("PIPER_WS_ORIGINS", "https://koenvanwijk.github.io").split(",") if o]
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
