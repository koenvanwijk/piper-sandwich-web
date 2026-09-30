"""Hugging Face Hub-upload van een LOKAAL geschreven LeRobot v3-dataset (DESIGN §4/§5).

Regels:
- Standaard UIT: er wordt alleen geüpload met `push` of `serve --push`; `--dry-run` valideert alles zonder netwerk.
- Token UITSLUITEND uit de omgevingsvariabele HF_TOKEN op deze machine. Nooit een CLI-argument, nooit gelogd,
  nooit in de dataset/kaart, nooit naar de browser. (Een `hf auth login`-cache wordt bewust NIET gebruikt.)
- Repo is standaard PRIVÉ (`private=True`); publiek alleen met expliciet `public=True`.
- De echte upload (`LeRobotDataset.push_to_hub`) is in deze PR NIET tegen de echte Hub getest; wel tegen een mock.
"""
from __future__ import annotations

import json
import logging
import os
import re
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

log = logging.getLogger("piper_sim.hub")
REPO_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*/[A-Za-z0-9][A-Za-z0-9._-]*$")
SIM_INFO = "meta/piper_sim_info.json"


class PushError(RuntimeError):
    pass


def git_sha(path: Path) -> str | None:
    try:
        out = subprocess.run(["git", "-C", str(path), "rev-parse", "--short=12", "HEAD"], capture_output=True, text=True, timeout=5)
        dirty = subprocess.run(["git", "-C", str(path), "status", "--porcelain"], capture_output=True, text=True, timeout=5).stdout.strip()
        return (out.stdout.strip() + ("+dirty" if dirty else "")) if out.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return os.environ.get("PIPER_SIM_GIT_SHA")


def write_sim_info(root: Path, hello: dict, server_scene_hash: str | None, repo_dir: Path | None) -> dict:
    """Schrijf `meta/piper_sim_info.json` (sim-versie + eenheden) bij de eerste browser-hello."""
    p = Path(root) / SIM_INFO
    info = {"app": hello.get("app"), "proto": hello.get("proto"), "fps": hello.get("fps"),
            "physics_timestep_s": hello.get("physics_timestep"), "substeps": hello.get("substeps"),
            "mode": hello.get("mode"), "browser_scene_hash": hello.get("scene_hash"),
            "server_scene_hash": server_scene_hash,
            "scene_hash_matches": (server_scene_hash == hello.get("scene_hash")) if server_scene_hash else None,
            "server_checkout_git_sha": git_sha(repo_dir) if repo_dir else None,
            "units": hello.get("units"), "cameras": hello.get("cameras"),
            "objects_dynamic": hello.get("objects_dynamic"), "objects_static": hello.get("objects_static")}
    p.parent.mkdir(parents=True, exist_ok=True)
    if p.exists():
        try:                                    # bij hervatten: behoud eerste info, noteer afwijkende scene-hash
            old = json.loads(p.read_text())
            if old.get("browser_scene_hash") != info["browser_scene_hash"]:
                info["WARNING"] = f"scene-hash wijkt af van eerdere sessie ({old.get('browser_scene_hash')})"
        except (OSError, json.JSONDecodeError):
            pass
    p.write_text(json.dumps(info, indent=2, ensure_ascii=False))
    return info


def dataset_description(root: Path, task: str | None, info: dict, episodes: list[dict]) -> str:
    ok = sum(1 for e in episodes if e.get("success") is True)
    bad = sum(1 for e in episodes if e.get("success") is False)
    frames = sum(e.get("frames", 0) for e in episodes)
    gaps = sum(e.get("seq_gaps", 0) for e in episodes)
    tasks = sorted({e.get("task") for e in episodes if e.get("task")}) or ([task] if task else [])
    return f"""**Gesimuleerde** teleoperatie (VR, Meta Quest) van twee AgileX Piper-armen in de browser-sim
[piper-sandwich-web](https://github.com/koenvanwijk/piper-sandwich-web) (MuJoCo WASM). Dit zijn géén echte-robot-data.

- **Taak/taken:** {'; '.join(tasks) or '(niet opgegeven)'}
- **Episodes:** {len(episodes)} ({ok} succes, {bad} mislukt, {len(episodes) - ok - bad} niet gemarkeerd); **frames:** {frames}; **fps:** {info.get('fps')}
- **Sim-versie:** app-checkout git `{info.get('server_checkout_git_sha')}`; scene-hash (browser) `{info.get('browser_scene_hash')}`, (server-checkout) `{info.get('server_scene_hash')}`{' — **AFWIJKEND**' if info.get('scene_hash_matches') is False else ''}; protocol v{info.get('proto')}; fysica-tijdstap {info.get('physics_timestep_s')} s ({info.get('substeps')} substappen per frame); modus `{info.get('mode')}`.
- **observation.state / action (14):** per arm `joint1..6` in **radialen** + `gripper` genormaliseerd **0 = open … 1 = dicht**; volgorde `left_joint1.pos … left_gripper.pos, right_joint1.pos … right_gripper.pos`. `action` = IK-doelstand (`qTarget`) + gripper-commando van dezelfde tick; `observation.state` is de gemeten stand vóór die actie.
- **Camera's:** {', '.join(c.get('name', '?') + f" {c.get('w')}x{c.get('h')}" for c in (info.get('cameras') or [])) or '-'} (vaste scene-camera's, JPEG vanuit de browser, daarna gecodeerd naar video).
- **Extra metadata** (niet in het LeRobot-standaardschema): `meta/piper_sim_episodes.jsonl` (succes, seq-gaten, beeld-lag per episode), `meta/piper_sim_info.json`.

**Beperkingen:** sim met bekende fysica-beperkingen (gripper opent ~7 cm terwijl brood 9 cm is; mes kantelt in de klem; geen beleg in de pot; linkerarm ~37 mm TCP-fout bij de pot door gewrichtslimieten). Beeld komt asynchroon uit de browser: per tick wordt het nieuwste beeld met `seq <=` tick gebruikt (`mean_image_lag_ticks` in het episodes-bestand); aantal seq-gaten in deze dataset: {gaps}. Geen beloning-/done-kolommen. Niet geschikt als bewijs van prestaties op de echte robot.
"""


@dataclass
class PushReport:
    repo_id: str
    private: bool
    dry_run: bool
    episodes: int = 0
    frames: int = 0
    files: int = 0
    bytes: int = 0
    video_keys: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    card_preview: str | None = None
    uploaded: bool = False
    hf_user: str | None = None


def validate(root: Path, repo_id: str) -> tuple[PushReport, object, list[dict], dict]:
    """Offline controle van de dataset; geeft (report, dataset, episodes-sidecar, sim-info). Geen netwerk."""
    from lerobot.datasets import LeRobotDataset
    root = Path(root)
    if not REPO_RE.match(repo_id or ""):
        raise PushError(f"repo_id {repo_id!r} moet de vorm 'gebruiker/naam' hebben (stel HF_REPO_ID of --repo-id in)")
    if not (root / "meta" / "info.json").is_file():
        raise PushError(f"{root} is geen LeRobot-dataset (meta/info.json ontbreekt)")
    prev = os.environ.get("HF_HUB_OFFLINE")
    os.environ["HF_HUB_OFFLINE"] = "1"                     # validatie mag nooit het netwerk op
    try:
        ds = LeRobotDataset(repo_id, root=root)
    finally:
        if prev is None:
            os.environ.pop("HF_HUB_OFFLINE", None)
        else:
            os.environ["HF_HUB_OFFLINE"] = prev
    rep = PushReport(repo_id=repo_id, private=True, dry_run=True)
    rep.episodes, rep.frames, rep.video_keys = ds.meta.total_episodes, ds.meta.total_frames, list(ds.meta.video_keys)
    if rep.episodes < 1 or rep.frames < 1:
        raise PushError("de dataset bevat geen episodes/frames")
    for vk in rep.video_keys:
        if not list((root / "videos" / vk).rglob("*.mp4")):
            raise PushError(f"geen mp4-bestanden voor {vk}")
    sample = ds[0]
    for k in ("observation.state", "action"):
        if tuple(sample[k].shape) != (14,):
            raise PushError(f"{k} heeft vorm {tuple(sample[k].shape)}, verwacht (14,)")
    last = ds[len(ds) - 1]                                  # laatste frame moet ook decodeerbaar zijn
    for vk in rep.video_keys:
        if vk not in last:
            raise PushError(f"videoframe {vk} niet te laden")
    files = [p for p in root.rglob("*") if p.is_file() and "images" not in p.relative_to(root).parts[:1]]
    rep.files, rep.bytes = len(files), sum(p.stat().st_size for p in files)
    eps: list[dict] = []
    side = root / "meta" / "piper_sim_episodes.jsonl"
    if side.is_file():
        eps = [json.loads(l) for l in side.read_text().splitlines() if l.strip()]
    else:
        rep.warnings.append("meta/piper_sim_episodes.jsonl ontbreekt (geen succes-informatie)")
    if eps and len(eps) != rep.episodes:
        rep.warnings.append(f"episodes-bestand ({len(eps)}) != dataset ({rep.episodes})")
    if any(e.get("seq_gaps") for e in eps):
        rep.warnings.append(f"{sum(1 for e in eps if e.get('seq_gaps'))} episode(s) met seq-gaten")
    info: dict = {}
    ip = root / SIM_INFO
    if ip.is_file():
        info = json.loads(ip.read_text())
        if info.get("scene_hash_matches") is False:
            rep.warnings.append("scene-hash van de browser wijkt af van de server-checkout")
        if info.get("WARNING"):
            rep.warnings.append(info["WARNING"])
    else:
        rep.warnings.append("meta/piper_sim_info.json ontbreekt (sim-versie onbekend)")
    return rep, ds, eps, info


def push(root: Path, repo_id: str | None = None, *, task: str | None = None, private: bool = True, public: bool = False,
         dry_run: bool = False, license: str = "apache-2.0", tags: list[str] | None = None) -> PushReport:
    """Valideer en (tenzij dry_run) upload. `private` blijft True tenzij `public=True`."""
    repo_id = repo_id or os.environ.get("HF_REPO_ID", "")
    make_private = not public
    rep, ds, eps, info = validate(Path(root), repo_id)
    rep.private, rep.dry_run = make_private, dry_run
    desc = dataset_description(Path(root), task, info, eps)
    card_kwargs = {"dataset_description": desc, "url": "https://github.com/koenvanwijk/piper-sandwich-web"}
    from lerobot.datasets.utils import create_lerobot_dataset_card
    card = create_lerobot_dataset_card(tags=["piper", "sim", "mujoco", "vr-teleoperation", *(tags or [])],
                                       dataset_info=ds.meta.info, license=license, repo_id=repo_id, **card_kwargs)
    preview = Path(str(Path(root).resolve()) + ".card-preview.md")      # buiten de dataset-map: wordt niet geüpload
    preview.write_text(str(card))
    rep.card_preview = str(preview)
    if dry_run:
        log.info("dry-run: %s (%s), %d episodes / %d frames / %d bestanden / %.1f MB; niets geüpload",
                 repo_id, "privé" if make_private else "PUBLIEK", rep.episodes, rep.frames, rep.files, rep.bytes / 1e6)
        return rep
    token = os.environ.get("HF_TOKEN", "")
    if not token:
        raise PushError("HF_TOKEN is niet gezet in de omgeving van deze proces (het token wordt nergens anders gelezen)")
    from huggingface_hub import HfApi
    try:
        who = HfApi(token=token).whoami()                   # controleert het token; we bewaren/tonen alleen de gebruikersnaam
        rep.hf_user = who.get("name")
    except Exception as e:                                  # noqa: BLE001
        raise PushError(f"HF_TOKEN werd geweigerd of Hub onbereikbaar ({type(e).__name__})") from None
    log.info("upload naar %s als %s (%s) ...", repo_id, rep.hf_user, "privé" if make_private else "PUBLIEK")
    ds.push_to_hub(tags=["piper", "sim", "mujoco", "vr-teleoperation", *(tags or [])], license=license, private=make_private,
                   dataset_description=desc, url=card_kwargs["url"])
    rep.uploaded = True
    return rep
