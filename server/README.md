# piper-sim-recorder (fase 2)

Python-kant van de VR-opname (zie `../DESIGN-vr-recording.md`): een WebSocket-server die het protocol van de
opname-client in de browser (`../src/recorder-client.js`, `?rec=...`) spreekt, een LeRobot-`Robot`-plugin (`piper_sim`)
en een recorder-loop die **LeRobot v3-datasets** schrijft. Standaard alleen **lokaal**; uploaden naar de Hugging Face Hub
is een aparte, expliciete stap (`--push`, standaard uit).

```
Quest-browser ──wss──► SimBridge (websockets/asyncio) ──► TickAssembler ──► EpisodeRecorder ──► LeRobotDataset (lokaal)
   (state JSON + JPEG-frames)        hello/token/Origin      state+beeld→tick    start/stop/discard      parquet + mp4 + meta
                                                                                                             └─► (optioneel) push_to_hub (privé)
```

## Installeren (Python >= 3.12)

```bash
cd server
uv venv --python 3.13 .venv && uv pip install --python .venv/bin/python -e ".[test]"   # installeert lerobot[dataset], websockets, pillow
# (of: python -m venv .venv && .venv/bin/pip install -e ".[test]")
.venv/bin/python -m pytest          # 13 tests, geen netwerk, geen token
```
Getest met lerobot 0.6.1, websockets 17, Python 3.13. `ffmpeg`-CLI is niet nodig (LeRobot codeert met PyAV).
Waarschuwing "torchcodec kan niet geladen worden" is onschuldig: LeRobot valt terug op `pyav`.

## Opnemen (alleen lokaal)

```bash
export PIPER_WS_TOKEN=$(openssl rand -hex 16)          # verplicht; dit zelfde token ga je in de browser-URL gebruiken
.venv/bin/piper-sim-recorder serve --root ~/lerobot_datasets/piper-sandwich-sim \
    --task "Broodje smeren (VR)" --host 127.0.0.1 --port 8765 \
    --origin https://koenvanwijk.github.io --cams front:640x480,top:640x480
```
Open in de browser: `https://koenvanwijk.github.io/piper-sandwich-web/?rec=wss://<jouw-host>/ws&cams=front,top&camsize=640x480#token=<PIPER_WS_TOKEN>`
(de `wss://`-voorkant: Caddy/Tailscale/tunnel, zie DESIGN §1; lokaal testen kan met `?rec=ws://127.0.0.1:8765/ws`).

- **Luisteren:** alleen op `--host` (standaard `127.0.0.1`), alleen pad `/ws`. Zonder token start de server niet
  (`--allow-no-auth` alleen op loopback, alleen voor tests). Token-vergelijking is constant-time, het token wordt nooit gelogd.
- **Origin-check:** alleen `--origin` (herhaalbaar, of env `PIPER_WS_ORIGINS`); een client zonder Origin wordt geweigerd
  tenzij `--allow-missing-origin`. Sluitcodes: `4401` token fout, `4403`/HTTP 403 origin, `4400` protocol/configuratie (bv. andere
  camera's dan `--cams`), `4409` vervangen door nieuwere browser-verbinding. De client herhaalt bij 4401/4403 niet.
- **Episodes via `cmd`-berichten** (in fase 3 komen daar Quest-knoppen voor; nu een testscript):
  ```bash
  .venv/bin/python scripts/send_cmds.py --url ws://127.0.0.1:8765/ws --script "start:mijn taak;wait=20;stop:success;start;wait=5;discard"
  ```
  `start[:taak]`, `stop[:success|:fail]`, `discard`, `success`, `status`. Tijdens een episode wegvallen van de browser of een fout gooit
  de lopende episode weg (nooit een half bestand). Bij stoppen van de server (`Ctrl-C`) wordt de dataset `finalize()`d.
- **Dataset:** `observation.state` (14, float32, namen `left_joint1.pos … right_gripper.pos`; rad, gripper 0 open … 1 dicht),
  `action` (14, `qTarget` + gripper van dezelfde tick), `observation.images.<cam>` (video), `task`, `timestamp = frame_index/30`.
  Extra (buiten het standaardschema): `meta/piper_sim_episodes.jsonl` (succes, seq-gaten, beeld-lag) en `meta/piper_sim_info.json`
  (sim-versie: scene-hash browser/server, git-sha van deze checkout, eenheden). Een bestaande map wordt hervat (zelfde features vereist).
- **Beeld is asynchroon:** de browser stuurt JPEG's niet elke tick (software-GL ±7–16 Hz). Per tick wordt per camera het nieuwste beeld met `seq <=` tick
  gebruikt (vertraging ≤ 6 ticks); ticks waarvoor nog geen beeld bestaat worden overgeslagen. `mean_image_lag_ticks` staat per episode in het sidecar-bestand.
- **Plugin:** `from lerobot_robot_piper_sim import PiperSimRobot, PiperSimRobotConfig` (type `piper_sim`; `camera_sizes` i.p.v. `cameras` omdat
  `RobotConfig` daar `CameraConfig`-objecten verwacht). `send_action()` clamp op gewrichtslimieten en `max_relative_target` (standaard 0,35 rad/stap) en stuurt
  **nog niets** naar hardware.

## Uploaden naar Hugging Face (standaard UIT)

De upload is niet in deze PR tegen de echte Hub getest (alleen dry-run en mock). Doe de **eerste upload bewust privé** en met `--dry-run` vooraf.

1. Maak op https://huggingface.co/settings/tokens een **fine-grained** token met alleen *write* op de ene dataset-repo
   (of maak eerst een lege privé dataset-repo `gebruiker/piper-sandwich-sim` aan).
2. Zet het token **alleen op de robot-pc**, in de omgeving van het proces dat uploadt — nooit in de browser-URL, nooit in git:
   ```bash
   read -rs HF_TOKEN && export HF_TOKEN        # typ/plak het token; het verschijnt niet op het scherm of in je shell-history
   export HF_REPO_ID=gebruiker/piper-sandwich-sim
   ```
   (Voor een service: `EnvironmentFile=/etc/piper-sim.env` met `chmod 600`.) Een `hf auth login`-cache wordt bewust **niet** gebruikt.
3. Valideer zonder netwerk (controleert dataset, 14-dim state/action, mp4's, laadt frame 0 en het laatste frame, schrijft een kaartvoorbeeld
   `<root>.card-preview.md` naast de dataset):
   ```bash
   .venv/bin/piper-sim-recorder push --root ~/lerobot_datasets/piper-sandwich-sim --dry-run
   ```
4. Eerste privé-upload (maakt de repo privé aan als die nog niet bestaat; `--public` is nodig om publiek te worden):
   ```bash
   .venv/bin/piper-sim-recorder push --root ~/lerobot_datasets/piper-sandwich-sim
   # of direct na het opnemen:  serve ... --push   (--dry-run = alleen valideren)
   ```
   Het commando controleert het token met `whoami` (alleen de gebruikersnaam wordt getoond), gebruikt `LeRobotDataset.push_to_hub(private=True)` en zet een
   datasetkaart met taak, sim-versie (git-sha + scene-hash), fps, eenheden en bekende beperkingen.
- Omvang: ≈ 0,65 MB voor 160 frames met één 320×240-camera (AV1, `crf 30`); reken op tientallen MB per uur bij twee 640×480-camera's (schatting, niet gemeten).
- Privacy: de dataset bevat alleen sim-data (geen echte camerabeelden); toch standaard privé.

## Bestanden
`lerobot_robot_piper_sim/` (plugin: `config`, `robot`, `bridge` = WS-server, `assembler`, `protocol`) · `piper_sim_recorder/` (`recorder`, `hub`, `cli`) ·
`scripts/send_cmds.py` · `tests/`.

## Beperkingen / nog niet gedaan
Geen Quest-knoppen voor start/stop (fase 3), geen echte arm/`piper_sdk` (fase 4+), geen TLS-certificaatbeheer (gebruik Caddy/Tailscale), echte Hub-upload
en Quest/echte `wss://` niet getest, `t_client_ms`-klok niet gesynchroniseerd (timestamps komen uit `seq/30`), bij seq-gaten in een episode is de tijdas niet exact 1/30 s (gewaarschuwd en in het sidecar-bestand).
