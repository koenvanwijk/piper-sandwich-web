# 🥪 Piper Sandwich VR

Two AgileX **Piper** 6-DoF arms in a "make a sandwich" scene, running **entirely
in the browser** with the official MuJoCo WebAssembly engine and rendered with
three.js. Put on a **Meta Quest**, press *Enter VR*, and teleoperate both arms
with the hand controllers — no install, no app, no local server.

**▶ Live:** https://koenvanwijk.github.io/piper-sandwich-web/

**▶ Quest passthrough table calibration:** https://koenvanwijk.github.io/piper-sandwich-web/table-ar/


![preview](preview.png)

## Open it on the Quest (typing a URL in VR is painful)

<img src="qr.png" alt="QR to the live app" width="200" align="right"/>

Pick whichever is easiest — you only need to do it once, then **bookmark** it:

- **Scan the QR** (right) with a QR scanner on the headset — e.g. the free
  *QR Scanner* app on Quest 3/3s uses the passthrough cameras to read a code off
  your phone or monitor and opens it in the browser. (Scanning it with your
  phone opens it on the phone, which is only useful for testing.)
- **Voice dictation:** in the Quest Browser tap the microphone in the address
  bar and say the address.
- **Type once, then bookmark** so it is one tap next time.

## Use it

Open the live link **in the Quest browser** and press **Enter VR** (or open it on
a desktop to orbit the scene with the mouse). Then:

- **Grip button = clutch.** Hold to make that arm follow your hand; release to
  reposition your hand without moving the arm.
- **Trigger = gripper** (analog close).
- Left controller drives the **left** arm, right controller the **right** arm.
- You start **sitting between the two arms, looking forward** at the board.

### Get comfortable (thumbsticks)

You can re-orient the whole workspace and the arms without leaving VR:

| Stick / button | Action |
|---|---|
| **Right stick ←/→** | turn (yaw) the whole workspace |
| **Right stick ↑/↓** | move it closer / further |
| **Left stick ←/→** | slide the workspace left / right |
| **Left stick ↑/↓** | raise / lower the table |
| **A / X button** | recenter to the default view |

The teleop stays correct no matter how you turn the scene: controller poses are
transformed into the workspace's own frame before driving the arms.

To toe the two arm bases inward permanently, set `ARM_YAW` in the scene builder
(`sim/build_scene.py`) — a static, physical mounting angle, no turntable.

Everything runs locally in the headset — physics, IK and rendering.

## Demo: broodje smeren

Een vaste, herhaalbare voorbeeldbeweging ("choreografie") waarin de twee armen een broodje
smeren: (1) ingrediënten verzamelen (rechts pakt het mes, links gaat boven de pot staan),
(2) boter smeren met het mes over `bread0`, (3) beleg toevoegen (linkerarm brengt beleg van de
pot naar het brood), (4) broodje sluiten (mes terug op het bord `plate`, daarna `bread1` met de gesloten
gripper tegen `bread0` aan schuiven — *niet* bovenop leggen, zie beperkingen).

**Starten**

- In de browser: open de pagina met `?demo=1` (of klik onderaan op de link *Demo: broodje smeren*;
  bv. lokaal `http://127.0.0.1:8000/?demo=1`, sneller afspelen: `?demo=1&speed=2`). Rechts staat een
  paneel (op een smal/staand scherm, bv. een autoscherm, onderaan) met de **stappenlijst** (huidige stap
  blauw, klaar = ✓), een **illustratie per stap** (`demo/step-*.svg`, gegenereerd met
  `python3 tools/make-step-images.py`) en grote knoppen **Pauze/Verder** en **Opnieuw**.
  Zonder `?demo=1` verandert er niets (teleop werkt zoals altijd). Tijdens de demo is teleop uit.
- Headless (Node, echte MuJoCo-WASM, zonder three.js):
  `node tools/run-sandwich-demo.mjs` (optioneel `--json`, `--tuning='{"PUSH_Z":0.02}'`,
  `--require-top`). Logt per stap de maximale TCP-fout, of het mes optilt, of boter/brood
  verschuift, of `bread1` is dichtgeschoven tegen `bread0` en het aantal boterblokjes op `bread0`.
  Exitcode 0 = `bread1` ligt tegen `bread0` (met `--require-top`: echt bovenop, dat lukt niet),
  2 = niet gelukt. Draait in ~8 s (Node ≥ 18, geen `npm install` nodig: MuJoCo-WASM zit in `vendor/`).

**Stappen aanpassen** — alles staat in `src/sandwich-motion.js`:

- `makeSandwichChoreography({ objects, tuning })` bouwt de stappen. Objectposities komen uit het
  model (`readObjectPositions`) of uit `DEFAULT_OBJECTS`; verschuif je een object in `scene.xml`,
  dan volgt de choreografie vanzelf.
- Constanten (hoogtes, gripper-oriëntatie per arm, smeer-slag, greeptijd) staan in `TUNING`/`GEOM`
  bovenaan en zijn te overschrijven via `tuning`.
- Een stap is `{ name, tracks: { left: [...], right: [...] } }` (armen parallel) of
  `{ name, arm, waypoints }`; een waypoint is `{ pos:[x,y,z], quat?, grip?, t }` (`t` = seconden vanaf
  het begin van de stap, `grip` 0 = open .. 1 = dicht). Stappen lopen na elkaar.
- `MotionPlayer(env, steps, { onStep, onDone, speed })` met `update(dt)`, `pause()`, `resume()`,
  `reset()` en `done`; `env` = `{ ik, qTarget, grip, tcpPose }` (de velden van `SandwichVR`).

**Bekende beperkingen** (gemeten met het headless-script, zie de PR voor de cijfers)

- **Mes pakken lukt maar half betrouwbaar.** De gripper opent ~7 cm en het handvat is 2 cm breed en
  1,6 cm dik; in de simulatie tilt het mes wel mee (ruim 10 cm), maar het kantelt in de klem
  (pitch tot ~60°) en schuift. Het werkt alleen met de vaste greeppositie/oriëntatie uit `TUNING`;
  kleine afwijkingen laten het mes vaak liggen. Het smeren is een bewegingsdemonstratie: het blad
  zweeft vlak boven het brood (blad-z ~5 cm bij de mes-oorsprong, met kanteling); er wordt dus niet
  echt "gesmeerd". De boterblokjes verschuiven wel (gem. ~18 mm, max ~56 mm in stap 2), maar het script
  kan niet onderscheiden of dat door het mes komt of doordat blokjes van het brood rollen.
- **Beleg is niet fysiek**: er zit geen beleg in de pot in `scene.xml`; de linkerarm "schept" met
  de gripper en opent boven het brood.
- **`bread1` grijpen lukt niet**: platliggend brood (9 cm) is breder dan de opening van de gripper
  (~7 cm); hoekklem (yaw ±30/±45°, theta 130/150, diverse diepte/hoogte, ook met hogere wrijving op de
  vingers) en zijkant-klem tilden het brood in geen enkele geteste combinatie op. Daarom **schuift** de
  demo `bread1` met de gesloten gripper naar `bread0` (stap 4b): dat werkt in de simulatie
  (tussenruimte 30 mm → ~15 mm, boterblokjes zitten er nog tussen), maar `bread1` ligt dus **niet
  bovenop** `bread0`.
- De demo zet in de browser (en in het script) botsingen van de statische basis-meshes uit
  (`relaxBaseContacts`): base_link en link1 overlappen ~6 mm in `scene.xml`, waardoor joint1 anders
  vastklemt en de armen niet zijwaarts kunnen zwenken.
- Linkerarm heeft in stap 3 tot ~37 mm TCP-fout rond de pot (gewrichtslimieten bij een neerwaartse tool).

## How it works

```
Quest browser (WebXR via three.js)
    │  controller grip pose + trigger/grip buttons
    ▼
teleop.js   clutched relative mapping (three-world → MuJoCo frame)
    ▼
ik.js       6×6 finite-difference damped-least-squares IK to each arm's TCP site
    ▼
@mujoco/mujoco (WASM)   position actuators + physics step
    ▼
scene-loader.js → three.js   meshes rebuilt from the compiled model each frame
```

- **Engine:** the official `@mujoco/mujoco` WebAssembly build (single-threaded,
  so it needs no cross-origin isolation and works on GitHub Pages).
- **Scene:** `assets/scene.xml` — the *same* MuJoCo scene as the Python sim
  (see the companion repo), with `meshdir` pointing at the bundled STL meshes.
- **IK** is ported 1:1 from the Python version and was validated headless in
  Node against this exact scene (0.0 mm convergence on both arms).

## Run locally

Any static file server works (module + WASM need HTTP, not `file://`):

```bash
python3 -m http.server 8000
# open http://localhost:8000
```

Desktop Chrome/Edge render the scene; WebXR needs a headset (or the browser's
WebXR emulator extension).

## Layout

```
index.html            importmap (three from CDN) + entry
src/app.js            init WASM, load scene, render/control loop, three.js WebXR
src/ik.js             finite-difference DLS IK per arm
src/teleop.js         clutch + three→MuJoCo mapping
src/qmath.js          quaternion helpers
src/sandwich-motion.js  demo-choreografie + MotionPlayer (DOM-vrij, ook headless)
src/demo-ui.js        demo-paneel (stappenlijst, plaatje, Pauze/Opnieuw), alleen bij ?demo=1
demo/step-*.svg       stap-illustraties (tools/make-step-images.py)
tools/                headless MuJoCo-test van de demo
src/scene-loader.js   MuJoCo model → three.js meshes (adapted from zalo/mujoco_wasm)
assets/scene.xml      the sandwich scene (shared with the Python sim)
assets/meshes/*.STL   Piper link meshes
vendor/mujoco/        official MuJoCo WASM engine (mujoco.js + mujoco.wasm)
```

## Tuning

| What | Where |
|------|-------|
| Motion scale, orientation lock | `HandTeleop` options in `src/app.js` |
| three→MuJoCo axis mapping | `QA`, `threeVecToMujoco` in `src/teleop.js` |
| Clutch thresholds | `CLUTCH_ON/OFF` in `src/teleop.js` |
| IK damping / step limit | `damping`, `maxStep` in `src/ik.js` |
| Gripper open/closed | `open7/closed7` in `ArmIK.apply` |
| Scene placement in VR | `mujocoRoot.position` in `src/app.js` |

## Known limitations & next steps

- Wrist orientation is **locked** by default (arm keeps the engage-time pose);
  flip `lockOrientation` to map controller rotation for full 6-DoF teleop.
- **Grasping** small props needs fingertip collision tuning (same as the Python
  sim); good next step for actually assembling the sandwich.
- **Butter is spreadable**: a heap of many small high-friction pats the knife
  pushes and smears across the bread. (A true elastic soft body would need
  MuJoCo's `elasticity.solid` flex plugin, which this WASM build does not expose
  for registration — and 100+ flex DoF would strain VR framerates anyway — so a
  granular heap is the robust, real-time choice, and reads well as spreading.)
- No headset camera streaming needed here — you're *in* the sim, which is the
  nice part of the browser version.

## Credits

- **MuJoCo** engine — `@mujoco/mujoco`, Apache-2.0, © Google DeepMind (vendored
  under `vendor/mujoco/`).
- **scene-loader.js** adapted from [`zalo/mujoco_wasm`](https://github.com/zalo/mujoco_wasm) (ISC).
- **three.js** — MIT, loaded from CDN.
- **Piper model** — AgileX `piper_ros` `piper_description` (BSD; repo MIT © 2024
  RosenYin), with a `tcp` site added.

See `NOTICE` for details. This project's own code is MIT (`LICENSE`).
