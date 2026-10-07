import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { VRButton } from 'three/addons/webxr/VRButton.js';
import load_mujoco from '../vendor/mujoco/mujoco.js';
import { loadSceneFromURL, getPosition, getQuaternion, drawTendonsAndFlex } from './scene-loader.js';
import { ArmIK } from './ik.js';
import { HandTeleop, teleopArm, orientationMode, rollAxisFromQuery, tiltAxisFromQuery, yawAxisFromQuery, tiltEnabledFromQuery, yawEnabledFromQuery } from './teleop.js';
import { mat2quat, rotErr } from './qmath.js';
import { parseTargetOpts, TargetStatus, createTargetGhosts } from './target-viz.js';
import { createDemoPanel } from './demo-ui.js';
import { MotionPlayer, makeSandwichChoreography, readObjectPositions, relaxBaseContacts } from './sandwich-motion.js';
import { parseRecConfig } from './recorder-client.js';
import { ctrlSummary } from './rec-state.js';
import { mjToWorld, headYaw, homeFromHead } from './xr-map.js';

const SIDES = ['left', 'right'];
// Demo (choreografie) is alleen actief met ?demo=1 in de URL; zonder verandert er niets.
const DEMO = new URLSearchParams(location.search).get('demo') === '1';
// Opname (WebSocket-recorder) is alleen actief met ?rec=wss://host/ws#token=... (zie README "Recording client").
// Zonder ?rec verandert er niets aan de frame-loop (rec-capture.js wordt dan niet geladen; recorder-client.js/rec-state.js zijn klein en zonder bijwerkingen).
const RECCFG = parseRecConfig();
// ?debug=1: overlay met per controller handedness/pose/doel + positie-/oriëntatiefout (zie src/debug-overlay.js).
const _Q = new URLSearchParams(location.search);
const DEBUG = _Q.get('debug') === '1', ROT = _Q.get('rot') === '1', HEADHOME = _Q.get('headhome') === '1';
// Teleop-modus (src/teleop.js): standaard '6dof' = TCP volgt de volledige relatieve controller-pose (6-DOF-IK over alle joints);
// ?mode=joints = PR #11 (yaw→j4 + tilt→j5 + rol→j6; ?tilt=0 / ?yaw=0, ?rollaxis=/?tiltaxis=/?yawaxis=);
// ?orient=0 (of ?rot=0 / ?roll=0) = alleen positie, pols vergrendeld.
const ORI_MODE = orientationMode(location.search);
const ROLL_AXIS_Q = rollAxisFromQuery(location.search), TILT_AXIS_Q = tiltAxisFromQuery(location.search), YAW_AXIS_Q = yawAxisFromQuery(location.search);
const TILT_ON = tiltEnabledFromQuery(location.search), YAW_ON = yawEnabledFromQuery(location.search);
// Doel-ghost per arm (src/target-viz.js): standaard tijdens clutch; ?target=0 uit, ?target=always, ?tgtok=8,3 / ?tgtbad=20,10 (mm,°), ?tgthaptic=0.
const TARGET = parseTargetOpts(location.search);
const newTeleop = () => new HandTeleop({ mode: ORI_MODE, rollAxis: ROLL_AXIS_Q, tiltAxis: TILT_AXIS_Q, yawAxis: YAW_AXIS_Q, tilt: TILT_ON, yaw: YAW_ON });
const MESHES = ['base_link', 'link1', 'link2', 'link3', 'link4', 'link5',
                'link6', 'gripper_base', 'link7', 'link8'].map(n => n + '.STL');

// Load the WASM engine, pointing it at the vendored binary.
const mujoco = await load_mujoco({
  locateFile: (path, prefix) => path.endsWith('.wasm')
    ? new URL('../vendor/mujoco/mujoco.wasm', import.meta.url).href
    : prefix + path,
});

// Populate the virtual file system with the scene + meshes.
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.mkdir('/working/meshes');
mujoco.FS.writeFile('/working/scene.xml',
  await (await fetch(new URL('../assets/scene.xml', import.meta.url))).text());
for (const f of MESHES) {
  const buf = new Uint8Array(await (await fetch(
    new URL('../assets/meshes/' + f, import.meta.url))).arrayBuffer());
  mujoco.FS.writeFile('/working/meshes/' + f, buf);
}

const status = document.getElementById('status');
const setStatus = s => { if (status) status.textContent = s; };

class SandwichVR {
  constructor() {
    this.mujoco = mujoco;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0.12, 0.14, 0.18);

    this.camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.01, 100);
    this.camera.position.set(0.0, 1.55, 0.55);   // roughly the user's head, looking forward
    this.scene.add(this.camera);

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const key = new THREE.DirectionalLight(0xffffff, 2.0);
    key.position.set(1, 3, 2); key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.near = 0.1; key.shadow.camera.far = 10;
    this.scene.add(key);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(devicePixelRatio);
    this.renderer.setSize(innerWidth, innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.xr.enabled = true;
    this.renderer.xr.setReferenceSpaceType('local-floor');
    document.body.appendChild(this.renderer.domElement);
    document.body.appendChild(VRButton.createButton(this.renderer));

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.95, -0.6);
    this.controls.enableDamping = true;
    this.controls.update();

    addEventListener('resize', () => {
      this.camera.aspect = innerWidth / innerHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(innerWidth, innerHeight);
    });

    this._tmpV = new THREE.Vector3();
    this._acc = 0; this._last = performance.now() / 1000;
  }

  async init() {
    [this.model, this.data, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, 'scene.xml', this);

    // Default view: sit *between* the two arms, looking forward at the board.
    // A +90° yaw makes MuJoCo +x (toward the board) point to the user's forward
    // (three -z); the arms (MuJoCo ±y) then sit on the user's left/right, with
    // the bases flanking them at z≈0. Adjust live with the thumbsticks (readNav).
    this.mujocoRoot.rotation.y = Math.PI / 2;
    this.mujocoRoot.position.set(0, 0.95, -0.32);
    this.mujocoRoot.updateMatrixWorld(true);
    this._homeView = { pos: this.mujocoRoot.position.clone(),
                       rotY: this.mujocoRoot.rotation.y };

    // Start from the 'home' keyframe.
    mujoco.mj_resetData(this.model, this.data);
    if (this.model.nkey > 0) this.data.qpos.set(this.model.key_qpos.slice(0, this.model.nq));
    mujoco.mj_forward(this.model, this.data);

    // FIX (links/rechts gespiegeld): base_link (world) overlapt ~6 mm met link1 en houdt joint1 vast (gemeten: joint1 blijft ~0 terwijl
    // de actuator 0,8 rad vraagt). De IK-doelen zijn dan fysiek onbereikbaar en de arm beweegt zijwaarts de verkeerde kant op.
    // Dit stond alleen in de demo (initDemo); teleop had het niet. Nu altijd (zie tools/test-teleop-mapping.mjs).
    relaxBaseContacts(this.model);

    this.ik = {}; this.qTarget = {}; this.qIK = {}; this.grip = {}; this.teleop = {};
    for (const s of SIDES) {
      this.ik[s] = new ArmIK(mujoco, this.model, this.data, s);
      this.qTarget[s] = this.ik[s].currentQ();
      this.grip[s] = 0.0;
      this.teleop[s] = newTeleop();
      this.ik[s].apply(this.qTarget[s], this.grip[s]);
    }
    this._mocap = {};
    for (const s of SIDES) {
      const bid = mujoco.mj_name2id(this.model, mujoco.mjtObj.mjOBJ_BODY.value, `${s}_target`);
      this._mocap[s] = this.model.body_mocapid[bid];
    }

    if (TARGET.mode !== 'off') {
      this._ghost = createTargetGhosts({ THREE, parent: this.mujocoRoot, sides: SIDES });
      this._tstat = {}; this._tinfo = {}; this._badPulseT = {};
      for (const s of SIDES) this._tstat[s] = new TargetStatus(TARGET);
    }
    if (DEMO) this.initDemo();
    if (RECCFG) await this.initRec();
    if (DEBUG) { const { createDebugOverlay } = await import('./debug-overlay.js'); this._dbg = createDebugOverlay({ THREE, camera: this.camera }); }

    this.renderer.setAnimationLoop(() => this.frame());
    setStatus(DEMO ? 'Demo: ' + this.demo.stepName
                   : 'Ready — press "Enter VR". Hold grip = clutch, trigger = gripper.');
  }

  // Demo "broodje smeren" (src/sandwich-motion.js). Alleen aangeroepen bij ?demo=1.
  initDemo() {
    relaxBaseContacts(this.model);      // zie uitleg in sandwich-motion.js (base/link1-overlap)
    const build = () => {
      const objects = readObjectPositions(this.mujoco, this.model, this.data);
      return new MotionPlayer(this, makeSandwichChoreography({ objects }), {
        speed: Number(new URLSearchParams(location.search).get('speed')) || 1,   // ?demo=1&speed=2 = sneller
        onStep: (name, i) => { setStatus('Demo: ' + name); if (this._demoUI) this._demoUI.setStep(i); },
        onDone: () => { setStatus('Demo: klaar — klik op "Opnieuw" om te herhalen');
                        if (this._demoUI) this._demoUI.setDone(); },
      });
    };
    this._buildDemo = build;
    this.demo = build();
    const steps = this.demo.steps;
    this._demoUI = createDemoPanel({
      names: steps.map(s => s.name),
      images: steps.map(s => s.image && new URL('../' + s.image, import.meta.url).href),
      onPause: () => this.demo.pause(),
      onResume: () => this.demo.resume(),
      onRestart: () => {
        // scene terug naar de 'home'-keyframe en choreografie herstarten
        this.mujoco.mj_resetData(this.model, this.data);
        if (this.model.nkey > 0) this.data.qpos.set(this.model.key_qpos.slice(0, this.model.nq));
        this.mujoco.mj_forward(this.model, this.data);
        for (const s of SIDES) { this.qTarget[s] = this.ik[s].currentQ(); this.grip[s] = 0; }
        this.demo = build();
        this._demoUI.setStep(0);
      },
    });
    this._demoUI.setStep(0);
  }

  // Opname-client (alleen met ?rec). Modules worden dynamisch geladen, zodat de normale pagina ongewijzigd blijft.
  async initRec() {
    if (RECCFG.error) { console.warn('[rec] ' + RECCFG.error); setStatus('REC uit: ' + RECCFG.error); return; }
    const [{ RecStateSampler, fnv1a, stateNames, REC_FPS, REC_DT, REC_SUBSTEPS, PROTO_VERSION },
           { RecorderClient, createRecBadge },
           { CameraCapture },
           { RecController, EdgeDetector, keyAction, padActions },
           { createRecHud }] = await Promise.all([
      import('./rec-state.js'), import('./recorder-client.js'), import('./rec-capture.js'),
      import('./rec-controls.js'), import('./rec-hud.js')]);
    const cfg = RECCFG.cfg;
    const sampler = new RecStateSampler(this);
    const sceneXml = await (await fetch(new URL('../assets/scene.xml', import.meta.url))).text();
    const capture = cfg.cams.length ? new CameraCapture(this, cfg.cams, { w: cfg.camW, h: cfg.camH, quality: cfg.camQuality }) : null;
    // vaste tijdstap: 16 substappen van (1/30)/16 s = 2,083 ms (in plaats van 2 ms) zodat 1 tick exact 1/30 s is
    this._recDtPhys = REC_DT / REC_SUBSTEPS;
    this.model.opt.timestep = this._recDtPhys;
    const setBadge = createRecBadge(cfg.host);
    let connState = 'connecting';
    const hud = createRecHud({ THREE, camera: this.camera, connLabel: () => connState });
    let ctl = null;                                   // RecController (hieronder); de callbacks lopen via de closure
    const client = new RecorderClient({
      url: cfg.url, token: cfg.token,
      onStatus: (st, msg) => { setBadge(st, msg); connState = st; if (ctl) ctl.onConn(st); },
      onEvent: m => { if (ctl) ctl.onEvent(m); },
      helloFn: () => ({ type: 'hello', proto: PROTO_VERSION, app: 'piper-sandwich-web', fps: REC_FPS, dt: REC_DT,
        physics_timestep: this._recDtPhys, substeps: REC_SUBSTEPS, mode: DEMO ? 'demo' : 'teleop',
        scene_hash: fnv1a(sceneXml), state_names: stateNames(), action_names: stateNames(),
        units: { joint: 'rad', gripper: 'norm 0=open..1=closed', pos: 'm', quat: 'wxyz (MuJoCo frame)' },
        cameras: capture ? capture.info : [], objects_dynamic: sampler.dynamic.map(b => b.name),
        objects_static: sampler.staticObjects(), frame_format: '[u32 seq LE][u8 cam_id][u8 fmt 0=jpeg][u16 0][jpeg]' }),
    });
    this.rec = { sampler, capture, client, seq: 0, tSim: 0, acc: 0, cfg, ticks: 0, hud, padActions };
    ctl = new RecController({
      send: (cmd, extra) => client.sendCmd(cmd, extra),
      resetScene: () => this.resetScene(),
      seqNow: () => this.rec.seq,
      haptic: (hand, intensity, ms, pulses) => this.haptic(hand, intensity, ms, pulses),
      onChange: snap => hud.update(snap),
    });
    this.rec.ctl = ctl; this.rec.edges = new EdgeDetector(300);
    hud.update(ctl.snapshot());
    addEventListener('keydown', e => { const a = keyAction(e); if (a) { ctl.act(a, null); e.preventDefault(); } });
    const hint = document.getElementById('hint');
    if (hint) hint.innerHTML = 'Grip = clutch · Trigger = gripper &nbsp;|&nbsp; <b>REC</b>: A = start/stop+bewaar · B = weggooien+reset · ' +
      'X = geslaagd (+stop) · Y = reset scene · stick-klik = recenter &nbsp;|&nbsp; toetsen: S · D · K · R';
    client.start();
  }

  // Scene terug naar de startstaat zonder de pagina te herladen (mj_resetData + keyframe 'home'). De vaste tijdstap
  // blijft gelijk (model.opt.timestep wordt niet geraakt) en `seq`/t_sim lopen door, zodat de server geen gat ziet.
  resetScene() {
    const mj = this.mujoco;
    mj.mj_resetData(this.model, this.data);
    if (this.model.nkey > 0) this.data.qpos.set(this.model.key_qpos.slice(0, this.model.nq));
    mj.mj_forward(this.model, this.data);
    for (const s of SIDES) {
      this.qTarget[s] = this.ik[s].currentQ(); this.grip[s] = 0;
      this.teleop[s] = newTeleop();       // clutch loslaten; volgende grip = nieuw anker
      this.ik[s].apply(this.qTarget[s], this.grip[s]);
    }
    if (this._buildDemo) { this.demo = this._buildDemo(); if (this._demoUI) this._demoUI.setStep(0); }
    this.syncScene();
  }

  // Haptische puls op de controller(s) (Quest): hand = 'left' | 'right' | null (beide). Feature-detect, nooit een fout.
  haptic(hand, intensity = 0.5, ms = 60, pulses = 1) {
    try {
      const session = this.renderer.xr.getSession(); if (!session) return;
      for (const src of session.inputSources) {
        if (hand && src.handedness !== hand) continue;
        const gp = src.gamepad; if (!gp) continue;
        const act = (gp.hapticActuators && gp.hapticActuators[0]) || null;
        const pulse = i => {
          if (act && act.pulse) act.pulse(intensity, ms);
          else if (gp.vibrationActuator && gp.vibrationActuator.playEffect)
            gp.vibrationActuator.playEffect('dual-rumble', { duration: ms, strongMagnitude: intensity, weakMagnitude: intensity });
        };
        for (let i = 0; i < pulses; i++) setTimeout(() => { try { pulse(i); } catch {} }, i * (ms + 60));
      }
    } catch { /* haptiek is optioneel */ }
  }

  // Knoppen van de Quest-controllers (rising edge) -> RecController. Alleen met ?rec.
  pollRecButtons() {
    const session = this.renderer.xr.getSession(); if (!session || !this.rec.ctl) return;
    for (const a of this.rec.padActions(session.inputSources, this.rec.edges, performance.now())) this.rec.ctl.act(a.action, a.hand);
  }

  // Eén opname-tick = exact REC_DT aan simulatietijd: observatie -> actie (teleop/demo + IK) -> 16 fysica-stappen -> versturen.
  recTick(cmds, dt) {
    const r = this.rec, obs = r.sampler.observe();
    const anyEngaged = this.control(cmds, dt);
    for (let i = 0; i < 16; i++) this.mujoco.mj_step(this.model, this.data);
    const seq = r.seq++; r.tSim = r.seq * dt; r.ticks++;
    r.client.sendState({ type: 'state', seq, t_sim: +(seq * dt).toFixed(6), t_client_ms: +performance.now().toFixed(1),
      state: obs.state, action: r.sampler.action(), tcp: obs.tcp, ctrl: ctrlSummary(cmds, this.teleop),
      objects: obs.objects, xr: this.renderer.xr.isPresenting, demo_step: this.demo ? this.demo.index : null });
    if (r.capture && r.client.wantFrames()) {
      this.syncScene();
      r.hud.mesh.visible = false;               // het 3D-HUD hoort niet in de opgenomen camerabeelden
      try { r.capture.grab((camId, ab) => r.client.sendFrame(seq, camId, ab)); } finally { r.hud.mesh.visible = true; }
    }
    return anyEngaged;
  }

  tcpPose(side) {
    const s = this.ik[side].site, d = this.data;
    const m = []; for (let k = 0; k < 9; k++) m.push(d.site_xmat[9 * s + k]);
    return { pos: [d.site_xpos[3*s], d.site_xpos[3*s+1], d.site_xpos[3*s+2]],
             quat: mat2quat(m) };
  }

  readControllers() {
    const out = { left: null, right: null };
    const session = this.renderer.xr.getSession();
    if (!session) return out;
    const frame = this.renderer.xr.getFrame();
    const ref = this.renderer.xr.getReferenceSpace();
    if (!frame || !ref) return out;
    // Controller poses are transformed into the scene root's LOCAL frame, so
    // the teleop mapping stays correct no matter how the joysticks have rotated
    // or moved the scene (the fixed three->MuJoCo swizzle then applies).
    const rootQinv = this.mujocoRoot.getWorldQuaternion(new THREE.Quaternion()).invert();
    for (const src of session.inputSources) {
      if (!src.gripSpace || !src.handedness) continue;
      const pose = frame.getPose(src.gripSpace, ref);
      if (!pose) continue;
      const p = pose.transform.position, o = pose.transform.orientation;
      const lp = this.mujocoRoot.worldToLocal(new THREE.Vector3(p.x, p.y, p.z));
      const lq = rootQinv.clone().multiply(new THREE.Quaternion(o.x, o.y, o.z, o.w));
      const gp = src.gamepad;
      const btn = i => (gp && gp.buttons[i]) ? gp.buttons[i].value : 0;
      const raw = { pos: [lp.x, lp.y, lp.z], quat: [lq.w, lq.x, lq.y, lq.z],
                    trigger: btn(0), grip: btn(1) };
      if (DEBUG) raw.world = [p.x, p.y, p.z];
      if (this.rec && gp) { raw.buttons = gp.buttons.map(b => +b.value.toFixed(3)); raw.axes = Array.from(gp.axes); }
      if (src.handedness === 'left') out.left = raw;
      else if (src.handedness === 'right') out.right = raw;
    }
    return out;
  }

  // Read thumbsticks / face buttons for navigating the scene.
  readNav() {
    const nav = { yaw: 0, dist: 0, height: 0, strafe: 0, reset: false };
    const session = this.renderer.xr.getSession();
    if (!session) return nav;
    const dz = v => Math.abs(v) < 0.2 ? 0 : v;
    for (const src of session.inputSources) {
      const gp = src.gamepad; if (!gp) continue;
      const ax = i => gp.axes.length > i ? gp.axes[i] : 0;
      const pressed = i => gp.buttons[i] && gp.buttons[i].pressed;
      if (src.handedness === 'right') {
        nav.yaw += dz(ax(2));            // right stick L/R : spin the workspace
        nav.dist += dz(ax(3));           // right stick U/D : move it closer/further
        if (this.rec ? pressed(3) : (pressed(4) || pressed(5))) nav.reset = true;   // A/B : recenter (met ?rec: thumbstick-druk; A/B = opname)
      } else if (src.handedness === 'left') {
        nav.strafe += dz(ax(2));         // left stick L/R : slide the workspace
        nav.height += dz(ax(3));         // left stick U/D : raise / lower the table
        if (this.rec ? pressed(3) : (pressed(4) || pressed(5))) nav.reset = true;   // X/Y : recenter (met ?rec: thumbstick-druk; X/Y = opname)
      }
    }
    return nav;
  }

  // OPT-IN (?headhome=1, niet op hardware getest): de scène staat vast t.o.v. de 'local-floor'-referentieruimte (kijkrichting -z). Keek u bij het
  // starten van de sessie een andere kant op, dan zit u niet "tussen de armen, kijkend naar het bord". Met ?headhome=1 wordt de thuispositie bij
  // sessiestart en bij elke recenter uit hoofdpositie + kijkrichting (yaw) berekend (xr-map.js). Standaard en buiten VR ongewijzigd.
  homeToHead(nav) {
    if (!HEADHOME) return;
    if (!this.renderer.xr.isPresenting) { this._xrHomed = false; return; }
    if (this._xrHomed && !nav.reset) return;
    const cam = this.renderer.xr.getCamera(); cam.updateMatrixWorld(true);
    const p = new THREE.Vector3(), q = new THREE.Quaternion();
    cam.matrixWorld.decompose(p, q, new THREE.Vector3());
    if (!(cam.cameras && cam.cameras.length) || (p.lengthSq() === 0 && q.w === 1)) return;   // nog geen geldige hoofdpose
    const yaw = headYaw(q); if (yaw == null) return;
    const h = homeFromHead([p.x, p.y, p.z], yaw);
    this._homeView = { pos: new THREE.Vector3(h.pos[0], this._homeView.pos.y, h.pos[2]), rotY: h.rotY };
    this.mujocoRoot.position.copy(this._homeView.pos); this.mujocoRoot.rotation.y = h.rotY;
    this._xrHomed = true;
  }

  debugInfo() {
    const out = { xr: this.renderer.xr.isPresenting, rot: ORI_MODE, rootRotY: this.mujocoRoot.rotation.y, headYaw: null, refSpace: null, sources: [], ctrl: {} };
    const session = this.renderer.xr.getSession();
    if (session) {
      out.refSpace = this.renderer.xr.getReferenceSpace() ? 'local-floor (ingesteld in app.js)' : null;
      for (const src of session.inputSources) out.sources.push({ handedness: src.handedness, profile: (src.profiles && src.profiles[0]) || '' });
      const cam = this.renderer.xr.getCamera(); const q = new THREE.Quaternion(); cam.matrixWorld.decompose(new THREE.Vector3(), q, new THREE.Vector3());
      out.headYaw = headYaw(q);
    }
    const cmds = this._lastCmds || {};
    for (const s of SIDES) {
      const c = cmds[s]; if (!c) { out.ctrl[s] = null; continue; }
      const t = (this._dbgT || {})[s], tp = this.tcpPose(s), tcp = tp.pos;
      const tgt = t && t.engaged ? t.pos : null;
      // fout doel ↔ werkelijke TCP (positie in m, oriëntatie in rad) + restfout van de IK-oplossing zelf
      const errPos = tgt ? Math.hypot(...tgt.map((v, i) => v - tcp[i])) : null;
      const errRot = tgt && t.quat ? Math.hypot(...rotErr(t.quat, tp.quat)) : null;
      out.ctrl[s] = { world: c.world, mj: [c.pos[0], -c.pos[2], c.pos[1]], grip: c.grip, trigger: c.trigger,
        engaged: this.teleop[s].engaged, roll: t && t.engaged ? t.roll : 0, tilt: t && t.engaged ? t.tilt : 0, yaw: t && t.engaged ? t.yaw : 0, target: tgt, tcp, delta: tgt ? tgt.map((v, i) => v - tcp[i]) : null, errPos, errRot, ik: t && t.err ? t.err : null, tstat: (this._tinfo || {})[s] || null };
    }
    return out;
  }

  applyNav(nav, dt) {
    const r = this.mujocoRoot;
    if (nav.reset) {
      r.position.copy(this._homeView.pos);
      r.rotation.y = this._homeView.rotY;
    } else {
      r.rotation.y -= nav.yaw * 1.2 * dt;          // spin scene
      r.position.z += nav.dist * 0.5 * dt;         // stick up (-) -> further away (-z)
      r.position.x += nav.strafe * 0.5 * dt;       // slide left / right
      r.position.y -= nav.height * 0.4 * dt;       // stick up (-) -> raise
    }
    r.updateMatrixWorld(true);
  }

  // Doel-ghost + kleurstatus (src/target-viz.js). Alleen in 6dof-modus telt de oriëntatie mee (in joints/lock is het doel
  // positie-only; de ghost neemt dan de oriëntatie van de echte TCP over). Haptiek: korte zwakke puls bij overgang naar rood (≤ 1×/1,5 s).
  updateTargetGhost(s, t, dt) {
    if (!this._ghost) return;
    const tp = this.tcpPose(s), six = this.teleop[s].mode === '6dof';
    if (t.engaged) {
      const posErr = Math.hypot(...t.pos.map((v, i) => v - tp.pos[i]));
      const rotE = six ? Math.hypot(...rotErr(t.quat, tp.quat)) : 0;
      const e = t.err || {};
      const st = this._tstat[s].update({ posErr, rotErr: rotE, ikPos: e.pos || 0, ikRot: six ? (e.rot || 0) : 0, atLimit: !!e.atLimit }, dt);
      this._tinfo[s] = st;
      this._ghost.update(s, { visible: true, pos: t.pos, quat: six ? t.quat : tp.quat, tcpPos: tp.pos, status: st.status });
      const now = performance.now();
      if (st.enteredBad && TARGET.haptic && now - (this._badPulseT[s] || -1e9) > 1500) { this._badPulseT[s] = now; this.haptic(s, 0.25, 35, 1); }
    } else {
      this._tstat[s].reset(); this._tinfo[s] = null;
      this._ghost.update(s, { visible: TARGET.mode === 'always', pos: tp.pos, quat: tp.quat, tcpPos: tp.pos, status: 'ok' });
    }
  }

  // Arm-aansturing voor één stap van `dt` seconden: demo of teleop -> IK -> ctrl. Geeft terug of een arm 'engaged' is.
  control(cmds, dt) {
    let anyEngaged = false;
    if (this.demo && !this.demo.done) {
      // Demo bestuurt beide armen; teleop-invoer wordt tijdens de demo genegeerd.
      this.demo.update(dt);
      for (const s of SIDES) if (this._mocap[s] >= 0 && this.demo.cmd) {
        const a = this._mocap[s] * 3, p = this.demo.cmd[s].pos;
        this.data.mocap_pos[a] = p[0]; this.data.mocap_pos[a+1] = p[1]; this.data.mocap_pos[a+2] = p[2];
      }
      if (this._ghost) this._ghost.hideAll();
    } else for (const s of SIDES) {
      const t = teleopArm(this, s, this.teleop[s], this.ik[s], cmds[s], this.tcpPose(s), 3, dt);   // 6-DOF pose-teleop (src/teleop.js)
      if (DEBUG) (this._dbgT ||= {})[s] = t;
      if (t.engaged) {
        this.grip[s] = t.grip;
        if (this._mocap[s] >= 0) {
          const a = this._mocap[s] * 3;
          this.data.mocap_pos[a] = t.pos[0];
          this.data.mocap_pos[a+1] = t.pos[1];
          this.data.mocap_pos[a+2] = t.pos[2];
        }
        anyEngaged = true;
      }
      this.updateTargetGhost(s, t, dt);
      this.ik[s].apply(this.qTarget[s], this.grip[s]);
    }
    return anyEngaged;
  }

  // Push MuJoCo transforms into three.js.
  syncScene() {
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition(this.data.xpos, b, this.bodies[b].position);
        getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
      }
    }
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.data.light_xpos, l, this.lights[l].position);
        getPosition(this.data.light_xdir, l, this._tmpV);
        this.lights[l].lookAt(this._tmpV.add(this.lights[l].position));
      }
    }
    drawTendonsAndFlex(this.mujocoRoot, this.model, this.data);
  }

  frame() {
    const now = performance.now() / 1000;
    const frameDt = Math.min(0.05, now - this._last);

    // navigate / orient the scene with the thumbsticks first
    const nav = this.readNav();
    this.homeToHead(nav);
    this.applyNav(nav, frameDt);

    const cmds = this.readControllers();
    if (DEBUG) this._lastCmds = cmds;
    if (this.rec) {
      this.pollRecButtons();
      if (this.rec.ctl.state === 'RECORDING' && now - (this._hudT || 0) > 0.1) { this._hudT = now; this.rec.hud.update(this.rec.ctl.snapshot()); }
      // vaste tijdstap: per tick precies 1/30 s simulatietijd; bij een trage client loopt de sim langzamer (max 3 ticks/frame)
      const dt = 1 / 30, r = this.rec; r.acc += frameDt; this._last = now;
      let n = 0;
      while (r.acc >= dt && n < 3) { this.recTick(cmds, dt); r.acc -= dt; n++; }
      if (r.acc > 3 * dt) r.acc = 0;
    } else {
      this.control(cmds, frameDt);
      // real-time-ish physics stepping
      this._acc += frameDt; this._last = now;
      const dt = this.model.opt.timestep;
      let n = 0;
      while (this._acc >= dt && n < 30) { mujoco.mj_step(this.model, this.data); this._acc -= dt; n++; }
    }

    this.syncScene();
    if (this._dbg) this._dbg.update(this.debugInfo());

    if (!this.renderer.xr.isPresenting) this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}

const app = new SandwichVR();
await app.init();
window.sandwichVR = app;
