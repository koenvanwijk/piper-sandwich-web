// Herbruikbare demo-choreografie "broodje smeren" voor de Piper-armen.
//
// Deze module heeft GEEN afhankelijkheid van three.js of de DOM, zodat hij ook
// headless in Node getest kan worden (zie tools/run-sandwich-demo.mjs).
// Alle coördinaten zijn in het MuJoCo-frame (z omhoog, meters), quaternions [w,x,y,z].
//
// Opbouw:
//   * Een choreografie is een lijst STAPPEN (fasen). Stappen lopen na elkaar.
//     Binnen één stap kunnen meerdere armen PARALLEL een eigen route volgen:
//       { name: 'butter', tracks: { right: [wp, wp, ...], left: [wp, ...] } }
//     Kortere vorm voor één arm:
//       { name: 'fetch-knife', arm: 'right', waypoints: [wp, ...] }
//   * Een waypoint: { pos:[x,y,z], quat?:[w,x,y,z], grip?:0..1, t:seconden }
//     `t` is de tijd vanaf het begin van de stap waarop het waypoint bereikt is
//     (oplopend). Een arm zonder quat/grip in een waypoint behoudt de vorige waarde.
//     Twee opeenvolgende waypoints met dezelfde pos en een andere grip = alleen
//     de gripper bewegen (bv. langzaam sluiten).
//   * `MotionPlayer` speelt dit af: per frame `update(dt)` interpoleert de doel-TCP
//     (lineair in positie, slerp in oriëntatie, easing), roept `ik.solve()` aan en
//     zet `qTarget`, `grip` en `ik.apply()`.
import { qmul } from './qmath.js';

// ---------------------------------------------------------------------------
// Kleine wiskundehulpjes
// ---------------------------------------------------------------------------
const D2R = Math.PI / 180;
export const qy = deg => { const a = deg * D2R / 2; return [Math.cos(a), 0, Math.sin(a), 0]; };
export const qz = deg => { const a = deg * D2R / 2; return [Math.cos(a), 0, 0, Math.sin(a)]; };

/**
 * Tool-oriëntatie (neerwaarts gericht): q = qz(yaw) * qy(theta) * (roll om de tool-as).
 * theta ~180 = tool-z wijst recht omlaag, ~100-160 = schuin naar voren/omlaag.
 * De gripper sluit langs de lokale y-as van de tool: met yaw=0 sluit hij dus langs
 * wereld-y (dwars op een mes dat langs x ligt).
 */
export function toolQuat(yawDeg = 0, thetaDeg = 140, rollDeg = 0) {
  let q = qmul(qz(yawDeg), qy(thetaDeg));
  if (rollDeg) {
    const a = rollDeg * D2R / 2;
    q = qmul(q, [Math.cos(a), 0, 0, Math.sin(a)]);
  }
  return q;
}

export const lerp = (a, b, u) => a + (b - a) * u;
export const lerp3 = (a, b, u) => [lerp(a[0], b[0], u), lerp(a[1], b[1], u), lerp(a[2], b[2], u)];

export function slerp(a, b, u) {
  let d = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
  let bb = b;
  if (d < 0) { d = -d; bb = b.map(x => -x); }
  let s0, s1;
  if (d > 0.9995) { s0 = 1 - u; s1 = u; }
  else {
    const th = Math.acos(d), sn = Math.sin(th);
    s0 = Math.sin((1 - u) * th) / sn; s1 = Math.sin(u * th) / sn;
  }
  const q = [0, 1, 2, 3].map(i => s0 * a[i] + s1 * bb[i]);
  const n = Math.hypot(...q) || 1; return q.map(x => x / n);
}

export const EASING = {
  linear: u => u,
  smooth: u => u * u * (3 - 2 * u),                 // smoothstep (zachte start/stop)
  smoother: u => u * u * u * (u * (6 * u - 15) + 10),
};

/**
 * Zet botsingen uit voor de statische basis-meshes (mesh-geoms op de world-body).
 * Reden: base_link (world) en link1 overlappen ~6 mm in de scene; dat contact houdt
 * joint1 (draaiing van de arm om de basis) bijna vast (gemeten: joint1 blijft ~0
 * terwijl de actuator 0.6 rad vraagt). Zonder dit kunnen de armen niet zijwaarts zwenken.
 * Wordt alleen door de demo aangeroepen; teleop/scene.xml blijven ongewijzigd.
 * Geeft het aantal aangepaste geoms terug.
 */
export function relaxBaseContacts(model) {
  const MESH = 7; let n = 0;
  for (let g = 0; g < model.ngeom; g++) {
    if (model.geom_bodyid[g] === 0 && model.geom_type[g] === MESH) {
      model.geom_contype[g] = 0; model.geom_conaffinity[g] = 0; n++;
    }
  }
  return n;
}

// ---------------------------------------------------------------------------
// Standaardposities (uit assets/scene.xml). Wordt overschreven door
// readObjectPositions() als je het model hebt, of via opts.objects.
// ---------------------------------------------------------------------------
export const DEFAULT_OBJECTS = {
  board:  [0.18, 0.0, 0.003],
  bread0: [0.16, 0.06, 0.025],
  bread1: [0.16, -0.06, 0.025],
  knife:  [0.13, -0.22, 0.031],
  jar:    [0.23, 0.33, 0.0],
  plate:  [0.23, -0.32, 0.0],
};

/** Lees de posities van de benoemde bodies uit het draaiende MuJoCo-model. */
export function readObjectPositions(mujoco, model, data, names = Object.keys(DEFAULT_OBJECTS)) {
  const out = {};
  for (const n of names) {
    const id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, n);
    out[n] = id >= 0
      ? [data.xpos[3*id], data.xpos[3*id+1], data.xpos[3*id+2]]
      : DEFAULT_OBJECTS[n];
  }
  return out;
}

// ---------------------------------------------------------------------------
// Geometrie / tuning-constanten van de standaard choreografie
// ---------------------------------------------------------------------------
export const GEOM = {
  bread: { half: [0.045, 0.045, 0.013] },         // 9 x 9 x 2.6 cm
  knifeHandleX: -0.045,                            // handvat-midden t.o.v. mes-origine (x)
  breadTopZ: 0.038,                                // bovenkant bread0 (z) = 0.025 + 0.013
  // De TCP-site ligt ~3.4 cm VOORBIJ de vingertoppen (langs de tool-as); met een neerwaartse
  // tool zit de vingertop dus ~3 cm boven de TCP-z. Zie tools/run-sandwich-demo.mjs.
  tcpBeyondFingertip: 0.034,
};

export const TUNING = {
  // Grip-waarden 0=open .. 1=dicht (ArmIK.apply). Open = ~7 cm tussen de vingers.
  GRIP_OPEN: 0.0,
  GRIP_CLOSED: 1.0,
  GRIP_JAR_NARROW: 0.7,     // half dicht: gripper past dan (7 cm) in de pot (binnenmaat 9 cm)
  // Tool-oriëntatie per arm [yaw, theta, roll] in graden (zie toolQuat). Gevonden met een
  // haalbaarheidsscan (IK, 30 iteraties, fout < 2 mm / 0.03 rad) over ALLE waypoints:
  // rechts (160, 15) en links (100, 0) zijn overal haalbaar. Eén vaste oriëntatie per arm
  // houdt de beweging rustig en het mes horizontaal.
  ORI: { right: [15, 160, 0], rightBread: [15, 150, 0], left: [15, 100, 0] },
  // Mes pakken: TCP-positie t.o.v. de mes-origine (x, y) en hoogte (TCP-z).
  // hx=+0.04 (i.e. aan het voorste eind van het handvat) bleek in de simulatie het
  // betrouwbaarst (zie README, "Bekende beperkingen").
  KNIFE_GRASP_DX: -0.005,
  KNIFE_GRASP_Z: -0.04,
  JAR_DIP_Z: 0.03,          // TCP-z bij het scheppen in de pot
  PARK_KNIFE: [0.10, -0.20],// TCP-(x,y) waar het mes wacht terwijl de linkerarm beleg brengt
  SAFE_Z: 0.14,             // veilige transporthoogte van de TCP
  KNIFE_CARRY_Z: 0.10,      // TCP-hoogte waarop het mes wordt vervoerd
  SPREAD_Z: 0.01,           // TCP-hoogte tijdens smeren (blad zweeft net boven het brood)
  SPREAD_DX: -0.07,         // TCP-x t.o.v. bread0-midden (blad ligt ~7 cm VOOR de TCP)
  SPREAD_SWEEP: 0.025,      // halve slag in x tijdens het smeren
  SPREAD_ROWS: [-0.03, 0.0, 0.03],   // y-offsets t.o.v. bread0 van de banen
  GRIP_CLOSE_TIME: 2.0,     // langzaam sluiten (s)
  PLATE_TCP_DX: -0.06,      // TCP-x t.o.v. plate-midden bij het neerleggen van het mes
  PLATE_Z: 0.0,             // TCP-z bij het neerleggen van het mes
  BREAD1_PINCH_DEPTH: 0.03, // hoek-klem: afstand van de hoek langs de diagonaal (m)
  BREAD1_PINCH_Z: 0.0,
  BREAD_PLACE_Z: 0.085,     // TCP-z waarop bread1 boven bread0 wordt losgelaten
};

const Q = a => toolQuat(a[0], a[1], a[2]);

// ---------------------------------------------------------------------------
// De standaard choreografie
// ---------------------------------------------------------------------------
/**
 * Bouw de choreografie (vijf fases voor de vier hoofdstappen). `opts.objects` (bv. van
 * readObjectPositions) bepaalt waar alles ligt; `opts.tuning` overschrijft TUNING-waarden.
 * De stap-namen zijn de identifiers die onStep() ontvangt.
 */
export function makeSandwichChoreography(opts = {}) {
  const obj = { ...DEFAULT_OBJECTS, ...(opts.objects || {}) };
  const T = { ...TUNING, ...(opts.tuning || {}) };
  T.ORI = { ...TUNING.ORI, ...((opts.tuning || {}).ORI || {}) };
  const P = (x, y, z) => [x, y, z];
  const b0 = obj.bread0, b1 = obj.bread1, jar = obj.jar, plate = obj.plate;
  const qR = Q(T.ORI.right), qL = Q(T.ORI.left);
  const kx = obj.knife[0] + GEOM.knifeHandleX + 0.045 + T.KNIFE_GRASP_DX;   // handvat-voorkant
  const ky = obj.knife[1];
  const C = T.GRIP_CLOSE_TIME;

  // --- Stap 1: ingrediënten verzamelen ---------------------------------------
  // Rechterarm: boven het mes, zakken, langzaam sluiten, optillen.
  // Linkerarm (parallel): naar boven de pot (half gesloten zodat hij later in de pot past).
  const fetchKnife = [
    { pos: P(kx, ky, T.SAFE_Z), quat: qR, grip: T.GRIP_OPEN, t: 3.0 },
    { pos: P(kx, ky, T.KNIFE_GRASP_Z), t: 5.5 },
    { pos: P(kx, ky, T.KNIFE_GRASP_Z), t: 6.0 },
    { pos: P(kx, ky, T.KNIFE_GRASP_Z), grip: T.GRIP_CLOSED, t: 6.0 + C },
    { pos: P(kx, ky, T.KNIFE_GRASP_Z), t: 6.5 + C },
    { pos: P(kx, ky, T.KNIFE_CARRY_Z), t: 9.0 + C },
  ];
  const leftToJar = [
    { pos: P(jar[0], jar[1], T.SAFE_Z + 0.03), quat: qL, grip: T.GRIP_JAR_NARROW, t: 5.0 },
  ];

  // --- Stap 2: boter smeren met het mes over bread0 -------------------------
  // De TCP beweegt in een zigzag; het blad ligt ~7 cm vóór de TCP (x-richting) en zweeft
  // net boven het brood. Drie banen (y-offsets) heen en weer in x.
  const sx = b0[0] + T.SPREAD_DX, sw = T.SPREAD_SWEEP, sz = T.SPREAD_Z;
  const spread = [{ pos: P(sx - sw, b0[1] + T.SPREAD_ROWS[0], T.KNIFE_CARRY_Z), t: 3.0 },
                  { pos: P(sx - sw, b0[1] + T.SPREAD_ROWS[0], sz), t: 4.5 }];
  let t = 4.5, dir = 1;
  T.SPREAD_ROWS.forEach((dy, i) => {
    if (i > 0) { t += 0.8; spread.push({ pos: P(sx + dir * -sw, b0[1] + dy, sz), t }); }
    dir = -dir;
    t += 1.6; spread.push({ pos: P(sx + dir * -sw, b0[1] + dy, sz), t });
  });
  spread.push({ pos: P(sx, b0[1], T.KNIFE_CARRY_Z + 0.02), t: t + 1.5 });
  // Parkeren: mes zijwaarts uit de weg (boven de rechterkant van het bord), zodat de
  // linkerarm in stap 3 boven bread0 kan komen zonder het mes aan te raken.
  spread.push({ pos: P(T.PARK_KNIFE[0], T.PARK_KNIFE[1], T.KNIFE_CARRY_Z + 0.04), t: t + 4.0 });

  // --- Stap 3: beleg toevoegen (linkerarm: pot -> brood) --------------------
  // Er zit in de scene geen fysiek beleg in de pot: de arm "schept" (gripper half dicht in
  // de pot, dicht = scheppen), tilt op, brengt het boven bread0 en opent de gripper.
  const topping = [
    { pos: P(jar[0], jar[1], T.SAFE_Z + 0.03), quat: qL, grip: T.GRIP_JAR_NARROW, t: 0.5 },
    { pos: P(jar[0], jar[1], T.JAR_DIP_Z), t: 3.0 },
    { pos: P(jar[0], jar[1], T.JAR_DIP_Z), t: 3.4 },
    { pos: P(jar[0], jar[1], T.JAR_DIP_Z), grip: T.GRIP_CLOSED, t: 4.6 },
    { pos: P(jar[0], jar[1], T.SAFE_Z + 0.03), t: 6.6 },
    { pos: P(b0[0], b0[1], T.SAFE_Z), t: 9.4 },
    { pos: P(b0[0], b0[1], 0.09), t: 10.8 },
    { pos: P(b0[0], b0[1], 0.09), grip: T.GRIP_OPEN, t: 12.0 },
    { pos: P(b0[0], b0[1], T.SAFE_Z + 0.03), t: 13.4 },
  ];

  // --- Stap 4a: mes terug op het bord (plate), loslaten -----------------------
  const px = plate[0] + T.PLATE_TCP_DX, py = plate[1];
  const knifeToPlate = [
    { pos: P(px, py, T.KNIFE_CARRY_Z), t: 3.5 },
    { pos: P(px, py, T.PLATE_Z), t: 5.5 },
    { pos: P(px, py, T.PLATE_Z), t: 6.0 },
    { pos: P(px, py, T.PLATE_Z), grip: T.GRIP_OPEN, t: 6.0 + C },
    { pos: P(px, py, T.SAFE_Z), t: 8.0 + C },
  ];
  const leftHome = [{ pos: [0.07, 0.22, 0.36], t: 5.0 }];

  // --- Stap 4b: bread1 op bread0 leggen ------------------------------------
  // De gripper opent maar ~7 cm en het brood is 9 cm breed: platliggend brood kan alleen
  // aan een HOEK geklemd worden (wrijvingsklem over de diagonaal). Zie README.
  const h = GEOM.bread.half[0], d = T.BREAD1_PINCH_DEPTH;
  const cornerX = b1[0] + h, cornerY = b1[1] + h;         // hoek (+x,+y) richting bread0/midden
  const ux = -Math.SQRT1_2, uy = -Math.SQRT1_2;           // diagonaal de hoek in
  const gx = cornerX + ux * d + 0, gy = cornerY + uy * d;
  const qB = toolQuat(T.ORI.rightBread[0], T.ORI.rightBread[1], T.ORI.rightBread[2]);
  const dxp = gx - b1[0], dyp = gy - b1[1];               // greeppunt t.o.v. bread1-midden
  const zc = T.BREAD1_PINCH_Z;
  const closeSandwich = [
    { pos: P(gx, gy, T.SAFE_Z), quat: qB, grip: T.GRIP_OPEN, t: 3.5 },
    { pos: P(gx, gy, zc + 0.03), t: 5.5 },
    { pos: P(gx, gy, zc), t: 6.5 },
    { pos: P(gx, gy, zc), grip: T.GRIP_CLOSED, t: 6.5 + C },
    { pos: P(gx, gy, zc), t: 7.0 + C },
    { pos: P(gx, gy, T.SAFE_Z), t: 9.5 + C },
    { pos: P(b0[0] + dxp, b0[1] + dyp, T.SAFE_Z), t: 12.5 + C },
    { pos: P(b0[0] + dxp, b0[1] + dyp, T.BREAD_PLACE_Z), t: 14.0 + C },
    { pos: P(b0[0] + dxp, b0[1] + dyp, T.BREAD_PLACE_Z), grip: T.GRIP_OPEN, t: 15.0 + C },
    { pos: P(b0[0] + dxp, b0[1] + dyp, T.SAFE_Z + 0.03), t: 16.5 + C },
  ];

  return [
    { name: '1. Ingrediënten verzamelen', tracks: { right: fetchKnife, left: leftToJar } },
    { name: '2. Boter smeren',            tracks: { right: spread } },
    { name: '3. Beleg toevoegen',         tracks: { left: topping } },
    { name: '4a. Mes terug op het bord',  tracks: { right: knifeToPlate, left: leftHome } },
    { name: '4b. Broodje sluiten',        tracks: { right: closeSandwich } },
  ];
}

// ---------------------------------------------------------------------------
// MotionPlayer
// ---------------------------------------------------------------------------
function normalizeStep(step) {
  const tracks = step.tracks ? { ...step.tracks }
    : (step.arm ? { [step.arm]: step.waypoints } : {});
  let duration = 0;
  for (const wps of Object.values(tracks))
    for (const w of wps) duration = Math.max(duration, w.t);
  return { name: step.name, tracks, duration: step.duration ?? duration };
}

/**
 * Speelt een choreografie af op een set armen.
 *
 * `env` = { ik: {left, right}, qTarget: {left, right}, grip: {left, right},
 *           tcpPose(side) -> {pos, quat} }
 * (precies de velden van SandwichVR in src/app.js; qTarget en grip worden in
 * place bijgewerkt).
 *
 * opts: { ikIters=6, easing='smooth', onStep(name, index), onDone() }
 */
export class MotionPlayer {
  constructor(env, steps, opts = {}) {
    this.env = env;
    this.steps = steps.map(normalizeStep);
    this.ikIters = opts.ikIters ?? 6;
    this.easing = EASING[opts.easing || 'smooth'] || EASING.smooth;
    this.onStep = opts.onStep || null;
    this.onDone = opts.onDone || null;
    this.paused = false;
    this.speed = opts.speed ?? 1;
    this.reset();
  }

  reset() {
    this.index = -1; this.time = 0; this.done = false;
    this.cmd = null;      // laatst gecommandeerde {pos, quat, grip} per arm
    this._seg = {};       // per arm: huidige segmentstart
    this.stepName = '';
    this._advance();
  }

  pause() { this.paused = true; }
  resume() { this.paused = false; }
  get stepIndex() { return this.index; }

  _ensureCmd() {
    if (this.cmd) return;
    this.cmd = {};
    for (const s of Object.keys(this.env.ik)) {
      const p = this.env.tcpPose(s);
      this.cmd[s] = { pos: p.pos.slice(), quat: p.quat.slice(), grip: this.env.grip[s] ?? 0 };
    }
  }

  _advance() {
    this.index++; this.time = 0;
    if (this.index >= this.steps.length) {
      const was = this.done; this.done = true; this.stepName = '';
      if (!was && this.onDone) this.onDone();
      return;
    }
    const st = this.steps[this.index];
    this.stepName = st.name;
    this._ensureCmd();
    // startpunt van elk track = laatste gecommandeerde pose
    this._starts = {};
    for (const s of Object.keys(st.tracks)) {
      const c = this.cmd[s]; this._starts[s] = { pos: c.pos.slice(), quat: c.quat.slice(), grip: c.grip };
    }
    if (this.onStep) this.onStep(st.name, this.index);
  }

  /** Doelpose van één track op tijd t (interpolatie tussen waypoints). */
  _sample(start, wps, t) {
    let prev = { ...start, t: 0 };
    for (const w of wps) {
      if (t <= w.t || w === wps[wps.length - 1]) {
        const span = Math.max(1e-6, w.t - prev.t);
        const u = Math.min(1, Math.max(0, (t - prev.t) / span));
        const e = this.easing(u);
        const wq = w.quat || prev.quat, wg = w.grip ?? prev.grip;
        return { pos: lerp3(prev.pos, w.pos, e), quat: slerp(prev.quat, wq, e),
                 grip: lerp(prev.grip, wg, e) };
      }
      prev = { pos: w.pos, quat: w.quat || prev.quat, grip: w.grip ?? prev.grip, t: w.t };
    }
    return start;
  }

  /** Eén frame: tijd verder, doel-TCP bepalen, IK oplossen, ctrl schrijven. */
  update(dt) {
    const env = this.env;
    if (!this.paused && !this.done) {
      this.time += dt * this.speed;
      const st = this.steps[this.index];
      for (const s of Object.keys(st.tracks)) {
        const tgt = this._sample(this._starts[s], st.tracks[s], Math.min(this.time, st.duration));
        this.cmd[s] = tgt;
      }
      if (this.time >= st.duration) {
        // eindpose van deze stap vastleggen zodat de volgende stap daar begint
        for (const s of Object.keys(st.tracks)) {
          const wps = st.tracks[s], last = wps[wps.length - 1];
          this.cmd[s] = { pos: last.pos.slice(),
            quat: (last.quat || this._sample(this._starts[s], wps, 0).quat).slice(),
            grip: last.grip ?? this.cmd[s].grip };
          this.cmd[s] = this._sample(this._starts[s], wps, st.duration);
        }
        this._advance();
      }
    }
    if (!this.cmd) return;
    for (const s of Object.keys(env.ik)) {
      const c = this.cmd[s]; if (!c) continue;
      env.qTarget[s] = env.ik[s].solve(env.qTarget[s], c.pos, c.quat, this.ikIters);
      env.grip[s] = c.grip;
      env.ik[s].apply(env.qTarget[s], env.grip[s]);
    }
  }

  get totalDuration() { return this.steps.reduce((a, s) => a + s.duration, 0); }
}
