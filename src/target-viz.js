// Doel-ghost voor 6-DOF teleop (PR #12, verzoek Koen: "teken het doel, dan zie je dat het onmogelijk is als het doel afwijkt").
// Per arm, tijdens clutch: een semi-transparant gripper-silhouet + compact assenkruis op de DOELpose, en een lijntje van de
// echte TCP naar het doel. Kleur = status: groen (bereikt), oranje (afwijking), rood (onmogelijk: IK haalt het doel niet,
// IK tegen een jointlimiet, of de arm blijft > 0,4 s ver van het doel — bv. geblokkeerd door de tafel).
// Licht: MeshBasicMaterial/LineBasicMaterial (geen belichting), geometrie één keer gebouwd en gedeeld, 3 draw calls per arm.
//
// Query:  ?target=0 uit · ?target=always ook buiten clutch · ?tgtok=5,3 (mm,°: groen/oranje-grens) · ?tgtbad=20,10 (mm,°: rood)
//         ?tgthaptic=0 geen trilpuls bij overgang naar rood.

export const TARGET_DEFAULTS = { mode: 'clutch', okMm: 8, okDeg: 3, badMm: 20, badDeg: 10, hyst: 0.7, tau: 0.15, dwell: 0.12, sustain: 0.4, haptic: true };
export const STATUS_COLOR = { ok: 0x33dd66, warn: 0xffa020, bad: 0xff3030 };

const pair = (v, d) => { const a = String(v || '').split(',').map(parseFloat); return a.length === 2 && a.every(x => Number.isFinite(x) && x > 0) ? a : d; };

/** Opties uit de query-string (zie kop). */
export function parseTargetOpts(search) {
  const q = new URLSearchParams(search || ''), o = { ...TARGET_DEFAULTS };
  const t = (q.get('target') || '').toLowerCase();
  if (t === '0' || t === 'off' || t === 'false') o.mode = 'off'; else if (t === 'always') o.mode = 'always';
  [o.okMm, o.okDeg] = pair(q.get('tgtok'), [o.okMm, o.okDeg]);
  [o.badMm, o.badDeg] = pair(q.get('tgtbad'), [o.badMm, o.badDeg]);
  if (o.badMm < o.okMm) o.badMm = o.okMm; if (o.badDeg < o.okDeg) o.badDeg = o.okDeg;
  if (q.get('tgthaptic') === '0') o.haptic = false;
  return o;
}

/**
 * Statusclassificatie met smoothing (EMA, tijdconstante tau), hysterese (terug pas onder hyst × drempel) en een minimale
 * verblijftijd (een nieuwe status moet `dwell` s aanhouden), zodat het niet flikkert.
 * update({ posErr, rotErr, ikPos, ikRot, atLimit }, dt): posErr/rotErr = doel ↔ werkelijke TCP (m, rad); ikPos/ikRot = restfout van de
 * IK-oplossing (m, rad); atLimit = IK-oplossing staat tegen een jointlimiet. Geeft { status, changed, enteredBad, pos, rot, ik } (mm/°).
 */
export class TargetStatus {
  constructor(opts = {}) { this.o = { ...TARGET_DEFAULTS, ...opts }; this.reset(); }
  reset() { this.status = 'ok'; this.s = null; this.farT = 0; this.pend = null; this.pendT = 0; }
  update({ posErr = 0, rotErr = 0, ikPos = 0, ikRot = 0, atLimit = false } = {}, dt = 1 / 72) {
    const o = this.o, raw = [posErr * 1000, rotErr * 180 / Math.PI, ikPos * 1000, ikRot * 180 / Math.PI].map(x => (Number.isFinite(x) ? x : 1e9));
    if (!this.s) this.s = raw.slice();
    else { const a = 1 - Math.exp(-Math.max(0, dt) / o.tau); this.s = this.s.map((v, i) => v + a * (raw[i] - v)); }
    const [p, r, ip, ir] = this.s;
    const okScore = Math.max(p / o.okMm, r / o.okDeg, ip / o.okMm, ir / o.okDeg);       // ≥ 1 → afwijking
    const ikBad = Math.max(ip / o.badMm, ir / o.badDeg);                                   // ≥ 1 → IK haalt het doel niet
    const farScore = Math.max(p / o.badMm, r / o.badDeg);                                  // arm ver van het doel
    const ikOff = Math.max(ip / o.okMm, ir / o.okDeg);
    const prev = this.status, inBad = prev === 'bad', h = o.hyst;
    this.farT = farScore >= (inBad ? h : 1) ? this.farT + dt : 0;
    const bad = ikBad >= (inBad ? h : 1) || (atLimit && ikOff >= (inBad ? h : 1)) || this.farT >= o.sustain;
    let want;
    if (bad) want = 'bad';
    else if (okScore >= (prev === 'ok' ? 1 : h)) want = 'warn';
    else want = 'ok';
    if (want === prev) { this.pend = null; this.pendT = 0; }
    else if (want === this.pend) this.pendT += dt;
    else { this.pend = want; this.pendT = dt; }
    const st = this.pend && this.pendT >= o.dwell ? this.pend : prev;
    if (st !== prev) { this.pend = null; this.pendT = 0; }
    this.status = st;
    return { status: st, changed: st !== prev, enteredBad: st === 'bad' && prev !== 'bad', pos: p, rot: r, ik: [ip, ir] };
  }
}

// ---- three.js-deel (alleen in de browser) ---------------------------------------------------------------------------------------
// MuJoCo-lokaal (x,y,z) → three-lokaal (x, z, −y): dezelfde swizzle als scene-loader.js (bodies én mesh-vertices).
const swz = (x, y, z) => [x, z, -y];
function boxTris(cx, cy, cz, sx, sy, sz, out) {           // as-uitgelijnde box (MuJoCo-maten) als losse driehoeken, geswizzled
  const h = [sx / 2, sy / 2, sz / 2], v = [];
  for (const dx of [-1, 1]) for (const dy of [-1, 1]) for (const dz of [-1, 1]) v.push(swz(cx + dx * h[0], cy + dy * h[1], cz + dz * h[2]));
  const F = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]];
  for (const [a, b, c, d] of F) for (const i of [a, b, c, a, c, d]) out.push(...v[i]);
}
let SHARED = null;
function shared(THREE) {
  if (SHARED) return SHARED;
  // Gripper-silhouet in het TCP-frame (TCP-site: +z = aanvliegrichting, vingers schuiven langs ±y): twee vingers, handpalm, steel.
  const p = [];
  boxTris(0, 0.022, -0.022, 0.012, 0.008, 0.044, p); boxTris(0, -0.022, -0.022, 0.012, 0.008, 0.044, p);
  boxTris(0, 0, -0.05, 0.03, 0.06, 0.012, p); boxTris(0, 0, -0.078, 0.014, 0.014, 0.044, p);
  const grip = new THREE.BufferGeometry(); grip.setAttribute('position', new THREE.Float32BufferAttribute(p, 3));
  const L = 0.045, ax = [], col = [];
  for (const [d, c] of [[[L, 0, 0], [1, 0.2, 0.2]], [[0, L, 0], [0.2, 1, 0.2]], [[0, 0, L], [0.3, 0.5, 1]]]) { ax.push(0, 0, 0, ...swz(...d)); col.push(...c, ...c); }
  const axes = new THREE.BufferGeometry(); axes.setAttribute('position', new THREE.Float32BufferAttribute(ax, 3)); axes.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  const axesMat = new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.9, depthTest: false });
  SHARED = { grip, axes, axesMat };
  return SHARED;
}

/** Ghost per arm onder `parent` (mujocoRoot). update(side, {visible, pos, quat ([w,x,y,z] MuJoCo), tcpPos, status}). */
export function createTargetGhosts({ THREE, parent, sides = ['left', 'right'] }) {
  const S = shared(THREE), arms = {};
  for (const s of sides) {
    const mat = new THREE.MeshBasicMaterial({ color: STATUS_COLOR.ok, transparent: true, opacity: 0.45, depthWrite: false, side: THREE.DoubleSide });
    const lineMat = new THREE.LineBasicMaterial({ color: STATUS_COLOR.ok, transparent: true, opacity: 0.8, depthTest: false });
    const g = new THREE.Group(); g.name = `${s}_target_ghost`;
    const mesh = new THREE.Mesh(S.grip, mat), axes = new THREE.LineSegments(S.axes, S.axesMat);
    mesh.renderOrder = axes.renderOrder = 10; g.add(mesh, axes);
    const lg = new THREE.BufferGeometry(); lg.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(6), 3));
    const line = new THREE.Line(lg, lineMat); line.frustumCulled = false; line.renderOrder = 10;
    g.visible = line.visible = false; parent.add(g, line);
    arms[s] = { g, mat, lineMat, line, status: null };
  }
  return {
    arms,
    update(side, { visible, pos, quat, tcpPos, status = 'ok' }) {
      const a = arms[side]; if (!a) return;
      a.g.visible = a.line.visible = !!visible && !!pos;
      if (!a.g.visible) return;
      a.g.position.set(...swz(...pos));
      a.g.quaternion.set(-quat[1], -quat[3], quat[2], -quat[0]);              // = getQuaternion-swizzle in scene-loader.js
      if (status !== a.status) { a.status = status; a.mat.color.setHex(STATUS_COLOR[status]); a.lineMat.color.setHex(STATUS_COLOR[status]); }
      const arr = a.line.geometry.attributes.position.array, t = tcpPos || pos;
      arr.set([...swz(...t), ...swz(...pos)]); a.line.geometry.attributes.position.needsUpdate = true;
    },
    hideAll() { for (const s of sides) arms[s].g.visible = arms[s].line.visible = false; },
  };
}
