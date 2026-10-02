// AprilTag -> tafel-werkoppervlak: pure geometrie + schatting (geen three.js/DOM; in Node te testen: tools/test-tag-surface.mjs).
//
// Keten (alles in de WebXR-referentieruimte, y omhoog, -z vooruit; quaternions [x,y,z,w] zoals XRRigidTransform):
//   tag-detectie (beeldpixels, evt. pose in cameraframe)  --camera-intrinsics-->  straal/pose in het CAMERAFRAME (OpenCV: x rechts, y omlaag, z vooruit)
//   --camera-extrinsics (aanname!)-->  viewerframe  --XRViewerPose op het opnamemoment-->  XR-wereld  --> tafelframe -> rechthoek.
//
// AANNAMEN (staan ook in README en PR; geen ervan is op een echte Quest geverifieerd):
//  A1  Cameraplek t.o.v. de bril (extrinsics): Quest Browser geeft die niet betrouwbaar door → vaste schatting (DEFAULT_CAM_OFFSET, geen pitch),
//      overschrijfbaar met ?camoff=x,y,z en ?campitch=graden. Een fout van 3 cm geeft ~3 cm fout in de tagpositie.
//  A2  Tijdsynchronisatie: beeld ↔ XR-pose is niet gekoppeld. We bewaren de viewer-pose per XR-frame (PoseHistory) en nemen de pose op
//      `captureTime` van requestVideoFrameCallback (indien aanwezig), anders `nu − latencyMs` (standaard 60 ms, ?camlat=). Frames waarbij de
//      kop sneller beweegt dan MAX_LIN_SPEED/MAX_ANG_SPEED (of de pose-historie te ver van het beeldtijdstip ligt) worden weggegooid.
//  A3  Intrinsics: uit track-metadata als aanwezig, anders schatting uit hfov (niet gekalibreerd). Daarom heeft 'ray' (straal door het tagcentrum ∩
//      horizontaal tafelvlak; hangt niet van tagmaat/diepte-schatting af) de voorkeur zodra de tafelhoogte bekend is; anders 'pose' (t uit de detector).
//  A4  De tags liggen PLAT op de tafel (normaal ≈ omhoog); tags met > MAX_TILT_DEG kanteling worden genegeerd.
//  A5  "Linksonder" = dichtst bij de gebruiker + links, "rechtsboven" = verst weg + rechts, gezien vanuit de kijkrichting van de gebruiker tijdens het scannen.
//      Tafelassen volgen de tag-oriëntatie (gemiddeld mod 90°) en worden gesnapt op de richting die het dichtst bij de kijkrichting ligt.

export const DEFAULT_CAM_OFFSET = { left: [-0.05, 0.02, -0.05], right: [0.05, 0.02, -0.05], auto: [0, 0.02, -0.05] };   // m in viewerframe (x rechts, y omhoog, -z vooruit)
export const MAX_LIN_SPEED = 0.5;       // m/s kopsnelheid waarboven een frame wordt weggegooid
export const MAX_ANG_SPEED = 60;        // graden/s
export const MAX_TILT_DEG = 20;         // maximale kanteling van een tag t.o.v. een plat tafelvlak
export const AXES_CONSISTENCY = 0.85;   // gemiddelde resultante van 4·yaw: ≥ 0.85 ≈ tags binnen ~8° van elkaar (mod 90°)
export const MIN_SURFACE_SIDE = 0.15;   // m (zelfde ondergrens als de handmatige kalibratie)

// ---------- kleine vector/quaternion-hulpjes ([x,y,z,w]) ----------
export const v3 = { add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]], sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s], dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  len: a => Math.hypot(a[0], a[1], a[2]), norm: a => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; } };
export const qmul = (a, b) => [a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1], a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
                               a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3], a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2]];
export const qconj = q => [-q[0], -q[1], -q[2], q[3]];
export const qnorm = q => { const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / l, q[1] / l, q[2] / l, q[3] / l]; };
export const qrot = (q, v) => { const [x, y, z, w] = q, t = [2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])];
  return [v[0] + w * t[0] + (y * t[2] - z * t[1]), v[1] + w * t[1] + (z * t[0] - x * t[2]), v[2] + w * t[2] + (x * t[1] - y * t[0])]; };
export const qaxis = (ax, deg) => { const h = deg * Math.PI / 360, s = Math.sin(h), n = v3.norm(ax); return [n[0] * s, n[1] * s, n[2] * s, Math.cos(h)]; };
export function slerp(a, b, t) {
  let d = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]; b = d < 0 ? b.map(x => -x) : b; d = Math.abs(d);
  if (d > 0.9995) return qnorm(a.map((x, i) => x + t * (b[i] - x)));
  const th = Math.acos(Math.min(1, d)), s = Math.sin(th), w1 = Math.sin((1 - t) * th) / s, w2 = Math.sin(t * th) / s;
  return qnorm(a.map((x, i) => w1 * x + w2 * b[i]));
}
export const qangle = (a, b) => 2 * Math.acos(Math.min(1, Math.abs(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]))) * 180 / Math.PI;
export const median = a => { if (!a.length) return NaN; const s = [...a].sort((x, y) => x - y), m = s.length >> 1; return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2; };
const rad = d => d * Math.PI / 180;

// ---------- viewer-pose-historie (tijdsynchronisatie) ----------
export class PoseHistory {
  constructor(maxAgeMs = 3000) { this.maxAgeMs = maxAgeMs; this.s = []; }
  /** t in ms (zelfde tijdbasis als performance.now()/rAF), pos [x,y,z], quat [x,y,z,w] van de viewer in de referentieruimte. */
  push(t, pos, quat) {
    const last = this.s[this.s.length - 1];
    if (last && t <= last.t) return;                                  // alleen oplopende tijd
    this.s.push({ t, pos: [...pos], quat: qnorm(quat) });
    while (this.s.length > 2 && t - this.s[0].t > this.maxAgeMs) this.s.shift();
  }
  get length() { return this.s.length; }
  clear() { this.s.length = 0; }
  /** geïnterpoleerde pose op tijd t. gap = afstand (ms) tot het dichtstbijzijnde sample (groot = extrapolatie/gat). null als leeg. */
  at(t) {
    const s = this.s; if (!s.length) return null;
    if (t <= s[0].t) return { pos: s[0].pos, quat: s[0].quat, gap: s[0].t - t, extrapolated: t < s[0].t };
    const L = s[s.length - 1]; if (t >= L.t) return { pos: L.pos, quat: L.quat, gap: t - L.t, extrapolated: t > L.t };
    let lo = 0, hi = s.length - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (s[m].t <= t) lo = m; else hi = m; }
    const a = s[lo], b = s[hi], f = (t - a.t) / (b.t - a.t);
    return { pos: a.pos.map((x, i) => x + f * (b.pos[i] - x)), quat: slerp(a.quat, b.quat, f), gap: Math.min(t - a.t, b.t - t), extrapolated: false };
  }
  /** kopsnelheid rond t (venster ±w ms): { lin: m/s, ang: graden/s } of null bij te weinig data. */
  speed(t, w = 60) {
    const s = this.s; if (s.length < 2) return null;
    const t0 = Math.max(t - w, s[0].t), t1 = Math.min(t + w, s[s.length - 1].t);       // venster geklemd op de beschikbare historie
    if (t1 - t0 < 30) return null;
    const a = this.at(t0), b = this.at(t1), dt = (t1 - t0) / 1000;
    return { lin: v3.len(v3.sub(b.pos, a.pos)) / dt, ang: qangle(a.quat, b.quat) / dt };
  }
}

// ---------- camera -> wereld ----------
/** Camera-extrinsics (in viewerframe, camera-assen als WebXR: x rechts, y omhoog, -z vooruit). side: 'left'|'right'|'auto'. */
export function cameraExtrinsics({ side = 'auto', offset = null, pitchDeg = 0, yawDeg = 0 } = {}) {
  const pos = offset || DEFAULT_CAM_OFFSET[side] || DEFAULT_CAM_OFFSET.auto;
  let q = [0, 0, 0, 1];
  if (pitchDeg) q = qmul(qaxis([1, 0, 0], pitchDeg), q);              // + = omhoog kijken
  if (yawDeg) q = qmul(qaxis([0, 1, 0], yawDeg), q);
  return { pos: [...pos], quat: q, side };
}
/** 'camera 2 1, facing back' -> 'left', 'camera 2 2' -> 'right' (aanname uit Quest-labels, zie apriltag-camera.js pickCamera). */
export function sideFromLabel(label = '') {
  const m = String(label).toLowerCase().match(/camera\s*\d+\s+(\d+)/);
  if (m && /back|environment/.test(label.toLowerCase())) return m[1] === '1' ? 'left' : m[1] === '2' ? 'right' : 'auto';
  return 'auto';
}
const F = [1, -1, -1];                                                // OpenCV-camera -> WebXR-assen
const camToViewerVec = v => [v[0] * F[0], v[1] * F[1], v[2] * F[2]];
/** vector in cameraframe (OpenCV) -> wereldrichting (alleen rotatie) */
export function camDirToWorld(d, head, ext) { return qrot(head.quat, qrot(ext.quat, camToViewerVec(d))); }
/** punt in cameraframe (OpenCV, m) -> wereldpunt */
export function camPointToWorld(p, head, ext) { return v3.add(head.pos, qrot(head.quat, v3.add(ext.pos, qrot(ext.quat, camToViewerVec(p))))); }
export const cameraWorldOrigin = (head, ext) => v3.add(head.pos, qrot(head.quat, ext.pos));

/** wereldrichting -> cameraframe (inverse van camDirToWorld) */
export function worldDirToCam(d, head, ext) { return camToViewerVec(qrot(qconj(ext.quat), qrot(qconj(head.quat), d))); }
/** Wereld -> cameraframe (inverse), voor tests/synthetische beelden. */
export function worldPointToCam(p, head, ext) {
  const inv = q => qconj(q), pv = qrot(inv(head.quat), v3.sub(p, head.pos)), pc = qrot(inv(ext.quat), v3.sub(pv, ext.pos));
  return camToViewerVec(pc);                                          // F is zijn eigen inverse
}

/**
 * Eén tagdetectie -> sample in wereldcoördinaten.
 * det: {id, corners, center, pose?}; K: {fx,fy,cx,cy} (pixels van het detectiebeeld); head: {pos,quat}; ext: cameraExtrinsics()
 * opts: { method: 'auto'|'ray'|'pose', tableY (m) }
 * Geeft { ok, reason?, id, center:[x,y,z], method, tilt (graden of null), yaw (rad mod 90° ruwe hoek, of null), dist (m) }.
 */
export function tagSample(det, K, head, ext, { method = 'auto', tableY = null, maxTilt = MAX_TILT_DEG } = {}) {
  let tilt = null, yaw = null, tagN = null;
  if (det.pose && det.pose.R) {
    const R = det.pose.R, zc = [R[0][2], R[1][2], R[2][2]], xc = [R[0][0], R[1][0], R[2][0]];   // tag-assen in cameraframe (z = het vlak in)
    const zw = camDirToWorld(zc, head, ext), xw = camDirToWorld(xc, head, ext);
    tilt = Math.acos(Math.max(-1, Math.min(1, -zw[1]))) * 180 / Math.PI; tagN = zw;
    yaw = Math.atan2(xw[2], xw[0]);
    if (tilt > maxTilt) return { ok: false, reason: `tag ${det.id} te gekanteld (${tilt.toFixed(0)}° > ${maxTilt}°)`, id: det.id, tilt };
  }
  const useRay = method === 'ray' || (method === 'auto' && tableY != null && K);
  let center, used, dist;
  if (useRay) {
    if (!K || tableY == null) return { ok: false, reason: 'ray-methode vraagt intrinsics en tafelhoogte', id: det.id };
    const d = camDirToWorld([(det.center.x - K.cx) / K.fx, (det.center.y - K.cy) / K.fy, 1], head, ext), o = cameraWorldOrigin(head, ext);
    if (d[1] > -0.05) return { ok: false, reason: `tag ${det.id}: straal wijst niet naar beneden`, id: det.id };
    const s = (tableY - o[1]) / d[1]; if (!(s > 0)) return { ok: false, reason: `tag ${det.id}: tafelvlak boven de camera`, id: det.id };
    center = v3.add(o, v3.scale(d, s)); used = 'ray'; dist = s * v3.len(d);
  } else if (det.pose && det.pose.t) {
    center = camPointToWorld(det.pose.t, head, ext); used = 'pose'; dist = v3.len(det.pose.t);
  } else return { ok: false, reason: `tag ${det.id}: geen pose en geen tafelhoogte (tagmaat/intrinsics ontbreken)`, id: det.id };
  if (!center.every(Number.isFinite)) return { ok: false, reason: 'niet-eindige positie', id: det.id };
  return { ok: true, id: det.id, center, method: used, tilt, yaw, dist, normal: tagN };
}

/**
 * Zelfkalibratie van de brandpuntsafstand uit de bekende tagmaat + bekende tafelhoogte.
 * De detector geeft t_z = f'·maat/px (met de AANGENOMEN f'); de straal door het tagcentrum snijdt het tafelvlak op diepte s(k) (k = f_echt / f').
 * Oplossen van s(k) = k·t_z (vaste-puntiteratie; s hangt maar zwak van k af) geeft k. Geeft null als er geen pose/tafelhoogte is of de oplossing onzinnig is.
 * Aanname: de tag ligt plat op het vlak y = tableY en de tagmaat klopt; bij een fout in tableY of tagmaat vangt k die fout op (k absorbeert dan een fout van die grootte).
 */
export function estimateFocalScale(det, K, head, ext, tableY) {
  if (!det.pose || !det.pose.t || !K || tableY == null) return null;
  const tz = det.pose.t[2], o = cameraWorldOrigin(head, ext); let k = 1;
  if (!(tz > 0.1)) return null;
  for (let i = 0; i < 8; i++) {
    const d = camDirToWorld([(det.center.x - K.cx) / (K.fx * k), (det.center.y - K.cy) / (K.fy * k), 1], head, ext);
    if (d[1] > -0.05) return null;
    const sdepth = (tableY - o[1]) / d[1]; if (!(sdepth > 0)) return null;
    const kn = sdepth / tz; if (Math.abs(kn - k) < 1e-6) { k = kn; break; } k = kn;
  }
  return k > 0.5 && k < 2 ? k : null;
}

// ---------- ruisonderdrukking: per tag een venster van samples, mediaan + uitbijterfilter ----------
export class TagTracker {
  constructor({ windowMs = 8000, maxSamples = 60, gate = 0.04, minDist = 0.15, maxDist = 1.6, resetAfterRejects = 12 } = {}) {
    Object.assign(this, { windowMs, maxSamples, gate, minDist, maxDist, resetAfterRejects }); this.tags = new Map(); this.stats = { accepted: 0, rejected: 0 };
  }
  reset() { this.tags.clear(); this.stats = { accepted: 0, rejected: 0 }; }
  add(t, s) {
    if (!s.ok) { this.stats.rejected++; return false; }
    if (s.dist != null && (s.dist < this.minDist || s.dist > this.maxDist)) { this.stats.rejected++; return false; }
    let e = this.tags.get(s.id); if (!e) { e = { id: s.id, samples: [], rejects: 0, lastT: t }; this.tags.set(s.id, e); }
    while (e.samples.length && (t - e.samples[0].t > this.windowMs || e.samples.length >= this.maxSamples)) e.samples.shift();
    if (e.samples.length >= 5) {
      const m = [0, 1, 2].map(i => median(e.samples.map(x => x.c[i]))), d = Math.hypot(s.center[0] - m[0], s.center[1] - m[1], s.center[2] - m[2]);
      if (d > this.gate) {
        e.rejects++; this.stats.rejected++;
        if (e.rejects >= this.resetAfterRejects) { e.samples.length = 0; e.rejects = 0; } else return false;     // tag is echt verplaatst -> opnieuw beginnen
      } else e.rejects = 0;
    }
    e.samples.push({ t, c: s.center, yaw: s.yaw, method: s.method }); e.lastT = t; this.stats.accepted++; return true;
  }
  /** Geschatte tags: mediaan per as, standaardafwijking (m), aantal samples, cirkelgemiddelde van de yaw (mod 90°). */
  estimates(now = null) {
    const out = [];
    for (const e of this.tags.values()) {
      const S = now == null ? e.samples : e.samples.filter(x => now - x.t <= this.windowMs); if (!S.length) continue;
      const c = [0, 1, 2].map(i => median(S.map(x => x.c[i])));
      const var_ = S.reduce((a, x) => a + (x.c[0] - c[0]) ** 2 + (x.c[1] - c[1]) ** 2 + (x.c[2] - c[2]) ** 2, 0) / S.length;
      const ys = S.filter(x => x.yaw != null); let yaw = null;
      if (ys.length) { const sx = ys.reduce((a, x) => a + Math.cos(4 * x.yaw), 0), sz = ys.reduce((a, x) => a + Math.sin(4 * x.yaw), 0); yaw = Math.atan2(sz, sx) / 4; }
      out.push({ id: e.id, center: c, std: Math.sqrt(var_), n: S.length, yaw, method: S[S.length - 1].method, lastT: e.lastT });
    }
    return out.sort((a, b) => a.id - b.id);
  }
}

// ---------- tags -> rechthoek ----------
const ang = (x, z) => Math.atan2(z, x);
const angDiff = (a, b) => { let d = a - b; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI; return Math.abs(d); };

/**
 * Schat het werkoppervlak uit ≥ 2 tags.
 * tags: TagTracker.estimates(); opts: { viewerForward:[fx,fz] (horizontaal, standaard [0,-1]), tableY (m, anders mediaan tag-y), frame: {x:[..],z:[..],origin:[..]} (gekalibreerd
 *       tafelframe: assen worden daaruit genomen i.p.v. uit de tags), tagSize (m), edge: 'center'|'outer'|'inner', minSide }
 * Resultaat: { ok, reason?, ... } met frame {origin (verste-linkerhoek, y=tafelhoogte), x (rechts), y (omhoog), z (naar de gebruiker toe), matrix (kolom-groot 4x4, three-compatibel)},
 *   width (langs x), depth (langs z), corners [FL, FR, NR, NL] in wereldcoördinaten, ll/ur (gekozen tags), tableY, axes.
 */
export function estimateSurface(tags, opts = {}) {
  const { viewerForward = [0, -1], tagSize = 0.08255, edge = 'center', minSide = MIN_SURFACE_SIDE } = opts;
  if (!tags || tags.length < 2) return { ok: false, reason: `minstens 2 tags nodig (nu ${tags ? tags.length : 0})`, nTags: tags ? tags.length : 0 };
  const tableY = opts.tableY ?? median(tags.map(t => t.center[1]));
  // assen
  const fl = Math.hypot(viewerForward[0], viewerForward[1]) || 1, fx = viewerForward[0] / fl, fz = viewerForward[1] / fl, rightAng = ang(-fz, fx);
  // theta = oriëntatie mod 90°: uit de gemiddelde tag-yaw (tags liggen langs de tafelranden), anders uit het gekalibreerde tafelframe, anders de kijkrichting; snap daarna op de as die het dichtst bij "rechts van de gebruiker" ligt
  const ys = tags.filter(t => t.yaw != null);
  let theta = rightAng, axesSource = 'view';
  if (ys.length >= 2) {                                                // tags onderling (mod 90°) consistent georiënteerd? dan volgen de assen de tags
    const sx = ys.reduce((a, t) => a + Math.cos(4 * t.yaw), 0) / ys.length, sz = ys.reduce((a, t) => a + Math.sin(4 * t.yaw), 0) / ys.length;
    if (Math.hypot(sx, sz) >= AXES_CONSISTENCY) { theta = Math.atan2(sz, sx) / 4; axesSource = 'tags'; }
  }
  if (axesSource === 'view' && opts.frame) { theta = ang(opts.frame.x[0], opts.frame.x[2]); axesSource = 'frame'; }
  let phi = theta, bd = Infinity; for (let k = 0; k < 4; k++) { const c = theta + k * Math.PI / 2, d = angDiff(c, rightAng); if (d < bd) { bd = d; phi = c; } }
  const u = [Math.cos(phi), 0, Math.sin(phi)], vAway = [u[2], 0, -u[0]];
  const P = tags.map(t => ({ ...t, u: v3.dot(t.center, u), v: v3.dot(t.center, vAway) }));
  let ll = P[0], ur = P[0];
  for (const p of P) { if (p.u + p.v < ll.u + ll.v) ll = p; if (p.u + p.v > ur.u + ur.v) ur = p; }
  if (ll.id === ur.id) return { ok: false, reason: 'linksonder- en rechtsboven-tag zijn dezelfde tag', nTags: P.length };
  const h = tagSize / 2, grow = edge === 'outer' ? h : edge === 'inner' ? -h : 0;
  const umin = Math.min(ll.u, ur.u) - grow, umax = Math.max(ll.u, ur.u) + grow, vmin = Math.min(ll.v, ur.v) - grow, vmax = Math.max(ll.v, ur.v) + grow;
  const width = umax - umin, depth = vmax - vmin;
  if (width < minSide || depth < minSide)
    return { ok: false, reason: `oppervlak te klein of tags op één lijn (${(width * 100).toFixed(0)} × ${(depth * 100).toFixed(0)} cm; minimaal ${(minSide * 100).toFixed(0)} cm): leg één tag linksonder en één rechtsboven`, nTags: P.length, width, depth };
  const at = (uu, vv) => [u[0] * uu + vAway[0] * vv, tableY, u[2] * uu + vAway[2] * vv];
  const x = u, y = [0, 1, 0], z = [-vAway[0], 0, -vAway[2]];           // z = naar de gebruiker toe; x × y = z? (zie test)
  const origin = at(umin, vmax);                                       // verste linkerhoek
  const matrix = [x[0], x[1], x[2], 0, y[0], y[1], y[2], 0, z[0], z[1], z[2], 0, origin[0], origin[1], origin[2], 1];
  const det = v3.dot(v3.cross(x, y), z);
  return { ok: true, nTags: P.length, tableY, width, depth, frame: { origin, x, y, z, matrix, det }, axes: { u, vAway, source: axesSource },
           corners: [at(umin, vmax), at(umax, vmax), at(umax, vmin), at(umin, vmin)], ll: { id: ll.id, center: ll.center }, ur: { id: ur.id, center: ur.center }, edge };
}

// ---------- scanner (tracker + historie + stabiliteit + time-out) ----------
export class TagTableScanner {
  constructor({ history, extrinsicsFor = () => cameraExtrinsics(), tracker = new TagTracker(), getTableY = () => null, getFrame = () => null,
                tagSize = 0.08255, edge = 'center', method = 'auto', latencyMs = 60, minSamples = 8, maxStd = 0.012, stableMs = 1000, stableTol = 0.01,
                timeoutMs = 25000, maxGapMs = 150, selfCalibrate = true, onUpdate = () => {}, onStable = () => {}, onFallback = () => {}, now = () => performance.now() } = {}) {
    Object.assign(this, { selfCalibrate, history, extrinsicsFor, tracker, getTableY, getFrame, tagSize, edge, method, latencyMs, minSamples, maxStd, stableMs, stableTol, timeoutMs, maxGapMs, onUpdate, onStable, onFallback, now });
    this.phase = 'idle'; this.focal = { ks: [], k: 1, used: 1 }; this.est = null; this.t0 = 0; this.trail = []; this.fwd = []; this.last = null;
    this.counters = { frames: 0, dets: 0, used: 0, dropGap: 0, dropMotion: 0, dropTilt: 0, dropOther: 0, dropNoHistory: 0 };
    this.lastReason = '';
  }
  start() { this.tracker.reset(); this.trail = []; this.fwd = []; this.est = null; this.focal = { ks: [], k: 1, used: 1 }; this.phase = 'scanning'; this.t0 = this.now(); this.lastReason = 'scannen…';
            Object.keys(this.counters).forEach(k => { this.counters[k] = 0; }); }
  stop() { this.phase = 'idle'; }
  markApplied() { this.phase = 'applied'; }
  get active() { return this.phase === 'scanning'; }

  /** dets: AprilTagDetector.detect()-uitvoer; meta: {t (capture, ms), w,h, intrinsics:{fx,fy,cx,cy}, device (label)}. */
  feed(dets, meta) {
    if (this.phase !== 'scanning') return;
    this.counters.frames++; this.counters.dets += dets.length;
    if (!dets.length) return;
    const t = meta.t ?? (this.now() - this.latencyMs), head = this.history.at(t);
    if (!head) { this.counters.dropNoHistory++; return; }
    if (head.gap > this.maxGapMs) { this.counters.dropGap++; return; }
    const sp = this.history.speed(t);
    if (sp && (sp.lin > MAX_LIN_SPEED || sp.ang > MAX_ANG_SPEED)) { this.counters.dropMotion++; return; }
    const K0 = meta.intrinsics, ext = this.extrinsicsFor(meta.device || '');
    const tableY = this.getTableY();
    if (this.selfCalibrate && K0 && tableY != null) {                   // brandpuntsafstand uit tagmaat + tafelhoogte (zie estimateFocalScale)
      for (const d of dets) { const k = estimateFocalScale(d, K0, head, ext, tableY); if (k != null) this.focal.ks.push(k); }
      if (this.focal.ks.length > 80) this.focal.ks.splice(0, this.focal.ks.length - 80);
      if (this.focal.ks.length >= 6) {
        this.focal.k = median(this.focal.ks);
        if (Math.abs(this.focal.k - this.focal.used) > 0.01) { this.focal.used = this.focal.k; this.tracker.reset(); }   // nieuwe f → eerdere posities waren met de oude f berekend
      }
    }
    const kf = this.selfCalibrate ? this.focal.used : 1, K = K0 && { ...K0, fx: K0.fx * kf, fy: K0.fy * kf };
    for (const d of dets) {
      const s = tagSample(d, K, head, ext, { method: this.method, tableY });
      if (!s.ok) { if (/gekanteld/.test(s.reason)) this.counters.dropTilt++; else this.counters.dropOther++; this.lastReason = s.reason; continue; }
      if (this.tracker.add(t, s)) this.counters.used++;
    }
    const f = qrot(head.quat, [0, 0, -1]), l = Math.hypot(f[0], f[2]); if (l > 0.3) { this.fwd.push([f[0] / l, f[2] / l]); if (this.fwd.length > 60) this.fwd.shift(); }
  }

  viewerForward() {
    if (!this.fwd.length) return [0, -1];
    const sx = this.fwd.reduce((a, f) => a + f[0], 0), sz = this.fwd.reduce((a, f) => a + f[1], 0), l = Math.hypot(sx, sz) || 1; return [sx / l, sz / l];
  }

  /** Aanroepen op ~5–10 Hz: schat, beoordeelt stabiliteit, time-out. Geeft de huidige toestand terug. */
  tick() {
    const now = this.now();
    if (this.phase !== 'scanning') return this.snapshot();
    const tags = this.tracker.estimates(now);
    const frame = this.getFrame();
    this.est = estimateSurface(tags, { viewerForward: this.viewerForward(), tableY: this.getTableY() ?? undefined, frame: frame || undefined, tagSize: this.tagSize, edge: this.edge });
    let stable = false;
    if (this.est.ok) {
      const q = [this.est.ll, this.est.ur].map(x => tags.find(t => t.id === x.id));
      const enough = q.every(t => t.n >= this.minSamples && t.std <= this.maxStd);
      this.trail.push({ t: now, w: this.est.width, d: this.est.depth, o: this.est.frame.origin, ok: enough });
      while (this.trail.length && now - this.trail[0].t > this.stableMs + 500) this.trail.shift();
      const win = this.trail.filter(x => now - x.t <= this.stableMs);
      if (enough && win.length >= 3 && now - this.trail[0].t >= this.stableMs - 50 && win.every(x => x.ok)) {
        const rng = f => Math.max(...win.map(f)) - Math.min(...win.map(f));
        stable = rng(x => x.w) < this.stableTol && rng(x => x.d) < this.stableTol && rng(x => x.o[0]) < this.stableTol && rng(x => x.o[2]) < this.stableTol;
      }
      this.lastReason = stable ? 'stabiel' : (enough ? 'wachten op stabiele meting…' : `meer samples nodig (≥${this.minSamples}, σ ≤ ${(this.maxStd * 1000).toFixed(0)} mm)`);
    } else this.lastReason = this.est.reason;
    this.onUpdate(this.snapshot(tags));
    if (stable) { this.phase = 'stable'; this.onStable(this.est); }
    else if (now - this.t0 > this.timeoutMs) {
      this.phase = 'failed';
      this.onFallback(`Geen stabiel oppervlak uit AprilTags binnen ${(this.timeoutMs / 1000).toFixed(0)} s (${this.lastReason}). Terugval: handmatige 3-punts kalibratie.`);
    }
    return this.snapshot(tags);
  }

  snapshot(tags = null) {
    return { phase: this.phase, reason: this.lastReason, est: this.est, tags: tags || this.tracker.estimates(this.now()), counters: { ...this.counters },
             tracker: { ...this.tracker.stats }, focalScale: this.focal.used, focalSamples: this.focal.ks.length, elapsedMs: this.phase === 'idle' ? 0 : this.now() - this.t0, forward: this.viewerForward() };
  }
}
