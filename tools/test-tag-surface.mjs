#!/usr/bin/env node
// Headless tests van de AprilTag-werkoppervlak-geometrie (src/tag-surface.js): rechthoek uit twee tags, spiegeling/rotatie, ruis,
// tijdsynchronisatie (PoseHistory), degeneraties, scanner-toestandsmachine en een end-to-end met de WASM-detector op synthetische beelden.
//   node tools/test-tag-surface.mjs
import assert from 'node:assert/strict';
import { createAprilTagDetector, DEFAULT_TAG_SIZE_M } from '../src/apriltag-detector.js';
import * as S from '../src/tag-surface.js';
import { formatTagDebug } from '../src/tag-debug.js';
import { renderScene } from './apriltag-synth.mjs';

const { v3, qaxis, qmul, qrot, PoseHistory, TagTracker, TagTableScanner, estimateSurface, tagSample, cameraExtrinsics, worldPointToCam, worldDirToCam, camPointToWorld } = S;
let n = 0; const ok = async (name, f) => { await f(); n++; console.log('ok', name); };
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg || ''} ${a} ≉ ${b} (±${tol})`);
const nearV = (a, b, tol, msg) => a.forEach((x, i) => near(x, b[i], tol, `${msg || ''}[${i}]`));
const deg = Math.PI / 180;
function rng(seed) { let s = seed >>> 0; return () => ((s = (s * 1664525 + 1013904223) >>> 0) / 4294967296); }
const gauss = r => Math.sqrt(-2 * Math.log(r() || 1e-12)) * Math.cos(2 * Math.PI * r());

const TABLE_Y = 0.75, TAG = DEFAULT_TAG_SIZE_M;
const cam = { w: 960, h: 720, fx: 700, fy: 700, cx: 480, cy: 360 };
const ext = cameraExtrinsics({ side: 'left' });
// kijk-pose: staat op (x, 1.3, z0), kijkt naar -z (yaw 0) en 50° omlaag; yawDeg draait om de verticale as
const headPose = (pos, yawDeg = 0, pitchDeg = -50) => ({ pos, quat: qmul(qaxis([0, 1, 0], yawDeg), qaxis([1, 0, 0], pitchDeg)) });
const fwdOf = h => { const f = qrot(h.quat, [0, 0, -1]), l = Math.hypot(f[0], f[2]); return [f[0] / l, f[2] / l]; };
// wereldtags (plat op de tafel): tag-x = (cosψ,0,sinψ), tag-z (het vlak in) = omlaag
const tagAxesWorld = psi => ({ x: [Math.cos(psi), 0, Math.sin(psi)], z: [0, -1, 0], y: [-Math.sin(psi), 0, Math.cos(psi)] });
function analyticDet(tag, head, e, K) {                      // exacte detectie (zonder beeld): centrum + pose
  const c = worldPointToCam(tag.c, head, e), ax = tagAxesWorld(tag.psi ?? 0);
  const X = worldDirToCam(ax.x, head, e), Y = worldDirToCam(ax.y, head, e), Z = worldDirToCam(tag.zdir || ax.z, head, e);
  return { id: tag.id, center: { x: K.fx * c[0] / c[2] + K.cx, y: K.fy * c[1] / c[2] + K.cy }, pose: { R: [[X[0], Y[0], Z[0]], [X[1], Y[1], Z[1]], [X[2], Y[2], Z[2]]], t: c } };
}
const K0 = { fx: cam.fx, fy: cam.fy, cx: cam.cx, cy: cam.cy, source: 'test' };
const LL = { id: 7, c: [-0.25, TABLE_Y, -0.45], psi: 0 }, UR = { id: 42, c: [0.25, TABLE_Y, -0.85], psi: 0 };
const H0 = headPose([0, 1.3, 0.1]);

await ok('quaternion/camera-keten: wereld → camera → wereld is exact (< 1e-9), ook met pitch/offset', () => {
  const e2 = cameraExtrinsics({ side: 'right', pitchDeg: 7, offset: [0.04, 0.03, -0.06] });
  for (const h of [H0, headPose([0.3, 1.1, -0.2], 33, -20), headPose([0, 1.6, 0], -170, 10)]) {
    const p = [0.4, 0.7, -0.9], c = worldPointToCam(p, h, e2); nearV(camPointToWorld(c, h, e2), p, 1e-9, 'roundtrip');
  }
  const c = worldPointToCam(S.cameraWorldOrigin(H0, ext), H0, ext); nearV(c, [0, 0, 0], 1e-9, 'camera-oorsprong');
  const d = S.camDirToWorld([0, 0, 1], H0, ext); const f = qrot(H0.quat, [0, 0, -1]); nearV(d, f, 1e-9, 'camera-z = kijkrichting');
});

await ok('PoseHistory: interpolatie (lerp/slerp), klem, gap en snelheid', () => {
  const h = new PoseHistory();
  for (let i = 0; i <= 10; i++) h.push(1000 + i * 10, [i * 0.001, 1.5, 0], qaxis([0, 1, 0], i));   // 0,1 m/s en 100°/s
  let a = h.at(1055); nearV(a.pos, [0.0055, 1.5, 0], 1e-9); near(S.qangle(a.quat, qaxis([0, 1, 0], 5.5)), 0, 1e-3, 'slerp');
  assert.ok(a.gap <= 5.0001 && !a.extrapolated);
  a = h.at(1200); assert.ok(a.extrapolated && Math.abs(a.gap - 100) < 1e-9); nearV(a.pos, [0.01, 1.5, 0], 1e-12);
  a = h.at(900); assert.ok(a.extrapolated && a.gap === 100);
  const sp = h.speed(1050, 20); near(sp.lin, 0.1, 1e-6); near(sp.ang, 100, 1e-6);
  assert.equal(h.speed(1005, 10), null, 'te weinig historie (< 30 ms venster)'); near(h.speed(1100, 60).lin, 0.1, 1e-6, 'aan de rand geklemd'); h.push(1005, [0, 0, 0], [0, 0, 0, 1]); assert.equal(h.length, 11, 'niet-oplopende tijd genegeerd');
  const h2 = new PoseHistory(100); for (let t = 0; t < 1000; t += 10) h2.push(t, [0, 0, 0], [0, 0, 0, 1]); assert.ok(h2.length <= 12, 'oude samples worden opgeruimd');
});

await ok('tagSample/ray: exact tagcentrum (< 0,1 mm) uit pixelstraal + tafelhoogte; pose-methode idem; tilt ≈ 0', () => {
  for (const h of [H0, headPose([0.2, 1.2, 0.3], 25, -45), headPose([-0.3, 1.4, 0], -15, -60)]) {
    for (const t of [LL, UR]) {
      const d = analyticDet(t, h, ext, K0);
      const r = tagSample(d, K0, h, ext, { method: 'ray', tableY: TABLE_Y }), p = tagSample(d, K0, h, ext, { method: 'pose' });
      assert.ok(r.ok && p.ok); nearV(r.center, t.c, 1e-4, 'ray'); nearV(p.center, t.c, 1e-4, 'pose'); near(r.tilt, 0, 1e-3, 'tilt'); near(Math.cos(r.yaw), 1, 1e-6, 'yaw');
    }
  }
  const auto = tagSample(analyticDet(LL, H0, ext, K0), K0, H0, ext, { tableY: TABLE_Y }); assert.equal(auto.method, 'ray');
  assert.equal(tagSample(analyticDet(LL, H0, ext, K0), K0, H0, ext, {}).method, 'pose', 'zonder tafelhoogte → pose');
});

await ok('tagSample: foute afstand/tagmaat → pose-methode schuift, ray-methode niet; verkeerde hfov geeft beperkte ray-fout', () => {
  const d = analyticDet(LL, H0, ext, K0); d.pose.t = d.pose.t.map(x => x * 1.8);            // tagmaat 1,8× te groot aangenomen
  const p = tagSample(d, K0, H0, ext, { method: 'pose' }), r = tagSample(d, K0, H0, ext, { method: 'ray', tableY: TABLE_Y });
  assert.ok(v3.len(v3.sub(p.center, LL.c)) > 0.2, 'pose-methode is gevoelig voor tagmaat'); nearV(r.center, LL.c, 1e-4, 'ray is dat niet');
  // intrinsics 10% te klein/groot: ray-fout is een paar cm, geen meters
  const Kbad = { ...K0, fx: K0.fx * 1.1, fy: K0.fy * 1.1 }; const rb = tagSample(analyticDet(LL, H0, ext, K0), Kbad, H0, ext, { method: 'ray', tableY: TABLE_Y });
  const err = v3.len(v3.sub(rb.center, LL.c)); console.log(`   ray-fout bij 10% foute brandpuntsafstand: ${(err * 100).toFixed(1)} cm`); assert.ok(err < 0.08);
});

await ok('brandpuntszelfkalibratie: uit tagmaat + tafelhoogte volgt k = f_echt/f_aangenomen binnen 0,5% (f 12% te groot én 12% te klein aangenomen), ook uit het midden van het beeld', () => {
  for (const [fac, head] of [[1.12, H0], [1 / 1.12, H0], [1.0, headPose([0.2, 1.2, 0.3], 25, -45)], [1.12, headPose([-0.3, 1.4, 0], -15, -60)]]) {
    const Kw = { ...K0, fx: K0.fx * fac, fy: K0.fy * fac };
    for (const t of [LL, UR]) {
      const d = analyticDet(t, head, ext, K0); d.pose.t = [d.pose.t[0], d.pose.t[1], d.pose.t[2] * fac];   // detector die met f' = fac·f rekent: t_z schaalt mee
      const k = S.estimateFocalScale(d, Kw, head, ext, TABLE_Y); near(k, 1 / fac, 0.005 / fac, `k voor fac ${fac}`);
    }
  }
  assert.equal(S.estimateFocalScale({ center: { x: 1, y: 1 } }, K0, H0, ext, TABLE_Y), null, 'zonder pose geen schatting');
});

await ok('tagSample: gekantelde tag (40°) en tag-richting omhoog/straal niet naar beneden worden geweigerd met reden', () => {
  const d = analyticDet({ ...LL, zdir: v3.norm([Math.sin(40 * deg), -Math.cos(40 * deg), 0]) }, H0, ext, K0);
  const r = tagSample(d, K0, H0, ext, { tableY: TABLE_Y }); assert.ok(!r.ok && /gekanteld/.test(r.reason), r.reason); near(r.tilt, 40, 0.01);
  const up = headPose([0, 1.3, 0], 0, 30); const d2 = analyticDet(LL, headPose([0, 1.3, 0.1]), ext, K0);
  const r2 = tagSample({ ...d2, center: { x: 480, y: 100 } }, K0, up, ext, { method: 'ray', tableY: TABLE_Y }); assert.ok(!r2.ok);
});

// --- rechthoek ---
const trk = (tags, head = H0, e = ext) => { const t = new TagTracker(); for (let i = 0; i < 12; i++) for (const g of tags) t.add(i * 30, tagSample(analyticDet(g, head, e, K0), K0, head, e, { tableY: TABLE_Y })); return t.estimates(); };

await ok('rechthoek uit twee tags: breedte/diepte/hoeken/frame exact; LL=#7, UR=#42; frame rechtshandig (det +1)', () => {
  const s = estimateSurface(trk([LL, UR]), { viewerForward: fwdOf(H0), tableY: TABLE_Y });
  assert.ok(s.ok, s.reason); assert.equal(s.ll.id, 7); assert.equal(s.ur.id, 42); assert.equal(s.axes.source, 'tags');
  near(s.width, 0.5, 1e-3); near(s.depth, 0.4, 1e-3); near(s.frame.det, 1, 1e-12);
  nearV(s.frame.origin, [-0.25, TABLE_Y, -0.85], 1e-3, 'verste-linkerhoek'); nearV(s.frame.x, [1, 0, 0], 1e-9); nearV(s.frame.z, [0, 0, 1], 1e-9, 'z naar de gebruiker');
  nearV(s.corners[0], [-0.25, TABLE_Y, -0.85], 1e-3); nearV(s.corners[2], [0.25, TABLE_Y, -0.45], 1e-3);
  const o = estimateSurface(trk([LL, UR]), { viewerForward: fwdOf(H0), tableY: TABLE_Y, edge: 'outer' }); near(o.width, 0.5 + TAG, 1e-3); near(o.depth, 0.4 + TAG, 1e-3);
  const i = estimateSurface(trk([LL, UR]), { viewerForward: fwdOf(H0), tableY: TABLE_Y, edge: 'inner' }); near(i.width, 0.5 - TAG, 1e-3);
  // het frame past op de bestaande kalibratie-conventie: punt (w/2, 0, d/2) in framecoördinaten = midden van de tafel
  const m = s.frame.matrix, mid = [m[0] * s.width / 2 + m[8] * s.depth / 2 + m[12], m[1] * s.width / 2 + m[9] * s.depth / 2 + m[13], m[2] * s.width / 2 + m[10] * s.depth / 2 + m[14]];
  nearV(mid, [0, TABLE_Y, -0.65], 2e-3, 'midden');
});

await ok('spiegeling/rotatie: kijkrichting 0/90/180/270° geeft consistente LL/UR (gezien vanuit de gebruiker), zelfde rechthoek, det +1; id-volgorde maakt niets uit', () => {
  const corners = [LL, UR];
  const ref = estimateSurface(trk(corners), { viewerForward: fwdOf(H0), tableY: TABLE_Y });
  const roles = {};
  for (const yaw of [0, 90, 180, 270, -90, 45]) {
    const off = qrot(qaxis([0, 1, 0], yaw), [0, 0, 0.75]), h = headPose([off[0], 1.3, -0.65 + off[2]], yaw), tg = [LL, UR];   // gebruiker loopt om de tafel heen en kijkt naar het tafelmidden
    // tags liggen vast in de wereld; de gebruiker draait → LL/UR t.o.v. de gebruiker wisselt
    const s = estimateSurface(trk(tg, h), { viewerForward: fwdOf(h), tableY: TABLE_Y });
    assert.ok(s.ok, `yaw ${yaw}: ${s.reason}`); near(s.frame.det, 1, 1e-12);
    const [fx, fz] = fwdOf(h), right = [-fz, fx], toward = [-fx, -fz];
    // gecontroleerd in viewer-termen (alleen bij 0°/180°: dan ligt het paar echt diagonaal LL/UR t.o.v. de gebruiker; bij ±90° ligt het op de andere diagonaal → rechthoek hetzelfde, rolnamen wisselen)
    const dl = [s.ur.center[0] - s.ll.center[0], s.ur.center[2] - s.ll.center[2]];
    if (yaw % 180 === 0) { assert.ok(dl[0] * right[0] + dl[1] * right[1] > 0.1, `yaw ${yaw}: UR rechts van LL`); assert.ok(-(dl[0] * toward[0] + dl[1] * toward[1]) > 0.1, `yaw ${yaw}: UR verder weg`); }
    // frame.x wijst naar rechts van de gebruiker (±45° snap), frame.z naar de gebruiker toe
    assert.ok(s.frame.x[0] * right[0] + s.frame.x[2] * right[1] > 0.3, `yaw ${yaw}: x rechts`); assert.ok(s.frame.z[0] * toward[0] + s.frame.z[2] * toward[1] > 0.3, `yaw ${yaw}: z naar gebruiker`);
    roles[yaw] = `${s.ll.id}→${s.ur.id}`;
    // zelfde fysieke tafeloppervlak (bounding box van hoeken, 90°-symmetrisch)
    const bb = c => [Math.min(...c.map(p => p[0])), Math.max(...c.map(p => p[0])), Math.min(...c.map(p => p[2])), Math.max(...c.map(p => p[2]))];
    if (yaw % 90 === 0) nearV(bb(s.corners), bb(ref.corners), 2e-3, `yaw ${yaw} bbox`); near(s.width * s.depth, 0.5 * 0.4, 2e-3, 'oppervlak');
  }
  console.log('   rollen per kijkrichting (LL→UR):', JSON.stringify(roles));
  assert.equal(roles[0], '7→42'); assert.equal(roles[180], '42→7');
  const sw = estimateSurface(trk([UR, LL]), { viewerForward: fwdOf(H0), tableY: TABLE_Y }); assert.equal(sw.ll.id, 7); near(sw.width, 0.5, 1e-3);
  // meerdere tags: de meest linksonder / rechtsboven wint (tag in het midden doet niet mee)
  const mid = { id: 100, c: [0, TABLE_Y, -0.65], psi: 0 }, far2 = { id: 1, c: [-0.1, TABLE_Y, -0.8], psi: 0 };
  const s3 = estimateSurface(trk([mid, LL, far2, UR]), { viewerForward: fwdOf(H0), tableY: TABLE_Y }); assert.ok(s3.ok, s3.reason); assert.equal(s3.ll.id, 7); assert.equal(s3.ur.id, 42); assert.equal(s3.nTags, 4);
});

await ok('tags onderling 0,3° verdraaid of 6° gedraaid: assen volgen de tags; > 8° spreiding → terugval op frame/kijkrichting', () => {
  const rot = (psi) => ({ ...LL, psi: psi * deg }), rot2 = (psi) => ({ ...UR, psi: psi * deg });
  const s = estimateSurface(trk([rot(6), rot2(6)]), { viewerForward: fwdOf(H0), tableY: TABLE_Y }); assert.equal(s.axes.source, 'tags');
  near(Math.atan2(s.frame.x[2], s.frame.x[0]) / deg, 6, 0.05, 'x-as volgt de tags (6°)');
  const t90 = estimateSurface(trk([rot(96), rot2(96)]), { viewerForward: fwdOf(H0), tableY: TABLE_Y }); near(Math.atan2(t90.frame.x[2], t90.frame.x[0]) / deg, 6, 0.05, '96° ≡ 6° (mod 90)');
  const mixed = estimateSurface(trk([rot(0), rot2(30)]), { viewerForward: fwdOf(H0), tableY: TABLE_Y }); assert.equal(mixed.axes.source, 'view');
  const withFrame = estimateSurface(trk([rot(0), rot2(30)]), { viewerForward: fwdOf(H0), tableY: TABLE_Y, frame: { x: [Math.cos(10 * deg), 0, Math.sin(10 * deg)], z: [0, 0, 1] } });
  assert.equal(withFrame.axes.source, 'frame'); near(Math.atan2(withFrame.frame.x[2], withFrame.frame.x[0]) / deg, 10, 0.05);
});

await ok('degeneraties: 0/1 tag, collineair (zelfde rij/kolom), te klein, dezelfde tag → duidelijke melding', () => {
  assert.match(estimateSurface([], {}).reason, /minstens 2/); assert.match(estimateSurface(trk([LL]), { tableY: TABLE_Y }).reason, /minstens 2.*nu 1/);
  const row = estimateSurface(trk([LL, { id: 42, c: [0.25, TABLE_Y, -0.45], psi: 0 }]), { viewerForward: fwdOf(H0), tableY: TABLE_Y });
  assert.ok(!row.ok && /te klein|één lijn/.test(row.reason), row.reason);
  const small = estimateSurface(trk([LL, { id: 42, c: [-0.15, TABLE_Y, -0.55], psi: 0 }]), { viewerForward: fwdOf(H0), tableY: TABLE_Y }); assert.ok(!small.ok, 'te klein'); assert.match(small.reason, /minimaal 15 cm/);
});

await ok('ruis: σ = 1 cm + 20% uitschieters (10–40 cm) → mediaan gem. < 3 mm, max < 7 mm na 40 samples (20 seeds); tracker volgt een echt verplaatste tag', () => {
  const errs = [];
  for (let seed = 1; seed <= 20; seed++) {
    const r = rng(seed), t = new TagTracker();
    for (let i = 0; i < 40; i++) {
      const out = r() < 0.2, mag = out ? 0.1 + 0.3 * r() : 0, dir = v3.norm([gauss(r), 0, gauss(r)]);
      const c = [LL.c[0] + 0.01 * gauss(r) + mag * dir[0], LL.c[1] + 0.003 * gauss(r), LL.c[2] + 0.01 * gauss(r) + mag * dir[2]];
      t.add(i * 30, { ok: true, id: 7, center: c, dist: 0.8, yaw: 0, method: 'ray' });
    }
    const e = t.estimates()[0]; errs.push(Math.hypot(e.center[0] - LL.c[0], e.center[2] - LL.c[2]));
  }
  const mean = errs.reduce((a, b) => a + b) / errs.length, max = Math.max(...errs);
  console.log(`   mediaanfout: gemiddeld ${(mean * 1000).toFixed(1)} mm, max ${(max * 1000).toFixed(1)} mm (σ per sample 10 mm, 20% uitschieters)`);
  assert.ok(mean < 0.003 && max < 0.007);
  const t = new TagTracker(); for (let i = 0; i < 20; i++) t.add(i * 30, { ok: true, id: 7, center: [0, 0.75, 0], dist: 1, yaw: 0 });
  for (let i = 0; i < 20; i++) t.add(1000 + i * 30, { ok: true, id: 7, center: [0.3, 0.75, 0], dist: 1, yaw: 0 });
  near(t.estimates()[0].center[0], 0.3, 1e-9, 'na een echte verplaatsing volgt de tracker de nieuwe plek');
  assert.ok(!t.add(5000, { ok: true, id: 9, center: [0, 0, 0], dist: 3.5 }) && !t.add(5000, { ok: true, id: 9, center: [0, 0, 0], dist: 0.05 }), 'afstandsfilter');
});

// --- scanner-toestandsmachine (nepklok, geen beeld) ---
function mkScanner(extra = {}) {
  const clock = { t: 0 }, ev = { stable: [], fallback: [], updates: 0 }, hist = new PoseHistory();
  const sc = new TagTableScanner({ history: hist, extrinsicsFor: () => ext, getTableY: () => TABLE_Y, now: () => clock.t, timeoutMs: 6000,
    onStable: s => ev.stable.push(s), onFallback: r => ev.fallback.push(r), onUpdate: () => { ev.updates++; }, ...extra });
  return { sc, clock, ev, hist };
}
function run(sc, clock, hist, { seconds, tags, head = H0, noise = 0.003, seed = 5, moving = false, lag = 0 }) {
  const r = rng(seed);
  for (let t = 0; t < seconds * 1000; t += 33) {
    clock.t += 33;
    const h = moving ? { pos: [0.5 * Math.sin(clock.t / 300) , head.pos[1], head.pos[2]], quat: head.quat } : head;
    hist.push(clock.t, h.pos, h.quat);
    const dets = tags.map(g => analyticDet({ ...g, c: [g.c[0] + noise * gauss(r), g.c[1], g.c[2] + noise * gauss(r)] }, h, ext, K0));
    sc.feed(dets, { t: clock.t - lag, w: cam.w, h: cam.h, intrinsics: K0, device: 'camera 2 1, facing back' });
    if (t % 198 === 0) sc.tick();
  }
}
await ok('scanner: scanning → stable (onStable exact 1×) binnen enkele seconden; resultaat ≈ 50 × 40 cm', () => {
  const { sc, clock, ev, hist } = mkScanner(); sc.start(); assert.equal(sc.phase, 'scanning');
  run(sc, clock, hist, { seconds: 5, tags: [LL, UR] }); sc.tick(); sc.tick();
  assert.equal(sc.phase, 'stable'); assert.equal(ev.stable.length, 1); assert.equal(ev.fallback.length, 0); assert.ok(ev.updates > 3);
  const s = ev.stable[0]; near(s.width, 0.5, 0.004); near(s.depth, 0.4, 0.004);
  sc.markApplied(); assert.equal(sc.phase, 'applied'); sc.feed([], { t: clock.t }); sc.tick(); assert.equal(ev.stable.length, 1, 'geen tweede onStable');
  const dbg = formatTagDebug(sc.snapshot(), { intrinsics: K0, tSource: 'test', extrinsics: ext, tagSize: TAG, device: 'cam' }).join('\n');
  for (const k of ['#7', '#42', 'LINKSONDER', 'RECHTSBOVEN', 'oppervlak:', 'hoek verst-links', 'intrinsics', 'extrinsics']) assert.ok(dbg.includes(k), 'debug bevat ' + k);
});
await ok('scanner: 1 tag of tags op één lijn → geen onStable; na time-out onFallback (→ handmatige 3-punts kalibratie), reden bevat 25/6 s-melding', () => {
  let { sc, clock, ev, hist } = mkScanner(); sc.start(); run(sc, clock, hist, { seconds: 8, tags: [LL] }); sc.tick();
  assert.equal(sc.phase, 'failed'); assert.equal(ev.stable.length, 0); assert.equal(ev.fallback.length, 1); assert.match(ev.fallback[0], /handmatige 3-punts/); assert.match(ev.fallback[0], /minstens 2/);
  ({ sc, clock, ev, hist } = mkScanner()); sc.start(); run(sc, clock, hist, { seconds: 8, tags: [LL, { id: 42, c: [0.25, TABLE_Y, -0.45], psi: 0 }] });
  assert.equal(ev.stable.length, 0); assert.equal(ev.fallback.length, 1);
});
await ok('scanner: te veel ruis (σ 4 cm) blijft onstabiel → terugval; snelle kopbeweging en beeldtijden buiten de pose-historie (gap > 150 ms) worden weggegooid (en geteld)', () => {
  let { sc, clock, ev, hist } = mkScanner(); sc.start(); run(sc, clock, hist, { seconds: 8, tags: [LL, UR], noise: 0.04 });
  assert.equal(ev.stable.length, 0, 'σ = 4 cm mag niet stabiel worden'); assert.equal(ev.fallback.length, 1);
  ({ sc, clock, ev, hist } = mkScanner()); sc.start(); run(sc, clock, hist, { seconds: 2, tags: [LL, UR], moving: true });
  assert.ok(sc.counters.dropMotion > 20 && sc.counters.used < sc.counters.dets * 0.5, `beweging gedropt: ${JSON.stringify(sc.counters)}`);
  ({ sc, clock, ev, hist } = mkScanner()); sc.start(); run(sc, clock, hist, { seconds: 1, tags: [LL, UR], lag: -500 });
  assert.ok(sc.counters.dropGap + sc.counters.dropNoHistory > 15 && sc.counters.used === 0, 'beeldtijd ver buiten de pose-historie: ' + JSON.stringify(sc.counters));
});
await ok('tijdsynchronisatie: 60 ms latentie-fout bij 0,3 m/s kopbeweging geeft ≈ 1–2 cm fout (gedocumenteerd), zonder correctie; correcte timestamp geeft ≈ 0', () => {
  const hist = new PoseHistory(); const pathAt = t => ({ pos: [0.3 * t / 1000, 1.3, 0.1], quat: H0.quat });
  for (let t = 0; t <= 1000; t += 11) { const p = pathAt(t); hist.push(t, p.pos, p.quat); }
  const tCap = 500, truth = pathAt(tCap), d = analyticDet(LL, truth, ext, K0);
  const good = tagSample(d, K0, hist.at(tCap), ext, { tableY: TABLE_Y }), bad = tagSample(d, K0, hist.at(tCap + 60), ext, { tableY: TABLE_Y });
  const eg = v3.len(v3.sub(good.center, LL.c)), eb = v3.len(v3.sub(bad.center, LL.c)); console.log(`   fout: juiste t ${(eg * 1000).toFixed(2)} mm; 60 ms te laat ${(eb * 1000).toFixed(1)} mm`);
  assert.ok(eg < 0.5e-3 + 1e-3 && eb > 0.01 && eb < 0.03);
  assert.ok(hist.speed(500).lin < S.MAX_LIN_SPEED, 'dit is een toegestane (trage) beweging');
});

// --- end-to-end met de WASM-detector op gerenderde beelden ---
const det = await createAprilTagDetector(); det.setTagSize(TAG);
await ok('end-to-end (gerenderde beelden + WASM-detector + pose-historie): rechthoek binnen 6 mm (ray) bij ruis & beweging; pose-methode en 12% verkeerde hfov vergeleken', async () => {
  const hist = new PoseHistory(), r = rng(11); let clock = 0;
  const heads = [];
  const worldTags = [LL, UR, { id: 100, c: [-0.02, TABLE_Y, -0.62], psi: 0 }];
  const scene = (head, e) => worldTags.map(g => { const d = analyticDet(g, head, e, K0); return { id: g.id, size: TAG, R: d.pose.R, t: d.pose.t }; });
  const results = { ray: new TagTracker(), pose: new TagTracker(), rayBadK: new TagTracker() };
  const KB = { ...K0, fx: K0.fx * 1.12, fy: K0.fy * 1.12 };
  const { sc: scB, clock: clB, hist: hiB } = mkScanner({ selfCalibrate: true, getTableY: () => TABLE_Y }); scB.start();
  const NF = 24;
  for (let i = 0; i < NF; i++) {
    clock += 33;
    const h = headPose([0.04 * Math.sin(i / 4), 1.3 + 0.02 * Math.cos(i / 5), 0.1], 2 * Math.sin(i / 6), -50 + 1.5 * Math.cos(i / 3)); hist.push(clock, h.pos, h.quat); heads.push(h);
    const { gray } = renderScene(scene(h, ext), cam, { noise: 4, seed: i + 1 });
    det.setIntrinsics(K0.fx, K0.fy, K0.cx, K0.cy); const dets = det.detect(gray, cam.w, cam.h);
    assert.ok(dets.length >= 2, `frame ${i}: ${dets.length} tags gevonden`);
    const head = hist.at(clock);
    for (const d of dets) {
      results.ray.add(clock, tagSample(d, K0, head, ext, { method: 'ray', tableY: TABLE_Y }));
      results.pose.add(clock, tagSample(d, K0, head, ext, { method: 'pose' }));
      det.setIntrinsics(KB.fx, KB.fy, KB.cx, KB.cy);    // een detector met verkeerde hfov (pose én ray lijden eronder)
    }
    det.setIntrinsics(KB.fx, KB.fy, KB.cx, KB.cy); const dB = det.detect(gray, cam.w, cam.h); for (const d of dB) results.rayBadK.add(clock, tagSample(d, KB, head, ext, { method: 'ray', tableY: TABLE_Y }));
    clB.t = clock; hiB.push(clock, h.pos, h.quat); scB.feed(dB, { t: clock, w: cam.w, h: cam.h, intrinsics: KB, device: 'camera 2 1' });
  }
  const rep = {};
  for (const [k, t] of Object.entries(results)) {
    const s = estimateSurface(t.estimates(), { viewerForward: fwdOf(H0), tableY: TABLE_Y }); assert.ok(s.ok, k + ': ' + s.reason);
    rep[k] = { dw: Math.abs(s.width - 0.5), dd: Math.abs(s.depth - 0.4), ll: s.ll.id, ur: s.ur.id, n: t.estimates().map(e => e.n) };
    console.log(`   ${k.padEnd(8)} Δbreedte ${(rep[k].dw * 1000).toFixed(1)} mm  Δdiepte ${(rep[k].dd * 1000).toFixed(1)} mm  LL=#${s.ll.id} UR=#${s.ur.id}  samples/tag ${rep[k].n}`);
    assert.equal(s.ll.id, 7); assert.equal(s.ur.id, 42);
  }
  const sB = estimateSurface(scB.tracker.estimates(), { viewerForward: fwdOf(H0), tableY: TABLE_Y }); assert.ok(sB.ok, sB.reason);
  console.log(`   zelfkalibratie bij 12% te grote f: k = ${scB.focal.used.toFixed(3)} (ideaal ${(1 / 1.12).toFixed(3)}); Δbreedte ${(Math.abs(sB.width - 0.5) * 1000).toFixed(1)} mm, Δdiepte ${(Math.abs(sB.depth - 0.4) * 1000).toFixed(1)} mm (zonder: ${(rep.rayBadK.dw * 1000).toFixed(0)} / ${(rep.rayBadK.dd * 1000).toFixed(0)} mm)`);
  assert.ok(Math.abs(sB.width - 0.5) < 0.008 && Math.abs(sB.depth - 0.4) < 0.008, 'met zelfkalibratie van f binnen 8 mm');
  assert.ok(rep.ray.dw < 0.006 && rep.ray.dd < 0.006, 'ray-methode binnen 6 mm');
  assert.ok(rep.pose.dw < 0.02 && rep.pose.dd < 0.02, 'pose-methode binnen 2 cm (met bekende intrinsics)');
  assert.ok(rep.rayBadK.dw < 0.08 && rep.rayBadK.dd < 0.08, '12% foute hfov: beperkte fout');
});

console.log(`\n${n} tests geslaagd`);
