#!/usr/bin/env node
// Unit-tests (Node, geen browser) voor de doel-ghost (src/target-viz.js): query-opties, statusclassificatie, smoothing en hysterese,
// en de ghost-opbouw met een minimale THREE-stub (gedeelde geometrie, zichtbaarheid, kleur, swizzle).
//   node tools/test-target-viz.mjs
import assert from 'node:assert/strict';
import { parseTargetOpts, TargetStatus, TARGET_DEFAULTS, STATUS_COLOR, createTargetGhosts } from '../src/target-viz.js';

let n = 0; const ok = (name, f) => { f(); n++; console.log('ok', name); };
const deg = Math.PI / 180, mm = 1e-3, DT = 1 / 72;
// De statustests gebruiken een groen-drempel van 5 mm (scherper dan de standaard 8 mm) zodat de grenzen in de tests rond getallen zijn.
const O5 = { okMm: 5 };
const run = (ts, e, secs) => { let r; for (let t = 0; t < secs; t += DT) r = ts.update(e, DT); return r; };

ok('parseTargetOpts: standaard aan tijdens clutch; ?target=0/always; drempels instelbaar en geldig', () => {
  const d = parseTargetOpts('');
  assert.equal(d.mode, 'clutch'); assert.deepEqual([d.okMm, d.okDeg, d.badMm, d.badDeg], [8, 3, 20, 10]); assert.equal(d.haptic, true);
  assert.equal(parseTargetOpts('?target=0').mode, 'off'); assert.equal(parseTargetOpts('?target=always').mode, 'always'); assert.equal(parseTargetOpts('?target=1').mode, 'clutch');
  const c = parseTargetOpts('?tgtok=8,4&tgtbad=30,15&tgthaptic=0');
  assert.deepEqual([c.okMm, c.okDeg, c.badMm, c.badDeg, c.haptic], [8, 4, 30, 15, false]);
  const bad = parseTargetOpts('?tgtok=x&tgtbad=-1,3'); assert.deepEqual([bad.okMm, bad.badMm], [8, 20], 'ongeldig → standaard');
  const inv = parseTargetOpts('?tgtok=25,12'); assert.ok(inv.badMm >= inv.okMm && inv.badDeg >= inv.okDeg, 'rood-drempel nooit onder groen');
});

ok('standaarddrempels (8 mm / 3°): 2,3 mm statische doorzakking onder zwaartekracht (gemeten in de browser) blijft groen', () => {
  assert.equal(run(new TargetStatus(), { posErr: 2.3 * mm, rotErr: 0.4 * deg }, 1).status, 'ok');
  assert.equal(run(new TargetStatus(), { posErr: 6 * mm, rotErr: 1 * deg }, 1).status, 'ok');
  assert.equal(run(new TargetStatus(), { posErr: 12 * mm, rotErr: 1 * deg }, 1).status, 'warn');
});

ok('classificatie: groen < 5 mm/3°, oranje daarboven, rood als IK het doel niet haalt (> 20 mm of > 10°) of tegen een jointlimiet', () => {
  assert.equal(run(new TargetStatus(O5), { posErr: 2 * mm, rotErr: 1 * deg }, 1).status, 'ok');
  assert.equal(run(new TargetStatus(O5), { posErr: 9 * mm, rotErr: 1 * deg }, 1).status, 'warn', 'positie-afwijking');
  assert.equal(run(new TargetStatus(O5), { posErr: 1 * mm, rotErr: 6 * deg }, 1).status, 'warn', 'oriëntatie-afwijking');
  assert.equal(run(new TargetStatus(O5), { posErr: 30 * mm, ikPos: 30 * mm }, 1).status, 'bad', 'IK-restfout 30 mm');
  assert.equal(run(new TargetStatus(O5), { rotErr: 15 * deg, ikRot: 15 * deg }, 1).status, 'bad', 'IK-restfout 15°');
  assert.equal(run(new TargetStatus(O5), { posErr: 7 * mm, ikPos: 7 * mm, atLimit: true }, 1).status, 'bad', 'limiet + restfout');
  assert.equal(run(new TargetStatus(O5), { posErr: 1 * mm, ikPos: 0.1 * mm, atLimit: true }, 1).status, 'ok', 'tegen de limiet maar doel gehaald = groen');
});

ok('rood ook als de arm > 0,4 s ver van een (volgens de IK) bereikbaar doel blijft (bv. geblokkeerd door de tafel); korte uitschieter (snel bewegen) = oranje', () => {
  const ts = new TargetStatus(O5); run(ts, { posErr: 1 * mm }, 0.5);
  let r = run(ts, { posErr: 40 * mm }, 0.25); assert.equal(r.status, 'warn', 'kort ver weg: oranje');
  r = run(ts, { posErr: 1 * mm }, 0.5); assert.equal(r.status, 'ok');
  r = run(ts, { posErr: 40 * mm }, 0.8); assert.equal(r.status, 'bad', 'aanhoudend ver weg: rood');
});

ok('smoothing + verblijftijd: een uitschieter van 1 frame (60 mm) verandert niets; 3 frames hooguit kort oranje, nooit rood', () => {
  const ts = new TargetStatus(O5); run(ts, { posErr: 1 * mm }, 0.5); let seen = new Set();
  ts.update({ posErr: 60 * mm, ikPos: 60 * mm }, DT);
  for (let t = 0; t < 0.5; t += DT) seen.add(ts.update({ posErr: 1 * mm }, DT).status);
  assert.deepEqual([...seen], ['ok'], '1 frame: blijft groen');
  seen = new Set(); for (let k = 0; k < 3; k++) seen.add(ts.update({ posErr: 60 * mm, ikPos: 60 * mm }, DT).status);
  for (let t = 0; t < 1; t += DT) seen.add(ts.update({ posErr: 1 * mm }, DT).status);
  assert.ok(!seen.has('bad'), '3 frames: niet rood ' + [...seen]); assert.equal(ts.status, 'ok', 'daarna weer groen');
});

ok('hysterese: ruis rond de 5 mm-grens flikkert niet; terug naar groen pas onder 0,7 × drempel; terug uit rood pas onder 0,7 × rood-drempel', () => {
  const ts = new TargetStatus(O5); let changes = 0, t = 0;
  run(ts, { posErr: 6 * mm }, 1);                                      // eerst oranje
  for (let i = 0; i < 2000; i++, t += DT) { const r = ts.update({ posErr: (5 + 0.8 * Math.sin(t * 40) + 0.4 * Math.sin(t * 7.3)) * mm }, DT); if (r.changed) changes++; }
  assert.ok(changes <= 1, `aantal wisselingen ${changes}`); assert.equal(ts.status, 'warn', 'blijft oranje boven 3,5 mm');
  assert.equal(run(ts, { posErr: 4 * mm }, 1).status, 'warn', '4 mm > 0,7×5: nog oranje');
  assert.equal(run(ts, { posErr: 3 * mm }, 1).status, 'ok', '3 mm < 3,5: groen');
  run(ts, { posErr: 25 * mm, ikPos: 25 * mm }, 1); assert.equal(ts.status, 'bad');
  assert.equal(run(ts, { posErr: 16 * mm, ikPos: 16 * mm }, 1).status, 'bad', '16 mm > 0,7×20: blijft rood');
  assert.equal(run(ts, { posErr: 10 * mm, ikPos: 10 * mm }, 1).status, 'warn', '10 mm: uit rood, oranje');
  let flick = 0; run(ts, { posErr: 25 * mm, ikPos: 25 * mm }, 1);
  for (let i = 0; i < 2000; i++, t += DT) { const e = (20 + 3 * Math.sin(t * 31)) * mm, r = ts.update({ posErr: e, ikPos: e }, DT); if (r.changed) flick++; }
  assert.equal(flick, 0, 'ruis rond de rood-grens: geen wisselingen');
});

ok('enteredBad alleen op de overgang naar rood (voor de haptische puls); reset() bij loslaten clutch; NaN = rood, geen crash', () => {
  const ts = new TargetStatus(O5); let entered = 0;
  for (let i = 0; i < 300; i++) if (ts.update({ posErr: 50 * mm, ikPos: 50 * mm }, DT).enteredBad) entered++;
  assert.equal(entered, 1); ts.reset(); assert.equal(ts.status, 'ok');
  assert.equal(run(new TargetStatus(O5), { posErr: NaN, ikPos: NaN }, 0.3).status, 'bad');
});

// ---- ghost met een minimale THREE-stub
class V { constructor() { this.x = this.y = this.z = 0; } set(x, y, z) { this.x = x; this.y = y; this.z = z; return this; } }
class Q { set(x, y, z, w) { Object.assign(this, { x, y, z, w }); return this; } }
class Obj { constructor() { this.children = []; this.visible = true; this.position = new V(); this.quaternion = new Q(); this.renderOrder = 0; } add(...c) { this.children.push(...c); return this; } }
class Attr { constructor(a, n) { this.array = a instanceof Float32Array ? a : Float32Array.from(a); this.itemSize = n; this.needsUpdate = false; } }
class Geo { constructor() { this.attributes = {}; } setAttribute(k, a) { this.attributes[k] = a; } }
class Mat { constructor(o) { Object.assign(this, o); const c = o.color; this.color = { hex: c, setHex(h) { this.hex = h; } }; } }
const THREE = { Group: Obj, Mesh: class extends Obj { constructor(g, m) { super(); this.geometry = g; this.material = m; } }, LineSegments: class extends Obj { constructor(g, m) { super(); this.geometry = g; this.material = m; } },
  Line: class extends Obj { constructor(g, m) { super(); this.geometry = g; this.material = m; } }, BufferGeometry: Geo, Float32BufferAttribute: Attr,
  MeshBasicMaterial: Mat, LineBasicMaterial: Mat, DoubleSide: 2 };

ok('ghost: licht (MeshBasic/LineBasic), geometrie gedeeld tussen armen, onzichtbaar tot update, kleur per status, positie/oriëntatie met de MuJoCo→three-swizzle', () => {
  const root = new Obj(), gh = createTargetGhosts({ THREE, parent: root });
  const L = gh.arms.left, R = gh.arms.right;
  assert.equal(root.children.length, 4, 'per arm: groep + lijn');
  assert.equal(L.g.children[0].geometry, R.g.children[0].geometry, 'silhouet-geometrie gedeeld'); assert.equal(L.g.children[1].geometry, R.g.children[1].geometry, 'assenkruis gedeeld');
  assert.ok(L.mat instanceof Mat && !('roughness' in L.mat), 'geen belicht materiaal');
  assert.equal(L.g.children[0].geometry.attributes.position.array.length, 4 * 36 * 3, '4 boxen = 144 driehoek-hoekpunten');
  assert.ok(!L.g.visible && !L.line.visible, 'standaard onzichtbaar');
  gh.update('left', { visible: true, pos: [0.3, 0.1, 0.2], quat: [1, 0, 0, 0], tcpPos: [0.3, 0.1, 0.18], status: 'bad' });
  assert.ok(L.g.visible && L.line.visible); assert.equal(L.mat.color.hex, STATUS_COLOR.bad); assert.equal(L.lineMat.color.hex, STATUS_COLOR.bad);
  assert.deepEqual([L.g.position.x, L.g.position.y, L.g.position.z].map(v => +v.toFixed(6)), [0.3, 0.2, -0.1], 'mj (x,y,z) → three (x,z,−y)');
  assert.deepEqual(Array.from(L.line.geometry.attributes.position.array).map(v => +v.toFixed(6)), [0.3, 0.18, -0.1, 0.3, 0.2, -0.1]);
  assert.ok(L.line.geometry.attributes.position.needsUpdate);
  assert.ok(!R.g.visible, 'andere arm ongemoeid');
  gh.update('left', { visible: false, pos: [0, 0, 0], quat: [1, 0, 0, 0] }); assert.ok(!L.g.visible && !L.line.visible);
  gh.update('right', { visible: true, pos: [0, 0, 0.3], quat: [1, 0, 0, 0], status: 'ok' }); gh.hideAll(); assert.ok(!R.g.visible);
});

console.log(`${n} tests ok`);
