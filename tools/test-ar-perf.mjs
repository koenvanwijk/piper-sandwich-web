#!/usr/bin/env node
// Tests (Node, zonder browser/three): AR-prestatie-opties (src/ar-perf.js) en de duty-cycle van de tag-detectie.
//   node tools/test-ar-perf.mjs
import assert from 'node:assert/strict';
import { parseArPerf, optimizeSandwichForAR, clusterDecimate, decimateToBudget } from '../src/ar-perf.js';
import { AprilTagCamera } from '../src/apriltag-camera.js';

let n = 0; const ok = (name, f) => { f(); n++; console.log('ok', name); };

ok('parseArPerf: standaard = geoptimaliseerd; ?perf=0 = alles oud gedrag; los per optie uit te zetten; waarden geklemd', () => {
  const d = parseArPerf('');
  assert.equal(d.enabled, true); assert.equal(d.fbScale, 0.8); assert.equal(d.foveation, 1); assert.equal(d.antialias, true); assert.equal(d.standardMaterials, true); assert.equal(d.freeze, true);
  assert.equal(d.tagFps, 10); assert.equal(d.tagProc, 960); assert.equal(d.tagDuty, 0.25); assert.equal(d.tagPreviewInXR, false); assert.equal(d.hudThrottleMs, 500);
  const o = parseArPerf('?perf=0');
  assert.deepEqual([o.enabled, o.fbScale, o.foveation, o.standardMaterials, o.freeze, o.tagFps, o.tagProc, o.tagDuty, o.tagPreviewInXR, o.hudThrottleMs], [false, 1, 1, false, false, 30, 960, 0, true, 0]);
  assert.equal(parseArPerf('?fbscale=1').fbScale, 1); assert.equal(parseArPerf('?fbscale=0.1').fbScale, 0.5); assert.equal(parseArPerf('?fbscale=9').fbScale, 1.5); assert.equal(parseArPerf('?fbscale=x').fbScale, 0.8);
  assert.equal(parseArPerf('?aa=0').antialias, false); assert.equal(parseArPerf('?mat=physical').standardMaterials, false); assert.equal(parseArPerf('?freeze=0').freeze, false);
  assert.equal(parseArPerf('?tagfps=5&tagproc=640&tagduty=0.5&tagpreview=1').tagFps, 5); assert.equal(parseArPerf('?tagproc=100').tagProc, 320); assert.equal(parseArPerf('?tagpreview=1').tagPreviewInXR, true);
  assert.equal(parseArPerf('?tags=1&debug=1').fbScale, 0.8, 'andere params veranderen niets');
});

// minimale nep-three
class Obj { constructor() { this.children = []; this.parent = null; this.visible = true; this.matrixAutoUpdate = true; this.updates = 0; }
  add(c) { c.parent = this; this.children.push(c); return this; } remove(c) { this.children = this.children.filter(x => x !== c); c.parent = null; }
  traverse(f) { f(this); for (const c of [...this.children]) c.traverse(f); } updateMatrix() { this.updates++; } updateMatrixWorld() {} }
const color = hex => ({ getHex: () => hex, clone: () => color(hex) });
const phys = (hex, extra = {}) => ({ isMeshPhysicalMaterial: true, color: color(hex), transparent: false, opacity: 1, map: null, roughness: 0.5, metalness: 0.1, side: 0, disposed: false, dispose() { this.disposed = true; }, ...extra });
class Mesh extends Obj { constructor(mat, extra = {}) { super(); this.isMesh = true; this.material = mat; this.castShadow = true; this.receiveShadow = true; this.geometry = { dispose() {} }; Object.assign(this, extra); } }
const THREE = { MeshStandardMaterial: class { constructor(o) { Object.assign(this, o); this.isMeshStandardMaterial = true; } dispose() {} } };

ok('optimizeSandwichForAR: verborgen/Reflector-objecten verwijderd, Physical → Standard gedeeld per kleur, schaduwvlaggen uit, matrices bevroren (root zelf niet)', () => {
  const root = new Obj(), body = new Obj(); root.add(body);
  const reds = [0, 1, 2, 3, 4].map(() => new Mesh(phys(0xff0000))); const blue = new Mesh(phys(0x0000ff)), glass = new Mesh(phys(0x00ff00, { transparent: true, opacity: 0.5 }));
  const floor = new Mesh(phys(0x888888), { visible: false }), mirror = new Mesh(phys(0x999999), { isReflector: true }), target = new Obj(); target.visible = false;
  for (const m of [...reds, blue, glass, floor, mirror, target]) body.add(m);
  const st = optimizeSandwichForAR(root, THREE, parseArPerf(''));
  assert.equal(st.removed, 3); assert.equal(body.children.length, 7); assert.equal(st.meshes, 7);
  assert.ok(reds.every(m => m.material.isMeshStandardMaterial && m.material === reds[0].material), 'zelfde kleur = één gedeeld materiaal');
  assert.notEqual(blue.material, reds[0].material); assert.equal(glass.material.transparent, true); assert.equal(glass.material.opacity, 0.5);
  assert.equal(st.materialsBefore, 7); assert.equal(st.materialsAfter, 3, 'rood, blauw, glas'); assert.equal(st.converted, 7);
  assert.ok([...reds, blue, glass].every(m => !m.castShadow && !m.receiveShadow && !m.matrixAutoUpdate && m.updates === 1));
  assert.equal(root.matrixAutoUpdate, true, 'root (positie/rotatie via placeOnTable) blijft vrij'); assert.equal(body.matrixAutoUpdate, false);
  // opt-outs
  const r2 = new Obj(); const m2 = new Mesh(phys(0xff0000)); r2.add(m2); optimizeSandwichForAR(r2, THREE, parseArPerf('?mat=physical&freeze=0'));
  assert.ok(m2.material.isMeshPhysicalMaterial && m2.matrixAutoUpdate === true);
});

function sphere(nu, nv, r = 0.05) {                                             // indexed UV-bol, 2·nu·nv driehoeken
  const pos = [], idx = [];
  for (let j = 0; j <= nv; j++) for (let i = 0; i <= nu; i++) { const th = Math.PI * j / nv, ph = 2 * Math.PI * i / nu; pos.push(r * Math.sin(th) * Math.cos(ph), r * Math.cos(th), r * Math.sin(th) * Math.sin(ph)); }
  for (let j = 0; j < nv; j++) for (let i = 0; i < nu; i++) { const a = j * (nu + 1) + i, b = a + nu + 1; idx.push(a, b, a + 1, b, b + 1, a + 1); }
  return { pos: Float32Array.from(pos), idx: Uint32Array.from(idx) };
}
ok('decimateToBudget: bol met 80 k driehoeken → ≤ 12 k (en niet veel minder), vorm binnen enkele mm, geldige indices, geen NaN; klein mesh blijft ongemoeid', () => {
  const m = sphere(200, 200); assert.equal(m.idx.length / 3, 80000);
  const t0 = performance.now(), r = decimateToBudget(m.pos, m.idx, 12000), ms = performance.now() - t0;
  const tris = r.index.length / 3; assert.ok(tris <= 12000 && tris > 4000, `driehoeken ${tris}`);
  assert.ok(r.index.every(i => i < r.position.length / 3) && r.position.every(Number.isFinite));
  const dev = Math.max(...Array.from({ length: r.position.length / 3 }, (_, v) => Math.abs(Math.hypot(r.position[3 * v], r.position[3 * v + 1], r.position[3 * v + 2]) - 0.05)));
  console.log(`   80000 → ${tris} driehoeken, max afwijking van de bol ${(dev * 1000).toFixed(2)} mm (straal 50 mm), ${ms.toFixed(0)} ms`);
  assert.ok(dev < 0.003, 'afwijking < 3 mm');
  assert.equal(decimateToBudget(sphere(20, 20).pos, sphere(20, 20).idx, 12000), null); assert.equal(decimateToBudget(m.pos, m.idx, 0), null, 'budget 0 = uit');
  const c = clusterDecimate(m.pos, m.idx, 1); assert.ok(c.index.length / 3 <= 24 && c.position.every(Number.isFinite), 'enorme cel: (bijna) alles klapt in elkaar zonder crash');
});
ok('optimizeSandwichForAR: geometrie wordt gedecimeerd en gedeeld (twee meshes, één geometrie), ?lod=0 laat het ongemoeid', () => {
  class Attr { constructor(array, itemSize) { this.array = array; this.itemSize = itemSize; } }
  class Geo { constructor() { this.attributes = {}; this.index = null; this.disposed = false; } setAttribute(n, a) { this.attributes[n] = a; } setIndex(a) { this.index = a; } computeVertexNormals() { this.normals = true; } computeBoundingSphere() {} dispose() { this.disposed = true; } }
  const T = { ...THREE, BufferGeometry: Geo, BufferAttribute: Attr };
  const mk = () => { const m = sphere(150, 150), g = new Geo(); g.attributes.position = new Attr(m.pos, 3); g.index = new Attr(m.idx, 1); const root = new Obj(), a = new Mesh(phys(0xaaaaaa)), b = new Mesh(phys(0xaaaaaa)); a.geometry = b.geometry = g; root.add(a); root.add(b); return { root, a, b, g }; };
  const x = mk(), st = optimizeSandwichForAR(x.root, T, parseArPerf(''));
  assert.equal(x.a.geometry, x.b.geometry, 'gedeelde geometrie blijft gedeeld'); assert.notEqual(x.a.geometry, x.g); assert.ok(x.g.disposed && x.a.geometry.normals);
  assert.equal(st.decimated, 1); assert.equal(st.trisBefore, 45000); assert.ok(st.trisAfter <= 12000, 'na: ' + st.trisAfter);
  const y = mk(); optimizeSandwichForAR(y.root, T, parseArPerf('?lod=0')); assert.equal(y.a.geometry, y.g);
  const z = mk(); optimizeSandwichForAR(z.root, T, parseArPerf('?perf=0')); assert.equal(z.a.geometry, z.g);
  assert.equal(parseArPerf('').lodTris, 12000); assert.equal(parseArPerf('?lod=0').lodTris, 0); assert.equal(parseArPerf('?lod=5000').lodTris, 5000);
});

ok('AprilTagCamera dutyCycle: wacht ≥ detMs/duty tussen frames (40 ms detectie, 25 % → ≥ 160 ms); dutyCycle 0 = oud gedrag (alleen maxFps)', () => {
  const mk = (dutyCycle, maxFps) => { const c = Object.create(AprilTagCamera.prototype); Object.assign(c, { video: { videoWidth: 640 }, maxFps, dutyCycle, _lastT: 0, fps: { detMs: 40 }, processed: 0, lastError: null, onStatus() {} });
    c.processFrame = function () { this.processed++; }; return c; };
  for (const [duty, fps, want] of [[0.25, 30, 6], [0, 30, 30], [0.25, 5, 5], [1, 30, 20]]) {   // 1 s aan rVFC-ticks van 16,7 ms
    const c = mk(duty, fps); for (let t = 100; t < 1100; t += 16.7) c.step(t, null, null);
    assert.ok(Math.abs(c.processed - want) <= Math.max(2, want * 0.15), `duty ${duty} fps ${fps}: ${c.processed} frames (verwacht ≈ ${want})`);
  }
});
console.log(`${n} tests ok`);
