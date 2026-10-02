#!/usr/bin/env node
// Test van de AprilTag-detector (WASM in Node, geen browser): synthetische beelden met bekende tag36h11-ID's en bekende pose.
//   node tools/test-apriltag.mjs
import assert from 'node:assert/strict';
import { createAprilTagDetector, rgbaToGray, DEFAULT_TAG_SIZE_M } from '../src/apriltag-detector.js';
import { pickCamera, intrinsicsFromTrack, AprilTagCamera } from '../src/apriltag-camera.js';
import { renderScene, rotXYZ, FIXTURE_IDS } from './apriltag-synth.mjs';

let n = 0; const ok = async (name, f) => { await f(); n++; console.log('ok', name); };
const cam = { w: 960, h: 720, fx: 700, fy: 700, cx: 480, cy: 360 };
const TAG = DEFAULT_TAG_SIZE_M;            // 0,08255 m: onze tags zijn 82,55 mm
assert.equal(TAG, 0.08255);
const det = await createAprilTagDetector();
const d2 = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
const I = rotXYZ(0, 0, 0);

await ok('7 tags (ID 0,1,7,42,100,321,586) frontaal: alle ID\'s gevonden, hoeken < 0,5 px', () => {
  const ids = FIXTURE_IDS, tags = ids.map((id, i) => ({ id, size: TAG, R: I, t: [-0.21 + (i % 4) * 0.14, -0.1 + Math.floor(i / 4) * 0.2, 0.9] }));
  const { gray, truth } = renderScene(tags, cam);
  det.setIntrinsics(0, 0, 0, 0);
  const t0 = performance.now(); const r = det.detect(gray, cam.w, cam.h); const ms = performance.now() - t0;
  assert.deepEqual(r.map(d => d.id).sort((a, b) => a - b), [...ids].sort((a, b) => a - b));
  let worst = 0;
  for (const t of truth) { const d = r.find(x => x.id === t.id); assert.ok(d, 'id ' + t.id);
    t.corners.forEach((c, i) => { worst = Math.max(worst, d2(c, d.corners[i])); }); worst = Math.max(worst, d2(t.center, d.center)); assert.equal(d.pose, undefined); }
  console.log(`   ${r.length}/${ids.length} tags, max hoek/centrum-afwijking ${worst.toFixed(3)} px, ${ms.toFixed(0)} ms voor ${cam.w}×${cam.h}`);
  assert.ok(worst < 0.5);
});

await ok('hoekvolgorde: 4 hoeken = LO, RO, RB, LB van de tag zoals gelezen; rotatie 90° verschuift de hoeken cyclisch', () => {
  const t = { id: 42, size: TAG, R: rotXYZ(0, 0, Math.PI / 2), t: [0, 0, 0.8] };
  const { gray, truth } = renderScene([t], cam); det.setIntrinsics(0, 0, 0, 0);
  const d = det.detect(gray, cam.w, cam.h)[0]; assert.equal(d.id, 42);
  d.corners.forEach((c, i) => assert.ok(d2(c, truth[0].corners[i]) < 0.6, `hoek ${i}`));
});

for (const [name, R, tz] of [['kantel 35° om y', rotXYZ(0, 0.61, 0), 0.8], ['kantel 30° om x + 20° roll', rotXYZ(0.52, 0, 0.35), 0.9], ['ver (1,5 m)', rotXYZ(0.2, -0.3, 0.1), 1.5]]) {
  await ok(`pose-schatting (${name}): t en R binnen tolerantie`, () => {
    const t = { id: 100, size: TAG, R, t: [0.05, -0.04, tz] };
    const { gray, truth } = renderScene([t], cam, { noise: 3 });
    det.setTagSize(TAG); det.setIntrinsics(cam.fx, cam.fy, cam.cx, cam.cy);
    const r = det.detect(gray, cam.w, cam.h); assert.equal(r.length, 1); const p = r[0].pose; assert.ok(p, 'pose aanwezig');
    const te = Math.hypot(p.t[0] - t.t[0], p.t[1] - t.t[1], p.t[2] - t.t[2]);
    const Rt = p.R.map((row, i) => row.map((_, j) => row.reduce((a, _, k) => a + p.R[k][i] * R[k][j], 0)));   // p.R^T · R
    const ang = Math.acos(Math.max(-1, Math.min(1, (Rt[0][0] + Rt[1][1] + Rt[2][2] - 1) / 2))) * 180 / Math.PI;
    const cs = r[0].corners.reduce((m, c, i) => Math.max(m, d2(c, truth[0].corners[i])), 0);
    console.log(`   t-fout ${(te * 1000).toFixed(1)} mm (afstand ${tz} m), rotatiefout ${ang.toFixed(1)}°, hoekfout ${cs.toFixed(2)} px, e=${p.e.toExponential(1)}`);
    assert.ok(te < 0.02 * tz + 0.003, 't-fout te groot'); assert.ok(ang < 6, 'rotatiefout te groot'); assert.ok(cs < 1.0);
  });
}

await ok('zonder intrinsics: geen pose (alleen hoeken/centrum), geen crash; lege/ruisige beelden → 0 detecties', () => {
  det.setIntrinsics(0, 0, 0, 0);
  const { gray } = renderScene([{ id: 7, size: TAG, R: I, t: [0, 0, 0.8] }], cam);
  const r = det.detect(gray, cam.w, cam.h); assert.equal(r.length, 1); assert.equal(r[0].pose, undefined); assert.equal(r[0].corners.length, 4);
  assert.equal(det.detect(new Uint8Array(cam.w * cam.h).fill(128), cam.w, cam.h).length, 0);
  assert.equal(renderScene([], cam, { noise: 40, seed: 5 }).gray.length, cam.w * cam.h);
  assert.equal(det.detect(renderScene([], cam, { noise: 40, seed: 5 }).gray, cam.w, cam.h).length, 0);
  assert.throws(() => det.detect(new Uint8Array(10), 100, 100), /te klein/);
});

await ok('RGBA→grijs (BT.601) en tag onder lage resolutie (640×480, 6 cm op 1,2 m) nog gevonden', () => {
  assert.deepEqual(Array.from(rgbaToGray(new Uint8Array([255, 255, 255, 255, 0, 0, 0, 255, 255, 0, 0, 255]), 3, 1)), [255, 0, 76]);
  const c2 = { w: 640, h: 480, fx: 466, fy: 466, cx: 320, cy: 240 };
  const { gray } = renderScene([{ id: 321, size: 0.06, R: rotXYZ(0.1, 0.2, 0), t: [0, 0, 1.2] }], c2); det.setIntrinsics(0, 0, 0, 0);
  const r = det.detect(gray, c2.w, c2.h); console.log(`   6 cm op 1,2 m: ${r.length ? 'gevonden id ' + r[0].id : 'NIET gevonden'}`);
  assert.deepEqual(r.map(d => d.id), [321]);
});

await ok('pickCamera: Quest-labels "camera 2 1, facing back" = links, "camera 2 2" = rechts; fallbacks', () => {
  const mk = (label, id) => ({ kind: 'videoinput', label, deviceId: id });
  const devs = [mk('camera 2 0, facing front', 'a'), mk('camera 2 2, facing back', 'c'), mk('camera 2 1, facing back', 'b'), { kind: 'audioinput', label: 'mic', deviceId: 'm' }];
  assert.equal(pickCamera(devs, 'left').deviceId, 'b'); assert.equal(pickCamera(devs, 'right').deviceId, 'c');
  assert.equal(pickCamera(devs, 'front').deviceId, 'a'); assert.equal(pickCamera(devs, 'auto').deviceId, 'c');   // eerste "back" in lijst
  assert.equal(pickCamera(devs, 'c').deviceId, 'c'); assert.equal(pickCamera(devs, '0').deviceId, 'a'); assert.equal(pickCamera([], 'left'), null);
  assert.equal(pickCamera([mk('webcam', 'w')], 'left').deviceId, 'w');
});

await ok('intrinsicsFromTrack: herkent focal/principal-velden, anders incomplete (geen verzonnen waarden)', () => {
  const tr = s => ({ getSettings: () => s, getCapabilities: () => ({}) });
  assert.equal(intrinsicsFromTrack(tr({ width: 1280, height: 720 })).complete, false);
  const k = intrinsicsFromTrack(tr({ focalLengthX: 800, focalLengthY: 801, principalPointX: 640, principalPointY: 360 }));
  assert.ok(k.complete && k.fx === 800 && k.fy === 801 && k.cx === 640 && k.cy === 360);
});

await ok('foutmeldingen: NotAllowedError → uitleg over "Headset cameras"; geen mediaDevices → HTTPS-melding', () => {
  assert.match(AprilTagCamera.explainError({ name: 'NotAllowedError' }), /Headset cameras/);
  assert.match(AprilTagCamera.explainError({ name: 'NotFoundError' }), /Experimental web platform features/);
  assert.match(AprilTagCamera.explainError({ name: 'NotReadableError' }), /in gebruik/);
  assert.match(AprilTagCamera.supportError(), /mediaDevices|getUserMedia/);
});

await ok('standaardmaat: een NIEUWE detector zonder setTagSize geeft t_z ≈ ware afstand voor een 82,55 mm tag (en 0,15 m default zou 1,8× te ver zijn)', async () => {
  const d = await createAprilTagDetector();
  const t = { id: 42, size: TAG, R: rotXYZ(0.1, -0.2, 0.3), t: [0.04, -0.03, 0.7] };
  const { gray } = renderScene([t], cam, { noise: 2 }); d.setIntrinsics(cam.fx, cam.fy, cam.cx, cam.cy);
  const p = d.detect(gray, cam.w, cam.h)[0].pose, err = Math.hypot(p.t[0] - t.t[0], p.t[1] - t.t[1], p.t[2] - t.t[2]);
  console.log(`   standaardmaat: t_z=${p.t[2].toFixed(4)} m (waar ${t.t[2]}), fout ${(err * 1000).toFixed(2)} mm, pose.size=${p.size}`);
  assert.ok(err < 0.003 && Math.abs(p.size - 0.08255) < 0.005);
  d.setTagSize(0.15); const q = d.detect(gray, cam.w, cam.h)[0].pose;          // verkeerde maat -> afstand schaalt mee (controle dat de maat echt gebruikt wordt)
  console.log(`   met 0,15 m: t_z=${q.t[2].toFixed(3)} m (verwacht ≈ ${(t.t[2] * 0.15 / TAG).toFixed(3)})`);
  assert.ok(Math.abs(q.t[2] / p.t[2] - 0.15 / TAG) < 0.05);
  d.setTagSize(0.2, 42); assert.ok(d.detect(gray, cam.w, cam.h)[0].pose.t[2] > q.t[2]);   // per-ID maat
  d.destroy();
});

await ok('AprilTagCamera: standaard tagSize = 0.08255 (0 = geen pose, expliciet overschrijfbaar)', () => {
  const mk = o => new AprilTagCamera({ videoWidth: 640 }, { getContext: () => ({}) }, o);
  assert.equal(mk({}).tagSize, 0.08255); assert.equal(mk({ tagSize: 0.1 }).tagSize, 0.1); assert.equal(mk({ tagSize: 0 }).intrinsicsFor(640, 480), null);
  const K = mk({}).intrinsicsFor(640, 480); assert.ok(K && K.fx > 0 && /schatting/.test(K.source));
});
det.destroy();
console.log(`${n} tests ok`);
process.exit(0);
