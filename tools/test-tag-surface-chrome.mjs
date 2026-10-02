#!/usr/bin/env node
// Optioneel (vereist puppeteer-core + Chrome; niet in de repo-dependencies; de three.js-CDN uit de importmap moet bereikbaar zijn):
//   python3 -m http.server 8130 &      (in de repo-root)
//   PPTR_DIR=/pad/naar/map-met-node_modules node tools/test-tag-surface-chrome.mjs http://127.0.0.1:8130
// Geen XR in headless Chrome: dit test de integratie (laden zonder fouten, opt-in-gedrag, scanner → TableCalibrator.setFromSurface → placeOnTable, terugval),
// niet de Quest-camera/pose.
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
const require = createRequire(process.env.PPTR_DIR ? process.env.PPTR_DIR.replace(/\/?$/, '/') : import.meta.url);
const puppeteer = require('puppeteer-core');
const base = process.argv[2] || 'http://127.0.0.1:8130';
const b = await puppeteer.launch({ executablePath: process.env.CHROME || '/usr/bin/google-chrome', headless: 'new', args: ['--no-sandbox', '--use-fake-device-for-media-stream', '--use-fake-ui-for-media-stream'] });
let n = 0; const ok = (name) => { n++; console.log('ok', name); };

async function open(query) {
  const p = await b.newPage(), errs = [];
  p.on('pageerror', e => errs.push(e.message)); const failed = []; p.on('response', r => { if (r.status() >= 400) failed.push(r.url()); });
  p.on('console', m => { if (m.type() === 'error' && !/Failed to load resource/.test(m.text())) errs.push('console:' + m.text().slice(0, 160)); });
  p.on('close', () => { const bad = failed.filter(u => !/favicon\.ico/.test(u)); if (bad.length) console.log('   HTTP-fouten:', bad); });
  await p.goto(`${base}/table-ar/index.html${query}`, { waitUntil: 'load' });
  await p.waitForFunction(() => /loaded|failed|unavailable|WebXR/i.test(document.getElementById('status').textContent), { timeout: 60000 });
  return { p, errs };
}

// 1. zonder parameters: ongewijzigd gedrag
{
  const { p, errs } = await open('');
  const r = await p.evaluate(() => ({ hook: typeof window.tableAR, dbg: !!document.getElementById('tag-debug'), scan: document.getElementById('scan')?.disabled, scene: !!document.getElementById('recalibrate') }));
  assert.equal(r.hook, 'undefined', 'geen test-hook zonder ?tags/?debug'); assert.equal(r.dbg, false); assert.equal(r.scan, true, 'scanknop uit tot AR-sessie'); assert.deepEqual(errs, []);
  ok('zonder parameters: laadt zonder fouten, geen debug-paneel, scanknop uit, geen tableAR-hook'); await p.close();
}

// 2. ?tags=1&debug=1
{
  const { p, errs } = await open('?tags=1&debug=1&camoff=-5,2,-5');
  const r = await p.evaluate(async () => {
    const A = window.tableAR, S = await import('/src/tag-surface.js'); const { qaxis, qmul, qrot, worldPointToCam, worldDirToCam, cameraExtrinsics } = S;
    const out = { hasPanel: !!document.getElementById('tag-debug'), cameraInScene: A.scene.children.includes(A.camera), panelMesh: !!A.camera.getObjectByName('tag-debug-overlay') };
    out.startNoAr = null; await A.startTagScan(); out.startNoAr = document.getElementById('status').textContent;
    // synthetische scan: tags plat op de tafel, viewer kijkt 50° omlaag; real-time klok (performance.now) zoals in de app
    const ext = cameraExtrinsics({ side: 'left', offset: [-0.05, 0.02, -0.05] }), K = { fx: 700, fy: 700, cx: 480, cy: 360, source: 'test' };
    const head = { pos: [0, 1.3, 0.1], quat: qmul(qaxis([0, 1, 0], 0), qaxis([1, 0, 0], -50)) };
    const tags = [{ id: 7, c: [-0.25, 0.75, -0.45] }, { id: 42, c: [0.25, 0.75, -0.85] }];
    const det = t => { const c = worldPointToCam(t.c, head, ext), X = worldDirToCam([1, 0, 0], head, ext), Y = worldDirToCam([0, 0, 1], head, ext), Z = worldDirToCam([0, -1, 0], head, ext);
      return { id: t.id, center: { x: K.fx * c[0] / c[2] + K.cx, y: K.fy * c[1] / c[2] + K.cy }, pose: { R: [[X[0], Y[0], Z[0]], [X[1], Y[1], Z[1]], [X[2], Y[2], Z[2]]], t: c } }; };
    A.scanner.start(); out.phase0 = A.scanner.phase;
    const t0 = performance.now();
    await new Promise(res => { const iv = setInterval(() => { const t = performance.now(); A.poseHistory.push(t, head.pos, head.quat);
      A.scanner.feed(tags.map(det), { t, w: 960, h: 720, intrinsics: K, device: 'camera 2 1, facing back' }); A.scanner.tick();
      if (A.scanner.phase !== 'scanning' || t - t0 > 8000) { clearInterval(iv); res(); } }, 33); });
    out.phase1 = A.scanner.phase; out.pending = A.applyPending() && { w: A.applyPending().width, d: A.applyPending().depth };
    out.previewVisible = A.tagView.group.visible;
    // zoals de XR-lus: toepassen op de calibrator (zonder XRFrame: geen anchor)
    const surf = A.applyPending(); A.setPendingSurface(null); A.scanner.markApplied(); A.tagView.hide();
    await A.calibrator.setFromSurface(null, surf, { source: 'apriltag' });
    const e = A.calibrator.root.matrix.elements; out.rootVisible = A.calibrator.root.visible; out.rootPos = [e[12], e[13], e[14]]; out.rootX = [e[0], e[1], e[2]]; out.rootZ = [e[8], e[9], e[10]];
    out.visChildren = A.calibrator.visualRoot.children.length; out.saved = A.calibrator.getSavedCalibration();
    out.details = document.getElementById('details').textContent;
    out.mj = A.sandwichScene?.mujocoRoot ? [A.sandwichScene.mujocoRoot.position.x, A.sandwichScene.mujocoRoot.position.y, A.sandwichScene.mujocoRoot.position.z] : null;
    out.tgt = [surf.width / 2 - 0.10, 0, surf.depth / 2];
    await new Promise(r => setTimeout(r, 600)); out.debugText = document.getElementById('tag-debug').textContent;
    // terugval: time-out → handmatige 3-punts kalibratie start
    A.calibrator.collecting = false; A.scanner.timeoutMs = 300; A.scanner.start(); await new Promise(r => setTimeout(r, 450)); A.scanner.tick();
    out.fallbackPhase = A.scanner.phase; out.fallbackCollecting = A.calibrator.collecting; out.fallbackStatus = document.getElementById('status').textContent;
    return out;
  });
  console.log('   ', JSON.stringify({ phase: [r.phase0, r.phase1], pending: r.pending, rootPos: r.rootPos.map(v => +v.toFixed(3)), mj: r.mj && r.mj.map(v => +v.toFixed(3)), saved: r.saved && r.saved.source, fallback: [r.fallbackPhase, r.fallbackCollecting] }));
  assert.deepEqual(errs, [], 'geen pagina-/consolefouten');
  assert.ok(r.hasPanel && r.cameraInScene && r.panelMesh, '?debug=1: DOM- én 3D-paneel, camera in scene'); ok('?tags=1&debug=1: laadt zonder fouten; debug-paneel (DOM + hoofd-vast 3D) aanwezig');
  assert.match(r.startNoAr, /Start eerst AR/); ok('scan-knop buiten een AR-sessie: duidelijke melding, geen crash');
  assert.equal(r.phase0, 'scanning'); assert.equal(r.phase1, 'stable'); assert.ok(Math.abs(r.pending.w - 0.5) < 0.005 && Math.abs(r.pending.d - 0.4) < 0.005, 'oppervlak 50 × 40 cm'); ok('scanner (real-time klok) wordt stabiel op 50 × 40 cm');
  assert.ok(r.rootVisible); assert.ok(Math.abs(r.rootPos[0] + 0.25) < 0.005 && Math.abs(r.rootPos[1] - 0.75) < 0.005 && Math.abs(r.rootPos[2] + 0.85) < 0.005, 'root = verste-linkerhoek');
  assert.ok(Math.abs(r.rootX[0] - 1) < 1e-6 && Math.abs(r.rootZ[2] - 1) < 1e-6); assert.ok(r.visChildren >= 5, 'oppervlak + rand + assen getekend'); assert.equal(r.saved.source, 'apriltag');
  ok('TableCalibrator.setFromSurface: root-frame, visualisatie (vlak + randstroken + lijn + assen), opslag met source');
  assert.ok(r.mj && r.mj.every((v, i) => Math.abs(v - r.tgt[i]) < 1e-6), 'sandwich-scène geplaatst via placeOnTable(width, depth)'); assert.match(r.details, /AprilTags/); ok('sandwich-scène volgt via onCalibrated → placeOnTable (zelfde pad als de handmatige kalibratie)');
  for (const k of ['TAGS fase=applied', '#7', '#42', 'LINKSONDER', 'RECHTSBOVEN', 'oppervlak: 50.0 × 40.0 cm']) assert.ok(r.debugText.includes(k), 'debug bevat ' + k); ok('?debug=1-overlay toont tagposities, rollen, hoeken en afmetingen');
  assert.equal(r.fallbackPhase, 'failed'); assert.equal(r.fallbackCollecting, true); assert.match(r.fallbackStatus, /handmatige 3-punts|Calibration/); ok('time-out zonder stabiel oppervlak → terugval: handmatige 3-punts kalibratie start (calibrator.collecting)');
  await p.close();
}
console.log(`\n${n} tests ok`); await b.close();
