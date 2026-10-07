#!/usr/bin/env node
// Headless Chrome (optioneel; vereist puppeteer-core + Chrome, niet in de repo-dependencies; three.js-CDN moet bereikbaar zijn):
// laadt index.html, stuurt beide armen met gesimuleerde controllers (readControllers wordt vervangen) en controleert de doel-ghost:
// links een bereikbaar doel (groen), rechts een onbereikbaar doel (rood); geen JS-fouten; ?target=0 = geen ghost. Optioneel screenshot.
//   python3 -m http.server 8141 &
//   PPTR_DIR=/pad/met/node_modules node tools/test-target-ghost-chrome.mjs http://127.0.0.1:8141 [screenshot.png]
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
const req = createRequire(process.env.PPTR_DIR ? process.env.PPTR_DIR.replace(/\/?$/, '/') : import.meta.url);
const puppeteer = (await import(pathToFileURL(req.resolve('puppeteer-core')).href)).default;
const base = process.argv[2] || 'http://127.0.0.1:8141', SHOT = process.argv[3];

async function page(query) {
  const b = await puppeteer.launch({ executablePath: process.env.CHROME || '/usr/bin/google-chrome', headless: 'new', protocolTimeout: 240000,
    args: ['--no-sandbox', '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--window-size=1280,800'] });
  const p = await b.newPage(); await p.setViewport({ width: 1280, height: 800 });
  const errs = [];
  p.on('pageerror', e => errs.push('pageerror: ' + e.message));
  p.on('console', m => { if (m.type() === 'error' && !/favicon/.test(m.text()) && !/Failed to load resource/.test(m.text())) errs.push('console: ' + m.text()); });
  p.on('response', r => { if (r.status() >= 400 && !/favicon/.test(r.url())) errs.push(`HTTP ${r.status()} ${r.url()}`); });
  await p.goto(`${base}/index.html${query}`, { waitUntil: 'domcontentloaded', timeout: 180000 });
  await p.waitForFunction(() => window.sandwichVR && window.sandwichVR.ik && window.sandwichVR.ik.right, { timeout: 180000 });
  return { b, p, errs };
}
// Gesimuleerde controllers in het scène-root-frame (three, y omhoog): links 4 cm + 15° draaien (bereikbaar), rechts 55 cm vooruit + omhoog.
const drive = () => {
  const app = window.sandwichVR, t0 = performance.now();
  const L0 = [0.0, 0.25, -0.22], R0 = [0.0, 0.25, 0.22];
  app.readControllers = () => {
    const a = Math.min(1, Math.max(0, (performance.now() - t0 - 500) / 1500)), h = 15 * Math.PI / 180 * a / 2;
    return { left: { pos: [L0[0] + 0.04 * a, L0[1] + 0.03 * a, L0[2]], quat: [Math.cos(h), 0, Math.sin(h), 0], trigger: 0, grip: 1 },
             right: { pos: [R0[0] + 0.55 * a, R0[1] + 0.25 * a, R0[2]], quat: [1, 0, 0, 0], trigger: 0, grip: 1 } };
  };
};
const state = () => { const app = window.sandwichVR, g = app._ghost;
  return { hasGhost: !!g, vis: g ? { left: g.arms.left.g.visible, right: g.arms.right.g.visible } : null,
    status: app._tinfo ? { left: app._tinfo.left && app._tinfo.left.status, right: app._tinfo.right && app._tinfo.right.status } : null,
    err: app._tinfo ? ['left', 'right'].map(s => app._tinfo[s] && [+app._tinfo[s].pos.toFixed(1), +app._tinfo[s].rot.toFixed(1)]) : null,
    calls: app.renderer.info.render.calls }; };

{
  const { b, p, errs } = await page('?debug=1');
  const before = await p.evaluate(state);
  assert.ok(before.hasGhost && !before.vis.left && !before.vis.right, 'ghost bestaat, onzichtbaar zonder clutch');
  await p.evaluate(drive);
  // headless (SwiftShader) haalt maar enkele fps en de sim-stap per frame is begrensd (50 ms) → wacht tot de arm is uitgeregeld (max 60 s)
  await p.waitForFunction(() => { const i = window.sandwichVR._tinfo; return i && i.left && i.left.status === 'ok' && i.right && i.right.status === 'bad'; }, { timeout: 60000, polling: 500 }).catch(() => {});
  const s = await p.evaluate(state);
  console.log('   status:', JSON.stringify(s));
  assert.ok(s.vis.left && s.vis.right, 'ghost zichtbaar tijdens clutch');
  assert.equal(s.status.left, 'ok', 'links bereikbaar = groen'); assert.equal(s.status.right, 'bad', 'rechts onbereikbaar = rood');
  if (SHOT) {
    // overzicht (beide ghosts in beeld) + close-up van de linker (groene) ghost; debugpaneel verborgen
    const aim = (who, dist) => p.evaluate((who, dist) => { const a = window.sandwichVR, c = a.camera, T = a.controls.target.constructor;
      document.getElementById('dbg-overlay')?.remove(); if (a._dbg) a._dbg.mesh.visible = false;
      const w = s => a._ghost.arms[s].g.getWorldPosition(new T());
      const L = w('left'), R = w('right'), tgt = who === 'both' ? L.clone().add(R).multiplyScalar(0.5) : w(who);
      a.controls.enabled = false; a.controls.target.copy(tgt); c.position.set(tgt.x + 0.75 * dist, tgt.y + 0.45 * dist, tgt.z + 0.75 * dist); c.lookAt(tgt); c.updateMatrixWorld(); }, who, dist);
    await aim('both', 1.1); await new Promise(r => setTimeout(r, 2500)); await p.screenshot({ path: SHOT }); console.log('   screenshot:', SHOT);
    const close = SHOT.replace(/\.png$/, '-close.png');
    await aim('left', 0.25); await new Promise(r => setTimeout(r, 2500)); await p.screenshot({ path: close }); console.log('   screenshot:', close);
  }
  assert.deepEqual(errs, [], 'geen JS-fouten'); await b.close();
  console.log('ok doel-ghost in de browser: groen (bereikbaar) / rood (onbereikbaar), geen JS-fouten');
}
{
  const { b, p, errs } = await page('?target=0');
  await p.evaluate(drive); await new Promise(r => setTimeout(r, 2500));
  const s = await p.evaluate(state); assert.ok(!s.hasGhost, '?target=0: geen ghost'); assert.deepEqual(errs, []); await b.close();
  console.log('ok ?target=0: geen ghost, geen JS-fouten');
}
