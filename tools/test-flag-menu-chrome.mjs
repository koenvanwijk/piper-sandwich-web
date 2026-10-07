#!/usr/bin/env node
// Headless Chrome (optioneel; vereist puppeteer-core + Chrome, niet in de repo-dependencies; three.js-CDN moet bereikbaar zijn):
// het flag-menu op de desktop-pagina (zelfde paneel als in VR, hier bediend met toets M + muisklik):
// openen, modus/ghost/debug wisselen (werkt meteen door in teleop/ghost/overlay), bewaard na herladen, URL wint, reset wist, geen JS-fouten.
//   python3 -m http.server 8141 &
//   PPTR_DIR=/pad/met/node_modules node tools/test-flag-menu-chrome.mjs http://127.0.0.1:8141 [screenshot.png]
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
const req = createRequire(process.env.PPTR_DIR ? process.env.PPTR_DIR.replace(/\/?$/, '/') : import.meta.url);
const puppeteer = (await import(pathToFileURL(req.resolve('puppeteer-core')).href)).default;
const base = process.argv[2] || 'http://127.0.0.1:8141', SHOT = process.argv[3];

const b = await puppeteer.launch({ executablePath: process.env.CHROME || '/usr/bin/google-chrome', headless: 'new', protocolTimeout: 240000,
  args: ['--no-sandbox', '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const p = await b.newPage(); await p.setViewport({ width: 1280, height: 800 });
const errs = [];
p.on('pageerror', e => errs.push('pageerror: ' + e.message));
p.on('console', m => { if (m.type() === 'error' && !/favicon|Failed to load resource/.test(m.text())) errs.push('console: ' + m.text()); });
p.on('response', r => { if (r.status() >= 400 && !/favicon/.test(r.url())) errs.push(`HTTP ${r.status()} ${r.url()}`); });
const load = async q => { await p.goto(`${base}/index.html${q}`, { waitUntil: 'domcontentloaded', timeout: 180000 });
  await p.waitForFunction(() => window.sandwichVR && window.sandwichVR._menu, { timeout: 180000 }); };
const flags = () => p.evaluate(() => ({ ...window.sandwichVR.flags.values, src: { ...window.sandwichVR.flags.source } }));
// klik midden-links op regel i van het paneel (projectie van het paneel-lokale punt naar schermpixels)
const clickRow = async label => {
  const xy = await p.evaluate(async label => {
    const m = await import('/src/flag-menu.js'), a = window.sandwichVR, R = m.menuRows(a.flags), i = R.findIndex(r => r.label.startsWith(label));
    const H = m.panelHeight(R.length), P = a._menu.panel; P.updateMatrixWorld();
    const v = P.localToWorld(new (a.camera.position.constructor)(-0.05, H / 2 - m.TITLE_H - (i + 0.5) * m.ROW_H, 0)).project(a.camera);
    const r = a.renderer.domElement.getBoundingClientRect(); return [r.left + (v.x + 1) / 2 * r.width, r.top + (1 - v.y) / 2 * r.height];
  }, label);
  await p.mouse.click(xy[0], xy[1]); await new Promise(r => setTimeout(r, 300));
  if (process.env.DBG) console.log('   klik', label, xy.map(Math.round), JSON.stringify(await p.evaluate(() => window.sandwichVR.flags.values)));
};

await load('');
let f = await flags();
assert.deepEqual([f.mode, f.target, f.debug, f.haptic], ['6dof', 'clutch', false, true], 'standaard');
assert.ok(await p.$('#flags-btn'), 'desktop-knop ⚙ Flags');
await p.keyboard.press('m'); await new Promise(r => setTimeout(r, 300));
assert.equal(await p.evaluate(() => window.sandwichVR._menu.isOpen), true, 'M opent het menu');
await clickRow('Teleop-modus');
await clickRow('Doel-ghost'); await clickRow('Doel-ghost');
await clickRow('Debug-overlay');
await new Promise(r => setTimeout(r, 1500));
f = await flags();
assert.deepEqual([f.mode, f.target, f.debug], ['joints', 'off', true], 'klikken wisselt: ' + JSON.stringify(f));
const live = await p.evaluate(() => { const a = window.sandwichVR; return { tl: a.teleop.left.mode, tr: a.teleop.right.mode, dbg: !!a._dbg && a._dbg.el.style.display !== 'none',
  stored: JSON.parse(localStorage.getItem('piper-sandwich-web.flags.v1')) }; });
assert.deepEqual([live.tl, live.tr, live.dbg], ['joints', 'joints', true], 'werkt meteen door in teleop + debug-overlay');
assert.deepEqual(live.stored, { mode: 'joints', target: 'off', debug: true }, 'bewaard in localStorage');
console.log('ok menu: M opent; klik wisselt modus (6dof→joints), ghost (clutch→altijd→uit), debug aan; meteen actief; in localStorage');
if (SHOT) { await p.evaluate(() => { const d = document.getElementById('dbg-overlay'); if (d) d.style.display = 'none'; if (window.sandwichVR._dbg) window.sandwichVR._dbg.mesh.visible = false; });
  await new Promise(r => setTimeout(r, 1500)); await p.screenshot({ path: SHOT }); console.log('   screenshot:', SHOT);
  await p.evaluate(() => window.sandwichVR.applyFlag('debug', true)); }

await load('');
f = await flags();
assert.deepEqual([f.mode, f.target, f.debug, f.src.mode], ['joints', 'off', true, 'opgeslagen'], 'na herladen bewaard');
assert.equal(await p.evaluate(() => window.sandwichVR.teleop.left.mode), 'joints');
assert.ok(await p.evaluate(() => !!document.getElementById('dbg-overlay')), 'debug-overlay na herladen');
console.log('ok herladen: keuzes bewaard (bron "bewaard"), teleop in joints-modus, debug-overlay actief');

await load('?mode=6dof&target=always');
f = await flags();
assert.deepEqual([f.mode, f.src.mode, f.target, f.src.target, f.debug], ['6dof', 'url', 'always', 'url', true], 'URL wint, rest bewaard');
console.log('ok URL-voorrang: ?mode=6dof&target=always wint van de bewaarde keuze; debug blijft bewaard');

await p.keyboard.press('m'); await new Promise(r => setTimeout(r, 300));
await clickRow('Reset');
f = await flags();
assert.deepEqual([f.mode, f.target, f.debug], ['6dof', 'always', false], 'reset → URL/standaard');
assert.equal(await p.evaluate(() => localStorage.getItem('piper-sandwich-web.flags.v1')), null, 'opslag gewist');
assert.equal(await p.evaluate(() => window.sandwichVR._dbg.el.style.display), 'none', 'debug-overlay weer uit');
await clickRow('Sluiten'); assert.equal(await p.evaluate(() => window.sandwichVR._menu.isOpen), false);
console.log('ok reset wist localStorage (terug naar URL/standaard, debug-overlay uit); "Sluiten" sluit');

// VR-pad (zonder headset): menu.update() met gesimuleerde controllers — rechter straal op de MENU-knop van de linker controller + trigger,
// dan op de regel "Teleop-modus" + trigger. Tijdens clutch (engaged) en met A/B/X/Y gebeurt niets.
const vr = await p.evaluate(async () => {
  const a = window.sandwichVR, M = a._menu, V = a.camera.position.constructor, Q = a.camera.quaternion.constructor, m = await import('/src/flag-menu.js');
  const head = { pos: new V(0, 1.6, 0), quat: new Q() }, L = { origin: new V(-0.2, 1.2, -0.3), dir: new V(0, 0, -1), trigger: 0, engaged: false, gripPos: new V(-0.2, 1.2, -0.3), gripQuat: new Q() };
  let t = 10000; const out = {};
  const aimR = (target, trigger, engaged = false) => { const o = new V(0.25, 1.25, -0.1); return { origin: o, dir: target.clone().sub(o).normalize(), trigger, engaged }; };
  const step = R => { t += 400; M.update({ left: L, right: R }, head, t); };
  step(aimR(new V(0, 0, -5), 0)); const gear = M.gear.position.clone(); out.gearVisible = M.gear.visible;
  step(aimR(gear, 1, true)); step(aimR(gear, 0, true)); out.openWhileClutch = M.isOpen;
  step(aimR(gear, 0)); step(aimR(gear, 1)); out.openAfterClick = M.isOpen; step(aimR(gear, 0));
  out.rayVisible = M.rays.right.visible;
  const R = m.menuRows(a.flags), H = m.panelHeight(R.length); M.panel.updateMatrixWorld();
  const row0 = M.panel.localToWorld(new V(-0.05, H / 2 - m.TITLE_H - 0.5 * m.ROW_H, 0));
  const before = a.flags.get('mode'); step(aimR(row0, 0)); step(aimR(row0, 1)); out.mode = [before, a.flags.get('mode'), a.teleop.right.mode];
  step(aimR(row0, 0)); step(aimR(row0, 1, true)); out.modeAfterClutchPress = a.flags.get('mode');
  return out;
});
assert.deepEqual(vr, { gearVisible: true, openWhileClutch: false, openAfterClick: true, rayVisible: true, mode: ['6dof', 'joints', 'joints'], modeAfterClutchPress: 'joints' }, JSON.stringify(vr));
console.log('ok VR-pad (gesimuleerde controllers): MENU-knop op linker controller + rechter straal/trigger opent; tijdens clutch niet; regel-klik wisselt modus live');
await p.evaluate(() => window.sandwichVR.flags.reset());

assert.deepEqual(errs, [], 'geen JS-fouten'); await b.close();
console.log('ok geen JS-fouten');
