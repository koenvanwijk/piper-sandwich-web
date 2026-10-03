#!/usr/bin/env node
// Optionele headless-meting (puppeteer-core + Chrome + three-CDN): vergelijkt de AR-pagina met ?perf=0 (oud) en de standaard (geoptimaliseerd).
// LET OP: headless Chrome gebruikt software-GL (SwiftShader), geen Quest-GPU, geen stereo en geen XR-compositor. De absolute tijden zeggen niets over de Quest;
// draw calls / driehoeken / materialen / shader-programma's / hoofdthread-tijd zijn wél vergelijkbaar. De framebufferscale (0.8² = 64 % pixels) wordt hier met pixelRatio nagebootst.
//   python3 -m http.server 8140 &   PPTR_DIR=/pad/met/node_modules node tools/measure-ar-perf.mjs http://127.0.0.1:8140
import { createRequire } from 'node:module';
const require = createRequire(process.env.PPTR_DIR ? process.env.PPTR_DIR.replace(/\/?$/, '/') : import.meta.url);
const puppeteer = require('puppeteer-core');
const base = process.argv[2] || 'http://127.0.0.1:8140', N = +(process.argv[3] || 40);
const launch = () => puppeteer.launch({ executablePath: process.env.CHROME || '/usr/bin/google-chrome', headless: 'new', args: ['--no-sandbox'], protocolTimeout: 600000 });

let shot = 0;
async function measure(query, pixelScale) {
  const b = await launch();   // één browser per variant (schone staat)
  const p = await b.newPage(); await p.setViewport({ width: 1280, height: 720 }); const errs = []; p.on('pageerror', e => errs.push(e.message));
  await p.goto(`${base}/table-ar/index.html?x=1${query}`, { waitUntil: 'domcontentloaded', timeout: 120000 });
  await p.waitForFunction(() => /Pipers \+ bread/.test(document.getElementById('status').textContent) || /failed/i.test(document.getElementById('status').textContent), { timeout: 90000 });
  const r = await p.evaluate(async (N, pixelScale) => {
    const A = window.tableAR, R = A.renderer, gl = R.getContext();
    await A.calibrator.setFromSurface(null, { width: 0.8, depth: 0.6, frame: { matrix: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1] } });
    A.scene.add(A.camera); A.camera.position.set(0.0, 0.7, 1.0); A.camera.lookAt(0.0, 0.15, 0.3); A.camera.updateMatrixWorld(true);
    R.setAnimationLoop(null); R.setPixelRatio(pixelScale); R.setSize(1280, 720, false);
    const frames = [], render = () => { const t0 = performance.now(); R.render(A.scene, A.camera); const t1 = performance.now(); gl.finish(); return [t1 - t0, performance.now() - t0]; };
    for (let i = 0; i < 5; i++) render();
    for (let i = 0; i < N; i++) frames.push(render());
    const med = a => [...a].sort((x, y) => x - y)[a.length >> 1];
    const info = R.info;
    return { calls: info.render.calls, triangles: info.render.triangles, geometries: info.memory.geometries, textures: info.memory.textures, programs: info.programs?.length,
      cpuMs: med(frames.map(f => f[0])), totalMs: med(frames.map(f => f[1])), perf: A.perfStats, fbScale: A.PERF.fbScale, std: A.PERF.standardMaterials, px: Math.round(1280 * pixelScale) + '×' + Math.round(720 * pixelScale) };
  }, N, pixelScale);
  if (process.env.SHOTS) {
  await p.evaluate(() => { document.getElementById('hud').style.display = 'none'; window.tableAR.renderer.render(window.tableAR.scene, window.tableAR.camera); });
  await p.screenshot({ path: `${SHOTS}-${shot++}.png` });
  }
  r.errs = errs; await b.close(); return r;
}
const SHOTS = process.env.SHOTS;
const rows = [['oud (?perf=0), 100 % pixels', '&perf=0', 1], ['alleen tendon-fix (?mat=physical&freeze=0&lod=0&fbscale=1)', '&perf=1&mat=physical&freeze=0&lod=0&fbscale=1', 1], ['+ materialen/opruiming, zonder decimatie (?lod=0&fbscale=1)', '&perf=1&lod=0&fbscale=1', 1], ['standaard (lod 12000, fbscale 0.8 → 64 % pixels)', '&perf=1', 0.8], ['standaard + ?aa=0', '&perf=1&aa=0', 0.8]];
const out = [];
const ONLY = process.env.ROWS?.split(',').map(Number);
for (const [name, q, ps] of rows.filter((_, i) => !ONLY || ONLY.includes(i))) { console.error('meting:', name, new Date().toISOString()); const r = await measure(q, ps); out.push({ name, ...r }); }
console.log(JSON.stringify(out.map(o => ({ variant: o.name, px: o.px, drawCalls: o.calls, triangles: o.triangles, geometries: o.geometries, programs: o.programs, 'cpu ms/frame': +o.cpuMs.toFixed(1), 'cpu+gpu(swiftshader) ms/frame': +o.totalMs.toFixed(1), opt: o.perf, errs: o.errs })), null, 1));
