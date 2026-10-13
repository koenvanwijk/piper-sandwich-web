#!/usr/bin/env node
// Headless end-to-end (optioneel; puppeteer-core + Chrome, niet in de repo-dependencies; three.js-CDN nodig):
// record.html -> verbindingstest (fout token, goed token) -> "Start opname-sessie" -> index.html?rec=… -> S (start) ->
// K (geslaagd + stop) -> episode_saved van de echte Python-server. Geen HF-upload.
//   PIPER_WS_TOKEN=test piper-sim-recorder serve --root /tmp/ds --port 8765 --cams front:160x120 \
//       --origin http://127.0.0.1:8150 --origin https://koenvanwijk.github.io &
//   python3 -m http.server 8150 &
//   PIPER_WS_TOKEN=test PPTR_DIR=/pad/met/node_modules node tools/test-record-e2e-chrome.mjs http://127.0.0.1:8150 ws://127.0.0.1:8765/ws
//   # alleen de sim (bv. de live site, record.html nog niet live):  … <base> <ws> --index-only
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
const req = createRequire(process.env.PPTR_DIR ? process.env.PPTR_DIR.replace(/\/?$/, '/') : import.meta.url);
const puppeteer = (await import(pathToFileURL(req.resolve('puppeteer-core')).href)).default;
const [base = 'http://127.0.0.1:8150', WSURL = 'ws://127.0.0.1:8765/ws'] = process.argv.slice(2).filter(a => !a.startsWith('--'));
const INDEX_ONLY = process.argv.includes('--index-only'), TOKEN = process.env.PIPER_WS_TOKEN;
assert.ok(TOKEN, 'PIPER_WS_TOKEN zetten');
const sleep = ms => new Promise(r => setTimeout(r, ms));

const b = await puppeteer.launch({ executablePath: process.env.CHROME || '/usr/bin/google-chrome', headless: 'new', protocolTimeout: 240000,
  args: ['--no-sandbox', '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', ...(process.env.CHROME_ARGS ? process.env.CHROME_ARGS.split(' ') : [])] });
const p = await b.newPage(); await p.setViewport({ width: 1000, height: 700 });
const errs = [], urls = [];
p.on('pageerror', e => errs.push('pageerror: ' + e.message));
p.on('console', m => { if (m.type() === 'error' && !/favicon|Failed to load resource/.test(m.text())) errs.push('console: ' + m.text()); });
p.on('request', r => urls.push(r.url()));
const status = () => p.$eval('#status', e => [e.className, e.textContent]);
const waitStatus = async () => { await p.waitForFunction(() => !/busy/.test(document.getElementById('status').className), { timeout: 20000 }); return status(); };

if (!INDEX_ONLY) {
  // 1) instel-link met FOUT token: velden gevuld, fragment meteen weg uit de adresbalk
  await p.goto(`${base}/record.html#` + new URLSearchParams({ rec: WSURL, token: 'fout-token', cams: 'front', camsize: '160x120' }), { waitUntil: 'networkidle0' });
  assert.equal(new URL(p.url()).hash, '', 'fragment uit de adresbalk');
  assert.equal(await p.$eval('#url', e => e.value), WSURL);
  await p.click('#test'); let [cls, msg] = await waitStatus();
  assert.equal(cls, 'err'); assert.match(msg, /token klopt niet/); console.log('  fout token :', msg);
  // 2) goed token
  await p.$eval('#token', (e, t) => { e.value = t; }, TOKEN);
  await p.click('#test'); [cls, msg] = await waitStatus();
  assert.equal(cls, 'ok', msg); console.log('  goed token :', msg);
  // 3) herladen zonder fragment: alles uit localStorage
  await p.goto(`${base}/record.html`, { waitUntil: 'networkidle0' });
  assert.deepEqual(await p.evaluate(() => ['url', 'token', 'cams', 'camsize'].map(i => document.getElementById(i).value)), [WSURL, TOKEN, 'front', '160x120']);
  // 4) slechte server-URL: mixed-content-melding
  await p.$eval('#url', e => { e.value = 'ws://192.168.1.9:8765/ws'; }); await p.click('#test');
  [cls, msg] = await status(); assert.match(msg, /mixed content/); console.log('  ws:// LAN  :', msg.slice(0, 70) + '…');
  await p.$eval('#url', (e, u) => { e.value = u; }, WSURL);
  // 5) start
  await Promise.all([p.waitForNavigation({ timeout: 60000 }), p.click('#start')]);
} else {
  await p.goto(`${base}/index.html?` + new URLSearchParams({ rec: WSURL, cams: 'front', camsize: '160x120' }) + '#' + new URLSearchParams({ token: TOKEN }), { timeout: 120000 });
}
const loc = new URL(p.url());
console.log('  sim-URL    :', loc.origin + loc.pathname + loc.search);
assert.equal(loc.pathname.replace(/.*\//, ''), 'index.html'); assert.equal(loc.searchParams.get('rec'), WSURL);
assert.ok(!loc.search.includes(TOKEN), 'token niet in de query');
await p.waitForFunction(() => window.sandwichVR && window.sandwichVR.rec && window.sandwichVR.rec.client && window.sandwichVR.rec.client.ready, { timeout: 180000 });
assert.ok(!p.url().includes(TOKEN), 'token uit de adresbalk (replaceState)');
console.log('  verbonden  : client ready');
await sleep(1500);
await p.keyboard.press('s'); await sleep(4000);
assert.equal(await p.evaluate(() => window.sandwichVR.rec.ctl.state), 'RECORDING');
await p.keyboard.press('k');
await p.waitForFunction(() => window.sandwichVR.rec.ctl.saved >= 1, { timeout: 120000 });
const snap = await p.evaluate(() => window.sandwichVR.rec.ctl.snapshot());
console.log('  episode    :', JSON.stringify({ state: snap.state, saved: snap.saved, last: snap.last }));
assert.ok(!urls.some(u => u.includes(TOKEN)), 'token in geen enkele HTTP-request-URL');
assert.deepEqual(errs, [], errs.join('\n'));
await b.close();
console.log('test-record-e2e-chrome: OK');
