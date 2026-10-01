// Optioneel (vereist puppeteer-core + Chrome; niet in de repo-dependencies):
//   node tools/make-tag-video.mjs /tmp/tags.y4m /tmp/tags-truth.json
//   python3 -m http.server 8121 import puppeteer from 'puppeteer-core';   node tools/test-apriltag-chrome.mjs http://127.0.0.1:8121 /tmp/tags.y4m /tmp/tags-truth.json '&tagsize=0.09&hfov=62'
import puppeteer from 'puppeteer-core';
import fs from 'node:fs';
const [base, video, truthFile, query = ''] = process.argv.slice(2);
const T = JSON.parse(fs.readFileSync(truthFile, 'utf8'));
const b = await puppeteer.launch({ executablePath: process.env.CHROME || '/usr/bin/google-chrome', headless: 'new',
  args: ['--no-sandbox', '--use-fake-device-for-media-stream', '--use-fake-ui-for-media-stream', `--use-file-for-fake-video-capture=${video}`] });
const p = await b.newPage(); const errs = [];
p.on('pageerror', e => errs.push(e.message)); p.on('console', m => { if (m.type() === 'error') errs.push('console:' + m.text().slice(0, 150)); });
await p.goto(`${base}/table-ar/tags.html?autostart=1&proc=640${query}`, { waitUntil: 'load' });
await p.waitForFunction(() => window.tagTest && window.tagTest.detections.length >= 3, { timeout: 60000 }).catch(() => {});
await new Promise(r => setTimeout(r, 3000));
const r = await p.evaluate(() => ({ dets: window.tagTest.detections, status: document.getElementById('status').textContent, hud: document.getElementById('hudline').textContent,
  info: document.getElementById('info').textContent, cam: window.tagCam?.device?.label }));
const d2 = (a, c) => Math.hypot(a.x - c.x, a.y - c.y);
let worst = 0; const ids = r.dets.map(d => d.id).sort((a, c) => a - c);
for (const t of T.truth) { const d = r.dets.find(x => x.id === t.id); if (!d) continue; t.corners.forEach((c, i) => { worst = Math.max(worst, d2(c, d.corners[i])); }); }
let poseErr = null; const d7 = r.dets.find(x => x.id === 7), t7 = T.truth.find(x => x.id === 7);
if (d7?.pose) poseErr = Math.hypot(...d7.pose.t.map((v, i) => v - t7.t[i]));
console.log(JSON.stringify({ ids, expected: T.truth.map(t => t.id).sort((a, c) => a - c), worstCornerPx: +worst.toFixed(3), poseErrM_id7: poseErr == null ? null : +poseErr.toFixed(4),
  camLabel: r.cam, status: r.status, hud: r.hud, hasPose: !!d7?.pose, errs }, null, 1));
await p.screenshot({ path: '/tmp/tags-shot.png' });
await b.close();
