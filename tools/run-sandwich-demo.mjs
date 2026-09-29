#!/usr/bin/env node
// Headless test: speelt de choreografie uit src/sandwich-motion.js af in de echte
// MuJoCo-simulatie (WASM, zonder three.js) en logt per stap:
//   - max TCP-positiefout (doelpositie van de choreografie vs. echte TCP), per arm
//   - mes: min/max z (optillen?) en of het mes met de TCP meebeweegt
//   - verplaatsing van bread0 en van de boter-blokjes (gemiddeld/max, in mm)
//   - na afloop: ligt bread1 op bread0 (xy binnen 2 cm, z hoger)?
// Gebruik: node tools/run-sandwich-demo.mjs [--json] [--speed=1] [--tuning='{"PUSH_END_DY":-0.05}'] [--require-top]
import { createHeadlessSim } from './headless-sim.mjs';
import { MotionPlayer, makeSandwichChoreography, readObjectPositions, GEOM } from '../src/sandwich-motion.js';

const args = process.argv.slice(2);
const asJson = args.includes('--json');
const speed = Number((args.find(a => a.startsWith('--speed=')) || '--speed=1').split('=')[1]);

const env = await createHeadlessSim();
const { mujoco, model, data } = env;
const FRAME = 0.05;                                   // zoals app.js: 25 x 0.002 s

// Eerst laten we het tafereel 1 s tot rust komen (mes zakt van 3.1 cm op de tafel).
env.stepPhysics(2.0);
const objects = readObjectPositions(mujoco, model, data);
const tuning = JSON.parse((args.find(a => a.startsWith('--tuning=')) || '--tuning={}').slice(9));
const steps = makeSandwichChoreography({ objects, tuning });

const butterNames = []; for (let i = 0; i < 14; i++) butterNames.push('butter' + i);
const snap = () => ({
  bread0: env.bodyPos('bread0'), bread1: env.bodyPos('bread1'), knife: env.bodyPos('knife'),
  butter: butterNames.map(n => env.bodyPos(n)),
});
const dist = (a, b) => Math.hypot(a[0]-b[0], a[1]-b[1], a[2]-b[2]);
const f = (v, n = 1) => (v * 1000).toFixed(n);

const rows = [];
let cur = null, prev = snap();
const initial = snap();
const player = new MotionPlayer(env, steps, {
  ikIters: 6,
  onStep: (name, i) => {
    cur = { name, i, t0: data.time, errMax: { left: 0, right: 0 }, kzMin: 9, kzMax: -9,
            start: snap(), knifeHeld: false, kzAtEnd: 0 };
    rows.push(cur);
    if (!asJson) console.log(`\n▶ stap ${i}: ${name}`);
  },
});

const t0 = performance.now();
let guard = 0;
while (!player.done && guard++ < 20000) {
  player.update(FRAME * speed);
  env.stepPhysics(FRAME);
  if (!cur) continue;
  // Let op: na update() kan de speler al naar de volgende stap zijn gesprongen; we meten
  // met de actuele 'cmd' (doelpose) tegen de echte TCP.
  for (const s of ['left', 'right']) {
    const c = player.cmd && player.cmd[s]; if (!c) continue;
    const e = dist(c.pos, env.tcpPose(s).pos);
    if (e > cur.errMax[s]) cur.errMax[s] = e;
  }
  const kz = env.bodyPos('knife')[2];
  cur.kzMin = Math.min(cur.kzMin, kz); cur.kzMax = Math.max(cur.kzMax, kz);
  cur.end = snap();
}
// laat alles nog 1.5 s uitrollen (bread1 valt / mes komt tot rust)
env.stepPhysics(1.5);
const final = snap();
rows[rows.length - 1].end = final;

const out = [];
for (const r of rows) {
  const s = r.start, e = r.end || final;
  const bd = s.butter.map((p, k) => dist(p, e.butter[k]));
  const row = {
    step: r.name,
    tcpErrMaxMm: { left: +f(r.errMax.left), right: +f(r.errMax.right) },
    knifeZ: { minMm: +f(r.kzMin), maxMm: +f(r.kzMax), liftedMm: +f(r.kzMax - s.knife[2]) },
    knifeMovedMm: +f(dist(s.knife, e.knife)),
    bread0MovedMm: +f(dist(s.bread0, e.bread0)),
    butterMeanMovedMm: +f(bd.reduce((a, b) => a + b, 0) / bd.length),
    butterMaxMovedMm: +f(Math.max(...bd)),
    bread1MovedMm: +f(dist(s.bread1, e.bread1)),
  };
  out.push(row);
}
const b0 = final.bread0, b1 = final.bread1;
const dxy = Math.hypot(b1[0]-b0[0], b1[1]-b0[1]);
const closed = { bread1: b1.map(x => +x.toFixed(4)), bread0: b0.map(x => +x.toFixed(4)),
  dxyMm: +f(dxy), dzMm: +f(b1[2]-b0[2]), onTop: dxy < 0.02 && b1[2] > b0[2] + 0.005 };
// Geslaagd = dichtgeschoven: zij aan zij, zelfde hoogte, randen <= 2 cm uit elkaar
// (begin: 30 mm tussenruimte; middelpuntafstand 90 mm = randen raken; boterblokjes zitten er nog tussen)
closed.gapMm = +f(dxy - 2 * GEOM.bread.half[0]);
closed.adjacent = dxy < 0.112 && Math.abs(b1[2]-b0[2]) < 0.005;
// Boter: hoeveel blokjes liggen na afloop binnen de bread0-omtrek, en hoe breed is de verdeling
const insideOf = c => p => Math.abs(p[0]-c[0]) < 0.045 && Math.abs(p[1]-c[1]) < 0.045 && p[2] > 0.02;
const spreadStd = pts => { const mx = pts.reduce((a,p)=>a+p[0],0)/pts.length, my = pts.reduce((a,p)=>a+p[1],0)/pts.length;
  return Math.sqrt(pts.reduce((a,p)=>a+(p[0]-mx)**2+(p[1]-my)**2,0)/pts.length); };
const butter = { onBread0Before: initial.butter.filter(insideOf(initial.bread0)).length, onBread0After: final.butter.filter(insideOf(final.bread0)).length,
  total: final.butter.length, stdBeforeMm: +f(spreadStd(initial.butter)), stdAfterMm: +f(spreadStd(final.butter)) };
const knifeEnd = final.knife, plate = objects.plate;
const knifeOnPlate = { knife: knifeEnd.map(x => +x.toFixed(4)),
  dxyToPlateMm: +f(Math.hypot(knifeEnd[0]-plate[0], knifeEnd[1]-plate[1])) };
knifeOnPlate.onPlate = knifeOnPlate.dxyToPlateMm < 90 && knifeEnd[2] < 0.03;   // plate-straal 9 cm

if (asJson) console.log(JSON.stringify({ steps: out, closed, butter, knifeOnPlate }, null, 2));
else {
  console.log('\n=== Resultaat per stap ===');
  for (const r of out) {
    console.log(`\n${r.step}`);
    console.log(`  max TCP-fout   : links ${r.tcpErrMaxMm.left} mm, rechts ${r.tcpErrMaxMm.right} mm`);
    console.log(`  mes z (mm)     : min ${r.knifeZ.minMm}, max ${r.knifeZ.maxMm}, opgetild ${r.knifeZ.liftedMm} mm; verplaatst ${r.knifeMovedMm} mm`);
    console.log(`  bread0 verpl.  : ${r.bread0MovedMm} mm; boter gem. ${r.butterMeanMovedMm} mm, max ${r.butterMaxMovedMm} mm; bread1 verpl. ${r.bread1MovedMm} mm`);
  }
  console.log('\n=== Einde ===');
  console.log(`bread1 ${JSON.stringify(closed.bread1)}  bread0 ${JSON.stringify(closed.bread0)}`);
  console.log(`bread1 t.o.v. bread0: dxy ${closed.dxyMm} mm, dz ${closed.dzMm} mm (tussenruimte ${closed.gapMm} mm, begin 30 mm) -> ${closed.adjacent ? 'ZIJ AAN ZIJ (dichtgeschoven) ✔' : 'niet dichtgeschoven ✘'}; ${closed.onTop ? 'ligt OP bread0 ✔' : 'ligt NIET bovenop bread0 (bekende beperking)'}`);
  console.log(`boter: ${butter.onBread0Before}/${butter.total} blokjes op bread0 vooraf, ${butter.onBread0After}/${butter.total} achteraf; spreiding (std xy) ${butter.stdBeforeMm} -> ${butter.stdAfterMm} mm`);
  console.log(`mes eind ${JSON.stringify(knifeOnPlate.knife)}; afstand xy tot bord-midden ${knifeOnPlate.dxyToPlateMm} mm -> ${knifeOnPlate.onPlate ? 'ligt op het bord ✔' : 'ligt NIET op het bord ✘'}`);
  console.log(`(simulatietijd ${data.time.toFixed(1)} s, rekentijd ${((performance.now()-t0)/1000).toFixed(1)} s)`);
}
// exit 0 = dichtgeschoven (of, met --require-top, echt bovenop); 2 = niet gelukt
process.exit((args.includes('--require-top') ? closed.onTop : closed.adjacent) ? 0 : 2);
