#!/usr/bin/env node
// Kaasstapel in de web-scène: laadt, ligt na 3 s stil (geen jitter/wegschieten), en kost weinig rekentijd per stap.
import { createHeadlessSim } from './headless-sim.mjs';
const env = await createHeadlessSim(); const { mujoco: mj, model, data } = env;
const ids = [...Array(10)].map((_, i) => mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, 'cheese' + i));
if (ids.some(i => i < 0)) { console.error('FAIL: cheese0..9 ontbreken'); process.exit(1); }
const p0 = ids.map(b => [data.xpos[3*b], data.xpos[3*b+1], data.xpos[3*b+2]]);
mj.mj_forward(model, data); const p1 = ids.map(b => [data.xpos[3*b], data.xpos[3*b+1], data.xpos[3*b+2]]);
const t0 = performance.now(); for (let i = 0; i < 1500; i++) mj.mj_step(model, data); const ms = (performance.now() - t0) / 1500;
let dmax = 0, vmax = 0;
ids.forEach((b, i) => { const d = Math.hypot(data.xpos[3*b]-p1[i][0], data.xpos[3*b+1]-p1[i][1]); dmax = Math.max(dmax, d);
  const va = model.jnt_dofadr[model.body_jntadr[b]]; vmax = Math.max(vmax, Math.hypot(data.qvel[va], data.qvel[va+1], data.qvel[va+2])); });
const ztop = data.xpos[3*ids[9]+2];
console.log(`kaas: max xy-drift ${(dmax*1000).toFixed(2)} mm, eindsnelheid ${(vmax*1000).toFixed(2)} mm/s, top z ${(ztop*1000).toFixed(1)} mm, ${(ms*1000).toFixed(0)} µs/stap`);
if (dmax > 0.002 || vmax > 0.005 || ztop < 0.017) { console.error('FAIL'); process.exit(1); } console.log('OK');
