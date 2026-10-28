#!/usr/bin/env node
// Headless "snap"-meting: zet een object (brood / mes / boter) tussen de open rechter vingers, sluit de gripper
// in één frame (zoals een trekker-klik in VR) en meet de piek-snelheid van het object (m/s) en hoe ver het wegschiet.
// Vóór de fix: ongelimiteerde vingerkracht (forcelimited=false, kp 10000, doel tot 0,0475 m > slag 0,035 m) → honderden N
// knijpkracht op een 30 g boterham → wegschieten. Gebruik: node tools/test-grab-snap.mjs [--xml=pad] [--json]
import fs from 'fs';
import { createHeadlessSim } from './headless-sim.mjs';
const args = process.argv.slice(2), arg = (k, d) => { const a = args.find(x => x.startsWith('--' + k + '=')); return a ? a.split('=').slice(1).join('=') : d; };
const xml = arg('xml', null);
const results = [];
for (const obj of ['bread1', 'knife', 'butter0', 'cheese9']) {
  const env = await createHeadlessSim({ sceneXml: xml ? fs.readFileSync(xml, 'utf8') : null });
  const { mujoco: mj, model, data } = env;
  const bid = n => mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, n);
  const B = bid(obj), F7 = bid('right_link7'), F8 = bid('right_link8');
  const act = n => mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR.value, n);
  const a7 = act('right_joint7'), a8 = act('right_joint8');
  data.ctrl[a7] = 0.035; data.ctrl[a8] = -0.035;
  for (let i = 0; i < 500; i++) mj.mj_step(model, data);
  // object op het midden tussen de vingertoppen zetten, zwaartekracht-vrij gehouden door het eerst stil te zetten
  const mid = [0,1,2].map(k => (data.xpos[3*F7+k] + data.xpos[3*F8+k]) / 2);
  const jadr = model.body_jntadr[B], qa = model.jnt_qposadr[jadr], va = model.jnt_dofadr[jadr];
  data.qpos[qa] = mid[0]; data.qpos[qa+1] = mid[1]; data.qpos[qa+2] = mid[2];
  // oriënteer het object met de lange as langs de gripper-y (knijp over de korte kant)
  for (let k = 0; k < 6; k++) data.qvel[va+k] = 0;
  mj.mj_forward(model, data);
  const p0 = [data.qpos[qa], data.qpos[qa+1], data.qpos[qa+2]];
  model.opt.gravity[2] = 0;                      // zonder zwaartekracht: alleen de knijp-impuls telt, geen val
  data.ctrl[a7] = 0; data.ctrl[a8] = 0;         // trekker in één klap dicht
  let vmax = 0, wmax = 0;
  for (let i = 0; i < 1000; i++) { mj.mj_step(model, data);
    const v = Math.hypot(data.qvel[va], data.qvel[va+1], data.qvel[va+2]); if (v > vmax) vmax = v;
    const w = Math.hypot(data.qvel[va+3], data.qvel[va+4], data.qvel[va+5]); if (w > wmax) wmax = w; }
  const d = Math.hypot(data.qpos[qa]-p0[0], data.qpos[qa+1]-p0[1], data.qpos[qa+2]-p0[2]);
  results.push({ obj, vmax: +vmax.toFixed(2), wmax: +wmax.toFixed(1), moved_mm: +(d*1000).toFixed(0) });
}
if (args.includes('--json')) console.log(JSON.stringify(results));
else for (const r of results) console.log(`${r.obj.padEnd(8)} piek ${r.vmax} m/s, ${r.wmax} rad/s, verplaatst ${r.moved_mm} mm (2 s)`);
