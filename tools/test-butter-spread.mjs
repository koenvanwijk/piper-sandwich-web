#!/usr/bin/env node
// Unit-tests (Node) voor de smeerlaag (src/butter-spread.js): opnemen bij druk, smeren alleen dicht boven het brood en in beweging,
// voorraad is eindig (massabehoud), rooster volgt het (gedraaide) brood, reset; plus een MuJoCo-check dat geparkeerde blokjes niet meer botsen.
//   node tools/test-butter-spread.mjs
import assert from 'node:assert/strict';
import { ButterSpread, SPREAD_DEFAULTS, createButterSim } from '../src/butter-spread.js';
import { createHeadlessSim } from './headless-sim.mjs';

let n = 0; const ok = async (name, f) => { await f(); n++; console.log('ok', name); };
const I = [1, 0, 0, 0], bread = { pos: [0.16, 0.06, 0.025], quat: I }, top = 0.025 + 0.013;
const bladeAt = (x, y, lowAbove, quat = I) => ({ pos: [x, y, top + lowAbove + 0.002], quat });   // pos = middelpunt; onderkant = lowAbove boven het brood
const DT = 1 / 30;

await ok('opnemen: blad drukt op een blokje (≤ 4 mm boven de bovenkant) → opgenomen + voorraad; 1 cm erboven = niet', () => {
  const s = new ButterSpread(), blk = [{ pos: [0.16, 0.06, top + 0.0035], active: true }];
  assert.deepEqual(s.update({ bread, blade: bladeAt(0.16, 0.06, 0.007 + 0.01), blocks: blk, dt: DT }), []);
  assert.deepEqual(s.update({ bread, blade: bladeAt(0.16, 0.06, 0.007 + 0.002), blocks: blk, dt: DT }), [0]);
  assert.equal(s.supply, SPREAD_DEFAULTS.blockCoverM2);
  blk[0].active = false; assert.deepEqual(s.update({ bread, blade: bladeAt(0.16, 0.06, 0.009), blocks: blk, dt: DT }), [], 'niet twee keer');
});

await ok('smeren: bewegend blad met voorraad dicht boven het brood → dekking groeit; stilstaand of 2 cm erboven → niets', () => {
  const s = new ButterSpread(); s.supply = 0.01;
  s.update({ bread, blade: bladeAt(0.16, 0.06, 0.003), dt: DT }); s.update({ bread, blade: bladeAt(0.16, 0.06, 0.003), dt: DT });
  assert.equal(s.coverage(0.01), 0, 'stilstand');
  for (let k = 0; k < 10; k++) s.update({ bread, blade: bladeAt(0.16 + 0.001 * k, 0.06, 0.02), dt: DT });
  assert.equal(s.coverage(0.01), 0, '2 cm boven het brood');
  for (let k = 0; k < 30; k++) s.update({ bread, blade: bladeAt(0.14 + 0.001 * k, 0.06, 0.002), dt: DT });
  const c = s.coverage(); assert.ok(c > 0.15 && c < 0.6, 'dekking ' + c);   // bladbreedte 2,8 cm van 9 cm brood
});

await ok('massabehoud: dekking × celoppervlak = opgenomen voorraad; zonder voorraad geen smeerlaag', () => {
  const s = new ButterSpread(); s.supply = 0.0006; const s0 = s.supply;
  for (let k = 0; k < 200; k++) s.update({ bread, blade: bladeAt(0.12 + 0.0004 * k, 0.03 + 0.0003 * k, 0.001), dt: DT });
  const used = s.cov.reduce((a, c) => a + c, 0) * s.o.cell ** 2;
  assert.ok(Math.abs(used + s.supply - s0) < 1e-9, `${used} + ${s.supply} = ${s0}`);
  const e = new ButterSpread(); for (let k = 0; k < 30; k++) e.update({ bread, blade: bladeAt(0.14 + 0.001 * k, 0.06, 0.002), dt: DT });
  assert.equal(e.coverage(0.001), 0);
});

await ok('rooster in brood-coördinaten: gedraaid brood (90°) → de laag ligt onder het blad in brood-y i.p.v. brood-x', () => {
  const q = [Math.SQRT1_2, 0, 0, Math.SQRT1_2], b = { pos: bread.pos, quat: q }, s = new ButterSpread(); s.supply = 0.01;
  // smal blad dwars (langs wereld-y, kort) bij wereld-x = brood-midden + 3 cm → brood-lokaal y = −3 cm
  const bq = [Math.SQRT1_2, 0, 0, Math.SQRT1_2];
  for (let k = 0; k < 20; k++) s.update({ bread: b, blade: { pos: [0.19, 0.04 + 0.001 * k, top + 0.003], quat: bq }, dt: DT });
  let sx = 0, sy = 0, m = 0;
  s.cov.forEach((c, k) => { if (c > 0) { const i = k % s.nx, j = (k / s.nx) | 0; sx += c * (-0.045 + (i + 0.5) * 0.003); sy += c * (-0.045 + (j + 0.5) * 0.003); m += c; } });
  assert.ok(m > 0); assert.ok(Math.abs(sy / m + 0.03) < 0.004, 'brood-y ≈ −3 cm: ' + (sy / m)); 
});

await ok('reset + coverageWithBlocks (voetafdruk van blokjes op het brood telt, blokjes ernaast niet)', () => {
  const s = new ButterSpread(); s.cov.fill(1); s.supply = 1; s.reset(); assert.equal(s.coverage(), 0); assert.equal(s.supply, 0);
  const c = s.coverageWithBlocks(bread, [{ pos: [0.16, 0.06, top + 0.0035] }, { pos: [0.4, 0.06, 0.0035] }]);
  assert.ok(Math.abs(c - 36 / 900) < 0.02, 'één blokje 16×16 mm ≈ 6×6 cellen: ' + c);
});

await ok('MuJoCo: opgenomen blokje wordt geparkeerd (geen botsing, vastgezet onder de tafel); reset maakt het weer actief', async () => {
  const env = await createHeadlessSim(), sim = createButterSim(env.mujoco, env.model, env.data);
  assert.equal(sim.blocks.length, 14);
  env.stepPhysics(1); const b = sim.blocks[0];
  // forceer opname: zet het blad vlak boven blokje 0
  sim.spread.update = () => [0]; sim.step(0.05); sim.spread.update = () => [];
  for (let k = 0; k < 20; k++) { env.stepPhysics(0.05); sim.step(0.05); }
  const { model: m, data: d } = env, z = d.xpos[3 * b + 2]; if (process.env.DBG) console.log('z', z, m.body_gravcomp[b], sim.active[0]);
  assert.ok(Math.abs(z + 0.5) < 0.02, 'geparkeerd onder de tafel, vastgezet (zakt hooguit 1 frame): ' + z);
  for (let g = 0; g < m.ngeom; g++) if (m.geom_bodyid[g] === b) assert.equal(m.geom_contype[g] + m.geom_conaffinity[g], 0);
  sim.reset(); for (let g = 0; g < m.ngeom; g++) if (m.geom_bodyid[g] === b) assert.equal(m.geom_contype[g], 1);
  assert.equal(sim.active[0], true);
});

console.log(`${n} tests ok`);
