#!/usr/bin/env node
// Headless meting: hoe vast zit het mes in de rechter gripper? Speelt de demo-greep (stap 1 "fetch-knife" uit
// src/sandwich-motion.js) af tot het mes op draaghoogte hangt, bewaart die toestand en voert vanaf daar verstoringen uit:
//   schudden (joint1/joint2 sinus), kantelen (pols joint5/joint6), rukken (snelle joint1-stap) en N willekeurige combinaties.
// Meet de slip = verandering van de mespose t.o.v. de gripper (mm, °) en of het mes valt (> 40 mm weg uit de greep).
// Ook: kantelhoek van het mes tijdens het dragen (blad zakt door om de greep-as) en rust-jitter van het mes op tafel.
//   node tools/test-knife-grip.mjs [--xml=pad/naar/scene.xml] [--json] [--trials=30]
import fs from 'fs';
import { createHeadlessSim } from './headless-sim.mjs';
import { MotionPlayer, makeSandwichChoreography, readObjectPositions } from '../src/sandwich-motion.js';
import { qmul, qconj } from '../src/qmath.js';

const args = process.argv.slice(2), arg = (k, d) => { const a = args.find(x => x.startsWith('--' + k + '=')); return a ? a.split('=').slice(1).join('=') : d; };
const xmlPath = arg('xml', null), TRIALS = Number(arg('trials', 30)), asJson = args.includes('--json');
const env = await createHeadlessSim({ sceneXml: xmlPath ? fs.readFileSync(xmlPath, 'utf8') : null });
const { mujoco: mj, model, data } = env;
const bid = n => mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, n);
const KN = bid('knife'), GR = bid('right_link6'), FL7 = bid('right_link7'), FL8 = bid('right_link8');
const pose = b => ({ p: [data.xpos[3*b], data.xpos[3*b+1], data.xpos[3*b+2]], q: [data.xquat[4*b], data.xquat[4*b+1], data.xquat[4*b+2], data.xquat[4*b+3]] });
const rotv = (q, v) => { const r = qmul(qmul(q, [0, ...v]), qconj(q)); return [r[1], r[2], r[3]]; };
// mespose in het gripperframe
const rel = () => { const k = pose(KN), g = pose(GR), d = [k.p[0]-g.p[0], k.p[1]-g.p[1], k.p[2]-g.p[2]], gi = qconj(g.q);
  return { p: rotv(gi, d), q: qmul(gi, k.q) }; };
const angDeg = (a, b) => { const d = qmul(qconj(a), b); return 2 * Math.acos(Math.min(1, Math.abs(d[0]))) * 180 / Math.PI; };
const slip = (r0, r) => ({ mm: Math.hypot(r.p[0]-r0.p[0], r.p[1]-r0.p[1], r.p[2]-r0.p[2]) * 1000, deg: angDeg(r0.q, r.q) });
// kantel van de meslengte-as (+x van het mes) t.o.v. horizontaal, in °
const knifeTilt = () => { const x = rotv(pose(KN).q, [1, 0, 0]); return Math.asin(Math.max(-1, Math.min(1, x[2]))) * 180 / Math.PI; };
const FRAME = 0.05;

// 1) rust op tafel: jitter
env.stepPhysics(2.0);
let vmax = 0; const kp0 = env.bodyPos('knife');
for (let i = 0; i < 100; i++) { env.stepPhysics(0.02); const dof = model.jnt_dofadr[model.body_jntadr[KN]]; vmax = Math.max(vmax, Math.hypot(data.qvel[dof], data.qvel[dof+1], data.qvel[dof+2])); }
const rest = { vmaxMmS: vmax * 1000, driftMm: Math.hypot(...env.bodyPos('knife').map((v, i) => v - kp0[i])) * 1000 };

// 2) demo-greep tot het mes op draaghoogte hangt (einde stap 1)
const steps = makeSandwichChoreography({ objects: readObjectPositions(mj, model, data), tuning: JSON.parse(arg('tuning', '{}')) });
const player = new MotionPlayer(env, [steps[0]], { ikIters: 6 });
const tilts = [];
let g = 0; while (!player.done && g++ < 4000) { player.update(FRAME); env.stepPhysics(FRAME); if (data.time > 0 && env.bodyPos('knife')[2] > 0.06) tilts.push(knifeTilt()); }
env.stepPhysics(0.5);
const held = env.bodyPos('knife')[2] > 0.06;
const carryTilt = { deg: knifeTilt(), minDeg: tilts.length ? Math.min(...tilts) : NaN, maxDeg: tilts.length ? Math.max(...tilts) : NaN };
const save = { qpos: data.qpos.slice(), qvel: data.qvel.slice(), act: data.act.slice(), ctrl: data.ctrl.slice(), time: data.time,
  // doel-joints + gripper exact zoals de speler ze in ctrl zette (env.grip/qTarget worden door MotionPlayer niet bijgewerkt)
  qT: env.ik.right.act.map(a => data.ctrl[a]), grip: 1 - data.ctrl[env.ik.right.grip[0]] / 0.035 };
const restore = () => { data.qpos.set(save.qpos); data.qvel.set(save.qvel); if (save.act.length) data.act.set(save.act); data.ctrl.set(save.ctrl); data.time = save.time;
  mj.mj_forward(model, data); };

// verstoring: f(t) → joint-offsets (6) voor de rechterarm; dt = 2 ms-substappen per 10 ms aansturing
function disturb(dur, f, settle = 1.0, name = '') {
  if (process.env.TRACE) console.log('== ' + name);
  restore(); const r0 = rel(); const others = new Set(); const dof = model.jnt_dofadr[model.body_jntadr[KN]]; let pv = null, amax = 0; let mx = { mm: 0, deg: 0 }, fell = false;
  const q0 = save.qT; const T = dur + settle;
  for (let t = 0; t < T; t += 0.01) {
    const o = f(Math.min(t, dur));                 // na afloop: laatste stand vasthouden (geen extra ruk terug)
    env.ik.right.apply(q0.map((v, i) => v + o[i]), save.grip);
    env.stepPhysics(0.01);
    { const v = [data.qvel[dof], data.qvel[dof+1], data.qvel[dof+2]]; if (pv) amax = Math.max(amax, Math.hypot(v[0]-pv[0], v[1]-pv[1], v[2]-pv[2]) / 0.01); pv = v; }
    const CV = Math.round(t * 100) % 5 === 0 ? data.contact : null;
    if (CV) for (let i = 0; i < data.ncon; i++) { const c = CV.get ? CV.get(i) : CV[i]; const b1 = model.geom_bodyid[c.geom1], b2 = model.geom_bodyid[c.geom2];
      if (b1 === KN || b2 === KN) { const o = b1 === KN ? b2 : b1; if (o !== FL7 && o !== FL8) others.add(mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY.value, o) || 'world'); }
      if (c && c.delete) c.delete(); }
    if (CV && CV.delete) CV.delete();
    if (process.env.TRACE && Math.round(t * 100) % 20 === 0) { const r = rel(); console.log('   t', t.toFixed(2), 'rel mm', r.p.map(v => ((v) * 1000).toFixed(1)).join(','), 'j7', data.qpos[model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT.value, 'right_joint7')]].toFixed(4), 'ctrl', data.ctrl[env.ik.right.grip[0]].toFixed(4)); }
    const s = slip(r0, rel()); mx = { mm: Math.max(mx.mm, s.mm), deg: Math.max(mx.deg, s.deg) };
    if (s.mm > 40) { fell = true; break; }
  }
  const end = fell ? { mm: NaN, deg: NaN } : slip(r0, rel());
  return { fell, aMaxG: amax / 9.81, touched: [...others].join(','), maxMm: mx.mm, maxDeg: mx.deg, endMm: end.mm, endDeg: end.deg };
}
const S = Math.sin, TAU = 2 * Math.PI;
const TESTS = [
  ['stilhouden (5 s, geen verstoring)',             5,   t => [0, 0, 0, 0, 0, 0], 0],
  ['schudden zijwaarts (j1 ±0,15 rad, 2 Hz, 3 s)',  3,   t => [0.15 * S(TAU * 2 * t), 0, 0, 0, 0, 0]],
  ['schudden op/neer (j2 ±0,10 rad, 3 Hz, 3 s)',    3,   t => [0, 0.10 * S(TAU * 3 * t), 0, 0, 0, 0]],
  ['kantelen pols (j5 −0,8 rad, langzaam)',         3,   t => [0, 0, 0, 0, -0.8 * Math.min(1, t / 1.5), 0]],
  ['rollen (j6 +1,5 rad = mes zijwaarts)',          3,   t => [0, 0, 0, 0, 0, 1.5 * Math.min(1, t / 1.5)]],
  // ruk: 0,4 rad in 0,15 s (smoothstep, piek ≈ 4 rad/s – ongeveer de max. jointsnelheid van de Piper; een echte sprong in 1 stap is onfysisch)
  ['ruk (j1 0,4 rad in 0,15 s)',                    1.5, t => { const u = Math.min(1, t / 0.15); return [0.4 * u * u * (3 - 2 * u), 0, 0, 0, 0, 0]; }],
];
const tests = Object.fromEntries(TESTS.map(([n, d, f, st]) => [n, disturb(d, f, st ?? 1.0, n)]));
// willekeurige combinaties (vaste seed): amplitude/frequentie van j1/j2/j5/j6
let seed = 12345; const rnd = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648;
let falls = 0; const rmm = [], rdeg = [];
for (let k = 0; k < TRIALS; k++) {
  // ongeveer wat een hand in VR doet: tot ~±6° schouder, ±5° elleboog, ±30° pols-kantel, ±45° rol, 0,5–2,5 Hz (piek ≈ 1–3 g)
  const a = [0.1 * rnd(), 0.08 * rnd(), 0, 0, 0.5 * rnd(), 0.8 * rnd()], fr = [0.5 + 2 * rnd(), 0.5 + 2 * rnd(), 0, 0, 0.3 + rnd(), 0.3 + rnd()], ph = a.map(() => TAU * rnd());
  const r = disturb(3, t => a.map((v, i) => v * S(TAU * fr[i] * t + ph[i]) - v * S(ph[i])));
  if (r.fell) falls++; else { rmm.push(r.endMm); rdeg.push(r.endDeg); }
}
const med = a => a.length ? [...a].sort((x, y) => x - y)[a.length >> 1] : NaN;
const out = { scene: xmlPath || 'assets/scene.xml', rest, held, carryTilt, tests, random: { trials: TRIALS, falls, fallRate: falls / TRIALS, medianEndMm: med(rmm), medianEndDeg: med(rdeg), maxEndMm: Math.max(...rmm), maxEndDeg: Math.max(...rdeg) } };
if (asJson) console.log(JSON.stringify(out, null, 1));
else {
  const f1 = v => Number.isFinite(v) ? v.toFixed(1) : '—';
  console.log(`scene: ${out.scene}`);
  console.log(`rust op tafel: max snelheid ${f1(rest.vmaxMmS)} mm/s, drift ${f1(rest.driftMm)} mm (2 s)`);
  console.log(`greep: mes ${held ? 'opgetild ✔' : 'NIET opgetild ✘'}; kantel mes-as t.o.v. horizontaal tijdens dragen ${f1(carryTilt.minDeg)}…${f1(carryTilt.maxDeg)}°, eind ${f1(carryTilt.deg)}°`);
  for (const [n, r] of Object.entries(tests)) console.log(`  ${n.padEnd(46)} ${r.fell ? 'VALT ✘' : 'blijft ✔'}  piek ${f1(r.aMaxG)} g, slip max ${f1(r.maxMm)} mm / ${f1(r.maxDeg)}°, blijvend ${f1(r.endMm)} mm / ${f1(r.endDeg)}°${r.touched ? '  (raakt ook: ' + r.touched + ')' : ''}`);
  console.log(`  willekeurig ${TRIALS}×: valt ${falls}× (${(100 * falls / TRIALS).toFixed(0)} %), blijvende slip mediaan ${f1(out.random.medianEndMm)} mm / ${f1(out.random.medianEndDeg)}°, max ${f1(out.random.maxEndMm)} mm / ${f1(out.random.maxEndDeg)}°`);
}
