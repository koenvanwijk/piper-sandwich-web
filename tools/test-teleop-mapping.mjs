#!/usr/bin/env node
// Test (Node, MuJoCo-wasm, geen browser): controller (WebXR-wereld: x rechts, y omhoog, -z vooruit) -> TCP-beweging in de scène
// zoals de gebruiker hem ziet. Gebruikt dezelfde stappen als app.js (HandTeleop + ArmIK + 16 substappen van 1/30 s).
//   node tools/test-teleop-mapping.mjs
import assert from 'node:assert/strict';
import { createHeadlessSim } from './headless-sim.mjs';
import { HandTeleop, teleopArm, twistAbout, orientationMode, rollAxisFromQuery, tiltAxisFromQuery, yawAxisFromQuery, ROLL_MAX_STEP, TILT_MAX_STEP, YAW_MAX_STEP, JOINT_SPEED } from '../src/teleop.js';
import { qmul as qmulW, qconj as qconjW, qlog, rotErr } from '../src/qmath.js';
import { relaxBaseContacts } from '../src/sandwich-motion.js';
import { controllerToRaw, mjToWorld, headYaw, homeFromHead, worldToRoot, rootToWorld, threeToMj } from '../src/xr-map.js';

const ROOT_POS = [0, 0.95, -0.32], HOME_ROT = Math.PI / 2;         // = SandwichVR.init()
const SIDES = ['left', 'right'];
let n = 0; const ok = (name, f) => Promise.resolve(f()).then(() => { n++; console.log('ok', name); });
const r3 = a => a.map(x => +x.toFixed(3));

async function rig({ relax, lockOrientation = true, mode = null, tilt = true, yaw = true, rootRotY = HOME_ROT, rootPos = ROOT_POS }) {
  const env = await createHeadlessSim({ relaxBase: false });
  if (relax) relaxBaseContacts(env.model);
  const teleop = { left: new HandTeleop({ lockOrientation, mode, tilt, yaw }), right: new HandTeleop({ lockOrientation, mode, tilt, yaw }) };
  env.qIK = {}; const log = { dq4: [], dq5: [], dq6: [], dqMax: { left: 0, right: 0 }, nan: false, err: {} };
  const ctrl = { left: { pos: [-0.22, 1.0, -0.3], orient: [0, 0, 0, 1], grip: 1 }, right: { pos: [0.22, 1.0, -0.3], orient: [0, 0, 0, 1], grip: 1 } };
  const tcpWorld = s => mjToWorld(env.tcpPose(s).pos, rootPos, rootRotY);
  const step = (k = 1) => {
    for (let i = 0; i < k; i++) {
      for (const s of SIDES) {
        const c = ctrl[s], raw = { ...controllerToRaw(c.pos, c.orient, rootPos, rootRotY), trigger: 0, grip: c.grip };
        const q4 = env.qTarget[s][3], q5 = env.qTarget[s][4], q6 = env.qTarget[s][5], qPrev = env.qTarget[s].slice();
        const t = teleopArm(env, s, teleop[s], env.ik[s], raw, env.tcpPose(s));      // dezelfde stap als app.js control()
        log.dqMax[s] = Math.max(log.dqMax[s], ...env.qTarget[s].map((v, i) => Math.abs(v - qPrev[i])));
        if (!env.qTarget[s].every(Number.isFinite)) log.nan = true;
        if (t.err) log.err[s] = t.err;
        if (s === 'right') { log.dq4.push(Math.abs(env.qTarget[s][3] - q4)); log.dq5.push(Math.abs(env.qTarget[s][4] - q5)); log.dq6.push(Math.abs(env.qTarget[s][5] - q6)); }
        env.grip[s] = t.grip; env.ik[s].apply(env.qTarget[s], env.grip[s]);
      }
      for (let j = 0; j < 16; j++) env.mujoco.mj_step(env.model, env.data);
    }
  };
  const clutch = on => { for (const s of SIDES) ctrl[s].grip = on ? 1 : 0; step(1); };
  return { env, ctrl, step, clutch, tcpWorld, teleop, log };
}

// beweging (wereld, m) van het TCP als de controller `d` (wereld) verplaatst wordt; clutch loslaten/vastpakken per meting
async function moveTest(r, side, d) {
  r.clutch(false); r.step(20); r.clutch(true);
  const p0 = r.tcpWorld(side), c0 = r.ctrl[side].pos.slice();
  r.ctrl[side].pos = c0.map((v, i) => v + d[i]); r.step(60);
  const p1 = r.tcpWorld(side); r.ctrl[side].pos = c0; r.step(40);
  return p1.map((v, i) => v - p0[i]);
}

await ok('xr-map: round trip + axes (root = 90° om y): gebruiker-rechts = MuJoCo -y, vooruit = MuJoCo +x', () => {
  const p = [0.1, 0.2, -0.3], l = worldToRoot(p, ROOT_POS, HOME_ROT);
  assert.deepEqual(rootToWorld(l, ROOT_POS, HOME_ROT).map(x => +x.toFixed(9)), p.map(x => +x.toFixed(9)));
  const dRight = threeToMj(worldToRoot([0.2 + ROOT_POS[0], ROOT_POS[1], ROOT_POS[2]], ROOT_POS, HOME_ROT));
  assert.ok(Math.abs(dRight[1] + 0.2) < 1e-9 && Math.abs(dRight[0]) < 1e-9, 'rechts = -y (MuJoCo)');
  const dFwd = threeToMj(worldToRoot([ROOT_POS[0], ROOT_POS[1], ROOT_POS[2] - 0.2], ROOT_POS, HOME_ROT));
  assert.ok(Math.abs(dFwd[0] - 0.2) < 1e-9, 'vooruit = +x (MuJoCo)');
});

await ok('armen: linker arm staat links van de gebruiker (wereld-x < 0), rechter rechts', async () => {
  const r = await rig({ relax: true });
  assert.ok(r.tcpWorld('left')[0] < -0.1 && r.tcpWorld('right')[0] > 0.1, 'links/rechts per arm: ' + r3(r.tcpWorld('left')) + ' ' + r3(r.tcpWorld('right')));
});

const DIRS = { rechts: [0.1, 0, 0], links: [-0.1, 0, 0], vooruit: [0, 0, -0.1], omhoog: [0, 0.1, 0], omlaag: [0, -0.1, 0] };
await ok('OORZAAK: zonder relaxBaseContacts is joint1 geblokkeerd en beweegt de arm verkeerd/gespiegeld (gemeten, gedocumenteerd)', async () => {
  const r = await rig({ relax: false });
  const d = await moveTest(r, 'right', DIRS.rechts);
  console.log('   zonder fix: controller 10 cm rechts -> TCP wereld-delta', r3(d));
  assert.ok(d[0] < 0.03, 'bug reproduceert: TCP gaat NIET (of de verkeerde kant op) 10 cm naar rechts');
});

for (const side of SIDES) {
  await ok(`MET fix: ${side} controller 10 cm rechts/links/vooruit/omhoog/omlaag -> TCP zelfde richting (±1,5 cm per as)`, async () => {
    const r = await rig({ relax: true });
    for (const [name, d] of Object.entries(DIRS)) {
      const m = await moveTest(r, side, d);
      const err = Math.max(...m.map((v, i) => Math.abs(v - d[i])));
      console.log(`   ${side} ${name}: verwacht ${r3(d)} gemeten ${r3(m)} (max afwijking ${(err * 1000).toFixed(1)} mm)`);
      assert.ok(err < 0.015, `${side} ${name}: ${r3(m)} vs ${r3(d)}`);
    }
  });
}

await ok('links en rechts gedragen zich identiek (geen verwisseling): linker controller beweegt alleen de linker arm', async () => {
  const r = await rig({ relax: true });
  r.clutch(false); r.step(10); r.clutch(true);
  const R0 = r.tcpWorld('right'); r.ctrl.left.pos = r.ctrl.left.pos.map((v, i) => v + [0.1, 0, 0][i]); r.step(40);
  assert.ok(Math.hypot(...r.tcpWorld('right').map((v, i) => v - R0[i])) < 0.004, 'rechter arm blijft stil');
});

await ok('scène-rotatie (stick-yaw 35°): controller-beweging blijft in WERELD-richting (wat de gebruiker ziet)', async () => {
  const rotY = HOME_ROT + 0.61; const r = await rig({ relax: true, rootRotY: rotY });
  const m = await moveTest(r, 'right', [0, 0, -0.08]);
  console.log('   vooruit 8 cm bij gedraaide scène ->', r3(m));
  assert.ok(Math.abs(m[2] + 0.08) < 0.015 && Math.abs(m[0]) < 0.015);
});

await ok('kijkrichting: headYaw/homeFromHead (hoofd 90° naar links gedraaid -> scène meedraaien, positie voor het hoofd)', () => {
  const s = Math.SQRT1_2, yaw = headYaw({ x: 0, y: s, z: 0, w: s });           // +90° om y = naar links kijken (-x)
  assert.ok(Math.abs(yaw - Math.PI / 2) < 1e-9);
  const h = homeFromHead([1, 1.6, 2], yaw);
  assert.ok(Math.abs(h.pos[0] - (1 - 0.32)) < 1e-9 && Math.abs(h.pos[2] - 2) < 1e-9 && Math.abs(h.rotY - Math.PI) < 1e-9);
  assert.equal(headYaw({ x: Math.sin(Math.PI / 4), y: 0, z: 0, w: Math.cos(Math.PI / 4) }), null);   // recht omhoog/omlaag
});

// ---- oriëntatie (?rot=1): relatieve rotatie vanaf de clutch
const qAxis = (ax, a) => { const s = Math.sin(a / 2); return [ax[0] * s, ax[1] * s, ax[2] * s, Math.cos(a / 2)]; };
const tcpAxes = (env, s) => {   // TCP-as-richtingen in wereld: kolommen van site_xmat (MuJoCo) -> wereld
  const id = env.ik[s].site, m = []; for (let k = 0; k < 9; k++) m.push(env.data.site_xmat[9 * id + k]);
  const col = c => { const v = [m[c], m[3 + c], m[6 + c]]; const t = [v[0], v[2], -v[1]];   // mj -> three
    const a = HOME_ROT; return [t[0] * Math.cos(a) + t[2] * Math.sin(a), t[1], -t[0] * Math.sin(a) + t[2] * Math.cos(a)]; };
  return [col(0), col(1), col(2)];
};
const angle = (a, b) => Math.acos(Math.max(-1, Math.min(1, a[0]*b[0]+a[1]*b[1]+a[2]*b[2]))) * 180 / Math.PI;
const qrot = (q, v) => { const [x, y, z, w] = q, t = [2*(y*v[2]-z*v[1]), 2*(z*v[0]-x*v[2]), 2*(x*v[1]-y*v[0])];
  return [v[0]+w*t[0]+(y*t[2]-z*t[1]), v[1]+w*t[1]+(z*t[0]-x*t[2]), v[2]+w*t[2]+(x*t[1]-y*t[0])]; };
for (const [name, ax, deg] of [['yaw (om wereld-y)', [0, 1, 0], 30], ['pitch (om wereld-x)', [1, 0, 0], 25], ['roll (om wereld-z)', [0, 0, 1], 25]]) {
  await ok(`oriëntatie ?rot=1: controller ${name} ${deg}° -> TCP draait dezelfde kant op (gemeten)`, async () => {
    const r = await rig({ relax: true, lockOrientation: false });
    r.clutch(false); r.step(20); r.clutch(true);
    const a0 = tcpAxes(r.env, 'right');
    const q = qAxis(ax, deg * Math.PI / 180); r.ctrl.right.orient = q; r.step(150);
    const a1 = tcpAxes(r.env, 'right');
    // verwacht: elke TCP-as is met dezelfde wereld-rotatie meegedraaid; de 6-DOF-arm heeft gewrichtslimieten, dus restfout ~ enkele graden
    const errs = a0.map((v, i) => angle(qrot(q, v), a1[i]));
    const turned = a0.map((v, i) => angle(v, a1[i]));
    console.log(`   ${name}: TCP-assen gedraaid x/y/z = ${turned.map(x => x.toFixed(1))}°; afwijking t.o.v. verwachte rotatie = ${errs.map(x => x.toFixed(1))}°`);
    assert.ok(Math.max(...errs) < 15, 'TCP-oriëntatie volgt de controllerdraai binnen 15° (gewrichtslimieten)');
  });
}

// ---- polsrol (standaard 'roll'-modus): controller-rol om de eigen (wijs)as -> joint6 ----
const qm = (a, b) => [a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1], a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0], a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3], a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]];
const P_AX = [0, 0, -1], deg = Math.PI / 180;
const rollBody = (q0, d) => qm(q0, qAxis(P_AX, d * deg));                  // controller draait om zijn EIGEN wijsas (lichaams-frame)
const qj = (env, s) => env.ik[s].currentQ();
const settle = async (r, k = 90) => r.step(k);
await ok('rol (?mode=joints): controller 40° om eigen as -> joint6 +40° (±3°), positie en joint1–5 ongewijzigd; −40° -> −40°; ook als de controller schuin gehouden wordt', async () => {
  for (const [name, q0] of [['recht', [0, 0, 0, 1]], ['40° omlaag gekanteld + 25° gedraaid', qm(qAxis([0, 1, 0], 25 * deg), qAxis([1, 0, 0], -40 * deg))]]) {
    for (const sign of [1, -1]) {
      const r = await rig({ relax: true, mode: 'roll' }); r.ctrl.right.orient = q0; r.clutch(false); r.step(20); r.clutch(true); await settle(r, 5);
      const p0 = r.tcpWorld('right'), qa = qj(r.env, 'right'), a0 = tcpAxes(r.env, 'right');
      r.ctrl.right.orient = rollBody(q0, sign * 40); await settle(r, 120);
      const qb = qj(r.env, 'right'), d6 = (qb[5] - qa[5]) / deg, others = Math.max(...[0, 1, 2, 3, 4].map(i => Math.abs(qb[i] - qa[i]) / deg)), dp = Math.hypot(...r.tcpWorld('right').map((v, i) => v - p0[i]));
      const a1 = tcpAxes(r.env, 'right'), approach = angle(a0[2], a1[2]);
      console.log(`   ${name} ${sign > 0 ? '+' : '−'}40°: Δjoint6 = ${d6.toFixed(1)}°, max Δjoint1–5 = ${others.toFixed(2)}°, TCP-verplaatsing ${(dp * 1000).toFixed(1)} mm, aanvliegas gedraaid ${approach.toFixed(2)}°`);
      assert.ok(Math.abs(d6 - sign * 40) < 3, 'joint6 volgt de rol'); assert.ok(others < 1.0, 'andere gewrichten blijven'); assert.ok(dp < 0.004, 'TCP-positie blijft'); assert.ok(approach < 1.0, 'aanvliegrichting blijft'); assert.ok(Math.abs(angle(a0[0], a1[0]) - 40) < 3.5, 'de gripper zelf (TCP-x-as) is ~40° gedraaid: ' + angle(a0[0], a1[0]).toFixed(1));
      r.ctrl.right.orient = q0; await settle(r, 150); assert.ok(Math.abs(qj(r.env, 'right')[5] - qa[5]) < 3 * deg, 'terugrollen zet de gripper terug');
    }
  }
});
await ok('rol: andere draaiingen (yaw/pitch van de controller zonder rol) laten de gripper NIET rollen; links/rechts onafhankelijk', async () => {
  const r = await rig({ relax: true, mode: 'roll' }); r.clutch(false); r.step(20); r.clutch(true); await settle(r, 5);
  const qa = qj(r.env, 'right'), ql = qj(r.env, 'left');
  r.ctrl.right.orient = qAxis([0, 1, 0], 35 * deg); await settle(r, 80); assert.ok(Math.abs(qj(r.env, 'right')[5] - qa[5]) < 2 * deg, 'yaw rolt niet');
  r.ctrl.right.orient = qAxis([1, 0, 0], 30 * deg); await settle(r, 80); assert.ok(Math.abs(qj(r.env, 'right')[5] - qa[5]) < 2 * deg, 'pitch rolt niet');
  r.ctrl.right.orient = [0, 0, 0, 1]; r.ctrl.left.orient = rollBody([0, 0, 0, 1], 50); await settle(r, 120);
  assert.ok(Math.abs(qj(r.env, 'left')[5] - ql[5] - 50 * deg) < 3 * deg, 'linker controller rolt linker gripper'); assert.ok(Math.abs(qj(r.env, 'right')[5] - qa[5]) < 2 * deg, 'rechter blijft');
});
await ok('rol: geen sprongen (≤ ROLL_MAX_STEP per stap), tot de jointlimiet (±120°) geklemd bij 170°-rol, geen NaN; terugrollen vanaf de limiet', async () => {
  const r = await rig({ relax: true, mode: 'roll' }); r.clutch(false); r.step(20); r.clutch(true); await settle(r, 5);
  const lim = r.env.ik.right.rng[5], q6a = qj(r.env, 'right')[5]; r.log.dq6.length = 0;
  r.ctrl.right.orient = rollBody([0, 0, 0, 1], 90); r.step(1); r.ctrl.right.orient = rollBody([0, 0, 0, 1], 170); await settle(r, 150);   // sprong in de invoer van 90° -> 170°
  const q6 = qj(r.env, 'right')[5]; assert.ok(Number.isFinite(q6) && q6 <= lim[1] + 1e-3 && q6 >= lim[0] - 1e-3, `q6 ${q6 / deg}° binnen limiet`);
  assert.ok(q6 > lim[1] - 0.1, 'bereikt ~de limiet'); assert.ok(Math.max(...r.log.dq6) <= ROLL_MAX_STEP + 1e-6, 'max stap per tick ' + Math.max(...r.log.dq6).toFixed(3) + ' rad');
  const t = r.env.data.ctrl, act = r.env.ik.right.act[5]; assert.ok(t[act] <= lim[1] + 1e-6, 'ctrl binnen limiet');
  console.log(`   joint6: ${(q6a / deg).toFixed(0)}° -> ${(q6 / deg).toFixed(0)}° (limiet ${(lim[1] / deg).toFixed(0)}°), max stap ${(Math.max(...r.log.dq6) / deg).toFixed(1)}°/tick`);
  r.ctrl.right.orient = rollBody([0, 0, 0, 1], 20); await settle(r, 150); assert.ok(qj(r.env, 'right')[5] < 40 * deg, 'terug van de limiet');
});
await ok('rol blijft na loslaten van de grip (gebakken), nieuwe grip = nieuw anker zonder sprong; modus lock (?rot=0/?roll=0) rolt niet', async () => {
  const r = await rig({ relax: true, mode: 'roll' }); r.clutch(false); r.step(20); r.clutch(true); await settle(r, 5);
  const q6a = qj(r.env, 'right')[5]; r.ctrl.right.orient = rollBody([0, 0, 0, 1], 45); await settle(r, 100);
  const q6b = qj(r.env, 'right')[5]; assert.ok(Math.abs(q6b - q6a - 45 * deg) < 3 * deg);
  r.clutch(false); r.step(30); assert.ok(Math.abs(qj(r.env, 'right')[5] - q6b) < 1.5 * deg, 'blijft staan na loslaten');
  r.ctrl.right.orient = [0, 0, 0, 1]; r.log.dq6.length = 0; r.clutch(true); await settle(r, 30);
  assert.ok(Math.abs(qj(r.env, 'right')[5] - q6b) < 2 * deg, 'geen sprong bij nieuwe grip (controller staat nu anders)'); assert.ok(Math.max(...r.log.dq6) < 0.01);
  r.ctrl.right.orient = rollBody([0, 0, 0, 1], -30); await settle(r, 100); assert.ok(Math.abs(qj(r.env, 'right')[5] - q6b + 30 * deg) < 3 * deg, 'verder rollen vanaf de nieuwe stand');
  const l = await rig({ relax: true, mode: 'lock' }); l.clutch(false); l.step(20); l.clutch(true); await settle(l, 5); const qa = qj(l.env, 'right')[5];
  l.ctrl.right.orient = rollBody([0, 0, 0, 1], 60); await settle(l, 100); assert.ok(Math.abs(qj(l.env, 'right')[5] - qa) < 2 * deg, 'lock-modus: geen rol (oud gedrag)');
});
await ok('rol + translatie tegelijk: positie volgt nog (≤ 8 mm) en joint6 rolt 35° t.o.v. dezelfde beweging zonder rol; aanvliegas blijft vergrendeld', async () => {
  const run = async rollDeg => {
    const r = await rig({ relax: true, mode: 'roll', tilt: false, yaw: false }); r.clutch(false); r.step(20); r.clutch(true); await settle(r, 5);
    const p0 = r.tcpWorld('right'), c0 = r.ctrl.right.pos.slice(), a0 = tcpAxes(r.env, 'right');
    r.ctrl.right.pos = [c0[0] + 0.08, c0[1] + 0.05, c0[2] - 0.08]; r.ctrl.right.orient = rollBody([0, 0, 0, 1], rollDeg); await settle(r, 150);
    return { dp: r.tcpWorld('right').map((v, i) => v - p0[i]), q6: qj(r.env, 'right')[5], appr: angle(a0[2], tcpAxes(r.env, 'right')[2]) };
  };
  const a = await run(0), b = await run(35), want = [0.08, 0.05, -0.08];
  console.log(`   TCP-verplaatsing ${r3(b.dp)} (gewenst ${want}); Δjoint6 door rol = ${((b.q6 - a.q6) / deg).toFixed(1)}° (35° verwacht); aanvliegas ${b.appr.toFixed(2)}° gedraaid`);
  assert.ok(Math.hypot(...b.dp.map((v, i) => v - want[i])) < 0.008); assert.ok(Math.abs(b.q6 - a.q6 - 35 * deg) < 3 * deg); assert.ok(b.appr < 1.5);
});
await ok('twistAbout/orientationMode/assen: pure wiskunde en query-parsing', () => {
  const id = [1, 0, 0, 0], Rz = a => [Math.cos(a / 2), 0, 0, Math.sin(a / 2)];
  assert.ok(Math.abs(twistAbout(id, Rz(-0.5)) - 0.5) < 1e-9, 'rotatie −0,5 rad om +z = +0,5 rad om de wijsas (−z)');
  assert.ok(Math.abs(twistAbout(id, [Math.cos(0.3), Math.sin(0.3), 0, 0])) < 1e-9, 'pure swing = 0 twist');
  assert.ok(Math.abs(twistAbout(id, Rz(-0.5).map(x => -x)) - 0.5) < 1e-9, 'dubbele dekking q ≡ −q');
  assert.equal(orientationMode(''), '6dof'); assert.equal(orientationMode('?rot=1'), '6dof'); assert.equal(orientationMode('?mode=6dof'), '6dof'); assert.equal(orientationMode('?mode=joints'), 'joints'); assert.equal(orientationMode('?mode=roll'), 'joints');
  assert.equal(orientationMode('?rot=0'), 'lock'); assert.equal(orientationMode('?roll=0'), 'lock'); assert.equal(orientationMode('?orient=0'), 'lock'); assert.equal(orientationMode('?mode=joints&orient=0'), 'lock'); assert.equal(orientationMode('?tags=1&debug=1'), '6dof');
  assert.equal(new HandTeleop({ mode: 'full' }).mode, '6dof'); assert.equal(new HandTeleop({ mode: 'roll' }).mode, 'joints'); assert.equal(new HandTeleop({ lockOrientation: false }).mode, '6dof'); assert.equal(new HandTeleop().mode, 'lock');
  assert.deepEqual(rollAxisFromQuery('?rollaxis=1,0,0'), [1, 0, 0]); assert.deepEqual(rollAxisFromQuery(''), [0, 0, -1]);
  assert.deepEqual(tiltAxisFromQuery(''), [1, 0, 0]); assert.deepEqual(yawAxisFromQuery(''), [0, 1, 0]);
  assert.deepEqual(tiltAxisFromQuery('?tiltaxis=0,1,0'), [0, 1, 0]); assert.deepEqual(yawAxisFromQuery('?yawaxis=0,0,1'), [0, 0, 1]);
});

// ---- tilt → joint5 (pitch om grip +x) ----
const tiltBody = (q0, d) => qm(q0, qAxis([1, 0, 0], d * deg));
const yawBody  = (q0, d) => qm(q0, qAxis([0, 1, 0], d * deg));
await ok('tilt (?mode=joints): controller 35° pitch om +x → joint5 +35° (±3°), j4/j6 ~0; −35° → −35°; ?tilt=0 negeert', async () => {
  for (const sign of [1, -1]) {
    const r = await rig({ relax: true, mode: 'roll', yaw: false }); r.clutch(false); r.step(20); r.clutch(true); await settle(r, 5);
    const qa = qj(r.env, 'right');
    r.ctrl.right.orient = tiltBody([0, 0, 0, 1], sign * 35); await settle(r, 120);
    const qb = qj(r.env, 'right'), d5 = (qb[4] - qa[4]) / deg;
    console.log(`   tilt ${sign > 0 ? '+' : '−'}35°: Δj5=${d5.toFixed(1)}° Δj4=${((qb[3]-qa[3])/deg).toFixed(2)}° Δj6=${((qb[5]-qa[5])/deg).toFixed(2)}°`);
    assert.ok(Math.abs(d5 - sign * 35) < 3, 'joint5 volgt tilt'); assert.ok(Math.abs(qb[3] - qa[3]) < 2 * deg && Math.abs(qb[5] - qa[5]) < 2 * deg, 'j4/j6 stil');
  }
  const off = await rig({ relax: true, mode: 'roll', tilt: false, yaw: false }); off.clutch(false); off.step(20); off.clutch(true); await settle(off, 5);
  const q0 = qj(off.env, 'right'); off.ctrl.right.orient = tiltBody([0, 0, 0, 1], 40); await settle(off, 100);
  assert.ok(Math.abs(qj(off.env, 'right')[4] - q0[4]) < 2 * deg, '?tilt=0: geen tilt');
});
await ok('tilt: limiet (±70° j5), max stap, clutch bakken, geen NaN', async () => {
  const r = await rig({ relax: true, mode: 'roll', yaw: false }); r.clutch(false); r.step(20); r.clutch(true); await settle(r, 5);
  const lim = r.env.ik.right.rng[4]; r.log.dq5.length = 0;
  r.ctrl.right.orient = tiltBody([0, 0, 0, 1], 50); r.step(1); r.ctrl.right.orient = tiltBody([0, 0, 0, 1], 90); await settle(r, 150);
  const q5 = qj(r.env, 'right')[4];
  assert.ok(Number.isFinite(q5) && q5 <= lim[1] + 1e-3 && q5 >= lim[0] - 1e-3);
  assert.ok(q5 > lim[1] - 0.15, 'nabij limiet'); assert.ok(Math.max(...r.log.dq5) <= TILT_MAX_STEP + 1e-6);
  const q5b = q5; r.clutch(false); r.step(20); assert.ok(Math.abs(qj(r.env, 'right')[4] - q5b) < 2 * deg, 'blijft na loslaten');
  console.log(`   j5 → ${(q5 / deg).toFixed(0)}° (lim ${(lim[1] / deg).toFixed(0)}°), max stap ${(Math.max(...r.log.dq5) / deg).toFixed(1)}°`);
});

// ---- yaw → joint4 (om grip +y) ----
await ok('yaw (?mode=joints): controller 30° om +y → joint4 +30° (±3°), j5/j6 ~0; −30°; ?yaw=0 negeert', async () => {
  for (const sign of [1, -1]) {
    const r = await rig({ relax: true, mode: 'roll', tilt: false }); r.clutch(false); r.step(20); r.clutch(true); await settle(r, 5);
    const qa = qj(r.env, 'right');
    r.ctrl.right.orient = yawBody([0, 0, 0, 1], sign * 30); await settle(r, 120);
    const qb = qj(r.env, 'right'), d4 = (qb[3] - qa[3]) / deg;
    console.log(`   yaw ${sign > 0 ? '+' : '−'}30°: Δj4=${d4.toFixed(1)}° Δj5=${((qb[4]-qa[4])/deg).toFixed(2)}° Δj6=${((qb[5]-qa[5])/deg).toFixed(2)}°`);
    assert.ok(Math.abs(d4 - sign * 30) < 3, 'joint4 volgt yaw'); assert.ok(Math.abs(qb[4] - qa[4]) < 2 * deg && Math.abs(qb[5] - qa[5]) < 2 * deg, 'j5/j6 stil');
  }
  const off = await rig({ relax: true, mode: 'roll', tilt: false, yaw: false }); off.clutch(false); off.step(20); off.clutch(true); await settle(off, 5);
  const q0 = qj(off.env, 'right'); off.ctrl.right.orient = yawBody([0, 0, 0, 1], 40); await settle(off, 100);
  assert.ok(Math.abs(qj(off.env, 'right')[3] - q0[3]) < 2 * deg, '?yaw=0: geen yaw');
});
await ok('yaw: limiet (±100° j4), max stap, lock-modus geen yaw', async () => {
  const r = await rig({ relax: true, mode: 'roll', tilt: false }); r.clutch(false); r.step(20); r.clutch(true); await settle(r, 5);
  const lim = r.env.ik.right.rng[3]; r.log.dq4.length = 0;
  r.ctrl.right.orient = yawBody([0, 0, 0, 1], 60); r.step(1); r.ctrl.right.orient = yawBody([0, 0, 0, 1], 120); await settle(r, 150);
  const q4 = qj(r.env, 'right')[3];
  assert.ok(Number.isFinite(q4) && q4 <= lim[1] + 1e-3 && q4 >= lim[0] - 1e-3);
  assert.ok(q4 > lim[1] - 0.15, 'nabij limiet'); assert.ok(Math.max(...r.log.dq4) <= YAW_MAX_STEP + 1e-6);
  console.log(`   j4 → ${(q4 / deg).toFixed(0)}° (lim ${(lim[1] / deg).toFixed(0)}°), max stap ${(Math.max(...r.log.dq4) / deg).toFixed(1)}°`);
  const l = await rig({ relax: true, mode: 'lock' }); l.clutch(false); l.step(20); l.clutch(true); await settle(l, 5); const qa = qj(l.env, 'right');
  l.ctrl.right.orient = yawBody([0, 0, 0, 1], 40); await settle(l, 100);
  assert.ok(Math.abs(qj(l.env, 'right')[3] - qa[3]) < 2 * deg, 'lock: geen yaw');
});
await ok('roll+tilt+yaw tegelijk + translatie: teleop-offsets ≈ rotvec, joints volgen, positie ≈ gewenst', async () => {
  const qRotVec = (vx, vy, vz) => { const a = Math.hypot(vx, vy, vz) || 1e-12, s = Math.sin(a / 2) / a; return [vx * s, vy * s, vz * s, Math.cos(a / 2)]; };
  const r = await rig({ relax: true, mode: 'roll' }); r.clutch(false); r.step(20); r.clutch(true); await settle(r, 5);
  const p0 = r.tcpWorld('right'), qa = qj(r.env, 'right'), c0 = r.ctrl.right.pos.slice();
  r.ctrl.right.orient = qRotVec(25 * deg, 20 * deg, -30 * deg);
  r.ctrl.right.pos = [c0[0] + 0.06, c0[1] + 0.04, c0[2] - 0.05]; await settle(r, 160);
  const qb = qj(r.env, 'right'), dp = r.tcpWorld('right').map((v, i) => v - p0[i]), want = [0.06, 0.04, -0.05];
  const d4 = (qb[3] - qa[3]) / deg, d5 = (qb[4] - qa[4]) / deg, d6 = (qb[5] - qa[5]) / deg;
  const ty = r.teleop.right.yaw / deg, tt = r.teleop.right.tilt / deg, tr = r.teleop.right.roll / deg;
  console.log(`   Δj4/j5/j6 = ${d4.toFixed(1)}/${d5.toFixed(1)}/${d6.toFixed(1)}°; teleop ${ty.toFixed(1)}/${tt.toFixed(1)}/${tr.toFixed(1)}°; TCP ${r3(dp)}`);
  assert.ok(Math.abs(tr - 30) < 4 && Math.abs(tt - 25) < 4 && Math.abs(ty - 20) < 4, 'teleop-offsets');
  assert.ok(Math.abs(d6 - tr) < 3 && Math.abs(d5 - tt) < 3 && Math.abs(d4 - ty) < 3, 'joints = teleop-offsets');
  assert.ok(Math.hypot(...dp.map((v, i) => v - want[i])) < 0.04, 'positie');
});
// ================= 6-DOF pose-teleop (standaard) =================
// Hulpjes: controller-oriëntatie (WebXR x,y,z,w, wereld) die in de MuJoCo-frame precies de rotatie dQ_mj ([w,x,y,z]) geeft.
const W2X = q => [q[1], q[2], q[3], q[0]], X2W = q => [q[3], q[0], q[1], q[2]];
const QA6 = [Math.SQRT1_2, Math.SQRT1_2, 0, 0];                                   // three → MuJoCo (zie teleop.js)
const RY = a => [Math.cos(a / 2), 0, Math.sin(a / 2), 0];
const qexp = v => { const a = Math.hypot(...v); return a < 1e-12 ? [1, 0, 0, 0] : [Math.cos(a / 2), ...v.map(x => x / a * Math.sin(a / 2))]; };
const mjDeltaToWorld = (dmj, rotY = HOME_ROT) => { const d3 = qmulW(qmulW(qconjW(QA6), dmj), QA6); return qmulW(qmulW(RY(rotY), d3), qconjW(RY(rotY))); };
const worldDeltaToMj = (dw, rotY = HOME_ROT) => { const d3 = qmulW(qmulW(qconjW(RY(rotY)), dw), RY(rotY)); return qmulW(qmulW(QA6, d3), qconjW(QA6)); };
const mjVecToWorld = (v, rotY = HOME_ROT) => { const t = [v[0], v[2], -v[1]]; return [t[0] * Math.cos(rotY) + t[2] * Math.sin(rotY), t[1], -t[0] * Math.sin(rotY) + t[2] * Math.cos(rotY)]; };
const angDeg = (qa, qb) => Math.hypot(...rotErr(qa, qb)) / deg;
let seedR = 7; const rnd = () => (seedR = (seedR * 16807) % 2147483647) / 2147483647;

// Beweeg de controller in `ticks` stappen van zijn engage-pose naar (pos + dpWorld, dQworld · orient0), en laat daarna `hold` ticks uitzwaaien.
async function drive(r, side, dpWorld, dQworld, ticks = 30, hold = 60) {
  const c = r.ctrl[side], p0 = c.pos.slice(), o0 = X2W(c.orient), lv = qlog(dQworld);
  for (let k = 1; k <= ticks; k++) { const a = k / ticks; c.pos = p0.map((v, i) => v + a * dpWorld[i]); c.orient = W2X(qmulW(qexp(lv.map(x => x * a)), o0)); r.step(1); }
  r.step(hold);
}
const engage6 = async (r, orient = [0, 0, 0, 1]) => { for (const s of SIDES) r.ctrl[s].orient = orient; r.clutch(false); r.step(20); r.clutch(true); r.step(3); };
const anchorPose = (r, s) => { const [p, q] = r.env.ik[s].fk(r.env.qTarget[s]); return { p, q }; };

await ok('qmath: rotErr/qlog nemen de KORTSTE weg (q ≡ −q) — oorzaak van het "wilde pols"-gedrag in het oude ?rot=1-pad', () => {
  const t = [0.9, 0.1, 0.3, 0.2].map((x, _, a) => x / Math.hypot(...a)), small = [Math.cos(0.005), Math.sin(0.005), 0, 0];
  const e = Math.hypot(...rotErr(qmulW(small, t).map(x => -x), t));
  console.log(`   0,01 rad verschil met omgekeerd teken: rotErr = ${e.toFixed(4)} rad (oude qlog: 6,273 rad = bijna een hele slag de verkeerde kant op)`);
  assert.ok(Math.abs(e - 0.01) < 1e-6);
  assert.ok(Math.hypot(...qlog(qexp([0, 0, 3.5]))) <= Math.PI + 1e-9, '|hoek| ≤ π');
});

await ok('6dof IK (zonder sim): 60 willekeurige bereikbare poses via een teleop-pad — nieuw vs. oud solve(3 it) — fout en max gewrichtsstap', async () => {
  const env = await createHeadlessSim({}); const res = {};
  for (const side of SIDES) {
    const ik = env.ik[side], q0 = ik.currentQ();
    for (let k = 0; k < 30; k++) {
      const qt = q0.map((v, i) => Math.min(ik.rng[i][1] - 0.05, Math.max(ik.rng[i][0] + 0.05, v + (rnd() * 2 - 1) * 0.6)));
      const [tp, tq] = ik.fk(qt); if (tp[2] < 0.12) { k--; continue; }
      const [p0, r0] = ik.fk(q0), lv = rotErr(tq, r0);
      for (const kind of ['oud', 'nieuw']) {
        let q = q0.slice(), mx = 0;
        for (let f = 1; f <= 60; f++) {
          const a = Math.min(1, f / 30), pp = p0.map((v, i) => v + (tp[i] - v) * a), qq = qmulW(qexp(lv.map(x => x * a)), r0);
          let qn = kind === 'oud' ? ik.solve(q, pp, qq, 3) : ik.solvePose(q, pp, qq, { rest: q0 });
          if (kind === 'nieuw') qn = qn.map((v, i) => q[i] + Math.max(-JOINT_SPEED / 30, Math.min(JOINT_SPEED / 30, v - q[i])));
          mx = Math.max(mx, ...qn.map((v, i) => Math.abs(v - q[i]))); q = qn;
        }
        const e = ik.poseError(q, tp, tq); (res[kind] ||= []).push({ pos: e.pos * 1000, rot: e.rot / deg, mx, nan: !q.every(Number.isFinite) });
      }
    }
  }
  const st = (a, f) => { const v = a.map(f).sort((x, y) => x - y); return [v[v.length >> 1], v[Math.floor(v.length * 0.95)], v[v.length - 1]]; };
  for (const k of ['oud', 'nieuw']) { const P = st(res[k], x => x.pos), R = st(res[k], x => x.rot), M = st(res[k], x => x.mx);
    console.log(`   ${k.padEnd(5)}: positie mediaan/95%/max ${P.map(x => x.toFixed(2)).join('/')} mm, oriëntatie ${R.map(x => x.toFixed(2)).join('/')}°, max gewrichtsstap per tick ${M[2].toFixed(2)} rad`); }
  const P = st(res.nieuw, x => x.pos), R = st(res.nieuw, x => x.rot);
  assert.ok(P[2] < 2 && R[2] < 1, 'nieuwe IK: max < 2 mm / 1°'); assert.ok(!res.nieuw.some(x => x.nan));
  assert.ok(Math.max(...res.nieuw.map(x => x.mx)) <= JOINT_SPEED / 30 + 1e-9, 'geen sprongen');
});

for (const side of SIDES) {
  await ok(`6dof (sim, ${side}): 10 willekeurige bereikbare doelposes (positie + oriëntatie) via de controller → TCP-fout < 5 mm / < 3°, geen NaN, geen sprongen`, async () => {
    const errs = [];
    for (let k = 0; k < 10; k++) {
      const r = await rig({ relax: true, mode: '6dof' }); await engage6(r);
      const ik = r.env.ik[side], q0 = r.env.qTarget[side].slice(), A = anchorPose(r, side);
      let qt, tp, tq; do { qt = q0.map((v, i) => Math.min(ik.rng[i][1] - 0.05, Math.max(ik.rng[i][0] + 0.05, v + (rnd() * 2 - 1) * 0.5))); [tp, tq] = ik.fk(qt); } while (tp[2] < 0.12);
      await drive(r, side, mjVecToWorld(tp.map((v, i) => v - A.p[i])), mjDeltaToWorld(qmulW(tq, qconjW(A.q))), 40, 80);
      const T = r.env.tcpPose(side), ep = Math.hypot(...T.pos.map((v, i) => v - tp[i])) * 1000, er = angDeg(tq, T.quat);
      errs.push([ep, er]); assert.ok(!r.log.nan); assert.ok(r.log.dqMax[side] <= JOINT_SPEED / 30 + 1e-9, 'max stap ' + r.log.dqMax[side]);
    }
    const mp = Math.max(...errs.map(e => e[0])), mr = Math.max(...errs.map(e => e[1])), med = a => a.sort((x, y) => x - y)[a.length >> 1];
    console.log(`   ${side}: TCP-fout (werkelijk, na servo) mediaan ${med(errs.map(e => e[0])).toFixed(2)} mm / ${med(errs.map(e => e[1])).toFixed(2)}°, max ${mp.toFixed(2)} mm / ${mr.toFixed(2)}°`);
    assert.ok(mp < 5 && mr < 3);
  });
}

await ok('6dof: pure controller-rol/-tilt/-yaw (om de eigen assen, ±20°, ook met schuin gehouden controller) → gripper draait dezelfde wereld-rotatie, TCP blijft staan', async () => {
  const lines = [];
  for (const [label, o0] of [['recht', [0, 0, 0, 1]], ['schuin (40° omlaag, 25° gedraaid)', qm(qAxis([0, 1, 0], 25 * deg), qAxis([1, 0, 0], -40 * deg))]]) {
    for (const [name, ax] of [['rol (−z)', [0, 0, -1]], ['tilt (+x)', [1, 0, 0]], ['yaw (+y)', [0, 1, 0]]]) for (const sg of [1, -1]) {
      const r = await rig({ relax: true, mode: '6dof' }); await engage6(r, o0);
      const A = anchorPose(r, 'right'), T0 = r.env.tcpPose('right'), C0 = X2W(r.ctrl.right.orient);
      const C1 = qmulW(C0, qexp(ax.map(x => x * sg * 20 * deg)));                 // draai om de EIGEN (lichaams-)as
      await drive(r, 'right', [0, 0, 0], qmulW(C1, qconjW(C0)), 30, 60);
      const want = qmulW(worldDeltaToMj(qmulW(C1, qconjW(C0))), A.q), T = r.env.tcpPose('right');
      const err = angDeg(want, T.quat), turned = angDeg(T.quat, T0.quat), dp = Math.hypot(...T.pos.map((v, i) => v - T0.pos[i])) * 1000;
      lines.push(`${label} ${name} ${sg > 0 ? '+' : '−'}20°: gripper ${turned.toFixed(1)}° gedraaid, afwijking ${err.toFixed(2)}°, TCP ${dp.toFixed(1)} mm`);
      assert.ok(err < 3, lines.at(-1)); assert.ok(Math.abs(turned - 20) < 3, lines.at(-1)); assert.ok(dp < 5, lines.at(-1)); assert.ok(!r.log.nan);
    }
  }
  for (const l of lines) console.log('   ' + l);
});

await ok('6dof: combinatie (9 cm verplaatsen + 15°/−20°/25° draaien tegelijk) en tweede clutch (verder vanaf nieuwe stand, zonder sprong bij engage)', async () => {
  const r = await rig({ relax: true, mode: '6dof' }); await engage6(r);
  const A = anchorPose(r, 'right'), dQw = qexp([15 * deg, -20 * deg, 25 * deg]), dp = [-0.04, 0.05, -0.06];
  await drive(r, 'right', dp, dQw, 40, 80);
  const want = qmulW(worldDeltaToMj(dQw), A.q), wantP = A.p.map((v, i) => v + threeToMj(worldToRoot([dp[0] + ROOT_POS[0], dp[1] + ROOT_POS[1], dp[2] + ROOT_POS[2]], ROOT_POS, HOME_ROT))[i]);
  let T = r.env.tcpPose('right'); const e1 = [Math.hypot(...T.pos.map((v, i) => v - wantP[i])) * 1000, angDeg(want, T.quat)];
  r.clutch(false); r.step(20); const Tr = r.env.tcpPose('right'); r.log.dqMax.right = 0; r.clutch(true); r.step(10);
  T = r.env.tcpPose('right'); const jump = [Math.hypot(...T.pos.map((v, i) => v - Tr.pos[i])) * 1000, angDeg(T.quat, Tr.quat)], dqEngage = r.log.dqMax.right;
  const A2 = anchorPose(r, 'right'), dQ2 = qexp([0, 0, -20 * deg]); await drive(r, 'right', [0.03, -0.04, 0], dQ2, 30, 60);
  T = r.env.tcpPose('right'); const want2 = qmulW(worldDeltaToMj(dQ2), A2.q), e2 = angDeg(want2, T.quat);
  console.log(`   combinatie: fout ${e1[0].toFixed(2)} mm / ${e1[1].toFixed(2)}°; nieuwe grip: verschuiving ${jump[0].toFixed(2)} mm / ${jump[1].toFixed(2)}° (max Δq ${dqEngage.toFixed(4)} rad); 2e beweging fout ${e2.toFixed(2)}°`);
  assert.ok(e1[0] < 5 && e1[1] < 3); assert.ok(jump[0] < 2 && jump[1] < 1 && dqEngage < 0.02, 'geen sprong bij engage'); assert.ok(e2 < 3);
});

await ok('6dof: onbereikbaar (170° draai + 60 cm weg) → binnen jointlimieten, geen NaN, ≤ JOINT_SPEED·dt per tick; terug → weer op het startpunt', async () => {
  const r = await rig({ relax: true, mode: '6dof' }); await engage6(r);
  const T0 = r.env.tcpPose('right'), ik = r.env.ik.right;
  r.log.dqMax.right = 0; await drive(r, 'right', [0.3, 0.2, -0.5], qexp([0, 170 * deg, 0]), 10, 80);
  const q = ik.currentQ(), qt = r.env.qTarget.right, inLim = qt.every((v, i) => v >= ik.rng[i][0] - 1e-9 && v <= ik.rng[i][1] + 1e-9);
  const eUn = r.log.err.right;
  assert.ok(!r.log.nan && q.every(Number.isFinite) && inLim, 'binnen limieten'); assert.ok(r.log.dqMax.right <= JOINT_SPEED / 30 + 1e-9);
  const c = r.ctrl.right; const back = async () => { const pT = [0.22, 1.0, -0.3], o = X2W(c.orient), lv = qlog(qconjW(o)); const p0 = c.pos.slice();
    for (let k = 1; k <= 40; k++) { const a = k / 40; c.pos = p0.map((v, i) => v + a * (pT[i] - v)); c.orient = W2X(qmulW(qexp(lv.map(x => x * a)), o)); r.step(1); } r.step(120); };
  await back();
  const T = r.env.tcpPose('right'), dp = Math.hypot(...T.pos.map((v, i) => v - T0.pos[i])) * 1000, dr = angDeg(T.quat, T0.quat);
  console.log(`   onbereikbaar: IK-rest ${(eUn.pos * 1000).toFixed(0)} mm / ${(eUn.rot / deg).toFixed(0)}° (dichtstbijzijnde), max Δq ${r.log.dqMax.right.toFixed(3)} rad/tick; terug: ${dp.toFixed(2)} mm / ${dr.toFixed(2)}° van start`);
  assert.ok(dp < 5 && dr < 3, 'terug op start');
});

await ok('6dof: pols-singulariteit (joint5 ≈ 0: joint4 ∥ joint6) — draaien door/naast de singulariteit zonder NaN of sprongen', async () => {
  const r = await rig({ relax: true, mode: '6dof' });
  r.env.qTarget.right[4] = 0.0; r.env.ik.right.apply(r.env.qTarget.right, 0); r.ctrl.right.grip = 0; r.ctrl.left.grip = 0; r.step(60);
  await engage6(r); const A = anchorPose(r, 'right'), j5s = r.env.qTarget.right[4]; r.log.dqMax.right = 0;
  const dQw = qexp([0, 0, 35 * deg]); await drive(r, 'right', [0, 0, 0], dQw, 30, 80);
  const T = r.env.tcpPose('right'), want = qmulW(worldDeltaToMj(dQw), A.q), e = angDeg(want, T.quat), dp = Math.hypot(...T.pos.map((v, i) => v - A.p[i])) * 1000;
  console.log(`   joint5 bij engage ${(j5s / deg).toFixed(1)}° → ${(r.env.qTarget.right[4] / deg).toFixed(1)}° (pols-herconfiguratie j4/j6); fout ${e.toFixed(2)}°, TCP ${dp.toFixed(1)} mm, max Δq ${r.log.dqMax.right.toFixed(3)} rad/tick`);
  assert.ok(!r.log.nan && r.log.dqMax.right <= JOINT_SPEED / 30 + 1e-9); assert.ok(e < 3 && dp < 5);
});

await ok('6dof: links/rechts — linker controller draait + verplaatst alleen de linker gripper (rechter < 1 mm / 0,3°); linker volgt correct', async () => {
  const r = await rig({ relax: true, mode: '6dof' }); await engage6(r);
  const R0 = r.env.tcpPose('right'), A = anchorPose(r, 'left'), dQw = qexp([0, 30 * deg, 0]);
  await drive(r, 'left', [-0.06, 0, -0.05], dQw, 30, 60);
  const R1 = r.env.tcpPose('right'), L = r.env.tcpPose('left'), want = qmulW(worldDeltaToMj(dQw), A.q);
  const dR = [Math.hypot(...R1.pos.map((v, i) => v - R0.pos[i])) * 1000, angDeg(R1.quat, R0.quat)], eL = angDeg(want, L.quat);
  const lw = mjToWorld(L.pos, ROOT_POS, HOME_ROT), aw = mjToWorld(A.p, ROOT_POS, HOME_ROT), dpL = lw.map((v, i) => v - aw[i]);
  console.log(`   rechter arm: ${dR[0].toFixed(2)} mm / ${dR[1].toFixed(2)}°; linker oriëntatiefout ${eL.toFixed(2)}°, linker verplaatsing (wereld) ${r3(dpL)} (gewenst -0.06,0,-0.05)`);
  assert.ok(dR[0] < 1 && dR[1] < 0.3); assert.ok(eL < 3); assert.ok(Math.hypot(dpL[0] + 0.06, dpL[1], dpL[2] + 0.05) < 0.005);
});
console.log(`${n} tests ok`);
process.exit(0);
