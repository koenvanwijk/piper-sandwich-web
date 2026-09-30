#!/usr/bin/env node
// Test (Node, MuJoCo-wasm, geen browser): controller (WebXR-wereld: x rechts, y omhoog, -z vooruit) -> TCP-beweging in de scène
// zoals de gebruiker hem ziet. Gebruikt dezelfde stappen als app.js (HandTeleop + ArmIK + 16 substappen van 1/30 s).
//   node tools/test-teleop-mapping.mjs
import assert from 'node:assert/strict';
import { createHeadlessSim } from './headless-sim.mjs';
import { HandTeleop } from '../src/teleop.js';
import { relaxBaseContacts } from '../src/sandwich-motion.js';
import { controllerToRaw, mjToWorld, headYaw, homeFromHead, worldToRoot, rootToWorld, threeToMj } from '../src/xr-map.js';

const ROOT_POS = [0, 0.95, -0.32], HOME_ROT = Math.PI / 2;         // = SandwichVR.init()
const SIDES = ['left', 'right'];
let n = 0; const ok = (name, f) => Promise.resolve(f()).then(() => { n++; console.log('ok', name); });
const r3 = a => a.map(x => +x.toFixed(3));

async function rig({ relax, lockOrientation = true, rootRotY = HOME_ROT, rootPos = ROOT_POS }) {
  const env = await createHeadlessSim({ relaxBase: false });
  if (relax) relaxBaseContacts(env.model);
  const teleop = { left: new HandTeleop({ lockOrientation }), right: new HandTeleop({ lockOrientation }) };
  const ctrl = { left: { pos: [-0.22, 1.0, -0.3], orient: [0, 0, 0, 1], grip: 1 }, right: { pos: [0.22, 1.0, -0.3], orient: [0, 0, 0, 1], grip: 1 } };
  const tcpWorld = s => mjToWorld(env.tcpPose(s).pos, rootPos, rootRotY);
  const step = (k = 1) => {
    for (let i = 0; i < k; i++) {
      for (const s of SIDES) {
        const c = ctrl[s], raw = { ...controllerToRaw(c.pos, c.orient, rootPos, rootRotY), trigger: 0, grip: c.grip };
        const t = teleop[s].step(raw, env.tcpPose(s));
        if (t.engaged) env.qTarget[s] = env.ik[s].solve(env.qTarget[s], t.pos, t.quat, 3);
        env.grip[s] = t.grip; env.ik[s].apply(env.qTarget[s], env.grip[s]);
      }
      for (let j = 0; j < 16; j++) env.mujoco.mj_step(env.model, env.data);
    }
  };
  const clutch = on => { for (const s of SIDES) ctrl[s].grip = on ? 1 : 0; step(1); };
  return { env, ctrl, step, clutch, tcpWorld };
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
console.log(`${n} tests ok`);
process.exit(0);
