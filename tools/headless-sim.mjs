// Headless MuJoCo-opzet voor Node (zonder three.js), gedeeld door de demo-scripts.
// Laadt assets/scene.xml + meshes in het MEMFS, zet de 'home'-keyframe en geeft
// dezelfde velden als SandwichVR in src/app.js (ik, qTarget, grip, tcpPose).
// LET OP: in Node worden mesh_vert NIET geswizzled (dat doet alleen de three.js-loader).
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import load_mujoco from '../vendor/mujoco/mujoco.js';
import { ArmIK } from '../src/ik.js';
import { mat2quat } from '../src/qmath.js';
import { relaxBaseContacts } from '../src/sandwich-motion.js';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');

export async function createHeadlessSim({ relaxBase = true } = {}) {
  const mj = await load_mujoco();
  mj.FS.mkdir('/working'); mj.FS.mount(mj.MEMFS, { root: '.' }, '/working');
  mj.FS.mkdir('/working/meshes');
  mj.FS.writeFile('/working/scene.xml', fs.readFileSync(path.join(ROOT, 'assets/scene.xml'), 'utf8'));
  const md = path.join(ROOT, 'assets/meshes');
  for (const f of fs.readdirSync(md)) mj.FS.writeFile('/working/meshes/' + f, fs.readFileSync(path.join(md, f)));
  const model = mj.MjModel.mj_loadXML('/working/scene.xml');
  const data = new mj.MjData(model);
  if (relaxBase) relaxBaseContacts(model);   // zie sandwich-motion.js
  mj.mj_resetData(model, data);
  if (model.nkey > 0) data.qpos.set(model.key_qpos.slice(0, model.nq));
  mj.mj_forward(model, data);

  const ik = {}, qTarget = {}, grip = {};
  for (const s of ['left', 'right']) {
    ik[s] = new ArmIK(mj, model, data, s);
    qTarget[s] = ik[s].currentQ(); grip[s] = 0;
    ik[s].apply(qTarget[s], grip[s]);
  }
  const bodyId = n => mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, n);
  const env = {
    mujoco: mj, model, data, ik, qTarget, grip,
    tcpPose(side) {
      const s = ik[side].site, d = data, m = [];
      for (let k = 0; k < 9; k++) m.push(d.site_xmat[9 * s + k]);
      return { pos: [d.site_xpos[3*s], d.site_xpos[3*s+1], d.site_xpos[3*s+2]], quat: mat2quat(m) };
    },
    bodyPos(name) {
      const id = bodyId(name);
      return [data.xpos[3*id], data.xpos[3*id+1], data.xpos[3*id+2]];
    },
    /** ~ app.js: 25 mj_step per 0.05 s frame (timestep 0.002) */
    stepPhysics(seconds) {
      const n = Math.max(1, Math.round(seconds / model.opt.timestep));
      for (let i = 0; i < n; i++) mj.mj_step(model, data);
    },
  };
  return env;
}
