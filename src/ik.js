// Per-arm finite-difference damped-least-squares IK, ported 1:1 from the
// Python sim/ik.py and validated headless against this exact scene.
import { mat2quat, rotErr } from './qmath.js';

const ARM = [1, 2, 3, 4, 5, 6];

function solve6(A, b) {                   // A x = b, A 6x6 row-major
  const n = 6, M = A.map(r => r.slice()), x = b.slice();
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]]; [x[c], x[p]] = [x[p], x[c]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k < n; k++) M[r][k] -= f * M[c][k];
      x[r] -= f * x[c];
    }
  }
  return x.map((v, i) => v / M[i][i]);
}

export class ArmIK {
  constructor(mujoco, model, data, side) {
    this.mj = mujoco; this.model = model; this.data = data; this.side = side;
    const OBJ = mujoco.mjtObj;
    const jid = n => mujoco.mj_name2id(model, OBJ.mjOBJ_JOINT.value, n);
    const aid = n => mujoco.mj_name2id(model, OBJ.mjOBJ_ACTUATOR.value, n);
    this.qadr = ARM.map(j => model.jnt_qposadr[jid(`${side}_joint${j}`)]);
    this.rng = ARM.map(j => { const id = jid(`${side}_joint${j}`);
      return [model.jnt_range[2*id], model.jnt_range[2*id + 1]]; });
    this.act = ARM.map(j => aid(`${side}_joint${j}`));
    this.grip = [aid(`${side}_joint7`), aid(`${side}_joint8`)];
    this.site = mujoco.mj_name2id(model, OBJ.mjOBJ_SITE.value, `${side}_tcp`);
    this.damping = 0.08; this.eps = 1e-4; this.maxStep = 0.25;
    // scratch state so FK probing never disturbs the live simulation
    this.scratch = new mujoco.MjData(model);
    mujoco.mj_resetData(model, this.scratch);
  }

  currentQ() { return this.qadr.map(a => this.data.qpos[a]); }

  fk(q) {
    const d = this.scratch, s = this.site;
    for (let i = 0; i < 6; i++) d.qpos[this.qadr[i]] = q[i];
    this.mj.mj_kinematics(this.model, d);
    this.mj.mj_comPos(this.model, d);
    const p = [d.site_xpos[3*s], d.site_xpos[3*s+1], d.site_xpos[3*s+2]];
    const m = []; for (let k = 0; k < 9; k++) m.push(d.site_xmat[9*s+k]);
    return [p, mat2quat(m)];
  }

  solve(q, tpos, tquat, iters = 3) {
    q = q.slice();
    for (let it = 0; it < iters; it++) {
      const [p0, q0] = this.fk(q);
      const err = [tpos[0]-p0[0], tpos[1]-p0[1], tpos[2]-p0[2], ...rotErr(tquat, q0)];
      const J = [[], [], [], [], [], []];
      for (let j = 0; j < 6; j++) {
        const qj = q.slice(); qj[j] += this.eps;
        const [p1, q1] = this.fk(qj);
        const dp = [(p1[0]-p0[0])/this.eps, (p1[1]-p0[1])/this.eps, (p1[2]-p0[2])/this.eps];
        const dr = rotErr(q1, q0).map(x => x / this.eps);
        for (let r = 0; r < 3; r++) { J[r][j] = dp[r]; J[r+3][j] = dr[r]; }
      }
      const l2 = this.damping*this.damping, A = [], JTe = [];
      for (let a = 0; a < 6; a++) {
        let s = 0; for (let k = 0; k < 6; k++) s += J[k][a]*err[k]; JTe.push(s);
        A.push(new Array(6).fill(0));
      }
      for (let a = 0; a < 6; a++) for (let b = 0; b < 6; b++) {
        let s = 0; for (let k = 0; k < 6; k++) s += J[k][a]*J[k][b];
        A[a][b] = s + (a === b ? l2 : 0);
      }
      const dq = solve6(A, JTe);
      for (let i = 0; i < 6; i++)
        q[i] = Math.min(this.rng[i][1], Math.max(this.rng[i][0],
                 q[i] + Math.max(-this.maxStep, Math.min(this.maxStep, dq[i]))));
      if (Math.hypot(...err) < 1e-4) break;
    }
    return q;
  }

  // write arm + gripper targets into ctrl. grip: 0 open .. 1 closed
  apply(q, grip) {
    const c = this.data.ctrl;
    for (let i = 0; i < 6; i++) c[this.act[i]] = q[i];
    const open7 = 0.035, closed7 = 0.0;
    const j7 = open7 + (closed7 - open7) * grip;
    c[this.grip[0]] = j7; c[this.grip[1]] = -j7;
  }
}
