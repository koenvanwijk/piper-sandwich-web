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

/** Eigen-ontbinding van een symmetrische 6x6-matrix (cyclische Jacobi). Geeft { d: eigenwaarden[6], V: kolommen = eigenvectoren }. */
export function eigSym6(A) {
  const n = 6, a = A.map(r => r.slice()), V = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
  for (let sweep = 0; sweep < 30; sweep++) {
    let off = 0; for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += a[i][j] * a[i][j];
    if (off < 1e-30) break;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(a[p][q]) < 1e-300) continue;
      const th = (a[q][q] - a[p][p]) / (2 * a[p][q]), t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < n; k++) { const akp = a[k][p], akq = a[k][q]; a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq; }
      for (let k = 0; k < n; k++) { const apk = a[p][k], aqk = a[q][k]; a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk; }
      for (let k = 0; k < n; k++) { const vkp = V[k][p], vkq = V[k][q]; V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq; }
    }
  }
  return { d: a.map((r, i) => r[i]), V };
}

// ---- 6-DOF pose-IK (standaard teleop) -----------------------------------------------------------------------------------------
// Gewogen damped-least-squares op [positie (m); W_ROT · rotatievector (rad)]:
//  - W_ROT: 1 rad oriëntatiefout telt als 0,10 m positiefout (3° ≈ 5 mm) (zonder weging telde 1 rad = 1 m, waardoor de oude ?rot=1-IK
//    positie opofferde voor oriëntatie zodra het doel net buiten bereik lag).
//  - SELECTIEVE singulariteits-demping: JᵀJ = V·diag(σ²)·Vᵀ (Jacobi); alleen richtingen met σ² < S0 krijgen extra demping
//    λ² = LAMBDA_MAX²·(1 − σ²/S0) (pols-singulariteit joint5 ≈ 0, waar joint4 ∥ joint6). De goed geconditioneerde richtingen
//    blijven ongedempt, dus de rest van de pose convergeert gewoon; geen wilde gewrichtssnelheden in de slechte richting.
//  - null-space-bias naar een rusthouding: + Σ v_i·λn²/(σ_i² + λn²)·v_iᵀ·k(q_rest − q) ≈ (I − J⁺J)·k(q_rest − q): werkt alleen in
//    richtingen die de taak niet nodig heeft (bv. j4/j6 tegen elkaar in bij joint5 ≈ 0) en houdt de arm bij de engage-houding.
//  - jointlimieten: actieve set — een gewricht op zijn limiet dat verder naar buiten wil, wordt voor die iteratie vastgezet.
//  - max stap per iteratie, warm start (vorige oplossing), beste iterand onthouden (onbereikbaar → dichtstbijzijnde oplossing).
export const IK6 = { W_ROT: 0.1, LAMBDA0: 0.002, LAMBDA_MAX: 0.02, S0: 5e-5, LAMBDA_NULL: 0.005, REST_GAIN: 0.05, ITER_STEP: 0.2, ITERS: 10,
                     TOL_POS: 2e-4, TOL_ROT: 1e-3 };

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

  /** freeze: joint-indices (0..5) die niet meebewegen — voor wrist-overlay (j4/j5/j6) zodat IK de offsets niet wegwerkt. */
  solve(q, tpos, tquat, iters = 3, { freeze = [] } = {}) {
    q = q.slice();
    const frozen = new Set(freeze);
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
      for (let i = 0; i < 6; i++) {
        if (frozen.has(i)) continue;
        q[i] = Math.min(this.rng[i][1], Math.max(this.rng[i][0],
                 q[i] + Math.max(-this.maxStep, Math.min(this.maxStep, dq[i]))));
      }
      if (Math.hypot(...err) < 1e-4) break;
    }
    return q;
  }

  /** Gewogen fout [ep; W_ROT·er] en ongewogen normen voor een gewrichtsstand. */
  poseError(q, tpos, tquat, wRot = IK6.W_ROT) {
    const [p, qq] = this.fk(q), er = rotErr(tquat, qq);
    const ep = [tpos[0]-p[0], tpos[1]-p[1], tpos[2]-p[2]];
    return { e: [...ep, ...er.map(x => x * wRot)], pos: Math.hypot(...ep), rot: Math.hypot(...er), p, q: qq };
  }

  /** Robuuste 6-DOF IK (zie IK6 hierboven). opts: { iters, wRot, rest (q-rusthouding), stats (object, wordt gevuld) }. */
  solvePose(q, tpos, tquat, opts = {}) {
    const o = { ...IK6, ...opts }, wR = o.wRot ?? o.W_ROT, iters = o.iters ?? o.ITERS, rest = o.rest || null;
    q = q.slice();
    let cur = this.poseError(q, tpos, tquat, wR), best = { q: q.slice(), cost: Math.hypot(...cur.e), cur }, lastL2 = 0, it = 0;
    for (; it < iters; it++) {
      if (cur.pos < o.TOL_POS && cur.rot < o.TOL_ROT) break;
      const J = [[], [], [], [], [], []];
      for (let j = 0; j < 6; j++) {
        const qj = q.slice(); qj[j] += this.eps;
        const [p1, q1] = this.fk(qj);
        const dr = rotErr(q1, cur.q);
        for (let r = 0; r < 3; r++) { J[r][j] = (p1[r] - cur.p[r]) / this.eps; J[r+3][j] = wR * dr[r] / this.eps; }
      }
      const JTJ = [], JTe = [];
      for (let a = 0; a < 6; a++) {
        let g = 0; for (let k = 0; k < 6; k++) g += J[k][a] * cur.e[k]; JTe.push(g);
        JTJ.push([]); for (let b = 0; b < 6; b++) { let v = 0; for (let k = 0; k < 6; k++) v += J[k][a] * J[k][b]; JTJ[a].push(v); }
      }
      const z = rest ? rest.map((r, i) => o.REST_GAIN * (r - q[i])) : null;
      let frozen = new Set(), dq;
      for (let pass = 0; pass < 3; pass++) {                       // actieve set voor jointlimieten
        const A = JTJ.map((r, a) => r.map((v, b) => frozen.has(a) || frozen.has(b) ? (a === b ? 1 : 0) : v));
        const g = JTe.map((v, a) => frozen.has(a) ? 0 : v);
        const { d, V } = eigSym6(A), ln2 = o.LAMBDA_NULL ** 2;
        dq = [0, 0, 0, 0, 0, 0]; let lmax = 0;
        for (let k = 0; k < 6; k++) {
          const s2 = Math.max(0, d[k]), l2 = o.LAMBDA0 ** 2 + (s2 < o.S0 ? o.LAMBDA_MAX ** 2 * (1 - s2 / o.S0) : 0);
          lmax = Math.max(lmax, l2);
          let vg = 0, vz = 0; for (let i = 0; i < 6; i++) { vg += V[i][k] * g[i]; if (z && !frozen.has(i)) vz += V[i][k] * z[i]; }
          const c = vg / (s2 + l2) + (z ? vz * ln2 / (s2 + ln2) : 0);
          for (let i = 0; i < 6; i++) if (!frozen.has(i)) dq[i] += c * V[i][k];
        }
        lastL2 = lmax;
        let changed = false;
        for (let i = 0; i < 6; i++) if (!frozen.has(i)) {
          const m = 1e-4;
          if ((q[i] <= this.rng[i][0] + m && dq[i] < 0) || (q[i] >= this.rng[i][1] - m && dq[i] > 0)) { frozen.add(i); changed = true; }
        }
        if (!changed) break;
      }
      const mx = Math.max(...dq.map(Math.abs)), sc = mx > o.ITER_STEP ? o.ITER_STEP / mx : 1;   // richting behouden
      for (let i = 0; i < 6; i++) q[i] = Math.min(this.rng[i][1], Math.max(this.rng[i][0], q[i] + sc * dq[i]));
      if (!q.every(Number.isFinite)) { q = best.q.slice(); break; }
      cur = this.poseError(q, tpos, tquat, wR);
      const cost = Math.hypot(...cur.e);
      if (cost < best.cost) best = { q: q.slice(), cost, cur };
    }
    if (opts.stats) Object.assign(opts.stats, { iters: it, pos: best.cur.pos, rot: best.cur.rot, lambda: Math.sqrt(lastL2) });
    return best.q;
  }

  /** Alleen positie-IK (geen oriëntatiefout); optioneel freeze van joint-indices. */
  solvePos(q, tpos, iters = 3, { freeze = [] } = {}) {
    q = q.slice();
    const frozen = new Set(freeze);
    for (let it = 0; it < iters; it++) {
      const [p0] = this.fk(q);
      const err = [tpos[0]-p0[0], tpos[1]-p0[1], tpos[2]-p0[2]];
      const J = [[], [], []];
      for (let j = 0; j < 6; j++) {
        const qj = q.slice(); qj[j] += this.eps;
        const [p1] = this.fk(qj);
        for (let r = 0; r < 3; r++) J[r][j] = (p1[r] - p0[r]) / this.eps;
      }
      // 6×6 DLS op positie: J is 3×6, bouw JTJ
      const l2 = this.damping * this.damping, A = [], JTe = [];
      for (let a = 0; a < 6; a++) {
        let s = 0; for (let k = 0; k < 3; k++) s += J[k][a] * err[k]; JTe.push(s);
        A.push(new Array(6).fill(0));
      }
      for (let a = 0; a < 6; a++) for (let b = 0; b < 6; b++) {
        let s = 0; for (let k = 0; k < 3; k++) s += J[k][a] * J[k][b];
        A[a][b] = s + (a === b ? l2 : 0);
      }
      const dq = solve6(A, JTe);
      for (let i = 0; i < 6; i++) {
        if (frozen.has(i)) continue;
        q[i] = Math.min(this.rng[i][1], Math.max(this.rng[i][0],
                 q[i] + Math.max(-this.maxStep, Math.min(this.maxStep, dq[i]))));
      }
      if (Math.hypot(...err) < 1e-4) break;
    }
    return q;
  }

  /** Offset-vensters [min,max] (rad) t.o.v. huidige q, binnen jointlimiet (kleine marge). */
  rollLimits(q) { const m = 0.02; return [this.rng[5][0] + m - q[5], this.rng[5][1] - m - q[5]]; }  // joint6
  tiltLimits(q) { const m = 0.02; return [this.rng[4][0] + m - q[4], this.rng[4][1] - m - q[4]]; }  // joint5
  yawLimits(q)  { const m = 0.02; return [this.rng[3][0] + m - q[3], this.rng[3][1] - m - q[3]]; }  // joint4
  withRoll(q, roll) { if (!roll) return q; const r = q.slice(); r[5] = Math.min(this.rng[5][1], Math.max(this.rng[5][0], q[5] + roll)); return r; }
  withTilt(q, tilt) { if (!tilt) return q; const r = q.slice(); r[4] = Math.min(this.rng[4][1], Math.max(this.rng[4][0], q[4] + tilt)); return r; }
  withYaw(q, yaw)   { if (!yaw)  return q; const r = q.slice(); r[3] = Math.min(this.rng[3][1], Math.max(this.rng[3][0], q[3] + yaw));  return r; }
  /** qIK + yaw→j4 + tilt→j5 + rol→j6 (geklemd). */
  withWrist(q, roll = 0, tilt = 0, yaw = 0) { return this.withRoll(this.withTilt(this.withYaw(q, yaw), tilt), roll); }

  // write arm + gripper targets into ctrl. grip: 0 open .. 1 closed
  apply(q, grip) {
    const c = this.data.ctrl;
    for (let i = 0; i < 6; i++) c[this.act[i]] = q[i];
    const open7 = 0.035, closed7 = 0.0;
    const j7 = open7 + (closed7 - open7) * grip;
    c[this.grip[0]] = j7; c[this.grip[1]] = -j7;
  }
}
