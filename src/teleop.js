// Clutched relative teleop: map a VR controller's motion (three.js world frame)
// to a MuJoCo TCP target. While the grip is held the arm mirrors the motion
// since the clutch engaged; the trigger closes the gripper.
import { qmul, qconj } from './qmath.js';
import { IK6 } from './ik.js';

// three.js (y-up) -> MuJoCo (z-up) is a +90° rotation about x (see scene-loader
// swizzles). Position: (x,y,z)_three -> (x,-z,y)_mujoco. Orientation: conjugate
// by q_A.
const QA = [Math.SQRT1_2, Math.SQRT1_2, 0, 0];
const QA_INV = qconj(QA);
const threeQuatToMujoco = q => qmul(qmul(QA, q), QA_INV);
const threeVecToMujoco = v => [v[0], -v[2], v[1]];

const CLUTCH_ON = 0.6, CLUTCH_OFF = 0.4;

// ---- pols (standaard 'roll'-modus: yaw→j4, tilt→j5, rol→j6) -----------------------------------------------------------------
// OORZAAK: TCP-oriëntatie stond standaard VERGRENDELD; ?rot=1 gaf volledige 6-DOF-IK die rol over j4+j6 spreidt.
// Nu: body-frame twist om de drie controller-assen → exact joint4/5/6 (geen IK-spreiding), positie blijft clutched.
export const ROLL_AXIS = [0, 0, -1];            // grip −z = wijsrichting (rol)
export const TILT_AXIS = [1, 0, 0];             // grip +x = rechts (pitch/tilt)
export const YAW_AXIS  = [0, 1, 0];             // grip +y = omhoog langs handvat (yaw)
export const ROLL_DEADZONE = 0.03, TILT_DEADZONE = 0.03, YAW_DEADZONE = 0.03;
export const ROLL_MAX_STEP = 0.12, TILT_MAX_STEP = 0.12, YAW_MAX_STEP = 0.12;

/** Signed twist (rad) van qa→qb ([w,x,y,z]) om een lokale as. */
export function twistAbout(qa, qb, axis = ROLL_AXIS) {
  const d = qmul(qconj(qa), qb);
  const n = Math.hypot(...axis) || 1;
  const proj = (d[1]*axis[0] + d[2]*axis[1] + d[3]*axis[2]) / n;
  return 2 * Math.atan2(proj, d[0] >= 0 ? d[0] : -d[0]) * (d[0] >= 0 ? 1 : -1);
}

/** Teleop-modus uit de query-string:
 *  '6dof'   (standaard; ook ?rot=1 of ?mode=6dof): TCP volgt de volledige relatieve controller-pose via 6-DOF-IK over alle joints;
 *  'joints' (?mode=joints, oude PR #11-mapping): positie-IK met vergrendelde oriëntatie + yaw→j4, tilt→j5, rol→j6;
 *  'lock'   (?orient=0, ?rot=0 of ?roll=0): alleen positie, pols-oriëntatie vast (oudste gedrag). */
export function orientationMode(search) {
  const q = new URLSearchParams(search || '');
  if (q.get('orient') === '0' || q.get('rot') === '0' || q.get('roll') === '0') return 'lock';
  const m = (q.get('mode') || '').toLowerCase();
  if (m === 'joints' || m === 'roll') return 'joints';
  if (m === 'lock') return 'lock';
  return '6dof';
}
const MODE_ALIAS = { full: '6dof', roll: 'joints' };
export function tiltEnabledFromQuery(search) {
  return new URLSearchParams(search || '').get('tilt') !== '0';
}
export function yawEnabledFromQuery(search) {
  return new URLSearchParams(search || '').get('yaw') !== '0';
}
function axisFromQuery(search, key, fallback) {
  const v = (new URLSearchParams(search || '').get(key) || '').split(',').map(parseFloat);
  return v.length === 3 && v.every(Number.isFinite) && Math.hypot(...v) > 1e-6 ? v : fallback;
}
export function rollAxisFromQuery(search) { return axisFromQuery(search, 'rollaxis', ROLL_AXIS); }
export function tiltAxisFromQuery(search) { return axisFromQuery(search, 'tiltaxis', TILT_AXIS); }
export function yawAxisFromQuery(search)  { return axisFromQuery(search, 'yawaxis',  YAW_AXIS); }

function rateLimitAxis(state, key, rawKey, th, deadzone, maxStep, limits) {
  th += 2 * Math.PI * Math.round((state[rawKey] - th) / (2 * Math.PI));
  state[rawKey] = th;
  const goal0 = Math.abs(th) < deadzone ? 0 : th - Math.sign(th) * deadzone;
  const goal = limits ? Math.min(limits[1], Math.max(limits[0], goal0)) : goal0;
  state[key] += Math.max(-maxStep, Math.min(maxStep, goal - state[key]));
  return state[key];
}

export class HandTeleop {
  /** mode: '6dof' (volledige relatieve pose) | 'joints' (yaw→j4 + tilt→j5 + rol→j6; alias 'roll') | 'lock'.
   *  'full' = alias van '6dof'. lockOrientation (oud): true = 'lock', false = '6dof'; `mode` wint.
   *  tilt/yaw: per-as aan/uit in joints-modus (default true; ?tilt=0 / ?yaw=0). */
  constructor({ scale = 1.0, lockOrientation = true, mode = null,
                rollAxis = ROLL_AXIS, tiltAxis = TILT_AXIS, yawAxis = YAW_AXIS,
                tilt = true, yaw = true } = {}) {
    this.scale = scale;
    const m = mode || (lockOrientation ? 'lock' : '6dof');
    this.mode = MODE_ALIAS[m] || m;
    this.lockOrientation = this.mode !== '6dof';
    this.rollAxis = rollAxis; this.tiltAxis = tiltAxis; this.yawAxis = yawAxis;
    this.tiltEnabled = tilt !== false; this.yawEnabled = yaw !== false;
    this.roll = 0; this._rollRaw = 0;
    this.tilt = 0; this._tiltRaw = 0;
    this.yaw  = 0; this._yawRaw  = 0;
    this.engaged = false;
    this.anchor = null;
    this.lastGrip = 0.0;
  }

  /** limits: { roll, tilt, yaw } elk [min,max], of legacy [min,max] = alleen roll.
   *  returns {engaged, pos, quat, grip, roll, tilt, yaw} */
  step(raw, tcp, limits = null) {
    const L = limits && !Array.isArray(limits) ? limits : { roll: limits };
    if (!raw) {
      this.engaged = false; this.anchor = null;
      this.roll = this.tilt = this.yaw = 0;
      return { engaged: false, grip: this.lastGrip, roll: 0, tilt: 0, yaw: 0 };
    }
    const grip = raw.grip || 0;
    const on = grip > (this.engaged ? CLUTCH_OFF : CLUTCH_ON);
    this.lastGrip = raw.trigger || 0;
    if (!on) {
      this.engaged = false; this.anchor = null;
      this.roll = this.tilt = this.yaw = 0;
      return { engaged: false, grip: this.lastGrip, roll: 0, tilt: 0, yaw: 0 };
    }

    const cpos = raw.pos, cquat = threeQuatToMujoco(raw.quat);
    if (!this.engaged || !this.anchor) {
      this.anchor = { cpos: cpos.slice(), cquat: cquat.slice(),
                      tpos: tcp.pos.slice(), tquat: tcp.quat.slice(), rquat: raw.quat.slice() };
      this.engaged = true;
      this.roll = this._rollRaw = this.tilt = this._tiltRaw = this.yaw = this._yawRaw = 0;
    }
    const a = this.anchor;
    const dThree = [(cpos[0]-a.cpos[0])/this.scale, (cpos[1]-a.cpos[1])/this.scale, (cpos[2]-a.cpos[2])/this.scale];
    const dMj = threeVecToMujoco(dThree);
    const pos = [a.tpos[0]+dMj[0], a.tpos[1]+dMj[1], a.tpos[2]+dMj[2]];

    let quat;
    if (this.lockOrientation) quat = a.tquat;
    else quat = qmul(qmul(cquat, qconj(a.cquat)), a.tquat);

    let roll = 0, tilt = 0, yaw = 0;
    if (this.mode === 'joints' && raw.quat) {
      roll = rateLimitAxis(this, 'roll', '_rollRaw',
        twistAbout(a.rquat, raw.quat, this.rollAxis), ROLL_DEADZONE, ROLL_MAX_STEP, L.roll);
      if (this.tiltEnabled) {
        tilt = rateLimitAxis(this, 'tilt', '_tiltRaw',
          twistAbout(a.rquat, raw.quat, this.tiltAxis), TILT_DEADZONE, TILT_MAX_STEP, L.tilt);
      }
      if (this.yawEnabled) {
        yaw = rateLimitAxis(this, 'yaw', '_yawRaw',
          twistAbout(a.rquat, raw.quat, this.yawAxis), YAW_DEADZONE, YAW_MAX_STEP, L.yaw);
      }
    }
    return { engaged: true, pos, quat, grip: this.lastGrip, roll, tilt, yaw };
  }
}

// Max gewrichtssnelheid in 6dof-modus (rad/s; ≈ 0,12 rad per tick bij 30 Hz): geen sprongen, ook niet als de IK-oplossing springt
// (bv. bij een onbereikbaar doel of een andere tak). De volgende frame start vanaf de begrensde stand (warm start).
export const JOINT_SPEED = 3.6;
export const SING_J5 = 0.2;          // |joint5| < 0,2 rad (~11°) = in de buurt van de pols-singulariteit

/**
 * Eén teleop-stap voor één arm (gedeeld door app.js en de headless tests): controller → doel-pose → IK → st.qTarget[side].
 * st = { qTarget, qIK } (per kant), ik = ArmIK, dt = tijd sinds de vorige stap (s, voor de snelheidsgrens).
 *  - '6dof'  : anker = FK van de gecommandeerde stand (geen sprong bij engage), doel = relatieve controller-pose,
 *              ik.solvePose (gewogen DLS, jointlimieten, null-space-bias naar de engage-houding); qIK = de IK-keten (warm start),
 *              qTarget volgt qIK met max JOINT_SPEED·dt per joint (geen sprongen).
 *              t.err = { pos (m), rot (rad) } = restfout van de IK-oplossing t.o.v. het doel (debug-overlay).
 *  - 'joints': PR #11 (positie-IK + yaw→j4 / tilt→j5 / rol→j6).
 *  - 'lock'  : positie-IK, oriëntatie van het engage-moment.
 */
export function teleopArm(st, side, teleop, ik, raw, tcp, iters = 3, dt = 1 / 30) {
  const wasEngaged = teleop.engaged;
  const seed = wasEngaged && st.qIK[side] ? st.qIK[side] : st.qTarget[side].slice();
  if (teleop.mode === '6dof') {
    let anchorTcp = tcp;
    if (!wasEngaged && raw) { const [p, q] = ik.fk(seed); anchorTcp = { pos: p, quat: q }; }
    const t = teleop.step(raw, anchorTcp);
    if (!t.engaged) { teleop._rest = null; return t; }
    if (!wasEngaged || !teleop._rest) teleop._rest = seed.slice();
    let stats = {};
    let sol = ik.solvePose(seed, t.pos, t.quat, { rest: teleop._rest, stats });
    // Pols-singulariteit (joint5 ≈ 0: joint4 ∥ joint6): daar is de gradiënt naar de juiste joint4-stand nul en blijft DLS in een
    // lokaal minimum hangen (gemeten: 6° fout). joint4 + δ / joint6 − δ verandert de TCP-pose dan (bijna) niet, dus probeer die
    // startpunten en neem een duidelijk betere oplossing; de snelheidsgrens hieronder maakt de overgang vloeiend.
    if (Math.abs(seed[4]) < SING_J5 && (stats.pos > 1e-3 || stats.rot > 0.01)) {
      const cost = s => Math.hypot(s.pos, IK6.W_ROT * s.rot);
      for (const dl of [Math.PI / 4, -Math.PI / 4, Math.PI / 2, -Math.PI / 2, 3 * Math.PI / 4, -3 * Math.PI / 4]) {
        const s2 = seed.slice(); s2[3] += dl; s2[5] -= dl;
        if (s2[3] < ik.rng[3][0] || s2[3] > ik.rng[3][1] || s2[5] < ik.rng[5][0] || s2[5] > ik.rng[5][1]) continue;
        const st2 = {}, q2 = ik.solvePose(s2, t.pos, t.quat, { rest: teleop._rest, stats: st2 });
        if (cost(st2) < 0.5 * cost(stats)) { sol = q2; stats = st2; }
      }
    }
    const prev = st.qTarget[side], maxd = JOINT_SPEED * dt;
    const q = sol.map((v, i) => prev[i] + Math.max(-maxd, Math.min(maxd, v - prev[i])));
    st.qIK[side] = sol;          // IK-keten (warm start, behoudt de gekozen tak) — niet begrensd
    st.qTarget[side] = q;        // gecommandeerd: volgt de IK-oplossing met max JOINT_SPEED
    const atLimit = sol.some((v, i) => v <= ik.rng[i][0] + 1e-3 || v >= ik.rng[i][1] - 1e-3);   // IK-oplossing tegen een jointlimiet
    t.err = { pos: stats.pos, rot: stats.rot, iters: stats.iters, lambda: stats.lambda, atLimit };
    return t;
  }
  const t = teleop.step(raw, tcp, {
    roll: ik.rollLimits(seed), tilt: ik.tiltLimits(seed), yaw: ik.yawLimits(seed),
  });
  if (!t.engaged) { teleop._wristBase = null; return t; }
  if (!wasEngaged || !teleop._wristBase) teleop._wristBase = seed.slice(3, 6);
  if (teleop.mode === 'joints') {
    let q = ik.solve(seed, t.pos, t.quat, iters).slice();
    const b = teleop._wristBase;
    const freeze = [5];
    q[5] = Math.min(ik.rng[5][1], Math.max(ik.rng[5][0], b[2] + (t.roll || 0)));
    if (teleop.yawEnabled) { q[3] = Math.min(ik.rng[3][1], Math.max(ik.rng[3][0], b[0] + (t.yaw || 0))); freeze.push(3); }
    if (teleop.tiltEnabled) { q[4] = Math.min(ik.rng[4][1], Math.max(ik.rng[4][0], b[1] + (t.tilt || 0))); freeze.push(4); }
    if (freeze.length > 1) q = ik.solvePos(q, t.pos, iters, { freeze });
    st.qIK[side] = q.slice();
    st.qIK[side][5] = b[2];
    if (teleop.yawEnabled) st.qIK[side][3] = b[0];
    if (teleop.tiltEnabled) st.qIK[side][4] = b[1];
    st.qTarget[side] = q;
  } else {
    st.qIK[side] = ik.solve(seed, t.pos, t.quat, iters);
    st.qTarget[side] = st.qIK[side];
  }
  return t;
}
