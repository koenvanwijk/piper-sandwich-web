// Clutched relative teleop: map a VR controller's motion (three.js world frame)
// to a MuJoCo TCP target. While the grip is held the arm mirrors the motion
// since the clutch engaged; the trigger closes the gripper.
import { qmul, qconj } from './qmath.js';

// three.js (y-up) -> MuJoCo (z-up) is a +90° rotation about x (see scene-loader
// swizzles). Position: (x,y,z)_three -> (x,-z,y)_mujoco. Orientation: conjugate
// by q_A.
const QA = [Math.SQRT1_2, Math.SQRT1_2, 0, 0];
const QA_INV = qconj(QA);
const threeQuatToMujoco = q => qmul(qmul(QA, q), QA_INV);
const threeVecToMujoco = v => [v[0], -v[2], v[1]];

const CLUTCH_ON = 0.6, CLUTCH_OFF = 0.4;

// ---- polsrol ('roll'-modus, standaard) -----------------------------------------------------------------------------------------
// OORZAAK van "rol ik de controller, dan rolt de gripper niet": de TCP-oriëntatie stond standaard VERGRENDELD (lockOrientation, zie hieronder);
// alleen met ?rot=1 volgde de volledige oriëntatie, en dan verdeelt de damped-least-squares-IK een rol over joint4 én joint6 (zelfde as als
// joint5 ≈ 0) en meet 'm tegen alle andere draaiingen. Nu: alleen de TWIST van de controller om zijn eigen (wijs)as -> exact joint6.
export const ROLL_AXIS = [0, 0, -1];            // lokale as van de controller (WebXR gripSpace: −z = wijsrichting); overschrijfbaar via ?rollaxis=x,y,z
export const ROLL_DEADZONE = 0.03;              // rad (~1,7°): handtrilling rolt de gripper niet
export const ROLL_MAX_STEP = 0.12;              // rad per stap: geen sprongen (≈ 3,6 rad/s bij 30 Hz, 8,6 rad/s bij 72 Hz)

/** Signed twist (rad, rechtsdraaiend om `axis`) van de lokale-frame-rotatie qa -> qb ([w,x,y,z]) om een lokale as. */
export function twistAbout(qa, qb, axis = ROLL_AXIS) {
  const d = qmul(qconj(qa), qb);                // rotatie in het LICHAAMS-frame van de controller: qb = qa · d
  const n = Math.hypot(...axis) || 1;
  const proj = (d[1]*axis[0] + d[2]*axis[1] + d[3]*axis[2]) / n;
  return 2 * Math.atan2(proj, d[0] >= 0 ? d[0] : -d[0]) * (d[0] >= 0 ? 1 : -1);   // dubbele dekking: qa·(−d) == qa·d
}

/** 'roll' (standaard) | 'full' (?rot=1) | 'lock' (?rot=0 of ?roll=0: oud gedrag) uit een query-string. */
export function orientationMode(search) {
  const q = new URLSearchParams(search || '');
  if (q.get('rot') === '1') return 'full';
  if (q.get('rot') === '0' || q.get('roll') === '0') return 'lock';
  return 'roll';
}
export function rollAxisFromQuery(search) {
  const v = (new URLSearchParams(search || '').get('rollaxis') || '').split(',').map(parseFloat);
  return v.length === 3 && v.every(Number.isFinite) && Math.hypot(...v) > 1e-6 ? v : ROLL_AXIS;
}

export class HandTeleop {
  /** mode: 'lock' (pols-oriëntatie vast), 'roll' (alleen rol om de controller-as -> joint6), 'full' (volledige relatieve oriëntatie).
   *  lockOrientation (oud): true = 'lock', false = 'full'; `mode` wint. */
  constructor({ scale = 1.0, lockOrientation = true, mode = null, rollAxis = ROLL_AXIS } = {}) {
    this.scale = scale;
    this.mode = mode || (lockOrientation ? 'lock' : 'full');
    this.lockOrientation = this.mode !== 'full';
    this.rollAxis = rollAxis;
    this.roll = 0; this._rollRaw = 0;
    this.engaged = false;
    this.anchor = null;              // {cpos, cquat, tpos, tquat}
    this.lastGrip = 0.0;
  }

  /** raw: {pos:[x,y,z] three-world, quat:[w,x,y,z] three, trigger, grip} | null
   *  tcp: {pos:[x,y,z] mujoco, quat:[w,x,y,z] mujoco}  (current arm TCP)
   *  limits: optioneel [min,max] (rad) van de toegestane rol-offset (jointlimieten van joint6 t.o.v. de huidige stand)
   *  returns {engaged, pos, quat, grip, roll}          (targets in mujoco frame; roll = extra joint6-offset in rad, alleen mode 'roll') */
  step(raw, tcp, limits = null) {
    if (!raw) { this.engaged = false; this.anchor = null; this.roll = 0;
                return { engaged: false, grip: this.lastGrip, roll: 0 }; }

    const grip = raw.grip || 0;
    const on = grip > (this.engaged ? CLUTCH_OFF : CLUTCH_ON);
    this.lastGrip = raw.trigger || 0;

    if (!on) { this.engaged = false; this.anchor = null; this.roll = 0;
               return { engaged: false, grip: this.lastGrip, roll: 0 }; }

    const cpos = raw.pos;
    const cquat = threeQuatToMujoco(raw.quat);

    if (!this.engaged || !this.anchor) {           // rising edge -> anchor
      this.anchor = { cpos: cpos.slice(), cquat: cquat.slice(),
                      tpos: tcp.pos.slice(), tquat: tcp.quat.slice(), rquat: raw.quat.slice() };
      this.engaged = true; this.roll = 0; this._rollRaw = 0;
    }
    const a = this.anchor;

    const dThree = [ (cpos[0]-a.cpos[0]) / this.scale,
                     (cpos[1]-a.cpos[1]) / this.scale,
                     (cpos[2]-a.cpos[2]) / this.scale ];
    const dMj = threeVecToMujoco(dThree);
    const pos = [a.tpos[0]+dMj[0], a.tpos[1]+dMj[1], a.tpos[2]+dMj[2]];

    let quat;
    if (this.lockOrientation) {
      quat = a.tquat;                              // keep engage-time wrist pose
    } else {
      const dQ = qmul(cquat, qconj(a.cquat));      // controller rotation delta
      quat = qmul(dQ, a.tquat);
    }
    let roll = 0;
    if (this.mode === 'roll' && raw.quat) {
      let th = twistAbout(a.rquat, raw.quat, this.rollAxis);
      th += 2 * Math.PI * Math.round((this._rollRaw - th) / (2 * Math.PI));   // doorlopend (unwrapped), zodat > 180° kan
      this._rollRaw = th;
      const goal0 = Math.abs(th) < ROLL_DEADZONE ? 0 : th - Math.sign(th) * ROLL_DEADZONE;   // dode zone zonder sprong op de rand
      const goal = limits ? Math.min(limits[1], Math.max(limits[0], goal0)) : goal0;        // nooit voorbij de jointlimiet sturen
      this.roll += Math.max(-ROLL_MAX_STEP, Math.min(ROLL_MAX_STEP, goal - this.roll));    // snelheidsbegrenzing: geen sprongen
      roll = this.roll;
    }
    return { engaged: true, pos, quat, grip: this.lastGrip, roll };
  }
}

/**
 * Eén teleop-stap voor één arm (gedeeld door app.js en de headless tests): controller -> doel -> IK (+ rol -> joint6).
 * st = { qTarget: {side: q[6]}, qIK: {side: q[6]} } wordt bijgewerkt; ik = ArmIK. Geeft het teleop-resultaat t terug.
 * qIK is het IK-zaad ZONDER rol-offset; qTarget = qIK + rol op joint6 (geklemd) — dat is wat naar de actuatoren en de opname gaat.
 */
export function teleopArm(st, side, teleop, ik, raw, tcp, iters = 3) {
  const seed = teleop.engaged && st.qIK[side] ? st.qIK[side] : st.qTarget[side].slice();   // nieuw anker = huidige stand (incl. eerdere rol)
  const t = teleop.step(raw, tcp, ik.rollLimits(seed));
  if (t.engaged) {
    st.qIK[side] = ik.solve(seed, t.pos, t.quat, iters);
    st.qTarget[side] = ik.withRoll(st.qIK[side], t.roll);
  }
  return t;
}
