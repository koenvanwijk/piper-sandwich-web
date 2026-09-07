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

export class HandTeleop {
  constructor({ scale = 1.0, lockOrientation = true } = {}) {
    this.scale = scale;
    this.lockOrientation = lockOrientation;
    this.engaged = false;
    this.anchor = null;              // {cpos, cquat, tpos, tquat}
    this.lastGrip = 0.0;
  }

  /** raw: {pos:[x,y,z] three-world, quat:[w,x,y,z] three, trigger, grip} | null
   *  tcp: {pos:[x,y,z] mujoco, quat:[w,x,y,z] mujoco}  (current arm TCP)
   *  returns {engaged, pos, quat, grip}                (targets in mujoco frame) */
  step(raw, tcp) {
    if (!raw) { this.engaged = false; this.anchor = null;
                return { engaged: false, grip: this.lastGrip }; }

    const grip = raw.grip || 0;
    const on = grip > (this.engaged ? CLUTCH_OFF : CLUTCH_ON);
    this.lastGrip = raw.trigger || 0;

    if (!on) { this.engaged = false; this.anchor = null;
               return { engaged: false, grip: this.lastGrip }; }

    const cpos = raw.pos;
    const cquat = threeQuatToMujoco(raw.quat);

    if (!this.engaged || !this.anchor) {           // rising edge -> anchor
      this.anchor = { cpos: cpos.slice(), cquat: cquat.slice(),
                      tpos: tcp.pos.slice(), tquat: tcp.quat.slice() };
      this.engaged = true;
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
    return { engaged: true, pos, quat, grip: this.lastGrip };
  }
}
