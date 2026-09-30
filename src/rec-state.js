// Opname-toestand voor de WebSocket-recorder (?rec=...). DOM- en three.js-vrij, dus ook
// headless in Node te testen. Zie DESIGN-vr-recording.md §1/§2 (fase 1: alleen de client).
//
// Toestandsdefinitie (14 waarden, volgorde vast):
//   left_joint1..6.pos [rad], left_gripper.pos [0 = open .. 1 = dicht],
//   right_joint1..6.pos [rad], right_gripper.pos [0..1]
// Actie (14 waarden, zelfde namen): qTarget (IK-uitkomst) + gecommandeerde gripper.
export const REC_FPS = 30;
export const REC_DT = 1 / REC_FPS;
export const REC_SUBSTEPS = 16;                 // 16 x (1/30/16 = 2.083 ms) = exact 1/30 s per tick
export const PROTO_VERSION = 1;

const SIDES = ['left', 'right'];
const r5 = x => Math.round(x * 1e5) / 1e5;       // 0.01 mm / 1e-5 rad; houdt JSON klein
const r5a = a => Array.from(a, r5);

export function stateNames() {
  const out = [];
  for (const s of SIDES) {
    for (let j = 1; j <= 6; j++) out.push(`${s}_joint${j}.pos`);
    out.push(`${s}_gripper.pos`);
  }
  return out;
}

/** FNV-1a (32 bit, hex) van een string; identificeert de scene-versie in `hello`. */
export function fnv1a(str) {
  let h = 0x811c9dc5;
  for (let i = 0; i < str.length; i++) { h ^= str.charCodeAt(i); h = Math.imul(h, 0x01000193); }
  return (h >>> 0).toString(16).padStart(8, '0');
}

export class RecStateSampler {
  /** env = { mujoco, model, data, ik, qTarget, grip, tcpPose(side) } (velden van SandwichVR / headless env) */
  constructor(env) {
    this.env = env;
    const { mujoco: mj, model } = env;
    const OBJ = mj.mjtObj;
    this.names = stateNames();
    this.gripAdr = {}; this.gripHi = {};
    for (const s of SIDES) {
      const jid = mj.mj_name2id(model, OBJ.mjOBJ_JOINT.value, `${s}_joint7`);
      this.gripAdr[s] = model.jnt_qposadr[jid];
      this.gripHi[s] = model.jnt_range[2 * jid + 1];       // open-stand (0.035 m)
    }
    // objecten: bodies met een vrije joint = dynamisch (in elke state), zonder joint = statisch (alleen in hello)
    this.dynamic = []; this.static = [];
    for (let b = 1; b < model.nbody; b++) {
      const name = mj.mj_id2name(model, OBJ.mjOBJ_BODY.value, b);
      if (!name || name.startsWith('left_') || name.startsWith('right_')) continue;
      const nj = model.body_jntnum[b];
      if (nj === 1 && model.jnt_type[model.body_jntadr[b]] === 0) this.dynamic.push({ name, id: b });
      else if (nj === 0 && model.body_mocapid[b] < 0) this.static.push({ name, id: b });
    }
  }

  gripperNorm(side) {
    const q = this.env.data.qpos[this.gripAdr[side]];
    return Math.min(1, Math.max(0, 1 - q / this.gripHi[side]));
  }

  bodyPose(id) {
    const d = this.env.data;
    return [d.xpos[3*id], d.xpos[3*id+1], d.xpos[3*id+2],
            d.xquat[4*id], d.xquat[4*id+1], d.xquat[4*id+2], d.xquat[4*id+3]];
  }

  staticObjects() {
    const o = {}; for (const b of this.static) o[b.name] = r5a(this.bodyPose(b.id)); return o;
  }

  /** Gemeten toestand (vóór de actie van deze tick). */
  observe() {
    const env = this.env, st = [], tcp = {}, objects = {};
    for (const s of SIDES) {
      const q = env.ik[s].currentQ();
      for (let i = 0; i < 6; i++) st.push(r5(q[i]));
      st.push(r5(this.gripperNorm(s)));
      const p = env.tcpPose(s);
      tcp[s] = { pos: r5a(p.pos), quat: r5a(p.quat) };
    }
    for (const b of this.dynamic) objects[b.name] = r5a(this.bodyPose(b.id));
    return { state: st, tcp, objects };
  }

  /** Gecommandeerde actie (na teleop/demo/IK van deze tick). */
  action() {
    const env = this.env, a = [];
    for (const s of SIDES) {
      for (let i = 0; i < 6; i++) a.push(r5(env.qTarget[s][i]));
      a.push(r5(env.grip[s] ?? 0));
    }
    return a;
  }
}

/** Controller-ruwdata (readControllers) -> compact JSON-veld. */
export function ctrlSummary(cmds, teleop) {
  const out = {};
  for (const s of SIDES) {
    const c = cmds && cmds[s];
    out[s] = c ? { connected: true, engaged: !!(teleop && teleop[s] && teleop[s].engaged),
                   trigger: r5(c.trigger || 0), grip: r5(c.grip || 0),
                   pos: r5a(c.pos), quat: r5a(c.quat),
                   buttons: c.buttons || [], axes: c.axes ? r5a(c.axes) : [] }
               : { connected: false, engaged: false };
  }
  return out;
}
