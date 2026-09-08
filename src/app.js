import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { VRButton } from 'three/addons/webxr/VRButton.js';
import load_mujoco from '../vendor/mujoco/mujoco.js';
import { loadSceneFromURL, getPosition, getQuaternion, drawTendonsAndFlex } from './scene-loader.js';
import { ArmIK } from './ik.js';
import { HandTeleop } from './teleop.js';
import { mat2quat } from './qmath.js';

const SIDES = ['left', 'right'];
const MESHES = ['base_link', 'link1', 'link2', 'link3', 'link4', 'link5',
                'link6', 'gripper_base', 'link7', 'link8'].map(n => n + '.STL');

// Load the WASM engine, pointing it at the vendored binary.
const mujoco = await load_mujoco({
  locateFile: (path, prefix) => path.endsWith('.wasm')
    ? new URL('../vendor/mujoco/mujoco.wasm', import.meta.url).href
    : prefix + path,
});

// Populate the virtual file system with the scene + meshes.
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.mkdir('/working/meshes');
mujoco.FS.writeFile('/working/scene.xml',
  await (await fetch(new URL('../assets/scene.xml', import.meta.url))).text());
for (const f of MESHES) {
  const buf = new Uint8Array(await (await fetch(
    new URL('../assets/meshes/' + f, import.meta.url))).arrayBuffer());
  mujoco.FS.writeFile('/working/meshes/' + f, buf);
}

const status = document.getElementById('status');
const setStatus = s => { if (status) status.textContent = s; };

class SandwichVR {
  constructor() {
    this.mujoco = mujoco;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0.12, 0.14, 0.18);

    this.camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.01, 100);
    this.camera.position.set(0.0, 1.55, 0.55);   // roughly the user's head, looking forward
    this.scene.add(this.camera);

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const key = new THREE.DirectionalLight(0xffffff, 2.0);
    key.position.set(1, 3, 2); key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.near = 0.1; key.shadow.camera.far = 10;
    this.scene.add(key);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(devicePixelRatio);
    this.renderer.setSize(innerWidth, innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.xr.enabled = true;
    this.renderer.xr.setReferenceSpaceType('local-floor');
    document.body.appendChild(this.renderer.domElement);
    document.body.appendChild(VRButton.createButton(this.renderer));

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.95, -0.6);
    this.controls.enableDamping = true;
    this.controls.update();

    addEventListener('resize', () => {
      this.camera.aspect = innerWidth / innerHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(innerWidth, innerHeight);
    });

    this._tmpV = new THREE.Vector3();
    this._acc = 0; this._last = performance.now() / 1000;
  }

  async init() {
    [this.model, this.data, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, 'scene.xml', this);

    // Default view: sit *between* the two arms, looking forward at the board.
    // A +90° yaw makes MuJoCo +x (toward the board) point to the user's forward
    // (three -z); the arms (MuJoCo ±y) then sit on the user's left/right, with
    // the bases flanking them at z≈0. Adjust live with the thumbsticks (readNav).
    this.mujocoRoot.rotation.y = Math.PI / 2;
    this.mujocoRoot.position.set(0, 0.95, -0.32);
    this.mujocoRoot.updateMatrixWorld(true);
    this._homeView = { pos: this.mujocoRoot.position.clone(),
                       rotY: this.mujocoRoot.rotation.y };

    // Start from the 'home' keyframe.
    mujoco.mj_resetData(this.model, this.data);
    if (this.model.nkey > 0) this.data.qpos.set(this.model.key_qpos.slice(0, this.model.nq));
    mujoco.mj_forward(this.model, this.data);

    this.ik = {}; this.qTarget = {}; this.grip = {}; this.teleop = {};
    for (const s of SIDES) {
      this.ik[s] = new ArmIK(mujoco, this.model, this.data, s);
      this.qTarget[s] = this.ik[s].currentQ();
      this.grip[s] = 0.0;
      this.teleop[s] = new HandTeleop({ lockOrientation: true });
      this.ik[s].apply(this.qTarget[s], this.grip[s]);
    }
    this._mocap = {};
    for (const s of SIDES) {
      const bid = mujoco.mj_name2id(this.model, mujoco.mjtObj.mjOBJ_BODY.value, `${s}_target`);
      this._mocap[s] = this.model.body_mocapid[bid];
    }

    this.renderer.setAnimationLoop(() => this.frame());
    setStatus('Ready — press "Enter VR". Hold grip = clutch, trigger = gripper.');
  }

  tcpPose(side) {
    const s = this.ik[side].site, d = this.data;
    const m = []; for (let k = 0; k < 9; k++) m.push(d.site_xmat[9 * s + k]);
    return { pos: [d.site_xpos[3*s], d.site_xpos[3*s+1], d.site_xpos[3*s+2]],
             quat: mat2quat(m) };
  }

  readControllers() {
    const out = { left: null, right: null };
    const session = this.renderer.xr.getSession();
    if (!session) return out;
    const frame = this.renderer.xr.getFrame();
    const ref = this.renderer.xr.getReferenceSpace();
    if (!frame || !ref) return out;
    // Controller poses are transformed into the scene root's LOCAL frame, so
    // the teleop mapping stays correct no matter how the joysticks have rotated
    // or moved the scene (the fixed three->MuJoCo swizzle then applies).
    const rootQinv = this.mujocoRoot.getWorldQuaternion(new THREE.Quaternion()).invert();
    for (const src of session.inputSources) {
      if (!src.gripSpace || !src.handedness) continue;
      const pose = frame.getPose(src.gripSpace, ref);
      if (!pose) continue;
      const p = pose.transform.position, o = pose.transform.orientation;
      const lp = this.mujocoRoot.worldToLocal(new THREE.Vector3(p.x, p.y, p.z));
      const lq = rootQinv.clone().multiply(new THREE.Quaternion(o.x, o.y, o.z, o.w));
      const gp = src.gamepad;
      const btn = i => (gp && gp.buttons[i]) ? gp.buttons[i].value : 0;
      const raw = { pos: [lp.x, lp.y, lp.z], quat: [lq.w, lq.x, lq.y, lq.z],
                    trigger: btn(0), grip: btn(1) };
      if (src.handedness === 'left') out.left = raw;
      else if (src.handedness === 'right') out.right = raw;
    }
    return out;
  }

  // Read thumbsticks / face buttons for navigating the scene.
  readNav() {
    const nav = { yaw: 0, dist: 0, height: 0, strafe: 0, reset: false };
    const session = this.renderer.xr.getSession();
    if (!session) return nav;
    const dz = v => Math.abs(v) < 0.2 ? 0 : v;
    for (const src of session.inputSources) {
      const gp = src.gamepad; if (!gp) continue;
      const ax = i => gp.axes.length > i ? gp.axes[i] : 0;
      const pressed = i => gp.buttons[i] && gp.buttons[i].pressed;
      if (src.handedness === 'right') {
        nav.yaw += dz(ax(2));            // right stick L/R : spin the workspace
        nav.dist += dz(ax(3));           // right stick U/D : move it closer/further
        if (pressed(4) || pressed(5)) nav.reset = true;   // A/B : recenter
      } else if (src.handedness === 'left') {
        nav.strafe += dz(ax(2));         // left stick L/R : slide the workspace
        nav.height += dz(ax(3));         // left stick U/D : raise / lower the table
        if (pressed(4) || pressed(5)) nav.reset = true;   // X/Y : recenter
      }
    }
    return nav;
  }

  applyNav(nav, dt) {
    const r = this.mujocoRoot;
    if (nav.reset) {
      r.position.copy(this._homeView.pos);
      r.rotation.y = this._homeView.rotY;
    } else {
      r.rotation.y -= nav.yaw * 1.2 * dt;          // spin scene
      r.position.z += nav.dist * 0.5 * dt;         // stick up (-) -> further away (-z)
      r.position.x += nav.strafe * 0.5 * dt;       // slide left / right
      r.position.y -= nav.height * 0.4 * dt;       // stick up (-) -> raise
    }
    r.updateMatrixWorld(true);
  }

  frame() {
    const now = performance.now() / 1000;
    const frameDt = Math.min(0.05, now - this._last);

    // navigate / orient the scene with the thumbsticks first
    this.applyNav(this.readNav(), frameDt);

    const cmds = this.readControllers();
    let anyEngaged = false;
    for (const s of SIDES) {
      const t = this.teleop[s].step(cmds[s], this.tcpPose(s));
      if (t.engaged) {
        this.qTarget[s] = this.ik[s].solve(this.qTarget[s], t.pos, t.quat, 3);
        this.grip[s] = t.grip;
        if (this._mocap[s] >= 0) {
          const a = this._mocap[s] * 3;
          this.data.mocap_pos[a] = t.pos[0];
          this.data.mocap_pos[a+1] = t.pos[1];
          this.data.mocap_pos[a+2] = t.pos[2];
        }
        anyEngaged = true;
      }
      this.ik[s].apply(this.qTarget[s], this.grip[s]);
    }

    // real-time-ish physics stepping
    this._acc += frameDt; this._last = now;
    const dt = this.model.opt.timestep;
    let n = 0;
    while (this._acc >= dt && n < 30) { mujoco.mj_step(this.model, this.data); this._acc -= dt; n++; }

    // push MuJoCo transforms into three.js
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition(this.data.xpos, b, this.bodies[b].position);
        getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
      }
    }
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.data.light_xpos, l, this.lights[l].position);
        getPosition(this.data.light_xdir, l, this._tmpV);
        this.lights[l].lookAt(this._tmpV.add(this.lights[l].position));
      }
    }
    drawTendonsAndFlex(this.mujocoRoot, this.model, this.data);

    if (!this.renderer.xr.isPresenting) this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}

const app = new SandwichVR();
await app.init();
window.sandwichVR = app;
