import * as THREE from 'three';
import load_mujoco from '../vendor/mujoco/mujoco.js';
import { loadSceneFromURL, getPosition, getQuaternion } from './scene-loader.js';

const MESHES = ['base_link', 'link1', 'link2', 'link3', 'link4', 'link5',
  'link6', 'gripper_base', 'link7', 'link8'].map(n => n + '.STL');

export class ARSandwichScene {
  constructor(scene, tableContentRoot, { onStatus = () => {} } = {}) {
    this.scene = scene;
    this.tableContentRoot = tableContentRoot;
    this.onStatus = onStatus;
    this.loaded = false;
    this.pendingTable = null;
    this.model = null;
    this.data = null;
    this.bodies = null;
    this.lights = null;
    this.mujocoRoot = null;
  }

  async init() {
    this.onStatus('Loading the existing Piper + sandwich MuJoCo scene…');

    this.mujoco = await load_mujoco({
      locateFile: (path, prefix) => path.endsWith('.wasm')
        ? new URL('../vendor/mujoco/mujoco.wasm', import.meta.url).href
        : prefix + path,
    });

    this.mujoco.FS.mkdir('/working');
    this.mujoco.FS.mount(this.mujoco.MEMFS, { root: '.' }, '/working');
    this.mujoco.FS.mkdir('/working/meshes');

    this.mujoco.FS.writeFile('/working/scene.xml',
      await (await fetch(new URL('../assets/scene.xml', import.meta.url))).text());

    for (const f of MESHES) {
      const buf = new Uint8Array(await (await fetch(
        new URL('../assets/meshes/' + f, import.meta.url))).arrayBuffer());
      this.mujoco.FS.writeFile('/working/meshes/' + f, buf);
    }

    [this.model, this.data, this.bodies, this.lights] =
      await loadSceneFromURL(this.mujoco, 'scene.xml', this);

    this.mujoco.mj_resetData(this.model, this.data);
    if (this.model.nkey > 0) {
      this.data.qpos.set(this.model.key_qpos.slice(0, this.model.nq));
      if (this.model.nu > 0) this.data.ctrl.set(this.model.key_ctrl.slice(0, this.model.nu));
    }
    this.mujoco.mj_forward(this.model, this.data);
    this.syncBodies();

    // Passthrough supplies the real table and room. Keep only the robot/cell items
    // from the shared MuJoCo scene.
    this.mujocoRoot.traverse(obj => {
      const geomName = obj.userData?.mujocoGeomName;
      if (geomName === 'floor' || geomName === 'table') obj.visible = false;
      if (obj.name === 'left_target' || obj.name === 'right_target') obj.visible = false;
    });
    for (const light of this.lights || []) light.visible = false;

    // loadSceneFromURL initially adds this to the Three scene; re-parent it to the
    // calibrated tabletop frame so all Piper/bread/butter coordinates stay shared
    // with the VR demo.
    this.tableContentRoot.add(this.mujocoRoot);
    this.mujocoRoot.rotation.set(0, 0, 0);
    this.mujocoRoot.scale.setScalar(1);

    this.loaded = true;
    if (this.pendingTable) this.placeOnTable(this.pendingTable.width, this.pendingTable.depth);
    this.onStatus('Pipers + bread + butter loaded. Enter AR and calibrate the table.');
  }

  placeOnTable(width, depth) {
    this.pendingTable = { width, depth };
    if (!this.loaded || !this.mujocoRoot) return;

    // Shared scene table is 1.10 x 1.10 m, centered at MuJoCo (x=.10,y=0).
    // scene-loader maps MuJoCo x/y/z -> Three x/-z/y. Keep physical scale 1:1
    // and center the whole existing cell on the measured real tabletop.
    this.mujocoRoot.position.set(width / 2 - 0.10, 0, depth / 2);
    this.mujocoRoot.rotation.set(0, 0, 0);
    this.mujocoRoot.updateMatrixWorld(true);
  }

  syncBodies() {
    for (let b = 0; b < this.model.nbody; b++) {
      if (!this.bodies[b]) continue;
      getPosition(this.data.xpos, b, this.bodies[b].position);
      getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
    }
  }
}
