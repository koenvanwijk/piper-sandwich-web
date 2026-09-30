// Offscreen camerabeelden voor de recorder: rendert de MuJoCo-camera's uit scene.xml
// (front/top/oblique, vast aan de wereld) met three.js naar een render-target en codeert JPEG.
// Alleen geladen met ?rec. Werkt ook tijdens WebXR (xr.enabled wordt tijdelijk uitgezet).
import * as THREE from 'three';

// MuJoCo (z omhoog) -> three (y omhoog): (x,y,z) -> (x,z,-y) = rotatie -90° om x (zie scene-loader getPosition).
const QA = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);

export class CameraCapture {
  constructor(app, names, { w = 640, h = 480, quality = 0.8 } = {}) {
    this.app = app; this.w = w; this.h = h; this.quality = quality;
    const mj = app.mujoco, m = app.model;
    this.cams = [];                                      // { id, name, camera, busy }
    names.forEach(name => {
      const id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_CAMERA.value, name);
      if (id < 0 || m.cam_bodyid[id] !== 0) { console.warn('[rec] camera niet gebruikt:', name); return; }
      const cam = new THREE.PerspectiveCamera(m.cam_fovy[id], w / h, 0.01, 50);
      const p = [m.cam_pos[3*id], m.cam_pos[3*id+1], m.cam_pos[3*id+2]];
      const q = new THREE.Quaternion(m.cam_quat[4*id+1], m.cam_quat[4*id+2], m.cam_quat[4*id+3], m.cam_quat[4*id]);
      cam.position.set(p[0], p[2], -p[1]);
      cam.quaternion.copy(QA).multiply(q);
      app.mujocoRoot.add(cam);                           // kind van de scene-root: volgt de thumbstick-navigatie niet apart
      this.cams.push({ id: this.cams.length, name, camera: cam, busy: false });
    });
    this.target = new THREE.WebGLRenderTarget(w, h, { colorSpace: THREE.SRGBColorSpace });
    this.buf = new Uint8Array(w * h * 4);
    this.flip = new Uint8ClampedArray(w * h * 4);
    this.canvas = typeof OffscreenCanvas !== 'undefined' ? new OffscreenCanvas(w, h)
                : Object.assign(document.createElement('canvas'), { width: w, height: h });
    this.ctx = this.canvas.getContext('2d');
    this.dropped_busy = 0;
  }

  get info() { return this.cams.map(c => ({ id: c.id, name: c.name, w: this.w, h: this.h, format: 'jpeg' })); }

  _encode() {
    const { w, h } = this, row = w * 4;
    for (let y = 0; y < h; y++)                          // readPixels is onderaan begonnen -> omkeren
      this.flip.set(this.buf.subarray((h - 1 - y) * row, (h - y) * row), y * row);
    this.ctx.putImageData(new ImageData(this.flip, w, h), 0, 0);
    return this.canvas.convertToBlob
      ? this.canvas.convertToBlob({ type: 'image/jpeg', quality: this.quality })
      : new Promise(res => this.canvas.toBlob(res, 'image/jpeg', this.quality));
  }

  /** Render alle camera's voor deze tick; onFrame(camId, ArrayBuffer) zodra de JPEG klaar is. */
  grab(onFrame) {
    const r = this.app.renderer, xr = r.xr.enabled, prev = r.getRenderTarget(), sh = r.shadowMap.autoUpdate;
    r.xr.enabled = false;
    r.shadowMap.autoUpdate = false;                      // schaduwmap van de laatste hoofd-render hergebruiken (scheelt veel GPU/CPU)
    // De vloer is een Reflector (extra scene-pass per render); voor de opname-camera's overslaan (vloer ligt 75 cm onder de tafel).
    if (!this._refl) { this._refl = []; this.app.mujocoRoot.traverse(o => { if (o.isReflector) this._refl.push(o); }); }
    const reflVis = this._refl.map(o => o.visible); this._refl.forEach(o => { o.visible = false; });
    const t0 = performance.now();
    try {
      for (const c of this.cams) {
        if (c.busy) { this.dropped_busy++; continue; }   // vorige JPEG van deze camera nog niet klaar
        r.setRenderTarget(this.target);
        r.render(this.app.scene, c.camera);
        r.readRenderTargetPixels(this.target, 0, 0, this.w, this.h, this.buf);
        c.busy = true;
        this._encode().then(blob => blob.arrayBuffer())
          .then(ab => { c.busy = false; onFrame(c.id, ab); })
          .catch(e => { c.busy = false; console.warn('[rec] JPEG-fout', e); });
      }
    } finally { r.setRenderTarget(prev); r.xr.enabled = xr; r.shadowMap.autoUpdate = sh; this._refl.forEach((o, i) => { o.visible = reflVis[i]; }); this.lastGrabMs = performance.now() - t0; }
  }
}
