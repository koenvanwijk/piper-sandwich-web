import * as THREE from 'three';

const M = new THREE.Matrix4();
const INV = new THREE.Matrix4();
const POS = new THREE.Vector3();
const QUAT = new THREE.Quaternion();
const SCALE = new THREE.Vector3();

const DEFAULT_STYLE = { fill: 0x33aaff, opacity: 0.10, edge: 0xffffff };   // 3-punts kalibratie (ongewijzigd)
const TAG_STYLE = { fill: 0x22dd77, opacity: 0.22, edge: 0xb6ffd6, border: 0.012 };       // oppervlak uit AprilTags

export class TableCalibrator {
  constructor(renderer, scene, { onStatus = () => {}, onCalibrated = () => {} } = {}) {
    this.renderer = renderer;
    this.scene = scene;
    this.onStatus = onStatus;
    this.onCalibrated = onCalibrated;
    this.session = null;
    this.referenceSpace = null;
    this.points = [];
    this.collecting = false;
    this.hitTestSources = new Map();
    this.anchor = null;
    this.anchorHandle = localStorage.getItem('piper-table-anchor-handle');

    this.root = new THREE.Group();
    this.root.name = 'piper-table-frame';
    this.root.matrixAutoUpdate = false;
    this.root.visible = false;
    scene.add(this.root);

    // Keep calibration graphics separate from calibrated scene content.
    // Recalibration may clear the overlay without deleting Piper/bread/butter.
    this.visualRoot = new THREE.Group();
    this.visualRoot.name = 'table-calibration-visuals';
    this.root.add(this.visualRoot);
    this.contentRoot = new THREE.Group();
    this.contentRoot.name = 'table-calibrated-content';
    this.root.add(this.contentRoot);

    this.markerRoot = new THREE.Group();
    scene.add(this.markerRoot);

    this._onSelect = e => this.onSelect(e);
    this._onInputSourcesChange = e => this.onInputSourcesChange(e);
  }

  async attachSession(session) {
    this.session = session;
    this.referenceSpace = this.renderer.xr.getReferenceSpace();
    session.addEventListener('select', this._onSelect);
    session.addEventListener('inputsourceschange', this._onInputSourcesChange);
    for (const source of session.inputSources) await this.prepareHitTestSource(source);

    const saved = this.getSavedCalibration();
    if (saved) this.buildTableVisualization(saved.width, saved.depth);

    if (this.anchorHandle && typeof session.restorePersistentAnchor === 'function') {
      try {
        this.anchor = await session.restorePersistentAnchor(this.anchorHandle);
        this.root.visible = true;
        if (saved) this.onCalibrated({ matrix: null, width: saved.width, depth: saved.depth, root: this.root });
        this.onStatus('Persistent table anchor restored — Piper + sandwich aligned.');
      } catch (error) {
        console.warn('Persistent anchor restore failed', error);
        localStorage.removeItem('piper-table-anchor-handle');
        this.anchorHandle = null;
      }
    }
  }

  detachSession() {
    if (!this.session) return;
    this.session.removeEventListener('select', this._onSelect);
    this.session.removeEventListener('inputsourceschange', this._onInputSourcesChange);
    for (const h of this.hitTestSources.values()) h?.cancel?.();
    this.hitTestSources.clear();
    this.session = null;
    this.referenceSpace = null;
  }

  start() {
    this.clearMarkers();
    this.points = [];
    this.collecting = true;
    this.root.visible = false;
    this.onStatus('Calibration: aim at table corner A (origin) and press trigger.');
  }

  async clear() {
    this.collecting = false;
    this.points = [];
    this.clearMarkers();
    this.root.visible = false;
    this.visualRoot.clear();
    try { this.anchor?.delete?.(); } catch {}
    if (this.anchorHandle && this.session?.deletePersistentAnchor) {
      try { await this.session.deletePersistentAnchor(this.anchorHandle); } catch {}
    }
    this.anchor = null;
    this.anchorHandle = null;
    localStorage.removeItem('piper-table-anchor-handle');
    localStorage.removeItem('piper-table-calibration');
    this.onStatus('Calibration cleared.');
  }

  getSavedCalibration() {
    try { return JSON.parse(localStorage.getItem('piper-table-calibration') || 'null'); }
    catch { return null; }
  }

  update(frame) {
    if (!frame || !this.referenceSpace || !this.anchor) return;
    const pose = frame.getPose(this.anchor.anchorSpace, this.referenceSpace);
    if (!pose) return;
    this.root.matrix.fromArray(pose.transform.matrix);
    this.root.matrixWorldNeedsUpdate = true;
    this.root.visible = true;
  }

  async onInputSourcesChange(event) {
    for (const source of event.removed || []) {
      this.hitTestSources.get(source)?.cancel?.();
      this.hitTestSources.delete(source);
    }
    for (const source of event.added || []) await this.prepareHitTestSource(source);
  }

  async prepareHitTestSource(inputSource) {
    if (!this.session?.requestHitTestSource || !inputSource.targetRaySpace || this.hitTestSources.has(inputSource)) return;
    try {
      this.hitTestSources.set(inputSource, await this.session.requestHitTestSource({ space: inputSource.targetRaySpace }));
    } catch (e) {
      console.debug('Hit-test source unavailable', e);
    }
  }

  async onSelect(event) {
    if (!this.collecting || this.points.length >= 3) return;
    const point = this.pointFromHitTest(event) || this.pointFromDetectedPlanes(event);
    if (!point) return this.onStatus('No table hit. Aim at the tabletop and press trigger again.');

    this.points.push(point.clone());
    this.addMarker(point, this.points.length);
    if (this.points.length === 1) return this.onStatus('A stored. Select B along the +X table edge.');
    if (this.points.length === 2) return this.onStatus('B stored. Select C along the second table edge.');
    this.collecting = false;
    await this.finish(event.frame);
  }

  pointFromHitTest(event) {
    const source = this.hitTestSources.get(event.inputSource);
    if (!source || !event.frame || !this.referenceSpace) return null;
    const result = event.frame.getHitTestResults?.(source)?.[0];
    const pose = result?.getPose(this.referenceSpace);
    if (!pose) return null;
    const p = pose.transform.position;
    return new THREE.Vector3(p.x, p.y, p.z);
  }

  pointFromDetectedPlanes(event) {
    const planes = event.frame?.detectedPlanes;
    if (!planes || !this.referenceSpace || !event.inputSource?.targetRaySpace) return null;
    const rayPose = event.frame.getPose(event.inputSource.targetRaySpace, this.referenceSpace);
    if (!rayPose) return null;

    const p = rayPose.transform.position;
    const o = rayPose.transform.orientation;
    const origin = new THREE.Vector3(p.x, p.y, p.z);
    const q = new THREE.Quaternion(o.x, o.y, o.z, o.w);
    const dir = new THREE.Vector3(0, 0, -1).applyQuaternion(q).normalize();

    let best = null, bestDistance = Infinity;
    for (const plane of planes) {
      const pose = event.frame.getPose(plane.planeSpace, this.referenceSpace);
      if (!pose) continue;
      M.fromArray(pose.transform.matrix);
      INV.copy(M).invert();
      const lo = origin.clone().applyMatrix4(INV);
      const ld = dir.clone().transformDirection(INV);
      if (Math.abs(ld.y) < 1e-5) continue;
      const t = -lo.y / ld.y;
      if (t <= 0) continue;
      const hit = lo.clone().addScaledVector(ld, t);
      if (!pointInPolygonXZ(hit, plane.polygon)) continue;
      const world = hit.applyMatrix4(M);
      const d = world.distanceTo(origin);
      if (d < bestDistance) { bestDistance = d; best = world; }
    }
    return best;
  }

  async finish(frame) {
    const [a, b, c] = this.points;
    const x = b.clone().sub(a);
    const width = x.length();
    if (width < 0.15) { this.onStatus('A→B is too short; recalibrate.'); return; }
    x.normalize();

    const ac = c.clone().sub(a);
    const z = ac.clone().addScaledVector(x, -ac.dot(x));
    const depth = z.length();
    if (depth < 0.15) { this.onStatus('C does not define a second table edge; recalibrate.'); return; }
    z.normalize();

    const y = new THREE.Vector3().crossVectors(z, x).normalize();
    if (y.y < 0) { y.negate(); z.negate(); }

    const matrix = new THREE.Matrix4().makeBasis(x, y, z);
    matrix.setPosition(a);
    this.root.matrix.copy(matrix);
    this.root.matrixWorldNeedsUpdate = true;
    this.root.visible = true;
    this.buildTableVisualization(width, depth);

    localStorage.setItem('piper-table-calibration', JSON.stringify({
      matrix: matrix.toArray(), width, depth, savedAt: new Date().toISOString()
    }));

    await this.createAnchor(frame, matrix);
    this.onCalibrated({ matrix: matrix.clone(), width, depth, root: this.root });
    this.onStatus(`Table calibrated: ${(width * 100).toFixed(1)} × ${(depth * 100).toFixed(1)} cm.`);
  }

  /**
   * Zet het tafelframe vanuit een AprilTag-oppervlak (src/tag-surface.js estimateSurface()) in plaats van de 3-punts kalibratie.
   * surface: { width, depth, frame: { matrix: kolom-groot 4x4 (x = rechts, y = omhoog, z = naar gebruiker; oppervlak beslaat x∈[0,width], z∈[0,depth]) } }.
   * Zelfde frameconventie, opslag, anchor en onCalibrated() als finish(), zodat de sandwich-scène (placeOnTable) ongewijzigd blijft werken.
   */
  async setFromSurface(frame, surface, { source = 'apriltag' } = {}) {
    const width = surface.width, depth = surface.depth;
    this.collecting = false; this.points = []; this.clearMarkers();
    try { this.anchor?.delete?.(); } catch {}                       // update() zou root.matrix anders elk frame terugzetten naar de oude anchor
    this.anchor = null; this.anchorHandle = null; localStorage.removeItem('piper-table-anchor-handle');
    const matrix = new THREE.Matrix4().fromArray(surface.frame.matrix);
    this.root.matrix.copy(matrix);
    this.root.matrixWorldNeedsUpdate = true;
    this.root.visible = true;
    this.buildTableVisualization(width, depth, source === 'apriltag' ? TAG_STYLE : undefined);
    localStorage.setItem('piper-table-calibration', JSON.stringify({
      matrix: matrix.toArray(), width, depth, source, savedAt: new Date().toISOString()
    }));
    await this.createAnchor(frame, matrix);
    this.onCalibrated({ matrix: matrix.clone(), width, depth, root: this.root, source });
    this.onStatus(`Table surface from ${source}: ${(width * 100).toFixed(1)} × ${(depth * 100).toFixed(1)} cm.`);
  }

  async createAnchor(frame, matrix) {
    if (!frame?.createAnchor || !this.referenceSpace || typeof XRRigidTransform === 'undefined') return;
    matrix.decompose(POS, QUAT, SCALE);
    const rigid = new XRRigidTransform(
      { x: POS.x, y: POS.y, z: POS.z },
      { x: QUAT.x, y: QUAT.y, z: QUAT.z, w: QUAT.w }
    );
    try {
      this.anchor = await frame.createAnchor(rigid, this.referenceSpace);
      if (typeof this.anchor.requestPersistentHandle === 'function') {
        this.anchorHandle = await this.anchor.requestPersistentHandle();
        localStorage.setItem('piper-table-anchor-handle', this.anchorHandle);
      }
    } catch (e) {
      console.warn('Anchor creation failed; keeping session-local table transform.', e);
    }
  }

  buildTableVisualization(width, depth, style = DEFAULT_STYLE) {
    this.visualRoot.clear();

    const surface = new THREE.Mesh(
      new THREE.PlaneGeometry(width, depth),
      new THREE.MeshBasicMaterial({
        color: style.fill,
        transparent: true,
        opacity: style.opacity,
        side: THREE.DoubleSide,
        depthWrite: false,
      })
    );
    surface.rotation.x = -Math.PI / 2;
    surface.position.set(width / 2, 0.002, depth / 2);
    this.visualRoot.add(surface);

    const edgePoints = [
      new THREE.Vector3(0, .006, 0), new THREE.Vector3(width, .006, 0),
      new THREE.Vector3(width, .006, depth), new THREE.Vector3(0, .006, depth),
      new THREE.Vector3(0, .006, 0)
    ];
    this.visualRoot.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(edgePoints),
      new THREE.LineBasicMaterial({ color: style.edge })
    ));

    if (style.border) {                                              // lijnen zijn in WebXR 1 px: echte randstroken (alleen voor het tag-oppervlak)
      const bm = new THREE.MeshBasicMaterial({ color: style.edge, transparent: true, opacity: 0.85, side: THREE.DoubleSide, depthWrite: false }), b = style.border;
      for (const [w, d, x, z] of [[width, b, width / 2, b / 2], [width, b, width / 2, depth - b / 2], [b, depth, b / 2, depth / 2], [b, depth, width - b / 2, depth / 2]]) {
        const m = new THREE.Mesh(new THREE.PlaneGeometry(w, d), bm); m.rotation.x = -Math.PI / 2; m.position.set(x, 0.004, z); this.visualRoot.add(m);
      }
    }

    const axes = new THREE.AxesHelper(Math.min(.25, width * .25, depth * .25));
    axes.position.y = .01;
    this.visualRoot.add(axes);
  }

  addMarker(position, index) {
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(.015, 16, 8),
      new THREE.MeshBasicMaterial({ color: index === 1 ? 0xff4444 : index === 2 ? 0x44ff44 : 0x4488ff })
    );
    marker.position.copy(position);
    this.markerRoot.add(marker);
  }

  clearMarkers() { this.markerRoot.clear(); }
}

function pointInPolygonXZ(point, polygon) {
  if (!polygon || polygon.length < 3) return true;
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].x, zi = polygon[i].z, xj = polygon[j].x, zj = polygon[j].z;
    const cross = ((zi > point.z) !== (zj > point.z)) &&
      (point.x < (xj - xi) * (point.z - zi) / ((zj - zi) || 1e-9) + xi);
    if (cross) inside = !inside;
  }
  return inside;
}
