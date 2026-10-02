import * as THREE from 'three';
// Voorbeeld (preview) van het AprilTag-oppervlak tijdens het scannen, in WERELDcoördinaten (direct onder scene). Na toepassen neemt
// TableCalibrator.buildTableVisualization het over (in het gekalibreerde frame) en verbergen we deze preview.
const COLORS = { ll: 0x00e5ff, ur: 0xff2bd6, other: 0x999999 };

export class TagSurfaceView {
  constructor(scene, tagSize = 0.08255) {
    this.tagSize = tagSize; this.group = new THREE.Group(); this.group.name = 'tag-surface-preview'; this.group.visible = false; scene.add(this.group);
    this.fill = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ color: 0x22dd77, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false }));
    this.fill.frustumCulled = false; this.group.add(this.fill);
    this.border = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ color: 0xb6ffd6, transparent: true, opacity: 0.85, side: THREE.DoubleSide, depthWrite: false }));
    this.border.frustumCulled = false; this.group.add(this.border);
    this.markers = new Map(); this.posts = [];
  }
  _marker(id) {
    let m = this.markers.get(id);
    if (!m) { m = new THREE.Mesh(new THREE.PlaneGeometry(this.tagSize, this.tagSize), new THREE.MeshBasicMaterial({ color: COLORS.other, transparent: true, opacity: 0.6, side: THREE.DoubleSide, depthWrite: false }));
              m.rotation.x = -Math.PI / 2; this.group.add(m); this.markers.set(id, m); }
    return m;
  }
  /** snap = TagTableScanner.snapshot() */
  update(snap) {
    const e = snap.est, tags = snap.tags || [], seen = new Set();
    for (const t of tags) {
      const m = this._marker(t.id); seen.add(t.id); m.visible = true; m.position.set(t.center[0], (e?.tableY ?? t.center[1]) + 0.003, t.center[2]);
      m.material.color.setHex(e?.ok && t.id === e.ll.id ? COLORS.ll : e?.ok && t.id === e.ur.id ? COLORS.ur : COLORS.other);
    }
    for (const [id, m] of this.markers) if (!seen.has(id)) m.visible = false;
    if (!e?.ok) { this.fill.visible = this.border.visible = false; this.group.visible = tags.length > 0; return; }
    this.group.visible = true; this.fill.visible = this.border.visible = true;
    const c = e.corners, y = e.tableY + 0.002, b = 0.012;
    const P = c.map(p => [p[0], y, p[2]]);
    this.fill.geometry.dispose(); this.fill.geometry = quads([[P[0], P[1], P[2], P[3]]]);
    // randstroken: binnenrand = hoeken, verschoven naar binnen langs de diagonalen
    const cx = (P[0][0] + P[2][0]) / 2, cz = (P[0][2] + P[2][2]) / 2, shrink = p => { const dx = cx - p[0], dz = cz - p[2], l = Math.hypot(dx, dz) || 1; return [p[0] + dx / l * b * 1.2, y + 0.002, p[2] + dz / l * b * 1.2]; };
    const I = P.map(shrink), strips = [];
    for (let i = 0; i < 4; i++) { const j = (i + 1) % 4; strips.push([P[i], P[j], I[j], I[i]]); }
    this.border.geometry.dispose(); this.border.geometry = quads(strips);
  }
  hide() { this.group.visible = false; }
  clear() { this.group.visible = false; for (const m of this.markers.values()) this.group.remove(m); this.markers.clear(); }
}

function quads(list) {
  const pos = []; for (const q of list) { for (const i of [0, 1, 2, 0, 2, 3]) pos.push(...q[i]); }
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); return g;
}
