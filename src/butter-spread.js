// Smeerbare boter: kinematische smeerlaag op bread0 + "opnemen" van boterblokjes door het mes.
// DOM/three-vrij kern (ButterSpread, in Node getest: tools/test-butter-spread.mjs); createSpreadLayer() = three.js-weergave.
//
// Waarom zo (en niet echte vloeistof/deeltjes): MuJoCo heeft geen vloeistof, en een flex/soft-body of honderden deeltjes kost
// op de Quest te veel (contactparen ∝ n², en de sim draait op de hoofdthread). Deze aanpak kost < 0,1 ms per frame.
//   1. De 14 boterblokjes zijn fysiek glad en zacht (lage wrijving, zacht contact): het mes duwt/schuift ze i.p.v. ze als
//      plakkerige blokken mee te sleuren.
//   2. Drukt het blad (onderkant ≤ PICK_MM boven de bovenkant van een blokje, blokje onder de bladvoetafdruk), dan wordt dat
//      blokje "uitgesmeerd": het verdwijnt uit de botsingen (geparkeerd onder de tafel, elke stap vastgezet) en zijn
//      volume gaat als voorraad op het blad.
//   3. Beweegt het blad met voorraad dicht boven/op bread0 (onderkant ≤ CONTACT_MM boven de broodbovenkant, snelheid > MIN_SPEED),
//      dan groeit de dekking van de broodcellen onder het blad (rooster in bread0-coördinaten, volgt het brood mee).
//   Score = bedekt aandeel van het broodoppervlak (cellen ≥ 0,5 dekking).

export const SPREAD_DEFAULTS = {
  half: [0.045, 0.045],   // bread0 bovenvlak (halve maten, m)
  topZ: 0.013,            // bovenvlak t.o.v. bread0-midden (m)
  cell: 0.003,            // celgrootte (m) → 30 × 30 cellen
  bladeHalf: [0.07, 0.014, 0.00075],   // knife_blade (box, halve maten; 1,5 mm tafelmesblad)
  blockHalf: [0.008, 0.008, 0.0035], // boterblokje
  blockCoverM2: 0.0006,   // oppervlak (m²) dat één blokje volledig kan bedekken (6 cm² ≈ 2,3 × zijn voetafdruk); 14 blokjes ≈ 1 × broodvlak
  PICK_MM: 4,             // blad-onderkant hoogstens zoveel boven de bovenkant van een blokje = "druk" → opnemen
  CONTACT_MM: 6,          // blad-onderkant hoogstens zoveel boven de broodbovenkant = smeren
  MIN_SPEED: 0.01,        // m/s
  RATE: 4,                // dekking per seconde onder een bewegend blad (0 → 1 in 0,25 s)
};

const rot = (q, v) => {   // MuJoCo-quat [w,x,y,z] · v
  const [w, x, y, z] = q, [a, b, c] = v;
  const ix = w * a + y * c - z * b, iy = w * b + z * a - x * c, iz = w * c + x * b - y * a, iw = -x * a - y * b - z * c;
  return [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x];
};
const conj = q => [q[0], -q[1], -q[2], -q[3]];

export class ButterSpread {
  constructor(opts = {}) {
    this.o = { ...SPREAD_DEFAULTS, ...opts };
    const o = this.o;
    this.nx = Math.round(2 * o.half[0] / o.cell); this.ny = Math.round(2 * o.half[1] / o.cell);
    this.cov = new Float32Array(this.nx * this.ny);
    this.supply = 0; this.absorbed = []; this.dirty = true; this._prevBlade = null;
  }
  reset() { this.cov.fill(0); this.supply = 0; this.absorbed = []; this.dirty = true; this._prevBlade = null; }
  /** Dekking: aandeel cellen ≥ thr. */
  coverage(thr = 0.5) { let n = 0; for (const c of this.cov) if (c >= thr) n++; return n / this.cov.length; }
  /** Aandeel cellen bedekt door laag OF door een blokje-voetafdruk op het brood (vergelijkbare maat voor de oude scène zonder laag). */
  coverageWithBlocks(bread, blocks, thr = 0.5) {
    const o = this.o, m = new Uint8Array(this.cov.length);
    for (let i = 0; i < m.length; i++) m[i] = this.cov[i] >= thr ? 1 : 0;
    const bi = conj(bread.quat);
    for (const b of blocks) {
      const l = rot(bi, [b.pos[0] - bread.pos[0], b.pos[1] - bread.pos[1], b.pos[2] - bread.pos[2]]);
      if (l[2] < o.topZ - 0.002 || l[2] > o.topZ + 0.02) continue;          // ligt niet op het brood
      this._cells(l[0] - o.blockHalf[0], l[0] + o.blockHalf[0], l[1] - o.blockHalf[1], l[1] + o.blockHalf[1], k => { m[k] = 1; });
    }
    let n = 0; for (const v of m) n += v; return n / m.length;
  }
  _cells(x0, x1, y0, y1, f) {
    const o = this.o, c = o.cell;
    const i0 = Math.max(0, Math.floor((x0 + o.half[0]) / c)), i1 = Math.min(this.nx - 1, Math.floor((x1 + o.half[0]) / c));
    const j0 = Math.max(0, Math.floor((y0 + o.half[1]) / c)), j1 = Math.min(this.ny - 1, Math.floor((y1 + o.half[1]) / c));
    for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) f(j * this.nx + i, i, j);
  }
  /**
   * Eén stap. bread = { pos, quat } (bread0, MuJoCo-wereld); blade = { pos, quat } (knife_blade geom);
   * blocks = [{ pos, active }] (boterblokjes). Geeft de indices van blokjes terug die deze stap zijn opgenomen.
   */
  update({ bread, blade, blocks = [], dt }) {
    const o = this.o, picked = [];
    if (!bread || !blade || !(dt > 0)) return picked;
    const [hx, hy, hz] = o.bladeHalf, bq = blade.quat, bqi = conj(bq);
    // laagste punt van het blad (8 hoeken) in wereld-z
    let low = Infinity;
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) { const p = rot(bq, [sx * hx, sy * hy, sz * hz]); low = Math.min(low, blade.pos[2] + p[2]); }
    // 1) opnemen: blokje onder de bladvoetafdruk en het blad drukt erop
    blocks.forEach((b, i) => {
      if (!b.active) return;
      const l = rot(bqi, [b.pos[0] - blade.pos[0], b.pos[1] - blade.pos[1], b.pos[2] - blade.pos[2]]);
      if (Math.abs(l[0]) > hx + o.blockHalf[0] * 0.5 || Math.abs(l[1]) > hy + o.blockHalf[1] * 0.5) return;
      const top = b.pos[2] + o.blockHalf[2];
      if (low - top > o.PICK_MM / 1000 || l[2] > 0) return;      // te hoog, of het blokje ligt boven het blad
      picked.push(i); this.absorbed.push(i); this.supply += o.blockCoverM2;
    });
    // 2) smeren
    const prev = this._prevBlade; this._prevBlade = blade.pos.slice();
    if (!prev || this.supply <= 0) return picked;
    const speed = Math.hypot(blade.pos[0] - prev[0], blade.pos[1] - prev[1], blade.pos[2] - prev[2]) / dt;
    const breadTop = bread.pos[2] + o.topZ;                       // (brood ligt plat; kleine kanteling verwaarloosd)
    if (speed < o.MIN_SPEED || low - breadTop > o.CONTACT_MM / 1000 || low < breadTop - 0.01) return picked;
    // bladvoetafdruk in bread-coördinaten (as-uitgelijnde omhullende van de 4 onderhoeken), daarna per cel exact testen
    const bi = conj(bread.quat), corners = [];
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) { const w = rot(bq, [sx * hx, sy * hy, -hz]);
      corners.push(rot(bi, [blade.pos[0] + w[0] - bread.pos[0], blade.pos[1] + w[1] - bread.pos[1], blade.pos[2] + w[2] - bread.pos[2]])); }
    const xs = corners.map(c => c[0]), ys = corners.map(c => c[1]);
    const cellA = o.cell * o.cell, inc = o.RATE * dt;
    const bl = rot(bi, [blade.pos[0] - bread.pos[0], blade.pos[1] - bread.pos[1], 0]), rq = conj(bq);
    this._cells(Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys), (k, i, j) => {
      if (this.supply <= 0) return;
      const cx = -o.half[0] + (i + 0.5) * o.cell, cy = -o.half[1] + (j + 0.5) * o.cell;
      const w = rot(bread.quat, [cx - bl[0], cy - bl[1], 0]), lb = rot(rq, w);           // celmidden in bladcoördinaten
      if (Math.abs(lb[0]) > hx || Math.abs(lb[1]) > hy) return;
      const add = Math.min(1 - this.cov[k], inc, this.supply / cellA);
      if (add <= 0) return;
      this.cov[k] += add; this.supply -= add * cellA; this.dirty = true;
    });
    return picked;
  }
}

/** MuJoCo-koppeling: lees bread0/blad/blokjes, voer update uit, parkeer opgenomen blokjes (geen botsing, vast onder de tafel). */
export function createButterSim(mujoco, model, data, opts = {}) {
  const id = (t, n) => mujoco.mj_name2id(model, t, n);
  const B = mujoco.mjtObj.mjOBJ_BODY.value, G = mujoco.mjtObj.mjOBJ_GEOM.value;
  const bread = id(B, 'bread0'), blade = id(G, 'knife_blade');
  const blocks = []; for (let i = 0; ; i++) { const b = id(B, 'butter' + i); if (b < 0) break; blocks.push(b); }
  if (bread < 0 || blade < 0 || !blocks.length) return null;
  const sp = new ButterSpread(opts), active = blocks.map(() => true);
  const bodyPose = b => ({ pos: [data.xpos[3*b], data.xpos[3*b+1], data.xpos[3*b+2]], quat: [data.xquat[4*b], data.xquat[4*b+1], data.xquat[4*b+2], data.xquat[4*b+3]] });
  const geomQuat = g => { const m = data.geom_xmat.slice(9 * g, 9 * g + 9), t = m[0] + m[4] + m[8];
    let w, x, y, z;
    if (t > 0) { const s = Math.sqrt(t + 1) * 2; w = s / 4; x = (m[7] - m[5]) / s; y = (m[2] - m[6]) / s; z = (m[3] - m[1]) / s; }
    else if (m[0] > m[4] && m[0] > m[8]) { const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2; w = (m[7] - m[5]) / s; x = s / 4; y = (m[1] + m[3]) / s; z = (m[2] + m[6]) / s; }
    else if (m[4] > m[8]) { const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2; w = (m[2] - m[6]) / s; x = (m[1] + m[3]) / s; y = s / 4; z = (m[5] + m[7]) / s; }
    else { const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2; w = (m[3] - m[1]) / s; x = (m[2] + m[6]) / s; y = (m[5] + m[7]) / s; z = s / 4; }
    return [w, x, y, z]; };
  const park = (k, b) => {
    for (let g = 0; g < model.ngeom; g++) if (model.geom_bodyid[g] === b) { model.geom_contype[g] = 0; model.geom_conaffinity[g] = 0; }
    pin(k, b);
  };
  // geparkeerd blokje elke stap terugzetten (body_gravcomp werkt alleen als het model al bij het compileren gravcomp had)
  const pin = (k, b) => {
    const j = model.body_jntadr[b], qa = model.jnt_qposadr[j], da = model.jnt_dofadr[j];
    data.qpos[qa] = 0.1 + 0.02 * k; data.qpos[qa + 1] = 0; data.qpos[qa + 2] = -0.5;
    data.qpos[qa + 3] = 1; data.qpos[qa + 4] = data.qpos[qa + 5] = data.qpos[qa + 6] = 0;
    for (let d = 0; d < 6; d++) data.qvel[da + d] = 0;
  };
  const api = {
    spread: sp, blocks, active,
    state() { return { bread: bodyPose(bread), blade: { pos: [data.geom_xpos[3*blade], data.geom_xpos[3*blade+1], data.geom_xpos[3*blade+2]], quat: geomQuat(blade) },
      blocks: blocks.map((b, i) => ({ pos: bodyPose(b).pos, active: active[i] })) }; },
    step(dt) {
      const s = api.state();
      const picked = sp.update({ bread: s.bread, blade: s.blade, blocks: s.blocks, dt });
      for (const i of picked) { active[i] = false; park(i, blocks[i]); }
      blocks.forEach((b, i) => { if (!active[i]) pin(i, b); });
      return picked;
    },
    score() { const s = api.state(); return { layer: sp.coverage(), withBlocks: sp.coverageWithBlocks(s.bread, s.blocks.filter(b => b.active)), absorbed: sp.absorbed.length, supply: sp.supply }; },
    reset() {   // na mj_resetData: blokjes weer actief + botsend
      sp.reset();
      blocks.forEach((b, i) => { active[i] = true;
        for (let g = 0; g < model.ngeom; g++) if (model.geom_bodyid[g] === b) { model.geom_contype[g] = 1; model.geom_conaffinity[g] = 1; } });
    },
  };
  return api;
}

/** three.js-laag: semi-transparant vlak op bread0 (kind van de bread0-groep uit scene-loader), DataTexture uit het rooster. */
export function createSpreadLayer({ THREE, breadGroup, spread }) {
  const sp = spread, o = sp.o, px = new Uint8Array(sp.nx * sp.ny * 4);
  const tex = new THREE.DataTexture(px, sp.nx, sp.ny, THREE.RGBAFormat);
  tex.magFilter = THREE.LinearFilter; tex.minFilter = THREE.LinearFilter; tex.needsUpdate = true;
  if (THREE.SRGBColorSpace) tex.colorSpace = THREE.SRGBColorSpace;
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(2 * o.half[0], 2 * o.half[1]),
    new THREE.MeshLambertMaterial({ map: tex, transparent: true, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -2 }));
  // scene-loader-swizzle mj (x,y,z) → three (x, z, −y): vlak in three-XZ, plane-y = mj y; bovenvlak + 0,6 mm
  mesh.rotation.x = -Math.PI / 2; mesh.position.set(0, o.topZ + 0.0006, 0); mesh.name = 'butter-layer'; mesh.renderOrder = 2;
  breadGroup.add(mesh);
  let last = 0;
  return {
    mesh,
    update(nowMs = performance.now()) {
      if (!sp.dirty || nowMs - last < 100) return; last = nowMs; sp.dirty = false;
      for (let k = 0; k < sp.cov.length; k++) { const c = Math.min(1, sp.cov[k]);
        px[4*k] = 250; px[4*k+1] = 222; px[4*k+2] = 110; px[4*k+3] = Math.round(235 * Math.min(1, c * 1.4)); }
      tex.needsUpdate = true;
    },
  };
}
