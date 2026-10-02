// Prestatie-instellingen voor de AR-pagina (table-ar/). Alles is opt-out: ?perf=0 zet ALLES terug naar het oude gedrag.
// Op een Quest zit de last in de GPU (twee ogen × veel pixels × MeshPhysicalMaterial) en in de hoofdthread (tag-detectie WASM ~40 ms/frame
// bij 960 px, DOM-overlay die bij elke wijziging opnieuw wordt gerasterd). De MuJoCo-sim draait in AR NIET (de scène is statisch: ARSandwichScene.syncBodies
// draait één keer bij het laden), dus daar valt niets te winnen.
//
//   ?perf=0           alles uit (oud gedrag)
//   ?fbscale=0.8      XR framebufferScaleFactor (standaard 0.8 i.p.v. three's 1.0; 0.5–1.5). Lager = minder pixels = sneller maar iets minder scherp
//   ?foveation=1      XR foveation 0..1 (three-standaard 1 = maximaal; hier expliciet gezet)
//   ?aa=0             geen MSAA (alleen bij het laden van de pagina te kiezen; standaard aan)
//   ?mat=physical     laat de zware MeshPhysicalMaterial uit het MuJoCo-model staan (standaard: MeshStandardMaterial, gedeeld per kleur)
//   ?freeze=0         laat matrixAutoUpdate aan voor de (statische) scène-objecten
//   ?lod=12000        max. driehoeken per mesh (vertex-clustering-decimatie bij het laden, alleen AR); ?lod=0 = originele meshes (≈ 683 k driehoeken per oog → bij 12000 ≈ 4× minder)
//   ?tagfps=8 ?tagproc=640 ?tagduty=0.25   tag-detectie: max fps, breedte detectiebeeld, max. aandeel hoofdthread-tijd
//   ?tagpreview=1     toon camerabeeld + overlay ook tijdens AR (kost DOM-overlay-herrasterisatie)
const num = (q, k, d, lo, hi) => { const v = parseFloat(q.get(k)); return Number.isFinite(v) ? Math.min(hi, Math.max(lo, v)) : d; };

export function parseArPerf(search) {
  const q = new URLSearchParams(search || ''), off = q.get('perf') === '0';
  return {
    enabled: !off,
    fbScale: off ? 1 : num(q, 'fbscale', 0.8, 0.5, 1.5),
    foveation: off ? 1 : num(q, 'foveation', 1, 0, 1),
    antialias: q.get('aa') !== '0',
    standardMaterials: !off && q.get('mat') !== 'physical',
    freeze: !off && q.get('freeze') !== '0',
    tagFps: off ? 30 : num(q, 'tagfps', 10, 1, 30),
    tagProc: off ? 960 : Math.round(num(q, 'tagproc', 960, 320, 1280)),
    tagDuty: off ? 0 : num(q, 'tagduty', 0.25, 0.05, 1),
    tagPreviewInXR: off || q.get('tagpreview') === '1',
    hudThrottleMs: off ? 0 : 500,
    lodTris: off ? 0 : Math.round(num(q, 'lod', 12000, 0, 1e7)),
  };
}

/**
 * Vertex-clustering-decimatie (indexed): punten die in dezelfde kubus (zijde `cell`) vallen worden één punt (gemiddelde), gedegenereerde driehoeken verdwijnen.
 * pos: Float32Array|number[] (xyz), idx: Uint32Array|number[]. Geeft { position: Float32Array, index: Uint32Array }.
 */
export function clusterDecimate(pos, idx, cell) {
  const nv = pos.length / 3, K = 1 << 17, map = new Map(), remap = new Int32Array(nv), sum = [], cnt = [];
  for (let v = 0; v < nv; v++) {
    const ix = Math.floor(pos[3 * v] / cell) + (K >> 1), iy = Math.floor(pos[3 * v + 1] / cell) + (K >> 1), iz = Math.floor(pos[3 * v + 2] / cell) + (K >> 1);
    const key = ix + iy * K + iz * K * K; let c = map.get(key);
    if (c === undefined) { c = cnt.length; map.set(key, c); cnt.push(0); sum.push(0, 0, 0); }
    remap[v] = c; cnt[c]++; sum[3 * c] += pos[3 * v]; sum[3 * c + 1] += pos[3 * v + 1]; sum[3 * c + 2] += pos[3 * v + 2];
  }
  const position = new Float32Array(cnt.length * 3); for (let c = 0; c < cnt.length; c++) { position[3 * c] = sum[3 * c] / cnt[c]; position[3 * c + 1] = sum[3 * c + 1] / cnt[c]; position[3 * c + 2] = sum[3 * c + 2] / cnt[c]; }
  const out = [];
  for (let t = 0; t < idx.length; t += 3) { const a = remap[idx[t]], b = remap[idx[t + 1]], c = remap[idx[t + 2]]; if (a !== b && b !== c && a !== c) out.push(a, b, c); }
  return { position, index: Uint32Array.from(out) };
}
/** Kies de fijnste cel waarvoor het resultaat ≤ budget driehoeken heeft. null als het mesh al klein genoeg is. */
export function decimateToBudget(pos, idx, budget) {
  if (!budget || idx.length / 3 <= budget) return null;
  let mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let v = 0; v < pos.length; v += 3) for (let k = 0; k < 3; k++) { mn[k] = Math.min(mn[k], pos[v + k]); mx[k] = Math.max(mx[k], pos[v + k]); }
  const diag = Math.hypot(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]) || 1;
  let best = null;
  for (let n = 24; n <= 1500; n = Math.ceil(n * 1.3)) {                  // grof → fijn; de laatste die binnen het budget past
    const r = clusterDecimate(pos, idx, diag / n); if (r.index.length / 3 > budget) break; best = r;
  }
  return best || clusterDecimate(pos, idx, diag / 24);
}

/**
 * Maak het geladen MuJoCo-model goedkoper om te renderen (statisch in AR). Geeft tellingen terug.
 *  - verborgen objecten (vloer/tafel/targets, Reflector) echt uit de scène halen (geen traversal, geen reflectie-pass)
 *  - MeshPhysicalMaterial -> MeshStandardMaterial, gedeeld per (kleur, opacity, map, roughness, metalness)
 *  - schaduwvlaggen uit (de renderer heeft in AR geen shadowMap)
 *  - matrixAutoUpdate uit voor alles onder `root` (positie/rotatie van root zelf blijft vrij)
 */
export function optimizeSandwichForAR(root, THREE, opts) {
  const st = { removed: 0, trisBefore: 0, trisAfter: 0, decimated: 0, meshes: 0, materialsBefore: 0, materialsAfter: 0, converted: 0, frozen: 0 };
  const dead = []; root.traverse(o => { if (o !== root && (o.visible === false || o.isReflector)) dead.push(o); });
  for (const o of dead) { if (o.parent) { o.parent.remove(o); st.removed++; o.traverse?.(c => { c.geometry?.dispose?.(); }); } }
  const geoMap = new Map();                                                  // oude geometrie -> nieuwe (gedecimeerd), gedeeld door meshes
  root.traverse(o => {
    if (!o.isMesh || !o.geometry || !o.geometry.index) return;
    const g = o.geometry; let ng = geoMap.get(g);
    if (ng === undefined) {
      const pos = g.attributes.position.array, idx = g.index.array; st.trisBefore += idx.length / 3;
      const r = decimateToBudget(pos, idx, opts.lodTris);
      if (r) { ng = new THREE.BufferGeometry(); ng.setAttribute('position', new THREE.BufferAttribute(r.position, 3)); ng.setIndex(new THREE.BufferAttribute(r.index, 1)); ng.computeVertexNormals(); ng.computeBoundingSphere(); st.decimated++; st.trisAfter += r.index.length / 3; g.dispose?.(); }
      else { ng = null; st.trisAfter += idx.length / 3; }
      geoMap.set(g, ng);
    }
    if (ng) o.geometry = ng;
  });
  const cache = new Map(), seen = new Set();
  root.traverse(o => {
    if (!o.isMesh) return; st.meshes++;
    o.castShadow = false; o.receiveShadow = false;
    const m = o.material; if (!m || Array.isArray(m)) return; seen.add(m);
    if (opts.standardMaterials && m.isMeshPhysicalMaterial) {
      const key = [m.color.getHex(), m.transparent, m.opacity, m.map?.uuid || '', m.roughness, m.metalness].join('|');
      let n = cache.get(key);
      if (!n) { n = new THREE.MeshStandardMaterial({ color: m.color.clone(), transparent: m.transparent, opacity: m.opacity, map: m.map || null, roughness: m.roughness, metalness: m.metalness, side: m.side }); cache.set(key, n); }
      o.material = n; st.converted++; m.dispose();
    }
  });
  st.materialsBefore = seen.size; st.materialsAfter = new Set(); const after = new Set();
  root.traverse(o => { if (o.isMesh && o.material && !Array.isArray(o.material)) after.add(o.material); }); st.materialsAfter = after.size;
  if (opts.freeze) root.traverse(o => { if (o !== root) { o.updateMatrix(); o.matrixAutoUpdate = false; st.frozen++; } });
  root.updateMatrixWorld(true);
  return st;
}
