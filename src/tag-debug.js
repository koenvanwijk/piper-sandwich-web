// ?debug=1 voor de AprilTag-oppervlakstap: tekst (DOM <pre>) + hoofd-vast 3D-paneel (een DOM-overlay is in immersive-ar mogelijk niet zichtbaar).
const cm = v => (v * 100).toFixed(1);
const p3 = c => `(${cm(c[0])}, ${cm(c[1])}, ${cm(c[2])}) cm`;

/** Pure formatter (in Node getest). snap = TagTableScanner.snapshot(); info = {intrinsics, tSource, device, extrinsics, tagSize, method, source}. */
export function formatTagDebug(snap, info = {}) {
  const L = [];
  const e = snap.est, c = snap.counters || {};
  L.push(`TAGS fase=${snap.phase}  ${snap.reason || ''}  (${((snap.elapsedMs || 0) / 1000).toFixed(1)} s)`);
  L.push(`bron=${info.source || 'apriltag'}  tag=${info.tagSize ? (info.tagSize * 1000).toFixed(2) + ' mm' : '?'}  methode=${snap.tags?.[0]?.method || info.method || '-'}  tafel-y=${e?.tableY != null ? cm(e.tableY) + ' cm' : '?'}`);
  const K = info.intrinsics;
  L.push(`intrinsics: ${K ? `fx=${K.fx.toFixed(0)} cx=${K.cx.toFixed(0)} (${K.source || '?'})` : 'geen'}  f-zelfkal. k=${(snap.focalScale ?? 1).toFixed(3)} (n=${snap.focalSamples || 0}${K ? `, hfov≈${(2 * Math.atan(K.cx / (K.fx * (snap.focalScale ?? 1))) * 180 / Math.PI).toFixed(0)}°` : ''})  tijd: ${info.tSource || '?'}`);
  const x = info.extrinsics;
  L.push(`camera-extrinsics (AANNAME): ${x ? `${x.side} off=(${x.pos.map(v => (v * 100).toFixed(0)).join(',')}) cm pitch=${(info.pitchDeg || 0)}°` : '?'}  device: ${(info.device || '-').slice(0, 28)}`);
  L.push(`frames=${c.frames || 0} dets=${c.dets || 0} gebruikt=${c.used || 0} drop: gap=${c.dropGap || 0} beweging=${c.dropMotion || 0} kanteling=${c.dropTilt || 0} overig=${c.dropOther || 0} geen-pose=${c.dropNoHistory || 0}`);
  const tags = snap.tags || [];
  if (!tags.length) L.push('geen tags gezien'); 
  for (const t of tags.slice(0, 8)) {
    const role = e?.ok ? (t.id === e.ll.id ? ' ◄ LINKSONDER' : t.id === e.ur.id ? ' ◄ RECHTSBOVEN' : '') : '';
    L.push(`#${String(t.id).padEnd(3)} ${p3(t.center)} n=${t.n} σ=${(t.std * 1000).toFixed(1)} mm yaw=${t.yaw == null ? '-' : (t.yaw * 180 / Math.PI).toFixed(0) + '°'}${role}`);
  }
  if (e?.ok) {
    L.push(`oppervlak: ${cm(e.width)} × ${cm(e.depth)} cm  (breedte × diepte)  oppervlakte ${(e.width * e.depth).toFixed(3)} m²`);
    const nm = ['verst-links', 'verst-rechts', 'dichtbij-rechts', 'dichtbij-links'];
    e.corners.forEach((p, i) => L.push(`  hoek ${nm[i].padEnd(15)} ${p3(p)}`));
    L.push(`  frame x=(${e.frame.x[0].toFixed(2)},${e.frame.x[2].toFixed(2)}) z=(${e.frame.z[0].toFixed(2)},${e.frame.z[2].toFixed(2)}) det=${e.frame.det.toFixed(0)}`);
  } else if (e) L.push(`oppervlak: ✗ ${e.reason}`);
  return L;
}

export function createTagDebugPanel({ THREE, camera }) {
  const el = document.createElement('pre');
  el.id = 'tag-debug';
  el.style.cssText = 'position:fixed;bottom:12px;left:12px;z-index:10;margin:0;background:rgba(0,0,0,.65);padding:8px 10px;border-radius:8px;' +
    'font:11px/1.35 ui-monospace,Menlo,monospace;color:#ffd479;pointer-events:none;max-width:95vw;white-space:pre';
  document.body.appendChild(el);
  const W = 1100, H = 640, cv = document.createElement('canvas'); cv.width = W; cv.height = H;
  const g = cv.getContext('2d'), tex = new THREE.CanvasTexture(cv);
  if (THREE.SRGBColorSpace) tex.colorSpace = THREE.SRGBColorSpace;
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(0.66, 0.384),
    new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthTest: false, depthWrite: false, toneMapped: false }));
  mesh.position.set(0, -0.14, -0.9); mesh.renderOrder = 997; mesh.frustumCulled = false; mesh.name = 'tag-debug-overlay';
  camera.add(mesh);
  let last = 0;
  return {
    el, mesh,
    update(snap, info, nowMs = performance.now()) {
      if (nowMs - last < 200) return; last = nowMs;
      const L = formatTagDebug(snap, info); el.textContent = L.join('\n');
      g.clearRect(0, 0, W, H); g.fillStyle = 'rgba(0,0,0,.72)'; g.fillRect(0, 0, W, H);
      g.fillStyle = '#ffd479'; g.font = '19px monospace'; g.textBaseline = 'top';
      L.slice(0, 24).forEach((line, i) => g.fillText(line.slice(0, 96), 8, 6 + i * 26));
      tex.needsUpdate = true;
    },
    dispose() { el.remove(); camera.remove(mesh); },
  };
}
