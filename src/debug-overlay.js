// ?debug=1: overlay met per controller handedness, profiel, pose (wereld + scène-lokaal), clutch, doel-TCP en werkelijke TCP,
// plus hoofd-yaw en scène-root. DOM-paneel (desktop) + hoofd-vast 3D-paneel (canvas-texture, zichtbaar in VR).
// Bedoeld om snel te bevestigen of links/rechts en richtingen kloppen: beweeg de controller 20 cm naar rechts en kijk of
// "Δdoel" (MuJoCo) y -0,2 wordt (gebruiker-rechts = MuJoCo -y; vooruit = +x; omhoog = +z) en of de TCP dat volgt.
const f = (a, d = 2) => a ? '[' + a.map(x => (x >= 0 ? '+' : '') + x.toFixed(d)).join(' ') + ']' : '—';

export function formatDebug(d) {
  const L = [];
  L.push(`DEBUG  xr=${d.xr ? 'ja' : 'nee'}  refspace=${d.refSpace || '—'}  scene rotY=${(d.rootRotY * 180 / Math.PI).toFixed(0)}°  head yaw=${d.headYaw == null ? '—' : (d.headYaw * 180 / Math.PI).toFixed(0) + '°'}`);
  L.push(`orient-mode: ${d.rot ? 'ROT (?rot=1)' : 'vergrendeld'}   inputSources: ${d.sources.map(s => s.handedness + (s.profile ? '(' + s.profile + ')' : '')).join(', ') || '—'}`);
  for (const s of ['left', 'right']) {
    const c = d.ctrl[s];
    if (!c) { L.push(`${s.toUpperCase()}: geen controller`); continue; }
    L.push(`${s.toUpperCase()}: grip=${c.grip.toFixed(2)} trig=${c.trigger.toFixed(2)} ${c.engaged ? 'ENGAGED' : 'los'}`);
    L.push(`  wereld ${f(c.world)}  scène-lokaal(mj) ${f(c.mj)}`);
    L.push(`  doel(mj) ${f(c.target)}  tcp(mj) ${f(c.tcp)}  Δdoel ${f(c.delta)}`);
  }
  L.push('as-conventie: gebruiker-rechts = mj −y · vooruit = mj +x · omhoog = mj +z');
  return L;
}

export function createDebugOverlay({ THREE, camera }) {
  const el = document.createElement('pre');
  el.id = 'dbg-overlay';
  el.style.cssText = 'position:fixed;top:48px;left:12px;z-index:10;margin:0;background:rgba(0,0,0,.65);padding:8px 10px;' +
    'border-radius:8px;font:11px/1.35 ui-monospace,Menlo,monospace;color:#9be9a8;pointer-events:none;max-width:95vw;white-space:pre';
  document.body.appendChild(el);
  const cv = document.createElement('canvas'); cv.width = 1024; cv.height = 400;
  const g = cv.getContext('2d'), tex = new THREE.CanvasTexture(cv);
  if (THREE.SRGBColorSpace) tex.colorSpace = THREE.SRGBColorSpace;
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(0.6, 0.234),
    new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthTest: false, depthWrite: false, toneMapped: false }));
  mesh.position.set(0, 0.22, -0.9); mesh.renderOrder = 998; mesh.frustumCulled = false; mesh.name = 'dbg-overlay';
  camera.add(mesh);
  let last = 0;
  return {
    el, mesh,
    update(d, nowMs = performance.now()) {
      if (nowMs - last < 150) return; last = nowMs;
      const L = formatDebug(d); el.textContent = L.join('\n');
      g.clearRect(0, 0, 1024, 400); g.fillStyle = 'rgba(0,0,0,.72)'; g.fillRect(0, 0, 1024, 400);
      g.fillStyle = '#9be9a8'; g.font = '19px monospace'; g.textBaseline = 'top';
      L.forEach((line, i) => g.fillText(line.slice(0, 92), 8, 6 + i * 25));
      tex.needsUpdate = true;
    },
  };
}
