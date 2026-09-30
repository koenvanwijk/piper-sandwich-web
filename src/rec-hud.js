// Fase 3: feedback bij het opnemen (alleen met ?rec=): DOM-paneel (desktop) + een klein hoofd-vast paneel in de
// 3D-scene (kind van de camera, zichtbaar in VR; een DOM-overlay is dat in immersive VR op de Quest niet).
// Toont status (IDLE/RECORDING/SAVED/DISCARDED/...), episodenummer, verstreken tijd en verbindingsstatus.
export const STATE_COLOR = { IDLE: '#8b949e', RECORDING: '#f85149', SAVING: '#d29922', SAVED: '#3fb950',
                             DISCARDED: '#db6d28', INTERRUPTED: '#a371f7' };

export function hudLines(s, connState) {
  const t = s.elapsed || 0, mm = String(Math.floor(t / 60)).padStart(2, '0'), ss = (t % 60).toFixed(1).padStart(4, '0');
  const conn = s.connected ? 'verbonden' : (connState || 'offline');
  const l1 = `${s.state}   ep ${s.episode}   ${mm}:${ss}`;
  let l2 = s.state === 'SAVED' && s.last ? `opgeslagen: ${s.last.frames} frames${s.last.success ? ', geslaagd' : ''}` : (s.message || '');
  return { l1, l2, l3: `server: ${conn}`, color: STATE_COLOR[s.state] || '#8b949e' };
}

export function createRecHud({ THREE, camera, connLabel = () => '' }) {
  // --- DOM-paneel
  const el = document.createElement('div');
  el.id = 'rec-hud';
  el.style.cssText = 'position:fixed;bottom:56px;left:12px;z-index:10;background:rgba(0,0,0,.6);padding:8px 12px;' +
    'border-radius:10px;font:14px/1.4 ui-monospace,Menlo,monospace;color:#e6edf3;pointer-events:none;min-width:230px;' +
    'border-left:6px solid #8b949e;white-space:pre';
  document.body.appendChild(el);

  // --- 3D-paneel (canvas-texture)
  const cv = document.createElement('canvas'); cv.width = 512; cv.height = 160;
  const g = cv.getContext('2d');
  const tex = new THREE.CanvasTexture(cv);
  if (THREE.SRGBColorSpace) tex.colorSpace = THREE.SRGBColorSpace;
  const mat = new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthTest: false, depthWrite: false, toneMapped: false });
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(0.32, 0.1), mat);
  mesh.position.set(0, -0.2, -0.75);              // net onder het midden van het beeld, 75 cm voor het hoofd
  mesh.renderOrder = 999; mesh.name = 'rec-hud'; mesh.frustumCulled = false;
  camera.add(mesh);

  let key = '';
  function update(snap) {
    const L = hudLines(snap, connLabel());
    const k = L.l1 + '|' + L.l2 + '|' + L.l3 + '|' + L.color;
    if (k === key) return; key = k;
    el.textContent = `${L.l1}\n${L.l2 || ' '}\n${L.l3}`; el.style.borderLeftColor = L.color;
    g.clearRect(0, 0, cv.width, cv.height);
    g.fillStyle = 'rgba(13,17,23,0.82)'; g.beginPath(); g.roundRect ? g.roundRect(0, 0, 512, 160, 24) : g.rect(0, 0, 512, 160); g.fill();
    g.fillStyle = L.color; g.fillRect(0, 0, 16, 160);
    g.fillStyle = '#e6edf3'; g.textBaseline = 'middle';
    g.font = 'bold 44px monospace'; g.fillText(L.l1, 34, 40);
    g.font = '30px monospace'; g.fillStyle = '#c9d1d9'; g.fillText(L.l2.slice(0, 32), 34, 90);
    g.fillStyle = '#8b949e'; g.font = '26px monospace'; g.fillText(L.l3, 34, 130);
    tex.needsUpdate = true;
  }
  return { el, mesh, update, setVisible(v) { mesh.visible = v; }, get text() { return el.textContent; } };
}
