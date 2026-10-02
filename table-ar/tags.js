import { AprilTagCamera, DEFAULT_TAG_SIZE_M } from '../src/apriltag-camera.js';

const $ = id => document.getElementById(id);
const q = new URLSearchParams(location.search);
const statusEl = $('status'), info = $('info'), hud = $('hudline');
const setStatus = (m, err = false) => { statusEl.textContent = m; statusEl.classList.toggle('err', err); };

if (q.get('camera')) { const sel = $('cam'); if (![...sel.options].some(o => o.value === q.get('camera'))) sel.add(new Option(q.get('camera'), q.get('camera'))); sel.value = q.get('camera'); }
$('size').value = q.get('tagsize') ?? DEFAULT_TAG_SIZE_M;     // standaard 0,08255 m (82,55 mm); 0 = geen pose
if (q.get('hfov')) $('hfov').value = q.get('hfov');

let cam = null, lastDets = [];
const f2 = (a, d = 1) => a.map(x => x.toFixed(d)).join(', ');

function makeCam() {
  const v = parseFloat($('size').value), size = Number.isFinite(v) && v >= 0 ? v : DEFAULT_TAG_SIZE_M;   // 0 = geen pose
  return new AprilTagCamera($('video'), $('tag-canvas'), {
    camera: $('cam').value, showVideo: false, tagSize: size, hfov: parseFloat($('hfov').value) || 77, procWidth: parseInt(q.get('proc') || '960', 10),
    onStatus: m => setStatus(m),
    onFrame: f => { hud.textContent = `${f.fps.toFixed(1)} FPS · detectie ${f.detMs.toFixed(1)} ms · ${f.w}×${f.h} · ${f.device || 'camera'}`; },
    onDetections: (dets, meta) => {
      lastDets = dets;
      const L = [`camera: ${cam.device?.label || '?'} (${cam.device?.deviceId?.slice(0, 8) || '-'}…)`,
                 `stream: ${cam.settings?.width || '?'}×${cam.settings?.height || '?'} @ ${cam.settings?.frameRate || '?'} fps; metadata-velden: ${Object.keys(cam.intrinsicsInfo?.raw || {}).join(', ') || 'geen'}`,
                 `intrinsics: ${meta.intrinsics ? `fx=${meta.intrinsics.fx.toFixed(0)} fy=${meta.intrinsics.fy.toFixed(0)} cx=${meta.intrinsics.cx.toFixed(0)} cy=${meta.intrinsics.cy.toFixed(0)} (${meta.intrinsics.source})` : 'geen (geef tag-zijde voor pose)'}`,
                 `tags: ${dets.length}`];
      for (const d of dets) {
        L.push(`#${d.id}  centrum (${f2([d.center.x, d.center.y])})  hoeken ${d.corners.map(c => `(${f2([c.x, c.y])})`).join(' ')}`);
        if (d.pose) L.push(`     t=(${f2(d.pose.t, 3)}) m  afstand ${Math.hypot(...d.pose.t).toFixed(3)} m  fout ${d.pose.e.toExponential(1)}`);
      }
      info.textContent = L.join('\n');
    },
  });
}

async function start() {
  const err = AprilTagCamera.supportError();
  if (err) return setStatus(err, true);
  cam?.stopCamera(); cam = makeCam(); window.tagCam = cam;
  $('start').disabled = true;
  try { await cam.startDetection(); $('stop').disabled = false; }
  catch (e) { console.error(e); setStatus(e.message, true); $('start').disabled = false; cam.stopCamera(); }
}
$('start').addEventListener('click', start);
$('stop').addEventListener('click', () => { cam?.stopCamera(); $('start').disabled = false; $('stop').disabled = true; setStatus('Gestopt.'); });
$('list').addEventListener('click', async () => {
  try {
    const c = await (cam || makeCam()).listCameras();
    info.textContent = c.length ? c.map((d, i) => `${i}: "${d.label || '(geen label: permissie nodig)'}"  ${d.deviceId.slice(0, 12)}…`).join('\n') : 'Geen videoinput-apparaten gevonden.';
  } catch (e) { setStatus(e.message, true); }
});
const unsupported = AprilTagCamera.supportError();
if (unsupported) setStatus(unsupported, true);
else if (q.get('autostart') === '1') start();
window.tagTest = { get detections() { return lastDets; }, start };
