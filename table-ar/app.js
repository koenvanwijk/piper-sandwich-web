import * as THREE from 'three';
import { ARButton } from 'three/addons/webxr/ARButton.js';
import { TableCalibrator } from '../src/table-calibration.js';
import { AprilTagCamera, DEFAULT_TAG_SIZE_M } from '../src/apriltag-camera.js';
import { ARSandwichScene } from '../src/ar-sandwich-scene.js';
import { PoseHistory, TagTableScanner, cameraExtrinsics, sideFromLabel, median } from '../src/tag-surface.js';
import { TagSurfaceView } from '../src/tag-surface-view.js';
import { createTagDebugPanel } from '../src/tag-debug.js';
import { parseArPerf, optimizeSandwichForAR } from '../src/ar-perf.js';

const statusEl = document.querySelector('#status');
const detailsEl = document.querySelector('#details');
const recalibrateButton = document.querySelector('#recalibrate');
const clearButton = document.querySelector('#clear');
const cameraButton = document.querySelector('#camera');
const tagsButton = document.querySelector('#tags');
const scanButton = document.querySelector('#scan');
const video = document.querySelector('#camera-video');
const tagCanvas = document.querySelector('#tag-canvas');

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(70, innerWidth / innerHeight, 0.01, 20);

// Prestatie-opties (src/ar-perf.js): standaard lagere framebuffer-schaal (0.8), goedkopere materialen, minder DOM-updates, gedoseerde tag-detectie. ?perf=0 = oud gedrag.
const PERF = parseArPerf(location.search);
const renderer = new THREE.WebGLRenderer({ antialias: PERF.antialias, alpha: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.setClearColor(0x000000, 0);
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType('local-floor');
renderer.xr.setFramebufferScaleFactor(PERF.fbScale);   // moet vóór het starten van de sessie
renderer.xr.setFoveation(PERF.foveation);
document.body.appendChild(renderer.domElement);

scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 2.0));
const key = new THREE.DirectionalLight(0xffffff, 1.5);
key.position.set(1, 2, 1);
scene.add(key);

for (let i = 0; i < 2; i++) {
  const controller = renderer.xr.getController(i);
  const geom = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 0, -1)
  ]);
  const line = new THREE.Line(geom, new THREE.LineBasicMaterial({ color: 0xffffff }));
  line.scale.z = 1.5;
  controller.add(line);
  scene.add(controller);
}

let sandwichScene;
const calibrator = new TableCalibrator(renderer, scene, {
  onStatus: setStatus,
  onCalibrated: ({ width, depth, source }) => {
    sandwichScene?.placeOnTable(width, depth);
    const frameInfo = source === 'apriltag' ? 'frame: far-left corner origin · +X=right · +Z=towards you · +Y=up (from AprilTags)' : 'frame: A origin · +X=A→B · +Z=A→C · +Y=up';
    detailsEl.textContent =
      `table.width=${width.toFixed(3)} m\ntable.depth=${depth.toFixed(3)} m\n${frameInfo}\nscene: 2× Piper + 2× bread + 14× butter + board/knife/jar/plate`;
  },
});

sandwichScene = new ARSandwichScene(scene, calibrator.contentRoot, { onStatus: setStatus, fixTendons: PERF.enabled });
let perfStats = null;
sandwichScene.init().then(() => {
  if (PERF.enabled && sandwichScene.mujocoRoot) perfStats = optimizeSandwichForAR(sandwichScene.mujocoRoot, THREE, PERF);
  const saved = calibrator.getSavedCalibration();
  if (saved) sandwichScene.placeOnTable(saved.width, saved.depth);
}).catch(error => {
  console.error(error);
  setStatus(`Sandwich scene failed to load: ${error.message}`);
});

// Tag-hook: alleen info (ID's); de handmatige 3-punts kalibratie blijft ongewijzigd en gezaghebbend.
// ?tagsize=<meter> overschrijft de standaard 0.08255 m (82,55 mm); ?tagsize=0 zet de pose-schatting uit, ?camera=left|right kiest de passthrough-camera (zie table-ar/tags.html).
const tagParams = new URLSearchParams(location.search);
const tagsAuto = tagParams.get('tags') === '1';          // opt-in: bij sessiestart eerst het tafeloppervlak uit AprilTags schatten (terugval: 3-punts kalibratie)
const debugOn = tagParams.get('debug') === '1';
const numParam = (k, d) => { const v = parseFloat(tagParams.get(k)); return Number.isFinite(v) ? v : d; };
const listParam = k => { const v = (tagParams.get(k) || '').split(',').map(parseFloat); return v.length === 3 && v.every(Number.isFinite) ? v.map(x => x / 100) : null; };   // ?camoff=x,y,z in cm
const tagSizeM = tagParams.has('tagsize') && Number.isFinite(parseFloat(tagParams.get('tagsize'))) ? parseFloat(tagParams.get('tagsize')) : DEFAULT_TAG_SIZE_M;
let lastTagMeta = null;
const poseHistory = new PoseHistory();
const tagView = new TagSurfaceView(scene, tagSizeM || DEFAULT_TAG_SIZE_M);
let tagDebug = null;
if (debugOn) { scene.add(camera); tagDebug = createTagDebugPanel({ THREE, camera }); }   // hoofd-vaste meshes vragen dat de camera in de scene zit
let planeY = null;                                       // hoogte van door de bril gedetecteerde 'table'-vlakken (XR plane-detection), indien aanwezig
let pendingSurface = null;                               // door de scanner goedgekeurd, wordt in de XR-lus (met XRFrame) toegepast
let lastTick = 0;

const extrinsicsFor = label => cameraExtrinsics({
  side: ['left', 'right'].includes(tagParams.get('camera')) ? tagParams.get('camera') : sideFromLabel(label),
  offset: listParam('camoff'), pitchDeg: numParam('campitch', 0),
});

const scanner = new TagTableScanner({
  history: poseHistory, extrinsicsFor, tagSize: tagSizeM || DEFAULT_TAG_SIZE_M, edge: tagParams.get('tagedge') || 'center',
  latencyMs: numParam('camlat', 60),
  getTableY: () => {
    if (calibrator.root.visible && (calibrator.anchor || calibrator.getSavedCalibration())) return calibrator.root.matrix.elements[13];   // bestaande kalibratie
    return planeY;                                                                                                                      // anders plane-detection of null → pose-methode
  },
  getFrame: () => {
    if (!calibrator.root.visible) return null;
    const e = calibrator.root.matrix.elements; return { x: [e[0], e[1], e[2]], z: [e[8], e[9], e[10]] };
  },
  onUpdate: snap => {
    tagView.update(snap);
    if (snap.phase === 'scanning' && performance.now() - lastStatusT >= PERF.hudThrottleMs * 2) { lastStatusT = performance.now(); setStatus(`Scanning AprilTags… ${snap.tags.length} tag(s)` + (snap.est?.ok ? ` · ${(snap.est.width * 100).toFixed(1)} × ${(snap.est.depth * 100).toFixed(1)} cm` : '') + ` · ${snap.reason}`); }
  },
  onStable: surface => { pendingSurface = surface; },
  onFallback: reason => { stopTagDetection(); tagView.hide(); setStatus(reason); calibrator.start(); },
});

const tagCamera = new AprilTagCamera(video, tagCanvas, {
  onStatus: setStatus,
  camera: tagParams.get('camera') || 'auto',
  tagSize: tagSizeM,
  latencyMs: numParam('camlat', 60),
  maxFps: PERF.tagFps, procWidth: PERF.tagProc, dutyCycle: PERF.tagDuty,   // detectie (WASM ~40 ms/frame) mag de render-hoofdthread niet verhongeren
  onDetections: (detections, meta) => {
    lastTagMeta = meta;
    if (scanner.active) scanner.feed(detections, meta);
    if (!detections.length) return;
    const nowMs = performance.now(); if (nowMs - lastDetailsT < PERF.hudThrottleMs) return; lastDetailsT = nowMs;   // DOM-overlay wordt bij elke wijziging opnieuw gerasterd
    detailsEl.textContent =
      `${detailsEl.textContent.split('\nAprilTags:')[0]}\nAprilTags: ${detections.map(d => d.id).join(', ')}`;
  },
});

function debugInfo() {
  const ext = extrinsicsFor(lastTagMeta?.device || '');
  return { intrinsics: lastTagMeta?.intrinsics, tSource: lastTagMeta?.tSource, device: lastTagMeta?.device, extrinsics: ext, pitchDeg: numParam('campitch', 0),
           tagSize: tagSizeM, source: calibrator.getSavedCalibration()?.source || 'apriltag' };
}
let lastDetailsT = 0, lastStatusT = 0;
// Na de scan: detectie én camerastream stoppen (anders blijft video decoderen) en het preview-beeld weghalen.
function stopTagDetection() { tagCamera.stopDetection(); if (renderer.xr.isPresenting) { tagCamera.stopCamera(); hideTagPreview(); } }
// In AR laat een zichtbare <video>/<canvas> in de DOM-overlay elke frame opnieuw rasteren: standaard weg (?tagpreview=1 om te houden).
function hideTagPreview() { video.style.display = 'none'; tagCanvas.style.display = 'none'; }
function applyTagPreview() { if (renderer.xr.isPresenting && !PERF.tagPreviewInXR) { tagCamera.draw = false; hideTagPreview(); } else tagCamera.draw = true; }

/** Start de scanstap: camera + detector aan, tags middelen, bij stabiel resultaat het oppervlak toepassen; time-out/fout → handmatige 3-punts kalibratie. */
async function startTagScan() {
  if (!renderer.xr.getSession()) return setStatus('Start eerst AR (Enter AR), daarna de tag-scan.');
  try {
    calibrator.collecting = false; calibrator.clearMarkers();
    poseHistory.clear(); pendingSurface = null;
    scanner.start();
    if (!tagCamera.stream) await tagCamera.openCamera();
    if (!tagCamera.running) await tagCamera.startDetection();
    applyTagPreview();
    setStatus('Scanning AprilTags: leg een tag linksonder en een tag rechtsboven op de tafel (plat) en kijk er rustig naar.');
  } catch (error) {
    console.error(error);
    scanner.stop(); tagView.hide();
    setStatus(`AprilTag-scan niet mogelijk (${error.name || 'Error'}: ${error.message}). Terugval: handmatige 3-punts kalibratie.`);
    calibrator.start();
  }
}

// Alleen met ?tags=1 of ?debug=1: debug/test-haak (geen effect op gedrag).
if (tagsAuto || debugOn || tagParams.has('perf')) window.tableAR = { THREE, renderer, PERF, get perfStats() { return perfStats; }, scene, camera, calibrator, scanner, poseHistory, tagCamera, tagView, params: tagParams, startTagScan, debugInfo,
  get sandwichScene() { return sandwichScene; }, applyPending: () => pendingSurface, setPendingSurface: surface => { pendingSurface = surface; } };

const button = ARButton.createButton(renderer, {
  requiredFeatures: ['local-floor'],
  optionalFeatures: ['hit-test', 'plane-detection', 'anchors', 'hand-tracking', 'dom-overlay'],
  domOverlay: { root: document.body },
});
button.style.bottom = '20px';
document.body.appendChild(button);

renderer.xr.addEventListener('sessionstart', async () => {
  const session = renderer.xr.getSession();
  await calibrator.attachSession(session);
  recalibrateButton.disabled = false;
  clearButton.disabled = false;
  // A successfully restored persistent anchor is already aligned; otherwise
  // collect A/B/C immediately.
  scanButton.disabled = false;
  if (tagsAuto && !calibrator.anchor) startTagScan();             // ?tags=1 → eerst AprilTags (valt bij time-out/fout terug op de 3 punten)
  else if (!calibrator.anchor) calibrator.start();
  session.addEventListener('end', () => {
    scanner.stop(); stopTagDetection(); tagView.hide(); scanButton.disabled = true; planeY = null;
    calibrator.detachSession();
    recalibrateButton.disabled = true;
    clearButton.disabled = true;
    setStatus('AR ended.');
  }, { once: true });
});

recalibrateButton.addEventListener('click', () => { if (scanner.active) { scanner.stop(); stopTagDetection(); tagView.hide(); } calibrator.start(); });
scanButton.addEventListener('click', () => startTagScan());
clearButton.addEventListener('click', () => calibrator.clear());

cameraButton.addEventListener('click', async () => {
  try {
    await tagCamera.openCamera();
    tagsButton.disabled = false;
  } catch (error) {
    console.error(error);
    setStatus(`Camera failed: ${error.name || 'Error'} — ${error.message}`);
    detailsEl.textContent += '\nCamera pixel access unavailable; manual WebXR table calibration still works.';
  }
});

tagsButton.addEventListener('click', async () => {
  try { await tagCamera.startDetection(); applyTagPreview(); }
  catch (error) { console.error(error); setStatus(error.message); }
});

renderer.setAnimationLoop((time, frame) => {
  calibrator.update(frame);
  if (frame && (scanner.active || pendingSurface)) trackViewer(time, frame);
  if (tagDebug) tagDebug.update(scanner.snapshot(), debugInfo(), time);
  renderer.render(scene, camera);
});

/** Per XR-frame: viewer-pose in de historie (tijdsynchronisatie met het camerabeeld, zie src/tag-surface.js), tafelvlak-hoogte, scanner-tick, toepassen. */
function trackViewer(time, frame) {
  const ref = renderer.xr.getReferenceSpace(), vp = ref && frame.getViewerPose(ref);
  if (vp) { const p = vp.transform.position, o = vp.transform.orientation; poseHistory.push(time, [p.x, p.y, p.z], [o.x, o.y, o.z, o.w]); }
  tagCamera.pump(time);                                           // alleen actief zonder requestVideoFrameCallback
  if (frame.detectedPlanes && ref) {
    const ys = [];
    for (const pl of frame.detectedPlanes) if (pl.semanticLabel === 'table' && pl.orientation !== 'vertical') { const pp = frame.getPose(pl.planeSpace, ref); if (pp) ys.push(pp.transform.position.y); }
    planeY = ys.length ? median(ys) : null;
  }
  const now = performance.now();
  if (scanner.active && now - lastTick > 200) { lastTick = now; scanner.tick(); }
  if (pendingSurface) {
    const surface = pendingSurface; pendingSurface = null;
    scanner.markApplied(); stopTagDetection(); tagView.hide();
    calibrator.setFromSurface(frame, surface, { source: 'apriltag' }).catch(e => { console.error(e); setStatus('Oppervlak toepassen mislukt: ' + e.message); calibrator.start(); });
  }
}

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

function setStatus(message) { if (statusEl.textContent !== message) statusEl.textContent = message; }

(async () => {
  if (!navigator.xr) return setStatus('WebXR unavailable. Open over HTTPS in Meta Quest Browser.');
  const supported = await navigator.xr.isSessionSupported('immersive-ar');
  setStatus(supported ? 'Ready — press Enter AR. Calibration will start automatically.' : 'immersive-ar is not supported here.');
})();
