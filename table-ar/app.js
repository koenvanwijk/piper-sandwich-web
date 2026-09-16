import * as THREE from 'three';
import { ARButton } from 'three/addons/webxr/ARButton.js';
import { TableCalibrator } from '../src/table-calibration.js';
import { AprilTagCamera } from '../src/apriltag-camera.js';
import { ARSandwichScene } from '../src/ar-sandwich-scene.js';

const statusEl = document.querySelector('#status');
const detailsEl = document.querySelector('#details');
const recalibrateButton = document.querySelector('#recalibrate');
const clearButton = document.querySelector('#clear');
const cameraButton = document.querySelector('#camera');
const tagsButton = document.querySelector('#tags');
const video = document.querySelector('#camera-video');
const tagCanvas = document.querySelector('#tag-canvas');

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(70, innerWidth / innerHeight, 0.01, 20);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.setClearColor(0x000000, 0);
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType('local-floor');
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
  onCalibrated: ({ width, depth }) => {
    sandwichScene?.placeOnTable(width, depth);
    detailsEl.textContent =
      `table.width=${width.toFixed(3)} m\ntable.depth=${depth.toFixed(3)} m\nframe: A origin · +X=A→B · +Z=A→C · +Y=up\nscene: 2× Piper + 2× bread + 14× butter + board/knife/jar/plate`;
  },
});

sandwichScene = new ARSandwichScene(scene, calibrator.contentRoot, { onStatus: setStatus });
sandwichScene.init().then(() => {
  const saved = calibrator.getSavedCalibration();
  if (saved) sandwichScene.placeOnTable(saved.width, saved.depth);
}).catch(error => {
  console.error(error);
  setStatus(`Sandwich scene failed to load: ${error.message}`);
});

const tagCamera = new AprilTagCamera(video, tagCanvas, {
  onStatus: setStatus,
  onDetections: detections => {
    if (!detections.length) return;
    detailsEl.textContent =
      `${detailsEl.textContent.split('\nAprilTags:')[0]}\nAprilTags: ${detections.map(d => d.id).join(', ')}`;
  },
});

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
  if (!calibrator.anchor) calibrator.start();
  session.addEventListener('end', () => {
    calibrator.detachSession();
    recalibrateButton.disabled = true;
    clearButton.disabled = true;
    setStatus('AR ended.');
  }, { once: true });
});

recalibrateButton.addEventListener('click', () => calibrator.start());
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
  try { await tagCamera.startDetection(); }
  catch (error) { console.error(error); setStatus(error.message); }
});

renderer.setAnimationLoop((_, frame) => {
  calibrator.update(frame);
  renderer.render(scene, camera);
});

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

function setStatus(message) { statusEl.textContent = message; }

(async () => {
  if (!navigator.xr) return setStatus('WebXR unavailable. Open over HTTPS in Meta Quest Browser.');
  const supported = await navigator.xr.isSessionSupported('immersive-ar');
  setStatus(supported ? 'Ready — press Enter AR. Calibration will start automatically.' : 'immersive-ar is not supported here.');
})();
