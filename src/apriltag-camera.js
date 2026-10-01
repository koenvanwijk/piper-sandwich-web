/**
 * Quest-camera + AprilTag (tag36h11) — getUserMedia → grijswaarden → WASM-detector (vendor/apriltag/, zie NOTICE).
 *
 * Quest Browser (>= 40.1) geeft de passthrough-camera's alleen via getUserMedia (permissie "headset cameras"); er is geen WebXR
 * camera-access en geen directe koppeling aan de XR-pose. Een tag-pose is dus een pose in het CAMERAFRAME, niet automatisch een
 * XR/tafel-pose; de bestaande handmatige 3-punts tafelkalibratie (table-calibration.js) blijft het gezaghebbende pad.
 *
 * Intrinsics: Meta noemt metadata (brandpuntsafstand, hoofdpunt, beeldgrootte), maar de exacte vorm in de browser is niet vastgelegd.
 * Daarom: (1) zoek in track.getSettings()/getCapabilities() naar velden die op focal/principal/intrinsic lijken, (2) anders een
 * ruwe schatting uit `hfov` (standaard 77° volgens een gemeten Quest 3-schatting; NIET gekalibreerd), (3) zonder tagsize geen pose.
 */
import { createAprilTagDetector, rgbaToGray } from './apriltag-detector.js';

export const DEFAULT_HFOV_DEG = 77;       // schatting voor de getUserMedia-stream van Quest 3 (1280×720, fx≈fy≈800); niet exact

/** Kies een passthrough-camera uit enumerateDevices(): 'left' | 'right' | 'front' | 'auto' | index | label-deel | deviceId. */
export function pickCamera(devices, want = 'auto') {
  const cams = devices.filter(d => d.kind === 'videoinput');
  if (!cams.length) return null;
  const lab = d => (d.label || '').toLowerCase();
  const back = cams.filter(d => /facing back|environment/.test(lab(d)));
  const num = d => { const m = lab(d).match(/camera\s*\d+\s+(\d+)/); return m ? Number(m[1]) : null; };   // "camera 2 1, facing back"
  if (want === 'left') return back.find(d => num(d) === 1) || back[0] || cams[0];
  if (want === 'right') return back.find(d => num(d) === 2) || back[back.length - 1] || cams[cams.length - 1];
  if (want === 'front') return cams.find(d => /facing front|user/.test(lab(d))) || cams[0];
  if (want === 'auto' || want == null || want === '') return back[0] || cams[0];
  if (/^\d+$/.test(String(want))) return cams[Number(want)] || cams[0];
  return cams.find(d => d.deviceId === want) || cams.find(d => lab(d).includes(String(want).toLowerCase())) || cams[0];
}

/** Zoek intrinsics-achtige velden in MediaTrack-instellingen (vorm bij Quest onbekend → alleen als ze er zijn). */
export function intrinsicsFromTrack(track) {
  const found = {};
  for (const src of [track?.getSettings?.(), track?.getCapabilities?.()]) {
    if (!src) continue;
    for (const [k, v] of Object.entries(src)) if (/focal|principal|intrinsic|fov|distortion/i.test(k)) found[k] = v;
  }
  const num = v => (typeof v === 'number' && isFinite(v) ? v : null);
  const pick = (...ks) => { for (const k of ks) if (num(found[k]) != null) return found[k]; return null; };
  const fx = pick('focalLengthX', 'fx', 'focalLength'), fy = pick('focalLengthY', 'fy', 'focalLength');
  const cx = pick('principalPointX', 'cx'), cy = pick('principalPointY', 'cy');
  return { raw: found, fx, fy, cx, cy, complete: fx != null && fy != null && cx != null && cy != null };
}

export class AprilTagCamera {
  /**
   * @param video   <video> (verborgen mag)
   * @param canvas  <canvas>: krijgt het (verkleinde) camerabeeld + overlay van gedetecteerde tags
   * @param opts    onStatus(msg), onDetections(list), onFrame(info) per verwerkt frame, camera:'auto'|'left'|'right'|..., deviceId,
   *                procWidth (breedte detectiebeeld, standaard 960), tagSize (m, null = geen pose), hfov (graden), maxFps, draw (bool)
   */
  constructor(video, canvas, opts = {}) {
    const { onStatus = () => {}, onDetections = () => {}, onFrame = () => {}, camera = 'auto', deviceId = null, procWidth = 960,
            tagSize = null, hfov = DEFAULT_HFOV_DEG, maxFps = 30, draw = true, showVideo = true } = opts;
    Object.assign(this, { video, canvas, onStatus, onDetections, onFrame, want: camera, deviceId, procWidth, tagSize, hfov, maxFps, draw, showVideo });
    this.ctx = canvas.getContext('2d', { willReadFrequently: true });
    this.stream = null; this.detector = null; this.running = false; this.device = null; this.devices = [];
    this.fps = { frames: 0, t0: 0, value: 0, detMs: 0 };
    this.intrinsicsInfo = null; this.lastError = null; this._gray = null; this._lastT = 0; this._gen = 0;
  }

  static supportError() {
    if (typeof navigator === 'undefined' || !navigator.mediaDevices) return 'navigator.mediaDevices ontbreekt: open de pagina via HTTPS (of localhost).';
    if (!navigator.mediaDevices.getUserMedia) return 'getUserMedia wordt niet ondersteund in deze browser.';
    if (typeof WebAssembly === 'undefined') return 'WebAssembly wordt niet ondersteund.';
    return null;
  }

  /** Vertaal getUserMedia-fouten naar een duidelijke melding. */
  static explainError(e) {
    const n = e?.name || 'Error';
    if (n === 'NotAllowedError' || n === 'SecurityError')
      return 'Camera-permissie geweigerd. Op de Quest: Browser → slotje/instellingen van de site → "Headset cameras" toestaan, en herlaad. (Ook nodig: HTTPS en de pagina in beeld; maar één site tegelijk mag de camera.)';
    if (n === 'NotFoundError' || n === 'OverconstrainedError') return 'Geen (passende) camera gevonden. Op de Quest 3 is Browser ≥ 40.1 nodig; zet zo nodig chrome://flags → "Experimental web platform features" aan.';
    if (n === 'NotReadableError' || n === 'AbortError') return 'Camera is in gebruik door een ander tabblad/app of niet leesbaar. Sluit andere camera-gebruikers en probeer opnieuw.';
    return `Camerafout ${n}: ${e?.message || e}`;
  }

  async listCameras() {
    const err = AprilTagCamera.supportError(); if (err) throw new Error(err);
    let devs = await navigator.mediaDevices.enumerateDevices();
    if (devs.some(d => d.kind === 'videoinput' && !d.label)) {         // labels pas zichtbaar na permissie: even een tijdelijke stream
      try { const tmp = await navigator.mediaDevices.getUserMedia({ video: true, audio: false }); tmp.getTracks().forEach(t => t.stop()); }
      catch (e) { throw new Error(AprilTagCamera.explainError(e)); }
      devs = await navigator.mediaDevices.enumerateDevices();
    }
    this.devices = devs.filter(d => d.kind === 'videoinput');
    return this.devices;
  }

  async openCamera() {
    const err = AprilTagCamera.supportError(); if (err) throw new Error(err);
    if (this.stream) return this.stream;
    let cams; try { cams = await this.listCameras(); } catch (e) { this.lastError = e.message; throw e; }
    if (!cams.length) { const m = 'Geen videoinput gevonden (enumerateDevices is leeg). Geen camera of geen permissie "headset cameras".'; this.lastError = m; throw new Error(m); }
    const dev = this.deviceId ? cams.find(d => d.deviceId === this.deviceId) || pickCamera(cams, this.deviceId) : pickCamera(cams, this.want);
    this.device = dev;
    const base = { width: { ideal: 1280 }, height: { ideal: 960 }, frameRate: { ideal: 30, max: 30 } };
    let stream;
    try { stream = await navigator.mediaDevices.getUserMedia({ video: { ...base, deviceId: { exact: dev.deviceId } }, audio: false }); }
    catch (e1) {
      this.onStatus(`Camera "${dev.label || dev.deviceId}" met exacte deviceId faalde (${e1.name}); probeer zonder resolutie-eisen.`);
      try { stream = await navigator.mediaDevices.getUserMedia({ video: { deviceId: { exact: dev.deviceId } }, audio: false }); }
      catch (e2) { const m = AprilTagCamera.explainError(e2); this.lastError = m; throw new Error(m); }
    }
    this.stream = stream; this.video.srcObject = stream; this.video.muted = true; this.video.playsInline = true;
    if (this.showVideo) this.video.style.display = 'block';
    try { await this.video.play(); } catch (e) { const m = 'Video kon niet starten: ' + (e.message || e.name); this.lastError = m; throw new Error(m); }
    const track = stream.getVideoTracks()[0], s = track?.getSettings?.() || {};
    this.intrinsicsInfo = intrinsicsFromTrack(track);
    this.settings = s;
    this.onStatus(`Camera actief: "${dev.label || dev.deviceId}" ${s.width || this.video.videoWidth || '?'}×${s.height || this.video.videoHeight || '?'} @ ${s.frameRate || '?'} fps; ${cams.length} video-ingang(en).`);
    track.addEventListener?.('ended', () => { this.onStatus('Camera-stream is gestopt (permissie ingetrokken of ander venster actief).'); this.stopDetection(); this.stream = null; });
    return stream;
  }

  stopCamera() { this.stopDetection(); this.stream?.getTracks().forEach(t => t.stop()); this.stream = null; this.video.srcObject = null; }

  async loadDetector() {
    if (this.detector) return this.detector;
    try { this.detector = await createAprilTagDetector({}, new URL('../vendor/apriltag/', import.meta.url)); }
    catch (e) { const m = 'AprilTag-detector laden mislukt: ' + (e.message || e); this.lastError = m; throw new Error(m); }
    return this.detector;
  }

  /** Intrinsics (in pixels van het detectiebeeld w×h) en bron-label. null → pose uit. */
  intrinsicsFor(w, h) {
    if (!this.tagSize) return null;
    const vw = this.video.videoWidth || w, ii = this.intrinsicsInfo;
    if (ii?.complete) { const k = w / vw; return { fx: ii.fx * k, fy: ii.fy * k, cx: ii.cx * k, cy: ii.cy * k, source: 'track-metadata' }; }
    const f = (w / 2) / Math.tan((this.hfov * Math.PI / 180) / 2);
    return { fx: f, fy: f, cx: w / 2, cy: h / 2, source: `schatting hfov=${this.hfov}° (niet gekalibreerd)` };
  }

  async startDetection() {
    await this.openCamera();
    await this.loadDetector();
    if (this.tagSize) this.detector.setTagSize(this.tagSize);
    this.running = true; const gen = ++this._gen;
    this.canvas.style.display = 'block';
    this.fps = { frames: 0, t0: performance.now(), value: 0, detMs: 0 };
    this.onStatus(`AprilTag-detectie actief (tag36h11${this.tagSize ? ', tagsize ' + this.tagSize + ' m' : ', zonder pose'}).`);
    const next = () => {
      if (!this.running || gen !== this._gen) return;
      if (this.video.requestVideoFrameCallback) this.video.requestVideoFrameCallback((now, meta) => this.step(now, meta, next));
      else requestAnimationFrame(now => this.step(now, null, next));
    };
    next();
  }

  stopDetection() { this.running = false; this._gen++; }

  /** Verwerk het huidige videoframe. Geeft de detecties terug (ook bruikbaar met een eigen <canvas>/Node-mock voor tests). */
  step(now, meta, next) {
    try {
      const v = this.video;
      if (v.videoWidth && now - this._lastT >= 1000 / this.maxFps - 2) {
        this._lastT = now; this.processFrame();
      }
    } catch (e) { this.lastError = e.message || String(e); this.onStatus('Detectiefout: ' + this.lastError); }
    next?.();
  }

  processFrame() {
    const v = this.video, w = Math.min(this.procWidth, v.videoWidth), h = Math.max(1, Math.round(w * v.videoHeight / v.videoWidth));
    if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; }
    if (!this._gray || this._gray.length !== w * h) this._gray = new Uint8Array(w * h);
    this.ctx.drawImage(v, 0, 0, w, h);
    const rgba = this.ctx.getImageData(0, 0, w, h).data;
    const gray = rgbaToGray(rgba, w, h, this._gray);
    const K = this.intrinsicsFor(w, h);
    if (K) this.detector.setIntrinsics(K.fx, K.fy, K.cx, K.cy); else this.detector.setIntrinsics(0, 0, 0, 0);
    const t0 = performance.now();
    const dets = this.detector.detect(gray, w, h);
    const ms = performance.now() - t0;
    const f = this.fps; f.frames++; f.detMs = f.detMs ? f.detMs * 0.9 + ms * 0.1 : ms;
    const el = performance.now() - f.t0; if (el >= 1000) { f.value = f.frames * 1000 / el; f.frames = 0; f.t0 = performance.now(); }
    if (this.draw) drawDetections(this.ctx, dets);
    this.onFrame({ fps: f.value, detMs: f.detMs, w, h, intrinsics: K, device: this.device?.label || '' });
    this.onDetections(dets, { w, h, intrinsics: K });
    return dets;
  }
}

/** Teken kader + hoekpunten + ID op een 2D-context (beeldcoördinaten). */
export function drawDetections(ctx, dets) {
  ctx.save(); ctx.lineWidth = 2; ctx.font = 'bold 16px system-ui,sans-serif'; ctx.textBaseline = 'top';
  for (const d of dets) {
    const c = d.corners;
    ctx.strokeStyle = '#00ff66'; ctx.beginPath(); ctx.moveTo(c[0].x, c[0].y);
    for (let i = 1; i < 4; i++) ctx.lineTo(c[i].x, c[i].y); ctx.closePath(); ctx.stroke();
    ctx.fillStyle = '#ff3b30'; ctx.beginPath(); ctx.arc(c[0].x, c[0].y, 4, 0, 6.283); ctx.fill();   // hoek 0 = oriëntatie
    const label = `#${d.id}` + (d.pose ? `  ${d.pose.t[2].toFixed(2)} m` : '');
    ctx.fillStyle = 'rgba(0,0,0,.65)'; const tw = ctx.measureText(label).width;
    ctx.fillRect(d.center.x - tw / 2 - 3, d.center.y - 10, tw + 6, 22);
    ctx.fillStyle = '#fff'; ctx.fillText(label, d.center.x - tw / 2, d.center.y - 8);
  }
  ctx.restore();
}
