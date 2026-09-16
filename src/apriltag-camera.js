/**
 * Quest camera + AprilTag hook.
 * Detection is intentionally separate from XR alignment: pixel access alone
 * does not provide the calibrated camera intrinsics/extrinsics needed to treat
 * a tag pose as an authoritative XR/table pose.
 */
export class AprilTagCamera {
  constructor(video, canvas, { onStatus = () => {}, onDetections = () => {} } = {}) {
    this.video = video;
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d', { willReadFrequently: true });
    this.onStatus = onStatus;
    this.onDetections = onDetections;
    this.stream = null;
    this.detector = null;
    this.running = false;
    this.inFlight = false;
    this.lastDetectionAt = 0;
  }

  async openCamera() {
    if (!navigator.mediaDevices?.getUserMedia) throw new Error('MediaDevices/getUserMedia unavailable.');
    if (this.stream) return this.stream;
    let stream;
    try {
      stream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: { exact: 'environment' }, width: { ideal: 1280 }, height: { ideal: 960 }, frameRate: { ideal: 30, max: 30 } },
        audio: false
      });
    } catch (firstError) {
      this.onStatus(`Exact environment camera failed (${firstError.name}); retrying with ideal facing mode.`);
      stream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: { ideal: 'environment' }, width: { ideal: 1280 }, height: { ideal: 960 }, frameRate: { ideal: 30, max: 30 } },
        audio: false
      });
    }
    this.stream = stream;
    this.video.srcObject = stream;
    await this.video.play();
    this.video.style.display = 'block';
    const s = stream.getVideoTracks()[0]?.getSettings?.() || {};
    const devices = await navigator.mediaDevices.enumerateDevices();
    const cameras = devices.filter(d => d.kind === 'videoinput');
    this.onStatus(`Camera active: ${s.width || '?'}×${s.height || '?'} @ ${s.frameRate || '?'} fps; ${cameras.length} video input(s).`);
    return stream;
  }

  async startDetection() {
    await this.openCamera();
    await this.loadDetector();
    this.running = true;
    this.canvas.style.display = 'block';
    this.onStatus('AprilTag detection running (tag36h11, ~5 Hz).');
    requestAnimationFrame(t => this.loop(t));
  }

  stopDetection() { this.running = false; }

  async loadDetector() {
    if (this.detector) return;
    if (!globalThis.Apriltag) {
      throw new Error('AprilTag WASM detector is not bundled yet; camera test is available, tag pose alignment is the next step.');
    }
  }

  async loop(now = performance.now()) {
    if (!this.running) return;
    requestAnimationFrame(t => this.loop(t));
    if (this.inFlight || now - this.lastDetectionAt < 200 || !this.video.videoWidth) return;
    this.lastDetectionAt = now;
    this.inFlight = true;
    try {
      const w = 640, h = Math.max(1, Math.round(w * this.video.videoHeight / this.video.videoWidth));
      if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; }
      this.ctx.drawImage(this.video, 0, 0, w, h);
      const rgba = this.ctx.getImageData(0, 0, w, h).data;
      const gray = new Uint8Array(w * h);
      for (let i = 0, j = 0; i < rgba.length; i += 4, j++) gray[j] = (rgba[i] * 77 + rgba[i+1] * 150 + rgba[i+2] * 29) >> 8;
      const detections = await this.detector.detect(gray, w, h);
      this.onDetections(detections || []);
    } finally {
      this.inFlight = false;
    }
  }
}
