// AprilTag-detector (tag36h11) bovenop de gebundelde WASM-build in vendor/apriltag/ (BSD-3-Clause, zie NOTICE).
// Browser én Node (voor tests). Geen externe CDN. De C-kant (apriltag_js.c) wordt hier alleen via cwrap aangeroepen.
//
// Conventies van de uitvoer van detect():
//   corners : 4 hoeken {x,y} in beeldpixels (y omlaag), subpixel; volgorde = AprilTag-volgorde (tegen de klok in in tag-coördinaten:
//             linksonder, rechtsonder, rechtsboven, linksboven van de tag zoals je hem leest)
//   center  : {x,y}
//   pose    : alleen met intrinsics: { R: 3x3 rij-groot (R[rij][kolom]), t: [x,y,z] in meter, e: objectruimte-fout, size }
//             Cameraframe: x rechts, y omlaag, z vooruit (zoals OpenCV). Tagframe: x rechts, y omlaag, z het vlak IN (van de kijker af).
//             R draait tag- naar cameracoördinaten: p_cam = R·p_tag + t.
export const DEFAULT_TAG_SIZE_M = 0.08255;    // 82,55 mm (3,25") zwarte vierkant van onze tags; de C-code zelf gebruikt 0,15 m, dat overschrijven we bij init

export const MAX_TAG_ID = 600;               // grootte van de tagsize-tabel in de C-code

export class AprilTagDetector {
  constructor(Module, { decimate = 2.0, sigma = 0.0, refineEdges = 1, maxDetections = 0, returnPose = 1 } = {}) {
    this.M = Module;
    const cw = (n, r, a) => Module.cwrap(n, r, a);
    this._init = cw('atagjs_init', 'number', []);
    this._destroy = cw('atagjs_destroy', 'number', []);
    this._setOptions = cw('atagjs_set_detector_options', 'number', ['number', 'number', 'number', 'number', 'number', 'number', 'number']);
    this._setPose = cw('atagjs_set_pose_info', 'number', ['number', 'number', 'number', 'number']);
    this._setBuf = cw('atagjs_set_img_buffer', 'number', ['number', 'number', 'number']);
    this._setTagSize = cw('atagjs_set_tag_size', null, ['number', 'number']);
    this._detect = cw('atagjs_detect', 'number', []);
    if (this._init() !== 0) throw new Error('AprilTag-detector kon niet initialiseren (atagjs_init)');
    this.options = { decimate, sigma, refineEdges, maxDetections, returnPose };
    this._applyOptions();
    this.intrinsics = null;
    this.setTagSize(DEFAULT_TAG_SIZE_M);       // alle ID's op de standaardmaat (de C-code start met 0,15 m)
  }

  _applyOptions() {
    const o = this.options;
    this._setOptions(o.decimate, o.sigma, 1, o.refineEdges, o.maxDetections, o.returnPose, 0);
  }
  setOptions(o) { Object.assign(this.options, o); this._applyOptions(); }

  /** Intrinsics in pixels van het beeld dat je aan detect() geeft. Zonder intrinsics: pose staat uit (alleen hoeken/centrum). */
  setIntrinsics(fx, fy, cx, cy) {
    this.intrinsics = (fx > 0 && fy > 0) ? { fx, fy, cx, cy } : null;
    if (this.intrinsics) this._setPose(fx, fy, cx, cy);
  }
  /** Zijde van de zwarte rand-tot-rand tag in meter (alle ID's, of één `id`). */
  setTagSize(sizeM, id = null) {
    if (id != null) { this._setTagSize(id, sizeM); return; }
    for (let i = 0; i < MAX_TAG_ID; i++) this._setTagSize(i, sizeM);
  }

  /** gray: Uint8Array/Uint8ClampedArray lengte w*h. Geeft [{id, corners, center, pose?}] */
  detect(gray, w, h) {
    if (gray.length < w * h) throw new Error(`grijswaardenbeeld te klein: ${gray.length} < ${w}×${h}`);
    const buf = this._setBuf(w, h, w);
    this.M.HEAPU8.set(gray.subarray ? gray.subarray(0, w * h) : gray, buf);
    const ptr = this._detect();
    const len = this.M.getValue(ptr, 'i32');
    if (!len) return [];
    const strPtr = this.M.getValue(ptr + 4, 'i32');
    const json = new TextDecoder().decode(new Uint8Array(this.M.HEAPU8.buffer, strPtr, len));
    let raw; try { raw = JSON.parse(json); } catch (e) { throw new Error('ongeldige detector-uitvoer: ' + json.slice(0, 80)); }
    if (!Array.isArray(raw)) throw new Error('detector: ' + (raw.result || json.slice(0, 80)));
    const withPose = this.intrinsics && this.options.returnPose;
    return raw.map(d => {
      const out = { id: d.id, corners: d.corners, center: d.center };
      if (withPose && d.pose) {
        const P = d.pose.R;                                  // de C-code schrijft kolom i van R als rij i: R[r][c] = P[c][r]
        out.pose = { R: [[P[0][0], P[1][0], P[2][0]], [P[0][1], P[1][1], P[2][1]], [P[0][2], P[1][2], P[2][2]]],
                     t: d.pose.t, e: d.pose.e, size: d.pose.size };
      }
      return out;
    });
  }

  destroy() { try { this._destroy(); } catch { /* al vrijgegeven */ } }
}

/** Laadt vendor/apriltag/apriltag_wasm.js (+ .wasm) en geeft een gereed emscripten-Module terug. Browser: via <script>, Node: via require. */
export async function loadAprilTagModule(baseUrl = new URL('../vendor/apriltag/', import.meta.url)) {
  const base = String(baseUrl).endsWith('/') ? String(baseUrl) : String(baseUrl) + '/';
  const opts = { print() {}, printErr(...a) { if (globalThis.__apriltagDebug) console.warn(...a); },
                 locateFile: p => base + p };
  let factory;
  if (typeof document === 'undefined') {                        // Node
    const { createRequire } = await import('node:module');
    const { fileURLToPath } = await import('node:url');
    factory = createRequire(import.meta.url)(fileURLToPath(base + 'apriltag_wasm.js'));
    const fs = await import('node:fs');
    opts.wasmBinary = fs.readFileSync(fileURLToPath(base + 'apriltag_wasm.wasm'));   // Node 20: geen fetch(file://)
    opts.locateFile = p => fileURLToPath(base + p);
  } else {
    if (typeof globalThis.AprilTagWasm !== 'function') {
      await new Promise((resolve, reject) => {
        const s = document.createElement('script');
        s.src = base + 'apriltag_wasm.js'; s.async = true;
        s.onload = resolve; s.onerror = () => reject(new Error('kon ' + s.src + ' niet laden (ontbreekt vendor/apriltag/?)'));
        document.head.appendChild(s);
      });
    }
    factory = globalThis.AprilTagWasm;
    if (typeof factory !== 'function') throw new Error('AprilTagWasm niet beschikbaar na laden van apriltag_wasm.js');
  }
  return await factory(opts);
}

export async function createAprilTagDetector(options = {}, baseUrl) {
  return new AprilTagDetector(await loadAprilTagModule(baseUrl), options);
}

/** RGBA -> grijswaarden (ITU-R BT.601, integer), schrijft in `out` (lengte w*h) of maakt een nieuwe. */
export function rgbaToGray(rgba, w, h, out = new Uint8Array(w * h)) {
  for (let i = 0, j = 0, n = w * h; j < n; i += 4, j++) out[j] = (rgba[i] * 77 + rgba[i + 1] * 150 + rgba[i + 2] * 29) >> 8;
  return out;
}
