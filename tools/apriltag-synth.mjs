// Synthetische testbeelden voor de AprilTag-detector: tag36h11-patronen (tools/fixtures/tag36h11-patterns.json, 10×10 cellen incl.
// witte rand; gegenereerd met het npm-pakket `apriltag` (MIT) uit de officiële codes) worden met een pinhole-camera
// (R, t, fx, fy, cx, cy) in een grijs beeld geprojecteerd. Geeft ook de ware hoeken terug (volgorde van de detector).
import fs from 'node:fs';
const PAT = JSON.parse(fs.readFileSync(new URL('./fixtures/tag36h11-patterns.json', import.meta.url), 'utf8'));
export const FIXTURE_IDS = Object.keys(PAT).map(Number);

const mulMV = (R, v) => [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2], R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2], R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]];
const mulMtV = (R, v) => [R[0][0]*v[0]+R[1][0]*v[1]+R[2][0]*v[2], R[0][1]*v[0]+R[1][1]*v[1]+R[2][1]*v[2], R[0][2]*v[0]+R[1][2]*v[1]+R[2][2]*v[2]];
export const rotXYZ = (ax, ay, az) => {          // R = Rz·Ry·Rx (radialen)
  const [cx, sx, cy, sy, cz, sz] = [Math.cos(ax), Math.sin(ax), Math.cos(ay), Math.sin(ay), Math.cos(az), Math.sin(az)];
  const Rx = [[1,0,0],[0,cx,-sx],[0,sx,cx]], Ry = [[cy,0,sy],[0,1,0],[-sy,0,cy]], Rz = [[cz,-sz,0],[sz,cz,0],[0,0,1]];
  const m = (A, B) => A.map((r, i) => [0,1,2].map(j => r[0]*B[0][j]+r[1]*B[1][j]+r[2]*B[2][j]));
  return m(Rz, m(Ry, Rx));
};

/** Ware hoeken (AprilTag-volgorde: links-onder, rechts-onder, rechts-boven, links-boven van de tag zoals gelezen). */
export function projectCorners(tag, cam) {
  const h = tag.size / 2, P = [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]];    // tag-frame: x rechts, y omlaag, z het vlak in
  return P.map(p => { const c = mulMV(tag.R, p).map((v, i) => v + tag.t[i]); return { x: cam.fx * c[0] / c[2] + cam.cx, y: cam.fy * c[1] / c[2] + cam.cy }; });
}

/** tags: [{id, size (m, zwart vierkant), R, t}] ; cam: {w,h,fx,fy,cx,cy}. Geeft {gray: Uint8Array, truth: [{id, corners, center}]}. */
export function renderScene(tags, cam, { bg = 128, noise = 0, ss = 3, seed = 1 } = {}) {
  const { w, h } = cam, gray = new Uint8Array(w * h).fill(bg);
  let rs = seed >>> 0; const rnd = () => ((rs = (Math.imul(rs, 1664525) + 1013904223) >>> 0) / 4294967296);
  for (const tg of tags) {
    const pat = PAT[tg.id]; if (!pat) throw new Error('geen patroon voor id ' + tg.id);
    const cell = tg.size / 8, half = 5 * cell;                       // 10 cellen breed, zwarte rand = middelste 8
    const n = mulMV(tg.R, [0, 0, 1]), nt = n[0]*tg.t[0] + n[1]*tg.t[1] + n[2]*tg.t[2];
    const cs = projectCorners({ ...tg, size: tg.size * 10 / 8 }, cam);
    const x0 = Math.max(0, Math.floor(Math.min(...cs.map(c => c.x)) - 2)), x1 = Math.min(w - 1, Math.ceil(Math.max(...cs.map(c => c.x)) + 2));
    const y0 = Math.max(0, Math.floor(Math.min(...cs.map(c => c.y)) - 2)), y1 = Math.min(h - 1, Math.ceil(Math.max(...cs.map(c => c.y)) + 2));
    for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
      let acc = 0, cnt = 0;
      for (let sy = 0; sy < ss; sy++) for (let sx = 0; sx < ss; sx++) {
        const ray = [(x + (sx + 0.5) / ss - cam.cx) / cam.fx, (y + (sy + 0.5) / ss - cam.cy) / cam.fy, 1];
        const d = n[0]*ray[0] + n[1]*ray[1] + n[2]*ray[2]; if (Math.abs(d) < 1e-9) continue;
        const s = nt / d; if (s <= 0) continue;
        const p = mulMtV(tg.R, [ray[0]*s - tg.t[0], ray[1]*s - tg.t[1], ray[2]*s - tg.t[2]]);
        const u = p[0], v = p[1];                                    // meter in het tagvlak, oorsprong in het midden
        if (Math.abs(u) >= half || Math.abs(v) >= half) continue;
        const col = Math.floor((u + half) / cell), row = Math.floor((v + half) / cell);
        const ch = pat[row][col]; acc += ch === 'b' ? 0 : 255; cnt++;
      }
      if (cnt) { const frac = cnt / (ss * ss); gray[y * w + x] = Math.round(gray[y * w + x] * (1 - frac) + (acc / cnt) * frac); }
    }
  }
  if (noise) for (let i = 0; i < gray.length; i++) gray[i] = Math.max(0, Math.min(255, gray[i] + Math.round((rnd() - 0.5) * 2 * noise)));
  const truth = tags.map(tg => { const corners = projectCorners(tg, cam);
    return { id: tg.id, corners, center: { x: corners.reduce((a, c) => a + c.x, 0) / 4, y: corners.reduce((a, c) => a + c.y, 0) / 4 }, R: tg.R, t: tg.t, size: tg.size }; });
  return { gray, truth };
}

/** PGM (P5) schrijven, voor ffmpeg → y4m/mjpeg of handmatige controle. */
export function pgm(gray, w, h) { return Buffer.concat([Buffer.from(`P5\n${w} ${h}\n255\n`), Buffer.from(gray)]); }
