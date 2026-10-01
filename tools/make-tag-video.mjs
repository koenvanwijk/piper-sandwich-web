#!/usr/bin/env node
// Maakt een y4m-testvideo (voor Chrome --use-file-for-fake-video-capture) met bekende tags + een JSON met de waarheid.
//   node tools/make-tag-video.mjs /tmp/tags.y4m /tmp/tags-truth.json
import fs from 'node:fs';
import { renderScene, rotXYZ } from './apriltag-synth.mjs';
const [out = '/tmp/tags.y4m', truthOut = '/tmp/tags-truth.json'] = process.argv.slice(2);
const cam = { w: 640, h: 480, fx: 520, fy: 520, cx: 320, cy: 240 };
const tags = [{ id: 7, size: 0.09, R: rotXYZ(0, 0.25, 0), t: [-0.12, -0.03, 0.8] },
              { id: 42, size: 0.09, R: rotXYZ(0.2, -0.2, 0.15), t: [0.1, 0.02, 0.8] },
              { id: 321, size: 0.09, R: rotXYZ(0, 0, 0.5), t: [0.0, 0.14, 1.0] }];
const { gray, truth } = renderScene(tags, cam, { noise: 2 });
// y4m 4:2:0: Y = grijs, U = V = 128; 15 identieke frames (Chrome loopt de file)
const w = cam.w, h = cam.h, chroma = Buffer.alloc((w / 2) * (h / 2), 128);
const hdr = Buffer.from(`YUV4MPEG2 W${w} H${h} F15:1 Ip A1:1 C420jpeg\n`), fh = Buffer.from('FRAME\n');
const parts = [hdr]; for (let i = 0; i < 15; i++) parts.push(fh, Buffer.from(gray), chroma, chroma);
fs.writeFileSync(out, Buffer.concat(parts));
fs.writeFileSync(truthOut, JSON.stringify({ cam, truth }, null, 1));
console.log(`geschreven: ${out} (${w}×${h}, 15 frames), ${truthOut}`);
