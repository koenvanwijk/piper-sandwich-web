#!/usr/bin/env node
// Minimale test-server (Node >= 18, geen dependencies) voor de opname-client (?rec=ws://127.0.0.1:PORT/ws#token=...).
// Dit is GEEN productieserver (geen TLS, geen opslag): hij valideert het protocol en meet aantallen.
//   - hello: controleert token (uit env REC_TOKEN, nooit gelogd) en stuurt `welcome`
//   - state: controleert seq (oplopend +1), 14 state/14 action-waarden; telt berichten en bytes
//   - binaire frames: leest header [u32 seq][u8 cam][u8 fmt][u16 0] en controleert JPEG-magic (FF D8 .. FF D9)
//   - ping -> pong
// Gebruik: REC_TOKEN=geheim node tools/rec-echo-server.mjs --port=8765 [--report=/tmp/rec-report.json] [--origin=http://127.0.0.1:8000] [--save-frames=/tmp/frames]
import http from 'node:http';
import crypto from 'node:crypto';
import fs from 'node:fs';

const arg = (k, d) => (process.argv.find(a => a.startsWith(`--${k}=`)) || `--${k}=${d}`).split('=').slice(1).join('=');
const PORT = Number(arg('port', 8765));
const REPORT = arg('report', '');
const ORIGIN = arg('origin', '');
const SAVE = arg('save-frames', '');           // map waarin het eerste JPEG per camera wordt bewaard (alleen voor handmatige controle)
const TOKEN = process.env.REC_TOKEN || '';
const GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';

const rep = { connections: 0, rejected: 0, hello: null, state: { n: 0, bytes: 0, seqGaps: 0, seqRepeats: 0, firstSeq: null, lastSeq: null,
  bad14: 0, t0: null, t1: null, tSimLast: null, rttSamples: 0 },
  frames: { n: 0, bytes: 0, badJpeg: 0, perCam: {}, sizes: [], seqs: 0 }, pings: 0, last: null };
let firstHelloMs = null;

function frame(op, payload) {
  const n = payload.length;
  const head = n < 126 ? Buffer.from([0x80 | op, n]) : n < 65536
    ? Buffer.from([0x80 | op, 126, n >> 8, n & 255]) : (() => { const b = Buffer.alloc(10); b[0] = 0x80 | op; b[1] = 127; b.writeBigUInt64BE(BigInt(n), 2); return b; })();
  return Buffer.concat([head, payload]);
}
const sendText = (sock, o) => sock.write(frame(1, Buffer.from(JSON.stringify(o))));

const server = http.createServer((req, res) => { res.writeHead(426); res.end('websocket only'); });
server.on('upgrade', (req, sock) => {
  const fail = (code, why) => { rep.rejected++; sock.end(`HTTP/1.1 ${code} ${why}\r\nConnection: close\r\n\r\n`); };
  if (req.url.split('?')[0] !== '/ws') return fail(404, 'Not Found');
  if (ORIGIN && req.headers.origin !== ORIGIN) return fail(403, 'Forbidden');
  const key = req.headers['sec-websocket-key']; if (!key) return fail(400, 'Bad Request');
  sock.write('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n' +
    `Sec-WebSocket-Accept: ${crypto.createHash('sha1').update(key + GUID).digest('base64')}\r\n\r\n`);
  rep.connections++;
  let buf = Buffer.alloc(0), authed = false, frag = null;
  const close = (code) => { const b = Buffer.alloc(2); b.writeUInt16BE(code); sock.write(frame(8, b)); sock.end(); };

  function onMessage(isText, data) {
    if (isText) {
      let m; try { m = JSON.parse(data.toString()); } catch { return; }
      if (m.type === 'hello') {
        if (TOKEN && m.token !== TOKEN) { rep.rejected++; return close(4401); }
        authed = true;
        const { token, ...safe } = m; rep.hello = { ...safe, objects_static: Object.keys(m.objects_static || {}) };
        sendText(sock, { type: 'welcome', proto: 1, server_ms: Date.now() });
      } else if (!authed) { return close(4401); }
      else if (m.type === 'ping') { rep.pings++; sendText(sock, { type: 'pong', id: m.id, t_client_ms: m.t_client_ms }); }
      else if (m.type === 'state') {
        const s = rep.state, now = performance.now();
        s.n++; s.bytes += data.length; s.t0 ??= now; s.t1 = now;
        if (s.lastSeq !== null) { if (m.seq === s.lastSeq) s.seqRepeats++; else if (m.seq !== s.lastSeq + 1) s.seqGaps++; }
        s.firstSeq ??= m.seq; s.lastSeq = m.seq; s.tSimLast = m.t_sim;
        if (!Array.isArray(m.state) || m.state.length !== 14 || !Array.isArray(m.action) || m.action.length !== 14 ||
            ![...m.state, ...m.action].every(Number.isFinite)) s.bad14++;
        if (!rep.last || m.seq % 30 === 0) rep.last = m;
      }
    } else {
      const f = rep.frames; f.n++; f.bytes += data.length;
      const cam = data.readUInt8(4), jpeg = data.subarray(8);
      const ok = data.length > 12 && jpeg[0] === 0xff && jpeg[1] === 0xd8 && jpeg[jpeg.length - 2] === 0xff && jpeg[jpeg.length - 1] === 0xd9;
      if (!ok || data.readUInt8(5) !== 0) f.badJpeg++;
      (f.perCam[cam] ??= { n: 0, lastSeq: -1, seqBackwards: 0 });
      const pc = f.perCam[cam]; pc.n++; if (data.readUInt32LE(0) < pc.lastSeq) pc.seqBackwards++; pc.lastSeq = data.readUInt32LE(0);
      if (f.sizes.length < 5000) f.sizes.push(jpeg.length);
      if (SAVE && pc.n === 1) { fs.mkdirSync(SAVE, { recursive: true }); fs.writeFileSync(`${SAVE}/cam${cam}-seq${data.readUInt32LE(0)}.jpg`, jpeg); }
    }
  }

  sock.on('data', chunk => {
    buf = Buffer.concat([buf, chunk]);
    for (;;) {
      if (buf.length < 2) return;
      const fin = !!(buf[0] & 0x80), op = buf[0] & 15, masked = !!(buf[1] & 0x80);
      let len = buf[1] & 127, off = 2;
      if (len === 126) { if (buf.length < 4) return; len = buf.readUInt16BE(2); off = 4; }
      else if (len === 127) { if (buf.length < 10) return; len = Number(buf.readBigUInt64BE(2)); off = 10; }
      if (len > 8 * 1024 * 1024) return close(1009);
      if (buf.length < off + (masked ? 4 : 0) + len) return;
      const mask = masked ? buf.subarray(off, off + 4) : null; off += masked ? 4 : 0;
      const p = Buffer.from(buf.subarray(off, off + len)); buf = buf.subarray(off + len);
      if (mask) for (let i = 0; i < p.length; i++) p[i] ^= mask[i & 3];
      if (op === 8) { sock.end(frame(8, p)); return; }
      if (op === 9) { sock.write(frame(10, p)); continue; }
      if (op === 10) continue;
      if (op === 1 || op === 2) frag = { text: op === 1, parts: [p] }; else if (op === 0 && frag) frag.parts.push(p);
      if (fin && frag) { onMessage(frag.text, Buffer.concat(frag.parts)); frag = null; }
    }
  });
  sock.on('error', () => {});
});

function summary() {
  const s = rep.state, f = rep.frames, secs = s.t1 && s.t0 ? (s.t1 - s.t0) / 1000 : 0;
  const sizes = f.sizes.slice().sort((a, b) => a - b);
  return { connections: rep.connections, rejected: rep.rejected, hello: rep.hello,
    state: { n: s.n, hz: secs ? +((s.n - 1) / secs).toFixed(2) : null, avgBytes: s.n ? Math.round(s.bytes / s.n) : 0,
             firstSeq: s.firstSeq, lastSeq: s.lastSeq, seqGaps: s.seqGaps, seqRepeats: s.seqRepeats, bad14: s.bad14, tSimLast: s.tSimLast },
    frames: { n: f.n, hz: secs ? +(f.n / secs).toFixed(2) : null, badJpeg: f.badJpeg, perCam: f.perCam,
              jpegBytes: sizes.length ? { min: sizes[0], median: sizes[sizes.length >> 1], max: sizes[sizes.length - 1],
                                          avg: Math.round(sizes.reduce((a, b) => a + b, 0) / sizes.length) } : null },
    pings: rep.pings, lastState: rep.last };
}
const dump = () => { const j = JSON.stringify(summary(), null, 2); if (REPORT) fs.writeFileSync(REPORT, j); return j; };
process.on('SIGINT', () => { console.log(dump()); process.exit(0); });
process.on('SIGTERM', () => { dump(); process.exit(0); });
server.listen(PORT, '127.0.0.1', () => console.log(`rec-echo-server op ws://127.0.0.1:${PORT}/ws (token ${TOKEN ? 'vereist' : 'niet vereist'})`));
