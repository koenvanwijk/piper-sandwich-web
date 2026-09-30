// WebSocket-client voor opname (opt-in via ?rec=wss://host/ws#token=...).
// Protocol: zie DESIGN-vr-recording.md §1 en README ("Recording client"). Geen three.js/DOM-afhankelijkheid
// behalve het kleine statusbadge. Het token staat alleen in het URL-fragment (nooit in query/logs/git).
import { PROTO_VERSION } from './rec-state.js';

const TOKEN_KEY = 'piper-rec-session';           // sessionStorage (alleen dit tabblad)
const LOOPBACK = new Set(['localhost', '127.0.0.1', '[::1]']);

/** Lees ?rec / #token / ?cams uit de URL. Geeft {cfg} of {error}; null als opname niet gevraagd is. */
export function parseRecConfig(loc = location, storage = globalThis.sessionStorage) {
  const q = new URLSearchParams(loc.search);
  const raw = q.get('rec');
  if (!raw) return null;
  let u;
  try { u = new URL(raw); } catch { return { error: 'ongeldige ?rec=-URL' }; }
  if (u.protocol !== 'wss:' && !(u.protocol === 'ws:' && LOOPBACK.has(u.hostname)))
    return { error: 'alleen wss:// (of ws:// naar localhost) toegestaan' };
  // token: uit #token=..., anders uit sessionStorage (zelfde rec-URL); fragment wordt uit de adresbalk gehaald
  let token = new URLSearchParams((loc.hash || '').replace(/^#/, '')).get('token') || '';
  try {
    if (token) {
      storage && storage.setItem(TOKEN_KEY, JSON.stringify({ url: raw, token }));
      if (globalThis.history && history.replaceState) history.replaceState(null, '', loc.pathname + loc.search);
    } else if (storage) {
      const s = JSON.parse(storage.getItem(TOKEN_KEY) || 'null');
      if (s && s.url === raw) token = s.token;
    }
  } catch { /* storage niet beschikbaar: alleen fragment */ }
  const [cw, ch] = (q.get('camsize') || '640x480').split('x').map(Number);
  const cams = (q.get('cams') ?? 'front,top').split(',').map(s => s.trim()).filter(Boolean);
  return { cfg: { url: u.href, host: u.host, token, cams,
                  camW: cw > 0 ? cw : 640, camH: ch > 0 ? ch : 480,
                  camQuality: Math.min(1, Math.max(0.1, Number(q.get('camq')) || 0.8)) } };
}

/** Binaire beeldframe: [u32 seq LE][u8 cam_id][u8 formaat (0 = JPEG)][u16 0][JPEG-bytes]. */
export function encodeFrame(seq, camId, jpeg) {
  const out = new Uint8Array(8 + jpeg.byteLength);
  const dv = new DataView(out.buffer);
  dv.setUint32(0, seq >>> 0, true); dv.setUint8(4, camId); dv.setUint8(5, 0); dv.setUint16(6, 0, true);
  out.set(new Uint8Array(jpeg), 8);
  return out;
}

export class RecorderClient {
  constructor({ url, token, helloFn, onStatus = () => {}, onCmd = null, onEvent = null, WS = globalThis.WebSocket }) {
    this.url = url; this.token = token; this.helloFn = helloFn; this.onStatus = onStatus; this.onCmd = onCmd; this.onEvent = onEvent;
    this.WS = WS; this.ws = null; this.state = 'idle'; this.stopped = false;
    this.attempt = 0; this.timer = null; this.pingTimer = null;
    this.lastPong = 0; this.pingId = 0; this.session = Math.random().toString(36).slice(2, 10);
    this.stats = { tx_state: 0, tx_frames: 0, tx_bytes: 0, drop_state_offline: 0, drop_state_backpressure: 0,
                   drop_frames_offline: 0, drop_frames_backpressure: 0, tx_cmd: 0, drop_cmd_offline: 0, rx_events: 0, reconnects: 0, rtt_ms: null,
                   last_error: null };
  }

  get ready() { return this.state === 'ready'; }
  _set(s, msg) { this.state = s; this.onStatus(s, msg); }

  start() { this.stopped = false; this._connect(); }
  stop() { this.stopped = true; clearTimeout(this.timer); clearInterval(this.pingTimer);
           if (this.ws) { try { this.ws.close(1000, 'client stop'); } catch {} } this._set('closed'); }

  _connect() {
    if (this.stopped) return;
    this._set('connecting');
    let ws;
    try { ws = new this.WS(this.url); } catch (e) { this.stats.last_error = String(e.message || e); return this._retry(); }
    ws.binaryType = 'arraybuffer';
    this.ws = ws;
    ws.onopen = () => {
      this._set('authenticating');
      const hello = { ...this.helloFn(), token: this.token, session: this.session,
                      t_client_ms: performance.now(), reconnect: this.attempt > 0 || this.stats.reconnects > 0 };
      ws.send(JSON.stringify(hello));
      this.lastPong = performance.now();
      clearInterval(this.pingTimer);
      this.pingTimer = setInterval(() => this._ping(), 2000);
    };
    ws.onmessage = ev => {
      if (typeof ev.data !== 'string') return;
      let m; try { m = JSON.parse(ev.data); } catch { return; }
      if (m.type === 'welcome') { this.attempt = 0; this.lastPong = performance.now(); this._set('ready'); }
      else if (m.type === 'pong') {
        this.lastPong = performance.now();
        if (typeof m.t_client_ms === 'number') this.stats.rtt_ms = +(performance.now() - m.t_client_ms).toFixed(1);
      } else if (m.type === 'ping') ws.send(JSON.stringify({ type: 'pong', id: m.id, t_client_ms: m.t_client_ms }));
      else if (m.type === 'cmd' && this.onCmd) this.onCmd(m);
      else if (m.type === 'event') {                                // fase 3: episode-events van de server (HUD)
        this.stats.rx_events++;
        if (this.onEvent) { try { this.onEvent(m); } catch (e) { this.stats.last_error = 'onEvent: ' + (e.message || e); } }
      }
    };
    ws.onerror = () => { this.stats.last_error = 'websocket-fout'; };
    ws.onclose = ev => {
      clearInterval(this.pingTimer);
      if (this.stopped) return;
      if (ev.code === 4401 || ev.code === 4403) {          // auth/origin geweigerd: niet blijven proberen
        this.stopped = true; this._set('auth-failed', `server weigerde (${ev.code})`); return;
      }
      this._retry(ev.code);
    };
  }

  _retry(code) {
    this.stats.reconnects++;
    const base = Math.min(10000, 500 * 2 ** this.attempt++);      // 0,5 s .. 10 s
    const delay = base * (0.5 + Math.random() * 0.5);             // jitter
    this._set('reconnecting', `over ${(delay / 1000).toFixed(1)} s${code ? ' (code ' + code + ')' : ''}`);
    clearTimeout(this.timer);
    this.timer = setTimeout(() => this._connect(), delay);
  }

  _ping() {
    if (!this.ws || this.ws.readyState !== 1) return;
    if (performance.now() - this.lastPong > 6000) {               // watchdog: geen pong -> herverbinden
      this.stats.last_error = 'pong time-out'; try { this.ws.close(); } catch {} return;
    }
    this.ws.send(JSON.stringify({ type: 'ping', id: ++this.pingId, t_client_ms: performance.now() }));
  }

  /** true als er JPEG-frames verstuurd mogen worden (verbonden + geen opgestapelde buffer). */
  wantFrames() { return this.ready && this.ws.bufferedAmount < 512 * 1024; }

  sendState(msg) {
    if (!this.ready) { this.stats.drop_state_offline++; return false; }
    if (this.ws.bufferedAmount > 2 * 1024 * 1024) { this.stats.drop_state_backpressure++; return false; }
    const s = JSON.stringify(msg); this.ws.send(s);
    this.stats.tx_state++; this.stats.tx_bytes += s.length; return true;
  }

  /** Fase 3: commando naar de server (start|stop|discard|success|reset|status). false = niet verstuurd (offline). */
  sendCmd(cmd, extra = {}) {
    if (!this.ready) { this.stats.drop_cmd_offline++; return false; }
    this.ws.send(JSON.stringify({ type: 'cmd', ...extra, cmd })); this.stats.tx_cmd++; return true;
  }

  sendFrame(seq, camId, jpeg) {
    if (!this.ready) { this.stats.drop_frames_offline++; return false; }
    if (this.ws.bufferedAmount > 512 * 1024) { this.stats.drop_frames_backpressure++; return false; }
    const b = encodeFrame(seq, camId, jpeg); this.ws.send(b);
    this.stats.tx_frames++; this.stats.tx_bytes += b.byteLength; return true;
  }
}

/** Klein statusbadge rechtsboven (alleen met ?rec). Toont host en toestand, nooit het token. */
export function createRecBadge(host) {
  const el = document.createElement('div');
  el.id = 'rec-badge';
  el.style.cssText = 'position:fixed;top:12px;right:12px;z-index:10;background:rgba(0,0,0,.55);' +
    'padding:6px 10px;border-radius:10px;font:12px/1.4 system-ui,sans-serif;color:#e6edf3;pointer-events:none';
  document.body.appendChild(el);
  return (state, msg) => {
    const dot = state === 'ready' ? '🔴' : state === 'auth-failed' ? '⛔' : '⚪';
    el.textContent = `${dot} REC ${host}: ${state}${msg ? ' ' + msg : ''}`;
  };
}
