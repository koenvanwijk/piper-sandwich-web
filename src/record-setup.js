// Logica voor record.html (opname-startpagina). DOM-vrij, in Node te testen (tools/test-record-setup.mjs).
// Het token gaat ALLEEN via het #fragment naar index.html (fragmenten worden niet naar servers gestuurd en
// komen niet in server-/proxylogs); het wordt nooit in een querystring of in git gezet.
import { PROTO_VERSION } from './rec-state.js';

export const STORE_KEY = 'piper-rec-setup';      // localStorage: {url, token?, cams, camsize, remember}
const LOOPBACK = new Set(['localhost', '127.0.0.1', '[::1]']);

/** Normaliseer wat Koen intypt: "host", "host:8443", "wss://host", "wss://host/ws" -> "wss://host[:p]/ws". */
export function normalizeServerUrl(input) {
  let s = String(input || '').trim();
  if (!s) return { error: 'vul de server in (bv. wss://robot-pc.tailnet.ts.net/ws)' };
  if (/^https?:\/\//i.test(s)) s = s.replace(/^http/i, 'ws');     // https://x -> wss://x
  if (!/^[a-z]+:\/\//i.test(s)) s = 'wss://' + s;
  let u;
  try { u = new URL(s); } catch { return { error: 'ongeldige server-URL' }; }
  if (u.protocol !== 'wss:' && u.protocol !== 'ws:') return { error: 'gebruik wss://' };
  if (u.protocol === 'ws:' && !LOOPBACK.has(u.hostname))
    return { error: 'ws:// werkt alleen naar localhost; vanaf https (GitHub Pages) is wss:// met geldig certificaat nodig (mixed content)' };
  if (u.search || u.hash) return { error: 'geen ?query of #fragment in de server-URL (token hoort in het tokenveld)' };
  if (u.pathname === '/' || u.pathname === '') u.pathname = '/ws';
  return { url: u.href };
}

export function normalizeCams(cams, camsize) {
  const list = String(cams ?? 'front,top').split(',').map(s => s.trim()).filter(Boolean);
  if (!list.length || list.some(c => !/^[a-z0-9_]+$/i.test(c))) return { error: 'camera-namen: komma-gescheiden, a-z/0-9/_' };
  const m = /^(\d{2,4})x(\d{2,4})$/.exec(String(camsize || '640x480').trim());
  if (!m) return { error: 'camsize als BxH, bv. 640x480' };
  return { cams: list.join(','), camsize: `${m[1]}x${m[2]}` };
}

/** URL van de opnamesessie: ./index.html?rec=..&cams=..&camsize=..#token=.. (token alleen in het fragment). */
export function buildSessionUrl(base, { url, token, cams = 'front,top', camsize = '640x480' }) {
  const u = new URL('index.html', base);
  u.search = new URLSearchParams({ rec: url, cams, camsize }).toString();
  u.hash = token ? new URLSearchParams({ token }).toString() : '';
  return u.href;
}

/** Instel-link voor record.html: alles in het fragment, bv. door de server geprint. */
export function buildSetupUrl(base, { url, token, cams, camsize }) {
  const u = new URL('record.html', base);
  const f = new URLSearchParams({ rec: url });
  if (token) f.set('token', token);
  if (cams) f.set('cams', cams);
  if (camsize) f.set('camsize', camsize);
  u.hash = f.toString();
  return u.href;
}

/** record.html#rec=..&token=..&cams=..&camsize=.. -> velden (of null). */
export function parseSetupFragment(hash) {
  const f = new URLSearchParams(String(hash || '').replace(/^#/, ''));
  if (!f.get('rec') && !f.get('token')) return null;
  const out = {};
  for (const k of ['rec', 'token', 'cams', 'camsize']) if (f.get(k)) out[k === 'rec' ? 'url' : k] = f.get(k);
  return out;
}

export function loadSetup(storage) {
  try { return JSON.parse(storage.getItem(STORE_KEY) || 'null') || {}; } catch { return {}; }
}
export function saveSetup(storage, { url, token, cams, camsize, remember }) {
  const v = { url, cams, camsize, remember: !!remember };
  if (remember && token) v.token = token;           // token alleen op dit apparaat, alleen als Koen dat wil
  try { storage.setItem(STORE_KEY, JSON.stringify(v)); } catch { /* privé-modus */ }
  return v;
}

/** Sluitcode/fout -> begrijpelijke tekst. */
export function explainClose(code, reason = '', { pageHttps = true } = {}) {
  switch (code) {
    case 4401: return 'token klopt niet (server weigert: auth mislukt)';
    case 4403: return 'Origin geweigerd: start de server met --origin https://koenvanwijk.github.io';
    case 4400: return 'protocolfout: ' + (reason || 'versie/hello komt niet overeen');
    case 4408: return 'server wachtte te lang op hello';
    case 4409: return 'vervangen door een nieuwere verbinding';
    case 1000: case 1001: return 'verbinding gesloten' + (reason ? ': ' + reason : '');
    default:
      return 'geen verbinding (code ' + code + '). Mogelijke oorzaken: server draait niet / verkeerde host of poort; ' +
        (pageHttps ? 'certificaat niet geldig voor wss:// (open https://<host>/ in de browser om te checken); ' : '') +
        'de browser blokkeert toegang tot het lokale netwerk (Chrome/Quest "Local Network Access": sta het toe als erom gevraagd wordt); ' +
        'of de server weigert deze Origin (HTTP 403 bij de handshake; start met --origin ' + (globalThis.location ? location.origin : 'https://koenvanwijk.github.io') + ').';
  }
}

/**
 * Verbindingstest: hello (rol "probe", zonder sim-controle) + ping -> pong. Bewaart niets op de server.
 * resolve({ok, rtt_ms, msg, code})
 */
export function probe(url, token, { WS = globalThis.WebSocket, timeoutMs = 6000, pageHttps = true, now = () => performance.now() } = {}) {
  return new Promise(resolve => {
    let ws, done = false, t0 = 0;
    const finish = r => { if (done) return; done = true; clearTimeout(tm); try { ws && ws.close(1000, 'probe klaar'); } catch {} resolve(r); };
    const tm = setTimeout(() => finish({ ok: false, code: 0, msg: 'time-out: geen antwoord binnen ' + timeoutMs / 1000 + ' s (' + explainClose(1006, '', { pageHttps }) + ')' }), timeoutMs);
    try { ws = new WS(url); } catch (e) { return finish({ ok: false, code: 0, msg: 'kan niet verbinden: ' + (e.message || e) }); }
    ws.onopen = () => ws.send(JSON.stringify({ type: 'hello', proto: PROTO_VERSION, role: 'probe', token: token || '' }));
    ws.onmessage = ev => {
      let m; try { m = JSON.parse(ev.data); } catch { return; }
      if (m.type === 'welcome') { t0 = now(); ws.send(JSON.stringify({ type: 'ping', id: 1, t_client_ms: Date.now() })); }
      else if (m.type === 'pong') finish({ ok: true, code: 0, rtt_ms: Math.round(now() - t0), msg: 'verbonden, token OK' });
    };
    ws.onclose = ev => finish({ ok: false, code: ev.code, msg: explainClose(ev.code, ev.reason, { pageHttps }) });
  });
}
