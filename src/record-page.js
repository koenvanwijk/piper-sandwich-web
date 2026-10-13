// UI van record.html. Logica in record-setup.js.
import { normalizeServerUrl, normalizeCams, buildSessionUrl, parseSetupFragment, loadSetup, saveSetup, probe } from './record-setup.js';

const $ = id => document.getElementById(id);
const st = (cls, msg) => { const s = $('status'); s.className = cls; s.textContent = msg; };
const pageHttps = location.protocol === 'https:';

// 1) instel-link van de server (record.html#rec=..&token=..): invullen, bewaren en het fragment direct uit de adresbalk halen
const frag = parseSetupFragment(location.hash);
if (frag) history.replaceState(null, '', location.pathname + location.search);
const saved = loadSetup(localStorage);
const init = { ...saved, ...(frag || {}) };
if (init.url) $('url').value = init.url;
if (init.token) $('token').value = init.token;
if (init.cams) $('cams').value = init.cams;
if (init.camsize) $('camsize').value = init.camsize;
if (saved.remember === false && !frag) $('remember').checked = false;

function read() {
  const u = normalizeServerUrl($('url').value);
  if (u.error) return u;
  const c = normalizeCams($('cams').value, $('camsize').value);
  if (c.error) return c;
  const token = $('token').value.trim();
  if (!token) return { error: 'vul het token in' };
  const v = { url: u.url, token, cams: c.cams, camsize: c.camsize, remember: $('remember').checked };
  $('url').value = v.url;
  saveSetup(localStorage, v);
  return v;
}
if (frag) { const v = read(); if (!v.error) st('', 'Instellingen uit de link overgenomen en bewaard. Test de verbinding of start.'); }

async function test() {
  const v = read(); if (v.error) { st('err', v.error); return null; }
  st('busy', 'Verbinden met ' + v.url + ' …');
  const r = await probe(v.url, v.token, { pageHttps });
  st(r.ok ? 'ok' : 'err', r.ok ? `✅ ${r.msg} — ping ${r.rtt_ms} ms` : '❌ ' + r.msg);
  return r.ok ? v : null;
}
$('test').onclick = () => test();
$('start').onclick = async () => {
  const v = await test(); if (!v) return;
  location.href = buildSessionUrl(location.href, v);
};
window.recordPage = { read, test };
