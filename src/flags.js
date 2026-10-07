// In-VR feature flags (src/flag-menu.js = het menu). DOM- en three.js-vrij, dus in Node te testen (tools/test-flags.mjs).
//
// Startwaarde per flag:  URL-queryparameter (als die in de URL staat)  >  in VR gekozen waarde (localStorage)  >  standaard.
// Een wijziging in het menu geldt meteen en wordt in localStorage bewaard (sleutel STORAGE_KEY); "Reset" wist die en zet alles
// terug naar query/standaard. Staat een flag in de URL, dan wint de URL bij het (her)laden — de in-VR keuze geldt dan alleen
// voor de lopende sessie (het menu toont "URL" als bron). Zo blijven bestaande links/bookmarks precies hetzelfde werken.
import { orientationMode } from './teleop.js';

export const STORAGE_KEY = 'piper-sandwich-web.flags.v1';

const has = (q, ...k) => k.some(x => q.has(x));
const bool01 = (q, k, def) => q.has(k) ? q.get(k) !== '0' && q.get(k) !== 'false' && q.get(k) !== 'off' : undefined;

/** Definities: key, label (menu), values (cyclus-volgorde), names (weergave), def (standaard), q (query → waarde | undefined = niet opgegeven). */
export const FLAG_DEFS = [
  { key: 'mode', label: 'Teleop-modus', values: ['6dof', 'joints', 'lock'], def: '6dof',
    names: { '6dof': '6-DOF pose', joints: 'joints j4/j5/j6', lock: 'alleen positie' },
    q: q => has(q, 'orient', 'rot', 'roll', 'mode') ? orientationMode('?' + q.toString()) : undefined },
  { key: 'target', label: 'Doel-ghost', values: ['clutch', 'always', 'off'], def: 'clutch',
    names: { clutch: 'tijdens clutch', always: 'altijd', off: 'uit' },
    q: q => { if (!q.has('target')) return undefined; const t = (q.get('target') || '').toLowerCase();
              return t === '0' || t === 'off' || t === 'false' ? 'off' : t === 'always' ? 'always' : 'clutch'; } },
  { key: 'haptic', label: 'Trilpuls bij rood doel', values: [true, false], def: true, q: q => bool01(q, 'tgthaptic') },
  { key: 'debug', label: 'Debug-overlay', values: [false, true], def: false, q: q => q.has('debug') ? q.get('debug') === '1' : undefined },
  { key: 'tilt', label: 'Tilt→j5 (joints-modus)', values: [true, false], def: true, q: q => bool01(q, 'tilt') },
  { key: 'yaw', label: 'Yaw→j4 (joints-modus)', values: [true, false], def: true, q: q => bool01(q, 'yaw') },
  { key: 'headhome', label: 'Thuispositie volgt hoofd', values: [false, true], def: false, q: q => q.has('headhome') ? q.get('headhome') === '1' : undefined },
];
const DEF = Object.fromEntries(FLAG_DEFS.map(d => [d.key, d]));
const valid = (k, v) => !!DEF[k] && DEF[k].values.some(x => x === v);

export function formatFlag(key, v) {
  const d = DEF[key]; if (!d) return String(v);
  if (d.names) return d.names[v] || String(v);
  return v ? 'aan' : 'uit';
}

/** localStorage die nooit gooit (privémodus / file:// / Node). */
export function safeStorage(s = (typeof localStorage !== 'undefined' ? localStorage : null)) {
  return {
    get(k) { try { return s ? s.getItem(k) : null; } catch { return null; } },
    set(k, v) { try { if (s) s.setItem(k, v); return true; } catch { return false; } },
    del(k) { try { if (s) s.removeItem(k); } catch { /* */ } },
  };
}

export class FeatureFlags {
  /** search = location.search; storage = { get, set, del } (safeStorage()). */
  constructor({ search = '', storage = safeStorage(null), key = STORAGE_KEY } = {}) {
    this.storage = storage; this.key = key; this.listeners = [];
    const q = new URLSearchParams(search || '');
    this.query = {};
    for (const d of FLAG_DEFS) { const v = d.q(q); if (v !== undefined && valid(d.key, v)) this.query[d.key] = v; }
    this.stored = this._load();
    this.values = {}; this.source = {};
    for (const d of FLAG_DEFS) this._init(d.key);
  }
  _load() {
    let o = {}; try { o = JSON.parse(this.storage.get(this.key) || '{}') || {}; } catch { o = {}; }
    const out = {}; for (const [k, v] of Object.entries(o)) if (valid(k, v)) out[k] = v;   // onbekend/ongeldig wordt genegeerd
    return out;
  }
  _init(k) {
    if (k in this.query) { this.values[k] = this.query[k]; this.source[k] = 'url'; }
    else if (k in this.stored) { this.values[k] = this.stored[k]; this.source[k] = 'opgeslagen'; }
    else { this.values[k] = DEF[k].def; this.source[k] = 'standaard'; }
  }
  get(k) { return this.values[k]; }
  /** Zet een flag (vanuit het menu): geldt meteen, wordt bewaard. Geeft true als de waarde veranderde. */
  set(k, v) {
    if (!valid(k, v)) return false;
    const prev = this.values[k]; this.values[k] = v; this.source[k] = 'vr';
    this.stored[k] = v; this.storage.set(this.key, JSON.stringify(this.stored));
    if (prev !== v) for (const f of this.listeners) f(k, v, prev);
    return prev !== v;
  }
  /** Volgende waarde in de cyclus (menu-klik). */
  cycle(k) { const vs = DEF[k].values, i = vs.indexOf(this.values[k]); this.set(k, vs[(i + 1) % vs.length]); return this.values[k]; }
  /** Wis de opgeslagen keuzes en zet alles terug naar URL/standaard. */
  reset() {
    this.storage.del(this.key); this.stored = {};
    const prev = { ...this.values };
    for (const d of FLAG_DEFS) this._init(d.key);
    for (const d of FLAG_DEFS) if (prev[d.key] !== this.values[d.key]) for (const f of this.listeners) f(d.key, this.values[d.key], prev[d.key]);
  }
  onChange(f) { this.listeners.push(f); return () => { this.listeners = this.listeners.filter(x => x !== f); }; }
  snapshot() { return FLAG_DEFS.map(d => ({ key: d.key, label: d.label, value: this.values[d.key], text: formatFlag(d.key, this.values[d.key]), source: this.source[d.key] })); }
}
