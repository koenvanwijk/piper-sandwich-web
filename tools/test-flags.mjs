#!/usr/bin/env node
// Unit-tests (Node, geen browser) voor de in-VR feature flags (src/flags.js) en het pure deel van het menu (src/flag-menu.js):
// standaardwaarden, gelijkwaardigheid met de bestaande query-parsers, persistentie (localStorage), query-voorrang, reset,
// robuustheid (kapotte opslag), trigger-klikdetectie en geen knopconflicten met opname/recenter/clutch.
//   node tools/test-flags.mjs
import assert from 'node:assert/strict';
import { FeatureFlags, FLAG_DEFS, STORAGE_KEY, safeStorage, formatFlag } from '../src/flags.js';
import { MenuPointer, MENU_BUTTON, menuRows, rowAt, panelHeight, PANEL_W, TITLE_H, ROW_H } from '../src/flag-menu.js';
import { orientationMode, tiltEnabledFromQuery, yawEnabledFromQuery } from '../src/teleop.js';
import { parseTargetOpts } from '../src/target-viz.js';
import { BTN, padActions, EdgeDetector, KEYMAP } from '../src/rec-controls.js';

let n = 0; const ok = (name, f) => { f(); n++; console.log('ok', name); };
const mem = (init = {}) => { const m = new Map(Object.entries(init)); return { m, getItem: k => m.has(k) ? m.get(k) : null, setItem: (k, v) => m.set(k, String(v)), removeItem: k => m.delete(k) }; };

ok('standaard (geen URL, niets bewaard) = bestaand gedrag: 6-DOF, ghost tijdens clutch, trilpuls aan, debug uit, tilt/yaw aan, headhome uit', () => {
  const f = new FeatureFlags({ search: '', storage: safeStorage(mem()) });
  assert.deepEqual(f.values, { mode: '6dof', target: 'clutch', haptic: true, debug: false, tilt: true, yaw: true, headhome: false });
  assert.ok(Object.values(f.source).every(s => s === 'standaard'));
});

ok('URL-startwaarden = precies de bestaande parsers (orientationMode, parseTargetOpts, tilt/yaw, ?debug=1, ?headhome=1)', () => {
  const qs = ['', '?orient=0', '?rot=0', '?roll=0', '?rot=1', '?mode=joints', '?mode=roll', '?mode=lock', '?mode=6dof', '?mode=full', '?mode=xyz', '?orient=0&mode=joints',
    '?target=0', '?target=off', '?target=always', '?target=1', '?tgthaptic=0', '?tgthaptic=1', '?debug=1', '?debug=0', '?debug=yes', '?tilt=0', '?yaw=0', '?headhome=1', '?headhome=0',
    '?mode=joints&tilt=0&yaw=0&debug=1&target=always&headhome=1&tgthaptic=0'];
  for (const q of qs) {
    const f = new FeatureFlags({ search: q, storage: safeStorage(mem()) }), u = new URLSearchParams(q), t = parseTargetOpts(q);
    assert.equal(f.get('mode'), orientationMode(q), 'mode ' + q);
    assert.equal(f.get('target'), t.mode, 'target ' + q); assert.equal(f.get('haptic'), t.haptic, 'haptic ' + q);
    assert.equal(f.get('tilt'), tiltEnabledFromQuery(q), 'tilt ' + q); assert.equal(f.get('yaw'), yawEnabledFromQuery(q), 'yaw ' + q);
    assert.equal(f.get('debug'), u.get('debug') === '1', 'debug ' + q); assert.equal(f.get('headhome'), u.get('headhome') === '1', 'headhome ' + q);
  }
});

ok('persistentie: in VR gekozen waarde wordt bewaard en bij herladen (zonder URL-flag) gebruikt', () => {
  const s = mem(), f = new FeatureFlags({ search: '', storage: safeStorage(s) });
  f.set('mode', 'joints'); f.set('debug', true); f.cycle('target');
  assert.deepEqual(JSON.parse(s.m.get(STORAGE_KEY)), { mode: 'joints', debug: true, target: 'always' });
  const g = new FeatureFlags({ search: '', storage: safeStorage(s) });
  assert.equal(g.get('mode'), 'joints'); assert.equal(g.get('debug'), true); assert.equal(g.get('target'), 'always'); assert.equal(g.get('haptic'), true);
  assert.equal(g.source.mode, 'opgeslagen'); assert.equal(g.source.haptic, 'standaard');
});

ok('query-voorrang: een flag in de URL wint van de bewaarde keuze; in VR wijzigen geldt meteen (bron VR); herladen met dezelfde URL = weer de URL', () => {
  const s = mem({ [STORAGE_KEY]: JSON.stringify({ mode: 'joints', target: 'off' }) });
  const f = new FeatureFlags({ search: '?orient=0', storage: safeStorage(s) });
  assert.equal(f.get('mode'), 'lock'); assert.equal(f.source.mode, 'url');
  assert.equal(f.get('target'), 'off'); assert.equal(f.source.target, 'opgeslagen', 'niet-opgegeven flag: bewaard');
  f.set('mode', '6dof'); assert.equal(f.get('mode'), '6dof'); assert.equal(f.source.mode, 'vr');
  assert.equal(new FeatureFlags({ search: '?orient=0', storage: safeStorage(s) }).get('mode'), 'lock');
  assert.equal(new FeatureFlags({ search: '', storage: safeStorage(s) }).get('mode'), '6dof', 'zonder URL: de laatste VR-keuze');
});

ok('reset: wist de opslag, terug naar URL/standaard, listeners krijgen alleen echte wijzigingen', () => {
  const s = mem(), f = new FeatureFlags({ search: '?debug=1', storage: safeStorage(s) }), seen = [];
  f.set('mode', 'lock'); f.set('debug', false); f.onChange((k, v) => seen.push(k + '=' + v));
  f.reset();
  assert.ok(!s.m.has(STORAGE_KEY)); assert.equal(f.get('mode'), '6dof'); assert.equal(f.get('debug'), true, 'URL blijft startwaarde');
  assert.deepEqual(seen.sort(), ['debug=true', 'mode=6dof']);
});

ok('cyclus + onChange: 6dof → joints → lock → 6dof; clutch → altijd → uit; dezelfde waarde zetten = geen event; ongeldige waarde geweigerd', () => {
  const f = new FeatureFlags({ storage: safeStorage(mem()) }); let ev = 0; f.onChange(() => ev++);
  assert.deepEqual([f.cycle('mode'), f.cycle('mode'), f.cycle('mode')], ['joints', 'lock', '6dof']);
  assert.deepEqual([f.cycle('target'), f.cycle('target'), f.cycle('target')], ['always', 'off', 'clutch']);
  assert.equal(f.cycle('debug'), true); assert.equal(ev, 7);
  assert.equal(f.set('debug', true), false); assert.equal(ev, 7);
  assert.equal(f.set('mode', 'banana'), false); assert.equal(f.get('mode'), '6dof');
  assert.equal(formatFlag('mode', 'lock'), 'alleen positie'); assert.equal(formatFlag('haptic', false), 'uit');
});

ok('robuust: kapotte JSON / onbekende of ongeldige waarden worden genegeerd; localStorage die gooit (privémodus) crasht niet', () => {
  assert.equal(new FeatureFlags({ storage: safeStorage(mem({ [STORAGE_KEY]: '{kapot' })) }).get('mode'), '6dof');
  const f = new FeatureFlags({ storage: safeStorage(mem({ [STORAGE_KEY]: JSON.stringify({ mode: 'xyz', debug: 'ja', foo: 1, tilt: false }) })) });
  assert.equal(f.get('mode'), '6dof'); assert.equal(f.get('debug'), false); assert.equal(f.get('tilt'), false);
  const boom = { getItem() { throw new Error('x'); }, setItem() { throw new Error('x'); }, removeItem() { throw new Error('x'); } };
  const g = new FeatureFlags({ storage: safeStorage(boom) }); g.set('mode', 'joints'); g.reset(); assert.equal(g.get('mode'), '6dof');
  assert.equal(new FeatureFlags({ storage: safeStorage(null) }).get('target'), 'clutch');
});

ok('menu-trigger: klik op rising edge (> 0,6), los pas < 0,3 (geen dubbele klik bij ruis), lockout 250 ms', () => {
  const p = new MenuPointer(); let t = 0; const c = [];
  for (const v of [0, 0.7, 0.5, 0.35, 0.7, 0.2, 0.1]) { const r = p.update('right', { trigger: v, target: 3 }, t += 100); if (r !== null) c.push([t, r]); }
  assert.deepEqual(c, [[200, 3]], 'ruis tussen 0,3 en 0,6 = geen tweede klik');
  assert.equal(p.update('right', { trigger: 0.9, target: 3 }, t += 100), 3, 'na loslaten weer klikbaar');
  assert.equal(p.update('right', { trigger: 0, target: 3 }, t += 50), null);
  assert.equal(p.update('right', { trigger: 0.9, target: 3 }, t += 50), null, 'binnen lockout');
  assert.equal(p.update('left', { trigger: 0.9, target: 'gear' }, t), 'gear', 'handen onafhankelijk');
});

ok('menu-trigger tijdens clutch telt nooit (trigger = gripper); grip loslaten met trigger nog ingedrukt = ook geen klik', () => {
  const p = new MenuPointer(); let t = 0;
  assert.equal(p.update('right', { trigger: 1, engaged: true, target: 'gear' }, t += 500), null);
  assert.equal(p.update('right', { trigger: 1, engaged: false, target: 'gear' }, t += 500), null, 'trigger nog vast na clutch');
  assert.equal(p.update('right', { trigger: 0, target: 'gear' }, t += 500), null);
  assert.equal(p.update('right', { trigger: 1, target: 'gear' }, t += 500), 'gear');
});

ok('geen knopconflicten: menu = alleen trigger (0); opname A/B/X/Y (4/5), thumbstick-druk (3), grip (1) blijven vrij; trigger levert geen opname-actie; toets M ≠ S/D/K/R', () => {
  assert.equal(MENU_BUTTON, 0);
  assert.ok(!Object.values(BTN).includes(MENU_BUTTON) && MENU_BUTTON !== 1);
  const pad = (h, idx) => ({ handedness: h, gamepad: { buttons: Array.from({ length: 7 }, (_, i) => ({ pressed: i === idx, value: i === idx ? 1 : 0 })) } });
  const e = new EdgeDetector(0);
  assert.deepEqual(padActions([pad('right', 0), pad('left', 0)], e, 1000), [], 'trigger = geen opname-actie');
  assert.equal(padActions([pad('right', BTN.A_X)], e, 2000).length, 1, 'A werkt nog');
  // menu leest alleen de trigger-waarde: A/B/thumbstick/grip ingedrukt geeft geen klik
  const p = new MenuPointer(); assert.equal(p.update('right', { trigger: 0, target: 2 }, 0), null);
  assert.ok(!('m' in KEYMAP));
});

ok('menu-indeling: alle flags + reset + sluiten; regel-hittest (titel = -1, naast het paneel = null)', () => {
  const f = new FeatureFlags({ storage: safeStorage(mem()) }), R = menuRows(f);
  assert.equal(R.length, FLAG_DEFS.length + 2); assert.deepEqual(R.slice(-2).map(r => r.key), ['_reset', '_close']);
  const H = panelHeight(R.length), y = i => H / 2 - TITLE_H - (i + 0.5) * ROW_H;
  for (let i = 0; i < R.length; i++) assert.equal(rowAt(0, y(i), R.length), i);
  assert.equal(rowAt(0, H / 2 - 0.01, R.length), -1); assert.equal(rowAt(PANEL_W, 0, R.length), null); assert.equal(rowAt(0, -H, R.length), null);
});

console.log(`${n} tests ok`);
