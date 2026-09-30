#!/usr/bin/env node
// Node-test (geen dependencies) voor src/rec-controls.js: knopdetectie met gesimuleerde gamepads + episode-statemachine.
//   node tools/test-rec-controls.mjs
import assert from 'node:assert/strict';
import { EdgeDetector, padActions, keyAction, RecController, BTN } from '../src/rec-controls.js';
import { hudLines } from '../src/rec-hud.js';

const pad = (h, down = []) => ({ handedness: h, gamepad: { buttons: Array.from({ length: 7 }, (_, i) => ({ pressed: down.includes(i), value: down.includes(i) ? 1 : 0 })) } });
let n = 0; const ok = (name, f) => { f(); n++; console.log('ok', name); };

ok('knoppen: A/B rechts, X/Y links, thumbstick geeft niets (is recenter)', () => {
  const e = new EdgeDetector(300);
  assert.deepEqual(padActions([pad('right', [BTN.A_X]), pad('left')], e, 0), [{ action: 'toggle', hand: 'right' }]);
  assert.deepEqual(padActions([pad('right', [BTN.A_X]), pad('left')], e, 50), []);             // ingedrukt houden = geen herhaling
  assert.deepEqual(padActions([pad('right', [BTN.B_Y]), pad('left', [BTN.A_X])], e, 100),
    [{ action: 'discard', hand: 'right' }, { action: 'success', hand: 'left' }]);
  assert.deepEqual(padActions([pad('right'), pad('left', [BTN.B_Y])], e, 200), [{ action: 'reset', hand: 'left' }]);
  assert.deepEqual(padActions([pad('right', [BTN.THUMBSTICK]), pad('left', [BTN.THUMBSTICK])], e, 300), []);
});
ok('lockout tegen dender (300 ms), daarna weer mogelijk', () => {
  const e = new EdgeDetector(300); const A = [pad('right', [4])], N = [pad('right')];
  assert.equal(padActions(A, e, 0).length, 1); padActions(N, e, 10);
  assert.equal(padActions(A, e, 100).length, 0); padActions(N, e, 110);
  assert.equal(padActions(A, e, 500).length, 1);
});
ok('toetsen S/D/K/R (hoofdletters ook), negeert modifiers/repeat/inputvelden', () => {
  assert.equal(keyAction({ key: 's' }), 'toggle'); assert.equal(keyAction({ key: 'D' }), 'discard');
  assert.equal(keyAction({ key: 'k' }), 'success'); assert.equal(keyAction({ key: 'r' }), 'reset');
  assert.equal(keyAction({ key: 'r', ctrlKey: true }), null); assert.equal(keyAction({ key: 's', repeat: true }), null);
  assert.equal(keyAction({ key: 's', target: { tagName: 'INPUT' } }), null); assert.equal(keyAction({ key: 'x' }), null);
});

function mk() {
  const sent = [], log = { reset: 0, haptic: [] }; let seq = 0, online = true;
  const c = new RecController({ send: (cmd, x) => { if (!online) return false; sent.push({ cmd, ...x }); return true; },
    resetScene: () => log.reset++, seqNow: () => seq, haptic: (...a) => log.haptic.push(a) });
  c.onConn('ready');
  return { c, sent, log, tick: k => { seq += k; }, offline: () => { online = false; c.onConn('closed'); } };
}
ok('start -> stop bewaart (episode_saved), teller en tijd', () => {
  const { c, sent, tick } = mk(); assert.equal(c.state, 'IDLE');
  c.onEvent({ event: 'status', state: 'IDLE', episodes_saved: 2, next_episode: 2 }); assert.equal(c.episode, 2);
  assert.ok(c.act('toggle', 'right')); assert.equal(c.state, 'RECORDING'); assert.deepEqual(sent.at(-1), { cmd: 'start' });
  c.onEvent({ event: 'episode_started', start_seq: 5 }); tick(35); assert.equal(c.elapsed, 1);   // (35-5)/30
  assert.ok(c.act('toggle', 'right')); assert.equal(c.state, 'SAVING'); assert.deepEqual(sent.at(-1), { cmd: 'stop', success: false });
  c.onEvent({ event: 'episode_saved', episode_index: 2, frames: 30, success: false, duration_s: 1 });
  assert.equal(c.state, 'SAVED'); assert.equal(c.episode, 3); assert.equal(c.elapsed, 1);
  assert.equal(hudLines(c.snapshot(), '').l1.split(/\s+/)[0], 'SAVED');
});
ok('X: success + stop; zonder episode geblokkeerd', () => {
  const { c, sent } = mk(); assert.equal(c.act('success', 'left'), false); assert.equal(c.counters.blocked, 1);
  c.act('toggle'); c.act('success', 'left');
  assert.deepEqual(sent.slice(-2), [{ cmd: 'success', value: true }, { cmd: 'stop', success: true }]); assert.equal(c.state, 'SAVING');
});
ok('B: weggooien + scene-reset; Y: reset (stuurt reset, server gooit lopende episode weg)', () => {
  const { c, sent, log } = mk(); c.act('toggle'); c.act('discard', 'right');
  assert.equal(c.state, 'DISCARDED'); assert.equal(sent.at(-1).cmd, 'discard'); assert.equal(log.reset, 1);
  c.act('toggle'); c.act('reset', 'left'); assert.equal(c.state, 'DISCARDED'); assert.equal(sent.at(-1).cmd, 'reset'); assert.equal(log.reset, 2);
  c.act('reset'); assert.equal(c.state, 'DISCARDED'); assert.equal(log.reset, 3);      // in IDLE-achtige toestand: alleen scene-reset
});
ok('offline: start geblokkeerd; verbinding weg tijdens opname -> INTERRUPTED; herstel via status', () => {
  const t = mk(); t.offline(); assert.equal(t.c.act('toggle', 'right'), false); assert.equal(t.c.state, 'IDLE');
  const u = mk(); u.c.act('toggle'); u.offline(); assert.equal(u.c.state, 'INTERRUPTED');
  u.c.onConn('ready'); u.c.onEvent({ event: 'status', state: 'IDLE', next_episode: 0 }); assert.equal(u.c.state, 'IDLE');
});
ok('server weigert start -> terug naar IDLE + foutmelding', () => {
  const { c } = mk(); c.act('toggle'); c.onEvent({ event: 'error', message: 'start genegeerd: toestand SAVING' });
  assert.equal(c.state, 'IDLE'); assert.match(c.message, /start genegeerd/);
});
console.log(`${n} tests ok`);
