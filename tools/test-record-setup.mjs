#!/usr/bin/env node
// Node-test voor src/record-setup.js (record.html-logica): URL-normalisatie, mixed-content-regel,
// token alleen in het #fragment, localStorage-gedrag, sluitcodes, en probe() tegen een nep-WebSocket.
import assert from 'node:assert/strict';
import * as R from '../src/record-setup.js';

const N = s => R.normalizeServerUrl(s);
assert.equal(N('robot.ts.net').url, 'wss://robot.ts.net/ws');
assert.equal(N('robot.ts.net:8443').url, 'wss://robot.ts.net:8443/ws');
assert.equal(N('https://robot.ts.net').url, 'wss://robot.ts.net/ws');
assert.equal(N('wss://robot.ts.net/rec').url, 'wss://robot.ts.net/rec');
assert.equal(N('ws://127.0.0.1:8765/ws').url, 'ws://127.0.0.1:8765/ws');
assert.match(N('ws://192.168.1.5:8765/ws').error, /mixed content/);
assert.match(N('').error, /vul/);
assert.match(N('wss://h/ws?token=x').error, /tokenveld/);
assert.match(N('wss://h/ws#token=x').error, /tokenveld/);

assert.deepEqual(R.normalizeCams(' front , top ', '320x240'), { cams: 'front,top', camsize: '320x240' });
assert.ok(R.normalizeCams('front;x', '640x480').error);
assert.ok(R.normalizeCams('front', '640').error);

const base = 'https://koenvanwijk.github.io/piper-sandwich-web/record.html';
const s = new URL(R.buildSessionUrl(base, { url: 'wss://h/ws', token: 'a&b=c#d', cams: 'front', camsize: '160x120' }));
assert.equal(s.pathname, '/piper-sandwich-web/index.html');
assert.equal(s.searchParams.get('rec'), 'wss://h/ws');
assert.equal(s.searchParams.get('cams'), 'front');
assert.ok(!s.search.includes('a%26b') && !s.search.includes('token'), 'token niet in de query');
assert.equal(new URLSearchParams(s.hash.slice(1)).get('token'), 'a&b=c#d', 'token heel in het fragment');

const su = new URL(R.buildSetupUrl(base, { url: 'wss://h/ws', token: 't', cams: 'front,top', camsize: '640x480' }));
assert.equal(su.search, ''); assert.deepEqual(R.parseSetupFragment(su.hash), { url: 'wss://h/ws', token: 't', cams: 'front,top', camsize: '640x480' });
assert.equal(R.parseSetupFragment('#foo=1'), null);

const mem = () => { const m = new Map(); return { getItem: k => m.get(k) ?? null, setItem: (k, v) => m.set(k, String(v)), m }; };
let st = mem(); R.saveSetup(st, { url: 'wss://h/ws', token: 't', cams: 'front', camsize: '1x1', remember: true });
assert.equal(R.loadSetup(st).token, 't');
st = mem(); R.saveSetup(st, { url: 'wss://h/ws', token: 't', cams: 'front', camsize: '1x1', remember: false });
assert.equal(R.loadSetup(st).token, undefined, 'niet onthouden -> token niet bewaard');
assert.deepEqual(R.loadSetup({ getItem: () => '{kapot' }), {});

assert.match(R.explainClose(4401), /token/);
assert.match(R.explainClose(4403), /--origin/);
assert.match(R.explainClose(1006), /certificaat/);
assert.doesNotMatch(R.explainClose(1006, '', { pageHttps: false }), /certificaat/);

// probe tegen nep-WebSocket
class FakeWS {
  constructor(url) { this.url = url; this.sent = []; FakeWS.last = this; setTimeout(() => this.onopen && this.onopen(), 0); }
  send(d) { const m = JSON.parse(d); this.sent.push(m);
    if (m.type === 'hello') setTimeout(() => m.token === 'ok' ? this.onmessage({ data: JSON.stringify({ type: 'welcome' }) }) : this.onclose({ code: 4401, reason: 'auth mislukt' }), 0);
    if (m.type === 'ping') setTimeout(() => this.onmessage({ data: JSON.stringify({ type: 'pong', id: m.id }) }), 5); }
  close() {}
}
let r = await R.probe('wss://h/ws', 'ok', { WS: FakeWS });
assert.equal(r.ok, true); assert.ok(r.rtt_ms >= 0);
assert.equal(FakeWS.last.sent[0].role, 'probe'); assert.equal(FakeWS.last.sent[0].token, 'ok');
r = await R.probe('wss://h/ws', 'fout', { WS: FakeWS });
assert.equal(r.ok, false); assert.equal(r.code, 4401); assert.match(r.msg, /token/);
class Silent { constructor() {} send() {} close() {} }
r = await R.probe('wss://h/ws', 'x', { WS: Silent, timeoutMs: 50 });
assert.equal(r.ok, false); assert.match(r.msg, /time-out/);
console.log('test-record-setup: OK');
