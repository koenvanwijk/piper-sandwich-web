// Fase 3: opname bedienen met de Quest-knoppen (of toetsen op desktop). Alleen actief met ?rec=.
// DOM- en three.js-vrij, dus in Node te testen (tools/test-rec-controls.mjs).
//
//   A (rechts)  start  /  stop + bewaar (success=false; niet als geslaagd gemarkeerd)
//   B (rechts)  episode weggooien + scene terug naar start
//   X (links)   episode als GESLAAGD markeren en stoppen + bewaren
//   Y (links)   scene terug naar start (een lopende episode wordt niet bewaard maar weggegooid)
//   thumbstick-druk (beide)  recenter (zat eerst op A/B/X/Y; zie readNav in app.js)
//   toetsen (desktop-test): S start/stop, D weggooien, K succes, R reset
//
// Knopindexen: WebXR "xr-standard" gamepad (Quest Touch): 0 trigger, 1 squeeze, 2 (leeg), 3 thumbstick, 4 A/X, 5 B/Y, 6 duimrust.
export const BTN = { THUMBSTICK: 3, A_X: 4, B_Y: 5 };
export const FPS = 30;

/** Rising-edge-detectie met een lockout tegen dubbele triggers (contactdender / dubbele klik). */
export class EdgeDetector {
  constructor(lockoutMs = 300) { this.lockoutMs = lockoutMs; this.down = new Map(); this.last = new Map(); }
  /** true precies op de overgang losgelaten -> ingedrukt (en niet binnen de lockout van dezelfde knop). */
  update(id, isDown, nowMs) {
    const was = this.down.get(id) || false;
    this.down.set(id, !!isDown);
    if (!isDown || was) return false;
    const t = this.last.get(id);
    if (t !== undefined && nowMs - t < this.lockoutMs) return false;
    this.last.set(id, nowMs);
    return true;
  }
}

/** Bron -> actie. inputs = [{ handedness:'left'|'right', buttons:[{pressed}|..] }] (XRInputSource.gamepad-achtig). */
export function padActions(inputs, edges, nowMs) {
  const out = [];
  for (const src of inputs) {
    const gp = src && src.gamepad ? src.gamepad : src, h = src && src.handedness;
    if (!gp || !gp.buttons || (h !== 'left' && h !== 'right')) continue;
    const down = i => !!(gp.buttons[i] && gp.buttons[i].pressed);
    if (edges.update(h + ':ax', down(BTN.A_X), nowMs)) out.push({ action: h === 'right' ? 'toggle' : 'success', hand: h });
    if (edges.update(h + ':by', down(BTN.B_Y), nowMs)) out.push({ action: h === 'right' ? 'discard' : 'reset', hand: h });
  }
  return out;
}

export const KEYMAP = { s: 'toggle', d: 'discard', k: 'success', r: 'reset' };
/** KeyboardEvent -> actie of null (negeert modifiers, key-repeat en tekstvelden). */
export function keyAction(e) {
  if (!e || e.repeat || e.ctrlKey || e.metaKey || e.altKey) return null;
  const t = e.target && e.target.tagName;
  if (t === 'INPUT' || t === 'TEXTAREA' || t === 'SELECT' || (e.target && e.target.isContentEditable)) return null;
  return KEYMAP[String(e.key || '').toLowerCase()] || null;
}

/** Episode-statemachine aan de browserkant; volgt de `event`-berichten van de server (bron van waarheid). */
export class RecController {
  /**
   * send(cmd, extra) -> bool     verstuur naar de server (RecorderClient.sendCmd)
   * resetScene()                  scene terug naar de startstaat (app.js)
   * seqNow() -> int               huidige tick-teller (voor verstreken sim-tijd)
   * haptic(hand, intensity, ms, pulses)   optioneel
   */
  constructor({ send, resetScene, seqNow = () => 0, haptic = () => {}, onChange = () => {}, now = () => performance.now() }) {
    Object.assign(this, { send, resetScene, seqNow, haptic, onChange, now });
    this.state = 'IDLE';            // IDLE | RECORDING | SAVING | SAVED | DISCARDED | INTERRUPTED
    this.connected = false;
    this.episode = 0;               // episodenummer (0-gebaseerd = episode_index in de dataset) van de lopende / volgende episode
    this.saved = 0;
    this.startSeq = null;           // tick-teller bij start (server-bevestigd via episode_started)
    this.frozenElapsed = 0;
    this.last = null;               // { frames, success, duration_s, episode_index } van de laatste bewaarde episode
    this.message = '';
    this.synced = false;
    this.counters = { start: 0, stop: 0, success: 0, discard: 0, reset: 0, blocked: 0 };
  }

  get recording() { return this.state === 'RECORDING'; }
  get elapsed() {
    if (this.state === 'RECORDING') return Math.max(0, (this.seqNow() - (this.startSeq ?? 0)) / FPS);
    return this.frozenElapsed;
  }
  snapshot() {
    return { state: this.state, connected: this.connected, episode: this.episode, elapsed: this.elapsed, message: this.message,
             last: this.last, saved: this.saved };
  }
  _set(state, message = '') { this.state = state; this.message = message; this.onChange(this.snapshot()); }
  _flash(message) { this.message = message; this.onChange(this.snapshot()); }
  _blocked(hand, message) { this.counters.blocked++; this._flash(message); this.haptic(hand, 0.25, 40, 1); return false; }

  /** Verbindingsstatus van RecorderClient ('ready' = verbonden en ingelogd). */
  onConn(s) {
    const was = this.connected; this.connected = (s === 'ready');
    if (this.connected && !was) { this.synced = false; this.send('status'); }
    if (!this.connected && was && (this.state === 'RECORDING' || this.state === 'SAVING')) {
      this.frozenElapsed = this.elapsed;
      this._set('INTERRUPTED', 'verbinding weg: episode onderbroken (server gooit hem weg)');
    } else this.onChange(this.snapshot());
  }

  /** `event`-bericht van de server. */
  onEvent(m) {
    switch (m.event) {
      case 'episode_started':
        this.startedOptimistic = false;
        if (typeof m.start_seq === 'number') this.startSeq = m.start_seq;
        this._set('RECORDING'); break;
      case 'saving':
        this.frozenElapsed = this.elapsed; this._set('SAVING'); break;
      case 'episode_saved':
        this.saved++; this.episode = (m.episode_index ?? this.episode) + 1;
        this.last = { episode_index: m.episode_index, frames: m.frames, success: m.success, duration_s: m.duration_s };
        this.frozenElapsed = m.duration_s ?? this.frozenElapsed;
        this._set('SAVED'); break;
      case 'episode_discarded':
        this.frozenElapsed = this.state === 'RECORDING' ? this.elapsed : this.frozenElapsed;
        this._set('DISCARDED', m.reason || ''); break;
      case 'scene_reset': break;                              // alleen ter info (de browser reset zelf)
      case 'status':
        if (typeof m.next_episode === 'number' && this.state !== 'RECORDING' && this.state !== 'SAVING') this.episode = m.next_episode;
        if (!this.synced) {                       // eerste status na (her)verbinden: neem de servertoestand over
          this.synced = true;
          if (m.state === 'RECORDING' && this.state === 'IDLE') { this.startSeq = this.seqNow() - (m.frames_in_episode || 0); this._set('RECORDING'); }
          else if (this.state === 'INTERRUPTED') this._set('IDLE');
          else this.onChange(this.snapshot());
        } else this.onChange(this.snapshot());
        break;
      case 'error': {
        const msg = m.message || 'fout';
        // optimistische toestand terugdraaien als de server een start/stop weigerde
        if (/^start genegeerd/.test(msg) && this.state === 'RECORDING' && this.startedOptimistic) this._set('IDLE', msg);
        else if (/^stop genegeerd/.test(msg) && this.state === 'SAVING') this._set('IDLE', msg);
        else this._flash('server: ' + msg);
        this.haptic(null, 0.5, 120, 2); break;
      }
    }
  }

  /** actie: 'toggle' | 'discard' | 'success' | 'reset'. hand: 'left'|'right'|null (voor haptiek). */
  act(action, hand = null) {
    switch (action) {
      case 'toggle':
        if (!this.connected) return this._blocked(hand, 'geen verbinding met de opname-server');
        if (this.state === 'SAVING') return this._blocked(hand, 'bezig met opslaan…');
        if (this.state === 'RECORDING') {
          if (!this.send('stop', { success: false })) return this._blocked(hand, 'stop niet verstuurd (offline)');
          this.counters.stop++; this.frozenElapsed = this.elapsed; this._set('SAVING'); this.haptic(hand, 0.5, 60, 2); return true;
        }
        if (!this.send('start')) return this._blocked(hand, 'start niet verstuurd (offline)');
        this.counters.start++; this.startedOptimistic = true; this.startSeq = this.seqNow(); this.frozenElapsed = 0; this._set('RECORDING'); this.haptic(hand, 0.6, 90, 1); return true;
      case 'success':
        if (!this.connected) return this._blocked(hand, 'geen verbinding met de opname-server');
        if (this.state !== 'RECORDING') return this._blocked(hand, 'geen lopende episode om als geslaagd te markeren');
        this.send('success', { value: true });
        this.send('stop', { success: true });                 // volgorde binnen één websocket is gegarandeerd
        this.counters.success++; this.frozenElapsed = this.elapsed; this._set('SAVING', 'geslaagd'); this.haptic(hand, 0.7, 60, 3); return true;
      case 'discard':
        if (this.state === 'RECORDING') {
          if (!this.send('discard')) return this._blocked(hand, 'discard niet verstuurd (offline)');
          this.frozenElapsed = this.elapsed; this._set('DISCARDED', 'weggegooid (B)');
        } else { this.send('reset'); this._flash('scene reset'); }
        this.counters.discard++; this.resetScene(); this.haptic(hand, 0.9, 200, 1); return true;
      case 'reset': {
        const wasRec = this.state === 'RECORDING';
        this.send('reset');                                   // server: lopende episode weggooien (nooit bewaren) + scene_reset-event
        if (wasRec) { this.frozenElapsed = this.elapsed; this._set('DISCARDED', 'scene reset: episode niet bewaard'); }
        else this._flash('scene reset');
        this.counters.reset++; this.resetScene(); this.haptic(hand, 0.4, 70, 1); return true;
      }
    }
    return false;
  }
}
