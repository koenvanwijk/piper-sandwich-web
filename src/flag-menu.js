// In-VR menu voor de feature flags (src/flags.js).
//
// OPENEN: de kleine "MENU"-knop die op de LINKER controller (bovenkant pols) zweeft: richt de straal van de RECHTER controller
//         erop en druk de trigger in. Op desktop: knop "⚙ Flags" rechtsboven of toets M.
// BEDIENEN: richt een controllerstraal (links of rechts) op een regel en druk de trigger: de waarde schuift door
//         (aan/uit, of 6-DOF → joints → positie). "Reset naar standaard" wist de bewaarde keuzes; "Sluiten" of nogmaals MENU sluit.
// GEEN KNOPCONFLICTEN: het menu gebruikt ALLEEN de trigger (knopindex 0), en alleen van een hand die op dat moment NIET clutcht
//         (grip los). Trigger = gripper geldt alleen tijdens clutch; A/B/X/Y (opname of recenter), thumbstick-druk (recenter met ?rec),
//         grip (clutch) en de thumbsticks (navigatie) blijven onaangeroerd. De Quest-menuknop zelf is niet beschikbaar voor WebXR-pagina's.
//
// Het pure deel (MenuPointer, menuRows, rowAt) is DOM/three-vrij en in Node getest (tools/test-flags.mjs).
import { FLAG_DEFS } from './flags.js';

export const MENU_BUTTON = 0;                 // xr-standard: 0 = trigger (zie rec-controls.js voor de rest van de indeling)
export const ACTION_ROWS = [{ key: '_reset', label: 'Reset naar standaard (wis bewaarde keuzes)' }, { key: '_close', label: 'Sluiten' }];
export const SRC_LABEL = { url: 'URL', vr: 'VR', opgeslagen: 'bewaard', standaard: 'std' };

/** Menuregels: alle flags + reset + sluiten. */
export function menuRows(flags) {
  return [...flags.snapshot(), ...ACTION_ROWS.map(r => ({ ...r, action: true }))];
}
export const ROW_H = 0.036, TITLE_H = 0.05, PANEL_W = 0.46;
export const panelHeight = nRows => TITLE_H + nRows * ROW_H + 0.012;
/** Regel onder het lokale punt (x, y) op het paneel (midden = 0,0; meters). -1 = titel/rand, null = naast het paneel. */
export function rowAt(x, y, nRows) {
  const H = panelHeight(nRows);
  if (Math.abs(x) > PANEL_W / 2 || Math.abs(y) > H / 2) return null;
  const fromTop = H / 2 - y - TITLE_H;
  if (fromTop < 0) return -1;
  const i = Math.floor(fromTop / ROW_H);
  return i < nRows ? i : -1;
}

/**
 * Trigger-klikken per hand (rising edge met hysterese: in > 0,6, los < 0,3) met lockout. Een hand die clutcht telt niet mee,
 * en een trigger die al ingedrukt was tijdens de clutch moet eerst los voordat hij weer kan klikken (geen klik bij het loslaten van de grip).
 * update(hand, { trigger, engaged, target }, nowMs) → target bij een klik, anders null.
 */
export class MenuPointer {
  constructor(lockoutMs = 250) { this.lockoutMs = lockoutMs; this.down = {}; this.blocked = {}; this.last = {}; }
  update(hand, { trigger = 0, engaged = false, target = null } = {}, nowMs = 0) {
    const was = !!this.down[hand];
    const isDown = was ? trigger > 0.3 : trigger > 0.6;
    this.down[hand] = isDown;
    if (engaged) { if (isDown) this.blocked[hand] = true; return null; }
    if (!isDown) { this.blocked[hand] = false; return null; }
    if (was || this.blocked[hand]) return null;
    if (nowMs - (this.last[hand] ?? -1e9) < this.lockoutMs) return null;
    this.last[hand] = nowMs;
    return target;
  }
}

/**
 * three.js-deel. scene = wereldscène (paneel en knop hangen in wereldcoördinaten, niet aan de verschuifbare scène-root).
 * onAction(key) wordt aangeroepen na een klik ('_reset', '_close' of een flag-key, nadat die al doorgeschoven is).
 */
export function createFlagMenu({ THREE, scene, flags, onAction = () => {} }) {
  const rows = () => menuRows(flags);
  const n = rows().length, H = panelHeight(n), CW = 1024, CH = Math.round(CW * H / PANEL_W);
  const cv = document.createElement('canvas'); cv.width = CW; cv.height = CH;
  const g = cv.getContext('2d'), tex = new THREE.CanvasTexture(cv);
  if (THREE.SRGBColorSpace) tex.colorSpace = THREE.SRGBColorSpace;
  const panel = new THREE.Mesh(new THREE.PlaneGeometry(PANEL_W, H),
    new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthTest: false, depthWrite: false, toneMapped: false, side: THREE.DoubleSide }));
  panel.name = 'flag-menu'; panel.renderOrder = 1000; panel.visible = false; panel.frustumCulled = false;
  scene.add(panel);

  // MENU-knop op de linker controller
  const gc = document.createElement('canvas'); gc.width = gc.height = 128;
  const gg = gc.getContext('2d'), gtex = new THREE.CanvasTexture(gc);
  if (THREE.SRGBColorSpace) gtex.colorSpace = THREE.SRGBColorSpace;
  const drawGear = hot => { gg.clearRect(0, 0, 128, 128); gg.fillStyle = hot ? '#3d8bfd' : 'rgba(20,24,32,.85)'; gg.beginPath(); gg.arc(64, 64, 60, 0, 7); gg.fill();
    gg.lineWidth = 6; gg.strokeStyle = '#9ecbff'; gg.stroke(); gg.fillStyle = '#fff'; gg.font = 'bold 30px sans-serif'; gg.textAlign = 'center'; gg.textBaseline = 'middle'; gg.fillText('MENU', 64, 66); gtex.needsUpdate = true; };
  drawGear(false);
  const gear = new THREE.Mesh(new THREE.CircleGeometry(0.02, 24),
    new THREE.MeshBasicMaterial({ map: gtex, transparent: true, depthWrite: false, toneMapped: false, side: THREE.DoubleSide }));
  gear.name = 'flag-menu-knop'; gear.renderOrder = 1000; gear.visible = false;
  scene.add(gear);

  // stralen (alleen zichtbaar als het menu open is, of als de rechter straal in de buurt van de knop wijst)
  const rays = {};
  for (const h of ['left', 'right']) {
    const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -1)]);
    const l = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0x9ecbff, transparent: true, opacity: 0.8, depthTest: false }));
    l.visible = false; l.renderOrder = 1001; l.frustumCulled = false; scene.add(l); rays[h] = l;
  }

  const pointer = new MenuPointer();
  let hover = -2, gearHot = false;
  const inv = new THREE.Matrix4(), o = new THREE.Vector3(), d = new THREE.Vector3(), tmp = new THREE.Vector3();

  function draw() {
    const R = rows(), sx = CW / PANEL_W;
    g.clearRect(0, 0, CW, CH);
    g.fillStyle = 'rgba(12,16,24,.92)'; g.fillRect(0, 0, CW, CH);
    g.strokeStyle = '#3d8bfd'; g.lineWidth = 4; g.strokeRect(2, 2, CW - 4, CH - 4);
    g.fillStyle = '#9ecbff'; g.font = 'bold 34px sans-serif'; g.textBaseline = 'middle';
    g.fillText('Instellingen (feature flags)', 20, TITLE_H * sx / 2);
    g.font = '20px sans-serif'; g.fillStyle = '#8b98a5'; g.textAlign = 'right';
    g.fillText('richt + trigger  ·  bron: URL / VR / bewaard / std', CW - 20, TITLE_H * sx / 2); g.textAlign = 'left';
    R.forEach((r, i) => {
      const y0 = (TITLE_H + i * ROW_H) * sx, h = ROW_H * sx;
      if (i === hover) { g.fillStyle = 'rgba(61,139,253,.35)'; g.fillRect(8, y0 + 2, CW - 16, h - 4); }
      g.font = '28px sans-serif'; g.fillStyle = r.action ? '#ffd27a' : '#e6edf3'; g.fillText(r.label, 22, y0 + h / 2);
      if (!r.action) {
        g.textAlign = 'right'; g.font = 'bold 28px sans-serif';
        g.fillStyle = r.value === false || r.value === 'off' ? '#ff8080' : '#7ee787'; g.fillText(r.text, CW - 130, y0 + h / 2);
        g.font = '20px sans-serif'; g.fillStyle = r.source === 'url' ? '#ffd27a' : '#8b98a5'; g.fillText(SRC_LABEL[r.source] || r.source, CW - 22, y0 + h / 2);
        g.textAlign = 'left';
      }
    });
    tex.needsUpdate = true;
  }

  function hitPanel(origin, dir) {
    if (!panel.visible) return null;
    panel.updateMatrixWorld(); inv.copy(panel.matrixWorld).invert();
    o.copy(origin).applyMatrix4(inv); d.copy(origin).add(dir).applyMatrix4(inv).sub(o);
    if (Math.abs(d.z) < 1e-6) return null;
    const t = -o.z / d.z; if (t <= 0) return null;
    const row = rowAt(o.x + d.x * t, o.y + d.y * t, rows().length);
    return row === null ? null : { row, dist: t * d.length() / dir.length() };
  }
  function hitGear(origin, dir) {
    if (!gear.visible) return null;
    tmp.copy(gear.position).sub(origin); const t = tmp.dot(dir) / dir.lengthSq(); if (t <= 0) return null;
    const miss = tmp.addScaledVector(dir, -t).length();
    return { near: miss < 0.15, hit: miss < 0.035, dist: t * dir.length() };
  }

  function click(row) {
    const R = rows(), r = R[row]; if (!r) return;
    if (r.key === '_close') api.close();
    else if (r.key === '_reset') flags.reset();
    else flags.cycle(r.key);
    onAction(r.key); draw();
  }

  const api = {
    panel, gear, rays, pointer,
    get isOpen() { return panel.visible; },
    /** Open ~0,6 m vóór het hoofd in de kijkrichting, naar het hoofd gedraaid (wereldvast, dus rustig om op te richten). */
    open(headPos, headQuat) {
      const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(headQuat).normalize();
      panel.position.copy(headPos).addScaledVector(fwd, 0.6);
      panel.up.set(0, 1, 0); panel.lookAt(headPos);
      hover = -2; draw(); panel.visible = true;
    },
    close() { panel.visible = false; for (const h in rays) rays[h].visible = false; },
    toggle(headPos, headQuat) { if (panel.visible) api.close(); else api.open(headPos, headQuat); },
    redraw: draw,
    /**
     * Per frame in VR. hands = { left|right: { origin: Vector3, dir: Vector3 (genormaliseerd), trigger, engaged, gripPos?, gripQuat? } }.
     * head = { pos, quat }. Geeft true als een trigger deze frame door het menu is gebruikt.
     */
    update(hands, head, nowMs = performance.now()) {
      const L = hands.left;
      gear.visible = !!(L && L.gripPos);
      if (gear.visible) {   // ~4 cm boven de grip, naar het hoofd gedraaid
        gear.position.set(0, 0.035, 0.03).applyQuaternion(L.gripQuat).add(L.gripPos);
        gear.lookAt(head.pos);
      }
      let used = false, newHover = -2, hot = false;
      for (const h of ['left', 'right']) {
        const c = hands[h], ray = rays[h];
        if (!c || !c.origin) { ray.visible = false; pointer.update(h, {}, nowMs); continue; }
        const gh = h === 'right' && !c.engaged ? hitGear(c.origin, c.dir) : null;
        const ph = !c.engaged ? hitPanel(c.origin, c.dir) : null;
        let target = null, len = 0.6;
        if (gh && gh.hit) { target = 'gear'; len = gh.dist; hot = true; }
        else if (ph && ph.row >= 0) { target = ph.row; len = ph.dist; newHover = ph.row; }
        else if (ph) len = ph.dist;
        ray.visible = !c.engaged && (panel.visible || !!(gh && gh.near));
        if (ray.visible) { ray.position.copy(c.origin); ray.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, -1), c.dir); ray.scale.set(1, 1, len); }
        const clicked = pointer.update(h, { trigger: c.trigger, engaged: c.engaged, target }, nowMs);
        if (clicked === 'gear') { api.toggle(head.pos, head.quat); used = true; }
        else if (typeof clicked === 'number') { click(clicked); used = true; }
      }
      if (hot !== gearHot) { gearHot = hot; drawGear(hot); }
      if (newHover !== hover && panel.visible) { hover = newHover; draw(); }
      return used;
    },
    /** Desktop: muisklik (Raycaster vanuit de camera). Geeft true als het paneel geraakt werd. */
    clickRay(origin, dir) { const ph = hitPanel(origin, dir); if (!ph) return false; if (ph.row >= 0) click(ph.row); return true; },
  };
  return api;
}
export { FLAG_DEFS };
