// UI voor de demo "broodje smeren": stappenlijst met de huidige stap gemarkeerd, een
// stap-illustratie en grote (touch/auto-scherm-vriendelijke) knoppen Pauze/Verder,
// Vorige/Volgende niet nodig; Opnieuw. Alleen DOM, geen three.js / MuJoCo.
// Wordt alleen geladen/gebruikt bij ?demo=1 (zie src/app.js).
const CSS = `
#demo-panel { position: fixed; z-index: 20; right: 12px; top: 12px; bottom: 12px; width: min(340px, 42vw);
  display: flex; flex-direction: column; gap: 10px; padding: 12px; box-sizing: border-box;
  background: rgba(13,17,23,.88); border: 1px solid #30363d; border-radius: 14px;
  font: 16px/1.35 system-ui, sans-serif; color: #e6edf3; overflow: auto; }
#demo-panel h2 { margin: 0; font-size: 18px; }
#demo-steps { list-style: none; margin: 0; padding: 0; display: flex; flex-direction: column; gap: 6px; }
#demo-steps li { padding: 10px 12px; border-radius: 10px; background: #161b22; border: 2px solid transparent;
  font-size: 17px; opacity: .75; }
#demo-steps li.done { opacity: .55; }
#demo-steps li.done::before { content: '✓ '; color: #3fb950; }
#demo-steps li.active { opacity: 1; background: #1f3a5f; border-color: #58a6ff; font-weight: 600; }
#demo-img { width: 100%; height: auto; border-radius: 10px; background: #1b2130; min-height: 60px; }
#demo-buttons { display: flex; gap: 10px; margin-top: auto; }
#demo-buttons button { flex: 1; min-height: 56px; font: 600 18px system-ui, sans-serif; border: 0; border-radius: 12px;
  color: #fff; background: #238636; cursor: pointer; touch-action: manipulation; }
#demo-buttons button.secondary { background: #30363d; }
#demo-buttons button:active { filter: brightness(1.25); }
#demo-note { font-size: 13px; opacity: .7; }
@media (max-width: 720px), (orientation: portrait) {
  #demo-panel { top: auto; left: 12px; right: 12px; width: auto; max-height: 55vh; bottom: 12px; }
  #demo-img { max-height: 22vh; object-fit: contain; }
}`;

/**
 * @param {object} o
 * @param {string[]} o.names     stapnamen (zelfde volgorde als de choreografie)
 * @param {string[]} o.images    URL per stap (of null)
 * @param {Function} o.onPause   () => void   (toggle wordt door de UI bijgehouden)
 * @param {Function} o.onResume
 * @param {Function} o.onRestart
 * @returns {{setStep(i:number), setDone(), setPaused(b:boolean), destroy()}}
 */
export function createDemoPanel({ names, images, onPause, onResume, onRestart }) {
  const style = document.createElement('style'); style.textContent = CSS; document.head.appendChild(style);
  const el = document.createElement('div'); el.id = 'demo-panel';
  el.innerHTML = `<h2>🥪 Demo: broodje smeren</h2><ul id="demo-steps"></ul>
    <img id="demo-img" alt="" />
    <div id="demo-note"></div>
    <div id="demo-buttons"><button id="demo-pause">⏸ Pauze</button><button id="demo-restart" class="secondary">↺ Opnieuw</button></div>`;
  document.body.appendChild(el);
  const ul = el.querySelector('#demo-steps'), img = el.querySelector('#demo-img');
  const note = el.querySelector('#demo-note'), pauseBtn = el.querySelector('#demo-pause');
  const lis = names.map(n => { const li = document.createElement('li'); li.textContent = n; ul.appendChild(li); return li; });
  let paused = false;
  const setPaused = b => { paused = b; pauseBtn.textContent = b ? '▶ Verder' : '⏸ Pauze'; };
  pauseBtn.onclick = () => { if (paused) { onResume(); setPaused(false); } else { onPause(); setPaused(true); } };
  el.querySelector('#demo-restart').onclick = () => { onRestart(); setPaused(false); };
  return {
    setStep(i) {
      lis.forEach((li, k) => { li.className = k < i ? 'done' : k === i ? 'active' : ''; });
      if (images[i]) { img.src = images[i]; img.alt = names[i]; img.style.display = ''; } else img.style.display = 'none';
      note.textContent = `Stap ${i + 1} van ${names.length}`;
      lis[i] && lis[i].scrollIntoView({ block: 'nearest' });
    },
    setDone() { lis.forEach(li => { li.className = 'done'; }); note.textContent = 'Klaar — druk op Opnieuw om te herhalen'; },
    setPaused,
    destroy() { el.remove(); style.remove(); },
  };
}
