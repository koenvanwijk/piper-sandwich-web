#!/usr/bin/env python3
"""Genereert de simpele stap-illustraties (bovenaanzicht van de scene) in demo/step-*.svg.
Gebruik: python3 tools/make-step-images.py   (geen dependencies)
Coördinaten: MuJoCo x = vooruit (omhoog in beeld), y = links (links in beeld)."""
import os
W, H = 320, 220
px = lambda y: 160 - y * 380
py = lambda x: 190 - (x + 0.05) * 480
POS = dict(bread0=(0.16, 0.06), bread1=(0.16, -0.06), knife=(0.13, -0.22), plate=(0.23, -0.32), jar=(0.23, 0.33))

def rect(c, w, h, fill, extra=''):
    return f'<rect x="{px(c[1])-w/2:.0f}" y="{py(c[0])-h/2:.0f}" width="{w}" height="{h}" rx="4" fill="{fill}" {extra}/>'
def circ(c, r, fill, extra=''):
    return f'<circle cx="{px(c[1]):.0f}" cy="{py(c[0]):.0f}" r="{r}" fill="{fill}" {extra}/>'
def arrow(a, b, col, dash=False):
    d = ' stroke-dasharray="6 4"' if dash else ''
    return (f'<line x1="{px(a[1]):.0f}" y1="{py(a[0]):.0f}" x2="{px(b[1]):.0f}" y2="{py(b[0]):.0f}" '
            f'stroke="{col}" stroke-width="4"{d} marker-end="url(#ah{col[1:]})"/>')
def base(items, label):
    cols = ['#ff9f43', '#4dabf7']
    defs = ''.join(f'<marker id="ah{c[1:]}" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto">'
                   f'<path d="M0,0 L8,4 L0,8 z" fill="{c}"/></marker>' for c in cols)
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}">'
            f'<defs>{defs}</defs><rect width="{W}" height="{H}" rx="10" fill="#1b2130"/>'
            f'<rect x="20" y="10" width="280" height="190" rx="8" fill="#d4b48c"/>'
            f'<rect x="{px(0.13)}" y="{py(0.30)}" width="{0.26*380:.0f}" height="{0.25*480:.0f}" rx="4" fill="#8c5c33" opacity=".9"/>'
            + items +
            f'<text x="12" y="{H-8}" font-family="sans-serif" font-size="13" fill="#e6edf3">{label}</text></svg>')
def scene(hl=()):
    hi = lambda n: 'stroke="#fff" stroke-width="3"' if n in hl else ''
    s = rect(POS['bread0'], 34, 34, '#e6c88a', hi('bread0')) + rect(POS['bread1'], 34, 34, '#e6c88a', hi('bread1'))
    s += circ(POS['plate'], 34, '#f0f0f5', hi('plate')) + circ(POS['jar'], 20, '#5c8de6', hi('jar'))
    k = POS['knife']
    s += (f'<rect x="{px(k[1])-4:.0f}" y="{py(k[0]+0.14)-0:.0f}" width="8" height="{0.19*480:.0f}" rx="2" fill="#222" {hi("knife")}/>')
    return s
def butter(n, box):
    import random; random.seed(3); c = POS['bread0']
    return ''.join(circ((c[0]+random.uniform(-.03, .03) if n else c[0]+random.uniform(-.008, .008),
                         c[1]+random.uniform(-.03, .03) if n else c[1]+random.uniform(-.008, .008)), 3.5, '#ffd93d') for _ in range(14))
R, L = '#ff9f43', '#4dabf7'
steps = {
 'step-1-verzamelen': base(scene(('knife', 'jar')) + arrow((0.02, -0.12), (0.10, -0.22), R) + arrow((0.02, 0.12), (0.20, 0.30), L),
                           '1. Mes pakken (rechts) · links naar de pot'),
 'step-2-smeren': base(scene(('bread0',)) + butter(0, 0) +
                       ''.join(f'<line x1="{px(0.075+dy)-12:.0f}" y1="{py(0.16)+o:.0f}" x2="{px(0.075+dy)+12:.0f}" y2="{py(0.16)+o:.0f}" stroke="{R}" stroke-width="3"/>' for dy, o in ((0, -10), (0, 0), (0, 10))),
                       '2. Boter smeren met het mes'),
 'step-3-beleg': base(scene(('jar', 'bread0')) + butter(1, 0) + arrow((0.23, 0.30), (0.17, 0.09), L), '3. Beleg van de pot naar het brood'),
 'step-4a-mes-terug': base(scene(('plate',)) + arrow((0.12, 0.0), (0.22, -0.30), R), '4a. Mes terug op het bord'),
 'step-4b-sluiten': base(scene(('bread1', 'bread0')) + arrow((0.16, -0.14), (0.16, -0.005), R), '4b. Broodje dichtschuiven'),
}
os.makedirs(os.path.join(os.path.dirname(__file__), '..', 'demo'), exist_ok=True)
for name, svg in steps.items():
    open(os.path.join(os.path.dirname(__file__), '..', 'demo', name + '.svg'), 'w').write(svg)
print('geschreven:', ', '.join(steps))
