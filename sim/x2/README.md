# AgiBot X2 in MuJoCo: boter smeren en kaas pakken

Simulatie met **MuJoCo 3.15, geen Isaac Sim**. Het URDF-model van de AgiBot X2 (Mulan PSL v2, zie `LICENSE.MulanPSL2` en `NOTICE`) wordt opgehaald met `fetch_x2_assets.sh`. De meshes worden niet meegecommit.

## Structuur
| Pad | Inhoud |
|---|---|
| `fetch_x2_assets.sh` | haalt URDF en meshes op naar `assets/X2/` |
| `butter_scene.py` | X2, tafel en boter (flexcomp dim=3) met 9 drukbewegingen |
| `knife.py` | tafelmes (heft 10×2×1,5 cm, lemmet 10×2 cm×1,5 mm, 40 g) in de linker OmniPicker |
| `cheese_scene.py` | 10 kaasplakken (8×8 cm, 2 mm, 2D-flex van 6×6 nodes) |
| `render_*.py` | video's (720p, ffmpeg) |
| `tests/` | `pytest sim/x2` |

## Gebruik
```
pip install -r sim/x2/requirements.txt
bash sim/x2/fetch_x2_assets.sh
pytest sim/x2              # snel
X2_SLOW=1 pytest sim/x2    # inclusief volledige kaas-sim (~80 s)
MUJOCO_GL=osmesa python sim/x2/render_knife_smooth.py
```
De video's staan in `docs/media/x2/`.

## Metingen
**Mes smeert boter**: vergeleken met de stand na 9× drukken, 4 halen onder 15°:
| | na drukken | na mes |
|---|---|---|
| gemiddelde dikte | 9,5 mm | 8,0 mm |
| max. hoogte (rand) | 15,1 mm | 12,7 mm |
| std van het bovenvlak | 4,76 mm | 3,60 mm |
| oppervlak | 16,0 cm² | 19,0 cm² |

Haal 2 t/m 4 voegen nog weinig toe.

**Kaasstapel**:
- 1 van de 10 plakken komt los: de bovenste wordt 26 mm opgetild (tijdens de film max. ~45 mm).
- De andere plakken verschuiven hooguit 1,6 mm.
- Plak 8 en 9 komen ~3 mm omhoog, doordat het mes ertussen zit.
- De opgetilde plak buigt ~60 mm door: hij hangt aan één hoek.
- Simulatie stabiel en eindig. Rekentijd ~11 s per gesimuleerde seconde (met renderen ~20 s).

## Beperkingen (eerlijk)
- Geen echt snijden of scheuren: MuJoCo-flex kan dat niet.
- Grepen zijn constraints. Het mes zit in de linkerhand via een weld-equivalent. De kaashoek wordt vastgehouden met `connect`-constraints op flex-vertices. Er is geen wrijvingsgreep.
- Plasticiteit van boter is benaderd: de rustvorm wordt herbouwd (rest += 0,8·(huidig − rest)).
- Hechting van kaas en boter is benaderd met wrijving (tafel 2,0).
- Mesbotsing via proxy's:
  - boter: 6 mm dik, in 5 segmenten (MuJoCo-limiet van 50 contacten);
  - kaas: het echte lemmet, met priority=1.
- Kaasstapel staat tegen een stopwand (holder), anders schuift het mes de stapel weg.
- Rechterarm-IK met tolerantie en restarts. De rest van het robotlichaam is kinematisch.
