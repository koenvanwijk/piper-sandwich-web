# Twee AgileX Piper-armen in MuJoCo: boter, mes, kaas en boterham

Simulatie in **MuJoCo 3.15, niet in Isaac Sim**. Het robotmodel is `assets/scene.xml` van de webdemo. Het mes met rubber handvat uit PR #13 zit in `main`; de test `test_web_scene_has_rubber_knife` controleert dat.

Waar het kan, wordt code uit `sim/x2` hergebruikt: dezelfde boterparameters, tet-topologie, kaasmodel en meetfuncties.

## Bestanden

| Bestand | Inhoud |
|---|---|
| `piper_scene.py` | basisscène, IK en boter 9× indrukken |
| `piper_knife.py` | mes in de linker gripper, boter gladstrijken |
| `piper_cheese.py` | stapel van 10 kaasplakken: mes links, rechter gripper pakt de hoek |
| `piper_bread.py` | boterham oppakken in 3 varianten |
| `render.py` | maakt de video's (`docs/media/piper/`) |
| `tests/` | `pytest sim/piper`; de volledige simulaties met `PIPER_SLOW=1` |

## Metingen en vergelijking met X2

| Experiment | Piper | X2 |
|---|---|---|
| Boter 9× indrukken: max. hoogte | 15 → 9,1 mm | 15 → 15,1 mm (rand), gem. dikte 9,5 mm |
| Boter 9× indrukken: oppervlak | 9 → 16,8 cm² | 9 → 16,0 cm² |
| Boter: verschuiving | 26 mm | ~2–3 cm |
| Mes: max. hoogte tijdens de halen | 9,1 → 6,5–7,7 mm | 15,1 → 12,7 mm |
| Mes: oppervlak | 16,8 → 19,1 cm² | 16,0 → 19,0 cm² |
| Mes: na loslaten | 9,1 mm / 17,2 cm² (vering, zie beperkingen) | – |
| Kaas: losgekomen plakken | 1 van 10; bovenste plak +31 mm, doorbuiging 83 mm | 1 van 10; +26 mm, doorbuiging 60 mm |
| Kaas: verschuiving overige plakken | plak 9: **40 mm**; plakken 1–8: ≤ 9 mm | ≤ 1,6 mm |
| Kaas: rekentijd per sim-seconde | ~16 s (met renderen ~22 s) | ~11 s |
| Boterham, randgreep (1 hand) | niet opgetild: kantelt uit de greep en valt | – |
| Boterham, bimanueel | **opgetild 39 mm**, slip 16 mm, kanteling 34° | – |
| Boterham, mes als spatel + steun | niet opgetild: +9 mm, kanteling 42°, slip 38 mm | – |

Bij de kaasstapel komt de grote verschuiving van plak 9 bij het optillen van het mes. Het lemmet ligt dan op de naad en sleept plak 9 mee. Het mes zakt onder de zwaartekracht ~1,8 mm door; daarvoor wordt gecorrigeerd.

## Beperkingen (eerlijk)

**Boter**
- Met de X2-regel voor de plasticiteit (rest += 0,8·(huidig − rest)) explodeert de flex bij de Piper al na de 3e druk.
  - Oorzaak: de Piper-vingertoppen zijn breed (2,5 × 1,5 cm, X2 0,7 × 2,3 cm) en drukken het blok vlak.
- Daarom wordt de plasticiteit begrensd: geen tet mag kleiner worden dan 10 % van zijn oorspronkelijke volume.
  - Daardoor is de plastische fractie na de 2e druk vrijwel 0.
  - Het blok veert en kreukelt; "gemiddelde dikte van de bovenlaag" is geen goede maat meer. Gebruik max. hoogte en oppervlak.
  - Bij X2 is ook 16 % van de tets omgeklapt.
- De vingertoppen stoppen 6 mm boven de tafel (X2: 3 mm).
- MuJoCo-limiet: maximaal 50 contacten per body–flex-paar.
  - Daarom zijn de vingertop-proxy (6×4), de messegmenten en de tafel (1 cm-tegels) gesplitst in losse bodies.
- In deze scènes botsen robotmeshes niet. Eigen-contacten lieten de solver en de flex trillen.
- Robotaanpassingen ten opzichte van de webdemo, alleen in de simulatie:
  - armature 0,01;
  - flex vraagt de discrete integrator (pyramidale kegel).

**Mes en kaas**
- De mesgreep is weld-equivalent: het mes is een star kind van `left_link6`, zonder slip.
- Bij de kaas is het X2-mes (lemmet 1,5 mm) gebruikt. Het webmes is 4 mm dik, dikker dan een plak van 2 mm.
- De kaasgreep zijn `connect`-constraints op 3 vertices. De hand nadert onder 30° omdat horizontaal onbereikbaar is.
- Echt snijden of scheuren is niet mogelijk.

**Boterham**
- De boterham is star (30 g, 9 × 9 × 2,6 cm) op een 5 cm brede, 4 cm hoge standaard, met 2 cm overstek. De gripper opent maar ~70 mm.
- Een 5 mm korststap aan de onderkant maakt ruimte voor het lemmet.
- De grepen zijn echte vingercontacten met wrijving.
- De rechterarm haalt geen vlakkere naderingen dan 30°.
