#!/usr/bin/env python3
"""Headless MuJoCo videos of the dual-Piper experiments (MuJoCo, NOT Isaac Sim).
MUJOCO_GL=osmesa python3 sim/piper/render.py butter|knife|cheese|bread [out.mp4]
Prints 'RESULT {json}' with the metrics of the rendered run."""
import json, os, subprocess, sys
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import piper_scene as P

EXP = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else f"/workspace/screens/piper-{EXP}.mp4"
W, H, FPS = 1280, 720, 30
FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 30)
SMALL = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 22)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                       "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "slow", "-crf", "28",
                       "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT], stdin=subprocess.PIPE)
st = dict(r=None)
cam = mujoco.MjvCamera()
TITLE = {"butter": "Piper: boter 9x indrukken (flex dim=3)", "knife": "Piper: mes strijkt boter glad",
         "cheese": "Piper: mes onder bovenste kaasplak, rechts pakt de hoek", "bread": "Piper: boterham oppakken"}


def make_cb(look, info):
    cam.lookat[:] = look

    def cb(m, d, t, ph):
        if st["r"] is None:
            st["r"] = mujoco.Renderer(m, H, W)
        z = min(t / 1.5, 1.0); cam.distance = 0.9 - 0.5 * P.smooth(z)
        cam.elevation = -20 - 8 * z; cam.azimuth = 150 + 15 * np.sin(0.3 * t)
        st["r"].update_scene(d, cam); img = Image.fromarray(st["r"].render()); g = ImageDraw.Draw(img, "RGBA")
        g.rectangle([0, 0, W, 96], fill=(0, 0, 0, 150)); g.text((20, 12), f"{TITLE[EXP]} | {ph}", font=FONT, fill="white")
        g.text((20, 56), info(m, d), font=SMALL, fill=(255, 235, 150))
        g.rectangle([0, H - 40, W, H], fill=(0, 0, 0, 150))
        g.text((20, H - 34), f"MuJoCo {mujoco.__version__} (geen Isaac Sim) | 2x AgileX Piper | t={t:4.1f}s", font=SMALL,
               fill="white")
        ff.stdin.write(img.tobytes())
    return cb


def butter_info(m, d):
    mt = P.butter_metrics(P.butter_verts(m, d))
    return f"max. hoogte {1000*mt['max_height']:4.1f} mm | oppervlak {1e4*mt['footprint']:4.1f} cm2"


if EXP == "butter":
    res = P.simulate_press(callback=make_cb([P.BUTTER_XY[0], P.BUTTER_XY[1], 0.01], butter_info))
    np.save("/tmp/piper_pressed.npy", res.pop("final_verts"))
elif EXP == "knife":
    import piper_knife as K
    rest = np.load("/tmp/piper_pressed.npy")
    res = K.simulate(rest, callback=make_cb([rest[:, 0].mean(), rest[:, 1].mean(), 0.01], butter_info))
elif EXP == "cheese":
    import piper_cheese as C
    def info(m, d):
        top = C.slice_verts(m, d, C.N - 1)
        return f"bovenste plak: hoogte {1000*top[:,2].mean():5.1f} mm | doorbuiging {1000*np.ptp(top[:,2]):5.1f} mm"
    res = C.simulate(callback=make_cb([C.STACK_XY[0], C.STACK_XY[1], 0.02], info))
else:
    import piper_bread as Br
    res = Br.simulate(Br.BEST, callback=make_cb([Br.BREAD_XY[0], Br.BREAD_XY[1], 0.03], Br.info))
ff.stdin.close(); ff.wait()
print("RESULT", json.dumps(res, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
