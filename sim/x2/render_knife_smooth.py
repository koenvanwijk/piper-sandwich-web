#!/usr/bin/env python3
"""Headless MuJoCo video: X2 smooths the pressed butter with a table knife (flexcomp; NOT Isaac Sim).

MUJOCO_GL=osmesa python3 sim/x2/render_knife_smooth.py [out.mp4]   (first runs the 9 presses unrendered)
"""
import os, subprocess, sys
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import butter_scene as B
import knife as K

OUT = sys.argv[1] if len(sys.argv) > 1 else "/workspace/x2-knife.mp4"
W, H, FPS = 1280, 720, 30
FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 30)
SMALL = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 22)
ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                       "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "slow", "-crf", "28",
                       "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT], stdin=subprocess.PIPE)
state = dict(r=None, m=None, m0=None, n=0)
cam = mujoco.MjvCamera(); cam.lookat[:] = [B.BUTTER_XY[0], B.BUTTER_XY[1], B.TABLE_TOP + 0.02]
opt = mujoco.MjvOption()


def cb(m, d, t, phase):
    if state["m"] is not m:  # model is rebuilt after every press (plastic yield)
        if state["r"] is not None: state["r"].close()
        state["r"] = mujoco.Renderer(m, H, W); state["m"] = m
    mt = B.butter_metrics(B.butter_verts(m, d))
    if state["m0"] is None: state["m0"] = mt
    z = min(t / 2.0, 1.0)
    cam.distance = 1.6 - 1.15 * (0.5 - 0.5 * np.cos(np.pi * z)); cam.elevation = -20 - 10 * z
    cam.azimuth = 150 + 25 * np.sin(0.25 * t)
    state["r"].update_scene(d, cam, opt); img = Image.fromarray(state["r"].render()); g = ImageDraw.Draw(img, "RGBA")
    g.rectangle([0, 0, W, 96], fill=(0, 0, 0, 150))
    labels = {"approach": "Mes: aanloop", "descend": "Mes omlaag (blad 15 graden)", "stroke": "Mes strijkt boter glad",
              "lift": "Mes omhoog", "return": "Mes terug voor volgende haal", "rest": "Resultaat"}
    g.text((20, 12), f"{labels[phase.split()[0]]}  {phase.split()[1] if ' ' in phase else ''}", font=FONT, fill="white")
    g.text((20, 56), f"dikte {mt['top_mean']*1000:4.1f} mm (na indrukken {state['m0']['top_mean']*1000:.1f})  max {mt['max_height']*1000:4.1f} mm ({state['m0']['max_height']*1000:.1f})  std {mt['top_std']*1000:.1f} mm ({state['m0']['top_std']*1000:.1f})  "
           f"opp. {mt['footprint']*1e4:4.1f} cm2 (start {state['m0']['footprint']*1e4:.1f})", font=SMALL, fill=(255, 235, 150))
    g.rectangle([0, H - 40, W, H], fill=(0, 0, 0, 150))
    g.text((20, H - 34), f"MuJoCo {mujoco.__version__} flexcomp (geen Isaac Sim) | E={B.BUTTER['young']/1e3:.0f} kPa, nu={B.BUTTER['poisson']}, "
           f"rho={B.BUTTER['density']:.0f} kg/m3 | mes star, vast in linker gripper (weld) | yield {B.YIELD}  t={t:4.1f}s", font=SMALL, fill="white")
    ff.stdin.write(img.tobytes()); state["n"] += 1
    if state["n"] in (45, 60, 200, 380): img.save(os.path.splitext(OUT)[0] + f"_f{state['n']}.png")


press = B.simulate(plastic=True)
res = K.simulate(press["final_verts"], plastic=True, callback=cb, dt_cb=1 / FPS)
ff.stdin.close(); ff.wait()
print("wrote", OUT, "frames", state["n"]); print(res["initial"], res["final"], "finite", res["finite"])
