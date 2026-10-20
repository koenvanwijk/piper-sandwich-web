#!/usr/bin/env python3
"""Headless MuJoCo video: X2 separates the top cheese slice with the knife and picks it (NOT Isaac Sim).
MUJOCO_GL=osmesa python3 sim/x2/render_cheese.py [out.mp4]"""
import json, os, subprocess, sys
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cheese_scene as C

OUT = sys.argv[1] if len(sys.argv) > 1 else "/workspace/x2-cheese.mp4"
W, H, FPS = 1280, 720, 30
FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 30)
SMALL = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 22)
ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                       "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "slow", "-crf", "28",
                       "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT], stdin=subprocess.PIPE)
LAB = {"settle": "10 kaasplakken (2D flex), mes in linkerhand", "knife_down": "Mes omlaag naar de bovenste naad",
       "insert": "Mes schuift tussen bovenste plak en de rest", "knife_lift": "Mes tilt de bovenste plak iets op",
       "reach": "Rechter gripper naar de hoek van de plak", "grasp": "Rechter gripper pakt (connect-constraint)",
       "pick": "Plak optillen", "hold": "Plak vasthouden"}
st = dict(r=None, n=0)
cam = mujoco.MjvCamera(); cam.lookat[:] = [C.STACK_XY[0], C.STACK_XY[1], C.TABLE_TOP + 0.03]


def cb(m, d, t, ph):
    if st["r"] is None: st["r"] = mujoco.Renderer(m, H, W)
    z = min(t / 1.5, 1.0); cam.distance = 1.4 - 0.85 * (0.5 - 0.5 * np.cos(np.pi * z))
    cam.elevation = -18 - 10 * z; cam.azimuth = 160 + 20 * np.sin(0.3 * t)
    st["r"].update_scene(d, cam); img = Image.fromarray(st["r"].render()); g = ImageDraw.Draw(img, "RGBA")
    top = C.slice_verts(m, d, C.N - 1); rest = C.slice_verts(m, d, C.N - 2)
    g.rectangle([0, 0, W, 96], fill=(0, 0, 0, 150)); g.text((20, 12), LAB[ph], font=FONT, fill="white")
    g.text((20, 56), f"bovenste plak: hoogte {1000*(top[:,2].mean()-C.TABLE_TOP):5.1f} mm, doorbuiging {1000*np.ptp(top[:,2]):5.1f} mm"
           f"   plak 9: hoogte {1000*(rest[:,2].mean()-C.TABLE_TOP):4.1f} mm", font=SMALL, fill=(255, 235, 150))
    g.rectangle([0, H - 40, W, H], fill=(0, 0, 0, 150))
    g.text((20, H - 34), f"MuJoCo {mujoco.__version__} (geen Isaac Sim) | geen echt snijden | grip = constraint | t={t:4.1f}s",
           font=SMALL, fill="white")
    ff.stdin.write(img.tobytes()); st["n"] += 1


res = C.simulate(callback=cb, dt_cb=1 / FPS)
ff.stdin.close(); ff.wait()
res["wall_per_sim_s_incl_render"] = res.pop("wall_per_sim_s")
print("RESULT", json.dumps({k: (list(map(float, v)) if isinstance(v, (list, np.ndarray)) else float(v) if not isinstance(v, (bool, type(None))) else v) for k, v in res.items()}))
