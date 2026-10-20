#!/usr/bin/env python3
"""Headless MuJoCo demo video of the AgiBot X2 model (MuJoCo, NOT Isaac Sim).

MUJOCO_GL=osmesa python3 sim/x2/render_mujoco_demo.py [out.mp4]
Requires: sim/x2/fetch_x2_assets.sh, mujoco, pillow, ffmpeg.
"""
import json, os, subprocess, sys
import numpy as np, mujoco, yaml
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.abspath(__file__))  # sim/x2
X2 = os.path.join(ROOT, "assets", "X2")
RC = json.load(open(os.path.join(X2, "X2_omnipicker_fixed_dual.json")))
ARMS = RC["robot"]["active_arm_joints"]
OUT = sys.argv[1] if len(sys.argv) > 1 else "/workspace/x2-demo.mp4"
W, H, FPS = 1280, 720, 30
T_LOAD, T_SWEEP, T_GRIP, T_HOLD = 5.0, 11.2, 4.0, 4.8
FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 30)
SMALL = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 22)

spec = mujoco.MjSpec.from_file(os.path.join(X2, "X2_omnipicker_fixed_dual.urdf"))
spec.compiler.meshdir = X2
spec.visual.global_.offwidth, spec.visual.global_.offheight = W, H
spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1], pos=[0, 0, -0.68],
                        rgba=[0.35, 0.4, 0.45, 1], contype=0, conaffinity=0)
spec.worldbody.add_light(pos=[0, -1, 3], dir=[0, 0.3, -1], diffuse=[0.8, 0.8, 0.8], castshadow=False)
spec.worldbody.add_light(pos=[1.5, 1.5, 2], dir=[-1, -1, -1], diffuse=[0.4, 0.4, 0.4], castshadow=False)
# Demo only: contacts off (2M-face meshes, no self-collision check in this video), small armature for PD stability
for g in spec.geoms: g.contype = 0; g.conaffinity = 0
for j in spec.joints: j.armature = 0.01; j.damping = [2.0, 0, 0]; j.actfrclimited = mujoco.mjtLimited.mjLIMITED_FALSE  # URDF effort caps ignored for the locked-joint demo
spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
for j in spec.joints:  # stiff position servo per joint (stands in for Isaac joint drives)
    a = spec.add_actuator(); a.name = j.name; a.target = j.name; a.trntype = mujoco.mjtTrn.mjTRN_JOINT
    stiff = j.name.endswith(("hip_pitch_joint", "hip_roll_joint", "hip_yaw_joint", "knee_joint", "ankle_pitch_joint",
                             "ankle_roll_joint")) or j.name.startswith(("waist_", "head_"))
    a.set_to_position(kp=5000.0 if stiff else 300.0, kv=50.0 if stiff else 0.0)
m = spec.compile(); d = mujoco.MjData(m)
adr = lambda n: m.joint(n).qposadr[0]
home = {}
for s in ARMS:
    home.update(zip(ARMS[s], yaml.safe_load(open(os.path.join(X2, f"X2_omnipicker_fixed_{s}.yaml")))["default_q"]))
q0 = np.zeros(m.nq)
for n, v in home.items(): q0[adr(n)] = v
locked = [n for n in RC["robot"]["lock_joints"] if not n.startswith(("L_hand", "R_hand"))]
claws = {"left": "left_claw_joint", "right": "right_claw_joint"}
mimic = {"left": "L_hand_wide1_joint", "right": "R_hand_wide1_joint"}
d.qpos[:] = q0; mujoco.mj_forward(m, d)
pelvis0 = d.xpos[m.body("left_hip_pitch_link").id].copy()

def target(t):
    q = q0.copy(); label, sub = "1/4  Model laden: X2-Ultra + OmniPicker (fixed base)", "rustige draai-camera, home-pose, zwaartekracht aan"
    if T_LOAD <= t < T_LOAD + T_SWEEP:
        k = int((t - T_LOAD) // (T_SWEEP / 7)); u = ((t - T_LOAD) % (T_SWEEP / 7)) / (T_SWEEP / 7)
        for s in ARMS:
            j = ARMS[s][k]; lo, hi = m.jnt_range[m.joint(j).id]
            a = np.sin(2 * np.pi * u)  # home -> hi -> lo -> home
            q[adr(j)] = home[j] + (0.8 * (hi - home[j]) if a > 0 else 0.8 * (home[j] - lo)) * a
        label, sub = "2/4  Joint-sweep per arm (80% van URDF-limieten)", f"joint {k+1}/7: {ARMS['left'][k]} / {ARMS['right'][k]}"
    elif T_LOAD + T_SWEEP <= t < T_LOAD + T_SWEEP + T_GRIP:
        u = (t - T_LOAD - T_SWEEP) / T_GRIP; c = 0.5 - 0.5 * np.cos(2 * np.pi * 2 * u)  # 0..1 twice
        for s in ARMS:
            q[adr(claws[s])] = -1.0 * c; q[adr(mimic[s])] = 1.0 * c
        label, sub = "3/4  Grippers open / dicht (2x)", "claw joint -1.0 (open) .. 0 (dicht), mimic-vinger volgt"
    elif t >= T_LOAD + T_SWEEP + T_GRIP:
        label, sub = "4/4  Benen + bekken stil onder zwaartekracht", "pelvis vast; benen/taille/hoofd via PD vastgehouden"
    return q, label, sub

cam = mujoco.MjvCamera(); cam.lookat[:] = [0, 0, 0.0]; cam.distance = 2.6; cam.elevation = -12
r = mujoco.Renderer(m, H, W)
ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                       "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "slow", "-crf", "28",
                       "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT], stdin=subprocess.PIPE)
steps = int(round(1 / (FPS * m.opt.timestep))); total = T_LOAD + T_SWEEP + T_GRIP + T_HOLD
maxdrift = 0.0; thumbs = {}
for f in range(int(total * FPS)):
    t = f / FPS; qt, label, sub = target(t)
    for i in range(m.nu): d.ctrl[i] = qt[m.jnt_qposadr[m.actuator_trnid[i, 0]]]
    mujoco.mj_step(m, d, nstep=steps)
    drift = max(abs(d.qpos[adr(n)]) for n in locked); maxdrift = max(maxdrift, drift)
    pdrift = np.linalg.norm(d.xpos[m.body("left_hip_pitch_link").id] - pelvis0)
    zoom = np.clip(min(t - (T_LOAD + T_SWEEP) + 0.5, T_LOAD + T_SWEEP + T_GRIP + 0.5 - t), 0, 1) if t > T_LOAD + T_SWEEP - 0.5 else 0
    cam.distance = 2.6 - 1.2 * zoom; cam.lookat[:] = [0.15 * zoom, 0, 0.15 * zoom]
    cam.azimuth = 150 + (60 * t / T_LOAD if t < T_LOAD else 60 + 10 * np.sin(0.4 * (t - T_LOAD)))
    r.update_scene(d, cam); img = Image.fromarray(r.render()); g = ImageDraw.Draw(img, "RGBA")
    g.rectangle([0, 0, W, 96], fill=(0, 0, 0, 150))
    g.text((20, 12), label, font=FONT, fill="white"); g.text((20, 56), sub, font=SMALL, fill=(220, 220, 220))
    g.rectangle([0, H - 40, W, H], fill=(0, 0, 0, 150))
    g.text((20, H - 34), f"MuJoCo {mujoco.__version__} (geen Isaac Sim)  t={t:4.1f}s  max drift benen/taille/hoofd: "
           f"{np.degrees(maxdrift):.2f} deg  hip-basis: {pdrift*1000:.1f} mm", font=SMALL, fill="white")
    ff.stdin.write(img.tobytes())
    if f % 60 == 0: print(f"frame {f}", flush=True)
    for key, tt in (("load", 4.0), ("sweep", 9.0), ("grip", 17.2), ("hold", 23.0)):
        if abs(t - tt) < 1e-6: thumbs[key] = img.copy()
ff.stdin.close(); ff.wait()
os.makedirs(os.path.dirname(os.path.abspath(OUT)) or ".", exist_ok=True)
for k, im in thumbs.items(): im.save(os.path.splitext(OUT)[0] + f"_{k}.png")
print(f"wrote {OUT}; max locked-joint drift {np.degrees(maxdrift):.3f} deg, pelvis {pdrift*1000:.2f} mm, NaN: {not np.isfinite(d.qpos).all()}")
