"""Render the lever motion as a video from the real exported meshes.

For each lever angle the collar closes exactly as far as the cam pushes it
(tools/kinematics.py), every part is posed with its rigid transform, and
OpenSCAD renders the frame. ffmpeg assembles the MP4.
"""
import math
import os
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import kinematics as K  # noqa: E402
from sweep3d import mat2d, poses  # noqa: E402

PETG, LEVER, TPU, WOOD = "#3b6e9c", "#5fb3a6", "#e2572b", "#d9b98c"


def scad_matrix(M):
    return "[" + ",".join("[" + ",".join(f"{v:.6f}" for v in row) + "]" for row in M) + "]"


def frame_scad(root, chk, c, th, f, wood=True):
    A, B = poses(c, f)
    P = c.g["P"]
    L = B @ mat2d(0, P[0], P[1]) @ mat2d(th) @ mat2d(0, -P[0], -P[1])
    fq = min((0, 0.25, 0.5, 0.75, 1), key=lambda q: abs(q - f))
    fs = {0: "0", 0.25: "0.25", 0.5: "0.5", 0.75: "0.75", 1: "1"}[fq]
    body = [f'color("{PETG}") import("{root}/anim/ring_{fs}.stl");',
            f'color("{TPU}") import("{root}/anim/tpu_{fs}.stl");',
            f'color("{PETG}") multmatrix({scad_matrix(A)}) import("{chk}/earA_closed.stl");',
            f'color("{PETG}") multmatrix({scad_matrix(B)}) import("{chk}/bgroup_closed.stl");',
            f'color("{LEVER}") multmatrix({scad_matrix(L)}) import("{chk}/lever_closed.stl");']
    if wood:
        body.append(f'color("{WOOD}") translate([0,0,-0.3]) cylinder(d=38, h=70.6, $fn=96);')
    return "\n".join(body)


def render(scad, out, camera, size):
    with tempfile.NamedTemporaryFile("w", suffix=".scad", delete=False) as fh:
        fh.write(scad)
        p = fh.name
    try:
        subprocess.run(["xvfb-run", "-a", "openscad", "-q", "--colorscheme=Tomorrow", "--projection=p",
                        f"--imgsize={size[0]},{size[1]}", f"--camera={camera}", "-o", out, p],
                       check=True, capture_output=True, timeout=600)
    finally:
        os.unlink(p)


def caption(png, text, sub):
    im = Image.open(png).convert("RGB")
    d = ImageDraw.Draw(im)
    try:
        f1 = ImageFont.truetype("DejaVuSans-Bold.ttf", 30)
        f2 = ImageFont.truetype("DejaVuSans.ttf", 24)
    except OSError:
        f1 = f2 = ImageFont.load_default()
    d.text((30, 24), text, fill=(31, 31, 29), font=f1)
    d.text((30, 64), sub, fill=(92, 91, 84), font=f2)
    im.save(png)


def main(root, out_mp4, camera="22,14,30,58,0,62,300", size=(1280, 960), test=False):
    root = os.path.abspath(root)
    chk = f"{root}/chk"
    c = K.Clamp(chk)
    S = c.sections(10.8)
    oa = c.g["open_angle"]
    closing = list(np.linspace(-oa, 0, 54))
    seq = [-oa] * 10 + closing + [0] * 24 + closing[::-1][::2] + [-oa] * 10
    if test:
        seq = [-oa, -oa / 2, 0]
    fcache = {}
    frames = []
    for i, th in enumerate(seq):
        key = round(float(th), 4)
        if key not in fcache:
            fcache[key] = c.contact_f(S, th)
        frames.append((i, th, fcache[key]))
    tmp = f"{root}/anim/frames_" + camera.replace(",", "_")
    os.makedirs(tmp, exist_ok=True)
    rendered = {}

    def job(item):
        i, th, f = item
        key = round(float(th), 4)
        png = f"{tmp}/k_{key:+09.4f}.png"
        if not os.path.exists(png):
            render(frame_scad(root, chk, c, th, f), png, camera, size)
        return key, png

    with ThreadPoolExecutor(max_workers=4) as ex:
        for key, png in ex.map(job, {round(float(th), 4): (i, th, f) for i, th, f in frames}.values()):
            rendered[key] = png
    for i, th, f in frames:
        src = rendered[round(float(th), 4)]
        dst = f"{tmp}/f_{i:04d}.png"
        Image.open(src).save(dst)
        sq = (1 - f) * c.g["open_dr"] - (c.g["open_dr"] - 0.45)
        travel = th + oa
        if f > 0.999:
            state = "open - the collar slides on and off the wood"
        elif th > -10:
            state = "past dead centre - LOCKED"
        else:
            state = "cam pushing the ear - slit closing, collar tightening"
        caption(dst, f"Lever {travel:5.0f} of {oa:.0f} deg   squeeze on wood {max(sq, 0):.2f} mm", state)
    if test:
        return
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "24", "-i", f"{tmp}/f_%04d.png",
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-vf", "scale=1280:-2", out_mp4], check=True)


if __name__ == "__main__":
    cam = sys.argv[3] if len(sys.argv) > 3 else "30,16,40,58,0,62,340"
    main(sys.argv[1], sys.argv[2], camera=cam)
