"""Preview images for the clamp.

3D views are rendered by OpenSCAD from the exported STLs (imported meshes
preview cleanly, without CSG z-fighting); the cross-section is drawn with
matplotlib straight from the meshes, so it shows exactly what gets printed.
"""
import os
import subprocess
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

PETG = "#3b6e9c"
TPU = "#e2572b"
WOOD = "#d9b98c"


def _scad_view(scad_body, out_png, camera, size=(1400, 1050)):
    with tempfile.NamedTemporaryFile("w", suffix=".scad", delete=False) as f:
        f.write(scad_body)
        path = f.name
    try:
        cmd = ["xvfb-run", "-a", "openscad", "-q", "--colorscheme=Tomorrow",
               "--projection=p", f"--imgsize={size[0]},{size[1]}",
               f"--camera={camera}", "-o", out_png, path]
        subprocess.run(cmd, check=True, capture_output=True, timeout=600)
    finally:
        os.unlink(path)


def views(shell_stl, liner_stl, wood_d, wood_h, clamp_h, out_dir, prefix=""):
    shell_stl, liner_stl = os.path.abspath(shell_stl), os.path.abspath(liner_stl)
    half = (f'color("{PETG}") import("{shell_stl}");\n'
            f'color("{TPU}") import("{liner_stl}");\n')
    wood = (f'color("{WOOD}") translate([0,0,{(clamp_h - wood_h) / 2}]) '
            f'cylinder(d={wood_d}, h={wood_h}, $fn=128);\n')
    out = {}
    jobs = {
        "assembly": (wood + half + "rotate(180) {" + half + "}",
                     f"0,0,{clamp_h / 2},62,0,32,{max(200, 3.2 * clamp_h)}"),
        "exploded": (wood + "translate([24,0,0]) {" + half + "}"
                     "rotate(180) translate([24,0,0]) {" + half + "}",
                     f"0,0,{clamp_h / 2},58,0,22,{max(230, 3.6 * clamp_h)}"),
        "half_print_orientation": (half, f"10,0,{clamp_h / 2},62,0,140,{max(220, 3.3 * clamp_h)}"),
    }
    for name, (body, cam) in jobs.items():
        png = os.path.join(out_dir, f"{prefix}{name}.png")
        _scad_view(body, png, cam)
        out[name] = png
    return out


def section(shell_stl, liner_stl, wood_d, z, out_png, title=None, notes=()):
    shell, liner = trimesh.load(shell_stl), trimesh.load(liner_stl)
    rot = trimesh.transformations.rotation_matrix(np.pi, [0, 0, 1])
    fig, ax = plt.subplots(figsize=(12.5, 8.4))
    extents = []
    for mesh, colour in ((shell, PETG), (liner, TPU)):
        for m in (mesh, mesh.copy().apply_transform(rot)):
            sec = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
            if sec is None:
                continue
            planar, _ = sec.to_2D(to_2D=np.eye(4))
            for poly in planar.polygons_full:
                xy = np.array(poly.exterior.coords)
                extents.append(xy)
                ax.add_patch(Polygon(xy, fc=colour, ec="black", lw=0.35))
                for hole in poly.interiors:
                    ax.add_patch(Polygon(np.array(hole.coords), fc="white", ec="black", lw=0.35))
    ax.add_patch(Circle((0, 0), wood_d / 2, fc=WOOD, ec="#7a5a30", alpha=0.5, lw=1.0, ls="--"))
    pts = np.vstack(extents)
    span = np.abs(pts).max() + 4
    ax.set_xlim(-span, span + (span * 0.9 if notes else 0))
    ax.set_ylim(-span, span)
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
    ax.set_xlabel("mm")
    ax.set_ylabel("mm")
    ax.set_title(title or f"Cross-section at z = {z:.1f} mm (blue PETG, orange TPU, dashed = wood)")
    if notes:
        ax.text(0.99, 0.5, "\n".join(notes), transform=ax.transAxes, fontsize=9.5,
                va="center", ha="right", multialignment="left", family="monospace",
                bbox=dict(fc="white", ec="0.7", alpha=0.9))
    fig.savefig(out_png, dpi=110, bbox_inches="tight")
    plt.close(fig)
    return out_png


def section_zoom(shell_stl, liner_stl, wood_d, z, out_png, window=((8, 26), (-9, 9))):
    """Close-up of the TPU spring fins against the wood surface."""
    liner = trimesh.load(liner_stl)
    shell = trimesh.load(shell_stl)
    fig, ax = plt.subplots(figsize=(8, 8))
    for mesh, colour in ((shell, PETG), (liner, TPU)):
        sec = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        planar, _ = sec.to_2D(to_2D=np.eye(4))
        for poly in planar.polygons_full:
            ax.add_patch(Polygon(np.array(poly.exterior.coords), fc=colour, ec="black", lw=0.4))
            for hole in poly.interiors:
                ax.add_patch(Polygon(np.array(hole.coords), fc="white", ec="black", lw=0.4))
    ax.add_patch(Circle((0, 0), wood_d / 2, fc="none", ec="#7a5a30", lw=1.4, ls="--"))
    (x0, x1), (y0, y1) = window
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_title("TPU spring fins before squeeze (dashed = wood surface)")
    fig.savefig(out_png, dpi=110, bbox_inches="tight")
    plt.close(fig)
    return out_png
