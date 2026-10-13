"""Stress maps from the FEA results: plan-view slices through a cam layer
and a knuckle layer, plus a vertical slice down the hinge pin showing the
stress across the printed layers (sigma_zz)."""
import json
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import shapely  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import fea  # noqa: E402

CMAP = matplotlib.colors.LinearSegmentedColormap.from_list(
    "blue", ["#eef4fc", "#9ec5f4", "#3987e5", "#1c5cab", "#0d366b"])


def slice_map(ax, d, axis, value, field, window, vmax, step=0.12):
    nodes, S = d["nodes"], d["S"]
    vals = fea.von_mises(S) if field == "vm" else S[:, 2]
    near = np.abs(nodes[:, axis] - value) < 0.7
    pts = nodes[near]
    keep = [i for i in range(3) if i != axis]
    tree = cKDTree(pts[:, keep])
    (x0, x1), (y0, y1) = window
    gx, gy = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    dist, idx = tree.query(np.c_[gx.ravel(), gy.ravel()])
    img = vals[near][idx].reshape(gx.shape)
    img = np.ma.masked_where(dist.reshape(gx.shape) > 0.45, img)   # outside the material
    im = ax.imshow(img, origin="lower", extent=(x0, x1, y0, y1), cmap=CMAP, vmin=0, vmax=vmax, interpolation="nearest")
    ax.set_aspect("equal")
    return im


def main(chk):
    res = json.load(open(f"{chk}/fea/results.json"))
    g = json.load(open(f"{chk}/geom.json"))
    P = g["P"]
    lever = np.load(f"{chk}/fea/lever_closed.npz")
    collar = np.load(f"{chk}/fea/collar_closed.npz")
    fig, axs = plt.subplots(1, 3, figsize=(16, 5.6), dpi=105)
    win = ((P[0] - 22, P[0] + 14), (P[1] - 20, P[1] + 14))
    im = slice_map(axs[0], lever, 2, 10.7, "vm", win, 30)
    slice_map(axs[0], collar, 2, 10.7, "vm", win, 30)
    axs[0].set_title("Plan slice through a cam layer\n(von Mises, MPa)", loc="left", fontsize=11)
    slice_map(axs[1], collar, 2, 2.6, "vm", win, 30)
    slice_map(axs[1], lever, 2, 2.6, "vm", win, 30)
    axs[1].set_title("Plan slice through a hinge knuckle\n(von Mises, MPa)", loc="left", fontsize=11)
    im2 = slice_map(axs[2], collar, 1, P[1], "zz", ((P[0] - 10, P[0] + 10), (0, 70)), 20)
    slice_map(axs[2], lever, 1, P[1], "zz", ((P[0] - 10, P[0] + 10), (0, 70)), 20)
    axs[2].set_title("Vertical slice down the pin\n(tension across layers, MPa)", loc="left", fontsize=11)
    for ax in axs:
        ax.set_xlabel("mm")
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    c1 = fig.colorbar(im, ax=axs[:2], shrink=0.8, pad=0.02)
    c1.set_label("von Mises stress (MPa) - printed PETG in-plane strength ~45")
    c2 = fig.colorbar(im2, ax=axs[2], shrink=0.8, pad=0.04)
    c2.set_label("tension across layers (MPa) - layer strength ~20")
    lc, lp, cc = res["lever_closed"], res["lever_peak"], res["collar_closed"]
    fig.suptitle(f"Worst case (stiff TPU, wood +0.4 mm): cam force {lc['cam_force']:.0f} N.  "
                 f"Peak stress: lever {max(lc['vm_max'], lp['vm_max']):.0f} MPa, collar/pin {cc['vm_max']:.0f} MPa, "
                 f"across layers {max(lc['szz_max'], lp['szz_max'], cc['szz_max']):.0f} MPa",
                 x=0.01, ha="left", fontsize=12)
    fig.savefig(f"{chk}/../img/stress_maps.png", bbox_inches="tight")


if __name__ == "__main__":
    main(sys.argv[1])
