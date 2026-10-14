"""Is the TPU held by shape alone? A zero-adhesion test for PETG + TPU parts.

Slides the TPU body out of the PETG body along 26 directions (the faces,
edges and corners of a cube) and checks whether it runs into PETG within a
short distance. A joint that relies on adhesion (e.g. a flat butt joint)
has a free direction; a mechanically captured joint has none.

It treats both bodies as rigid, so it is a necessary condition, not a
proof: soft TPU can still be pulled out by deforming it a lot. The report
also gives, for every direction, how far the TPU can slide freely.

Usage:
  python3 tools/capture_check.py petg.stl tpu.stl [--max 2.0]
  (or import and call check(petg_mesh, tpu_mesh))
Each TPU piece (connected body) is tested on its own.
"""
import itertools
import sys

import numpy as np
import trimesh

STEPS = (0.15, 0.3, 0.5, 0.8, 1.2, 2.0)
HIT = 0.05            # mm^3 of overlap counts as a collision


def directions():
    out = []
    for v in itertools.product((-1, 0, 1), repeat=3):
        if any(v):
            v = np.array(v, float)
            out.append(v / np.linalg.norm(v))
    return out


def free_travel(petg, piece, d, steps=STEPS):
    """How far `piece` can slide along d before it overlaps the PETG (inf if never within steps)."""
    for s in steps:
        moved = piece.copy()
        moved.apply_translation(d * s)
        try:
            vol = moved.intersection(petg).volume
        except Exception:
            vol = 0.0
        if vol > HIT:
            return s
    return float("inf")


def check(petg, tpu, max_free=2.0):
    pieces = tpu.split(only_watertight=False)
    report = []
    ok_all = True
    for k, piece in enumerate(pieces):
        rows = []
        for d in directions():
            t = free_travel(petg, piece, d)
            rows.append((tuple(float(x) for x in np.round(d, 2)), t))
        escapes = [r for r in rows if r[1] > max_free]
        ok = not escapes
        ok_all &= ok
        report.append(dict(piece=k, volume=float(piece.volume), captured=ok,
                           escapes=[e[0] for e in escapes],
                           worst_free_mm=max((r[1] for r in rows if r[1] != float("inf")), default=0.0)))
    return ok_all, report


if __name__ == "__main__":
    petg, tpu = trimesh.load(sys.argv[1]), trimesh.load(sys.argv[2])
    ok, rep = check(petg, tpu)
    for r in rep:
        status = "CAPTURED" if r["captured"] else "NOT CAPTURED"
        print(f"TPU piece {r['piece']} ({r['volume']:.0f} mm3): {status}", end="")
        print(f"; free to slide along {r['escapes']}" if r["escapes"] else
              f"; slides at most {r['worst_free_mm']:.2f} mm in any direction")
    sys.exit(0 if ok else 1)
