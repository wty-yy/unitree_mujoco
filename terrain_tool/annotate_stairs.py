#!/usr/bin/env python3
"""Annotate unitree_robots/go2/stairs.xml with one height label per stair series.

Adds a <site> above the centre of the *top landing* of every series (the
highest slab before the down flight). MuJoCo draws a site's *name* as a 3D
label in the viewer (Rendering -> Label -> Site, or press F7 until "Site" is
selected), so the name carries the text: "<riser>cm".

The riser of a series is encoded by the generator in the box half height
(size.z = riser/2); consecutive slabs with the same riser form one series
(= 6 up steps + top landing + 6 down steps + bottom landing). Only the top
landing gets a label, so the 12 series risers (5..38 cm) keep names unique.

Usage: annotate_stairs.py [--dry-run]
"""
import re
import sys

SRC = "/home/yy/Coding/robotics/unitree_mujoco/unitree_robots/go2/stairs.xml"
LABEL_Y = -0.6      # lateral offset: keep labels away from the walking line
Z_OFFSET = 0.002    # hover the site just above the top face
SITE_SIZE = 0.0005
SITE_GROUP = 2      # site group 2 is visible by default, can be toggled off

GEOM_RE = re.compile(
    r'^(?P<indent>\s*)<geom pos="(?P<pos>[^"]+)" type="box" '
    r'size="(?P<size>[^"]+)" quat="(?P<quat>[^"]+)" />\s*$')


def main():
    dry_run = "--dry-run" in sys.argv
    with open(SRC) as f:
        lines = f.read().split("\n")

    # (line_index, indent, x, top_z, riser) for every stair slab
    slabs = []
    for i, line in enumerate(lines):
        m = GEOM_RE.match(line)
        if not m:
            continue
        x, y, z = (float(v) for v in m.group("pos").split())
        sx, sy, sz = (float(v) for v in m.group("size").split())
        if abs(sy - 0.75) > 1e-9:  # not stair geometry (e.g. included robot)
            continue
        slabs.append((i, m.group("indent"), x, z + sz, 2.0 * sz))

    # group consecutive slabs by their series (same generator riser)
    groups, cur = [], []
    for s in slabs:
        if cur and abs(s[4] - cur[-1][4]) > 1e-9:
            groups.append(cur)
            cur = []
        cur.append(s)
    if cur:
        groups.append(cur)

    # label the top landing of each series
    labels = []
    for g in groups:
        riser_cm = int(round(g[0][4] * 100))
        apex = max(range(len(g)), key=lambda i: g[i][3])
        assert sum(abs(g[i][3] - g[apex][3]) < 1e-9 for i in range(len(g))) == 1, \
            "top landing is not unique in this series"
        i, indent, x, top, _ = g[apex]
        labels.append((i, indent, x, top, f"{riser_cm}cm"))

    names = [lbl[4] for lbl in labels]
    assert len(names) == len(set(names)), "duplicate site names"
    for _, _, x, top, name in labels:
        print(f"  {name:10s} x={x:6.3f} top={top:.3f}")

    if dry_run:
        print(f"dry run: {len(labels)} sites, nothing written")
        return

    for i, indent, x, top, name in sorted(labels, reverse=True):
        lines.insert(i + 1, f'{indent}<site name="{name}" '
                           f'pos="{x:g} {LABEL_Y:g} {top + Z_OFFSET:g}" '
                           f'size="{SITE_SIZE:g}" group="{SITE_GROUP}" />')
    with open(SRC, "w") as f:
        f.write("\n".join(lines))
    print(f"wrote {SRC}: {len(labels)} sites")


if __name__ == "__main__":
    main()
