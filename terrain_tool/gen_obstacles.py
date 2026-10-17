"""Generate unitree_robots/go2/obstacles.xml from the go2_rl_lab obstacle spec.

Replicates the highest-difficulty (1.0) random pillar terrain of the
go2_rl_lab `obstacle_toward` task on a 9 m x 9 m tile centred at the origin:
0.5 m square, 1.0 m high pillars, clusters of at most 2x2 cells, 15% obstacle
area ratio (+- the discrete placement overshoot), a flat centre spawn area
(1.5 m + cell rounding) and a one-cell flat perimeter.

Usage: python terrain_tool/gen_obstacles.py
"""
from pathlib import Path

import numpy as np

OUTPUT_PATH = Path(__file__).resolve().parent.parent / "unitree_robots" / "go2" / "obstacles.xml"

TERRAIN_SIZE = (9.0, 9.0)
DIFFICULTY = 1.0
PILLAR_SQUARE_SIZE = 0.5
PILLAR_HEIGHT = 1.0
CENTER_FLAT_SIZE = 1.5
PERIMETER_FLAT_CELLS = 1
OBSTACLE_AREA_RATIO_RANGE = (0.05, 0.15)
SEED = 2

GROUND_COLOR = "0.12 0.20 0.28 1"
PILLAR_COLOR = "0.85 0.25 0.04 1"


def cluster_templates(max_cells):
    """Return the connected pillar-cell templates bounded by a 2x2 footprint."""
    templates = [[(0, 0)]]
    if max_cells >= 2:
        templates.extend([[(0, 0), (1, 0)], [(0, 0), (0, 1)]])
    if max_cells >= 3:
        templates.extend([[(0, 0), (1, 0), (0, 1)], [(0, 0), (1, 0), (1, 1)]])
    if max_cells >= 4:
        templates.append([(0, 0), (1, 0), (0, 1), (1, 1)])
    return templates


def make_layout(difficulty, rng):
    """Generate the binary obstacle-cell layout for one terrain tile."""
    size_x, size_y = TERRAIN_SIZE
    cells_x = int(round(size_x / PILLAR_SQUARE_SIZE))
    cells_y = int(round(size_y / PILLAR_SQUARE_SIZE))
    level = float(np.clip(difficulty, 0.0, 1.0))
    max_cells = int(np.clip(int(np.floor(1.0 + 3.0 * level + 1.0e-6)), 1, 4))
    target_ratio = OBSTACLE_AREA_RATIO_RANGE[0] + level * (
        OBSTACLE_AREA_RATIO_RANGE[1] - OBSTACLE_AREA_RATIO_RANGE[0])
    target_cells = int(round(target_ratio * cells_x * cells_y))

    occupied = np.zeros((cells_x, cells_y), dtype=bool)
    center_x = size_x * 0.5
    center_y = size_y * 0.5
    spawn_half = CENTER_FLAT_SIZE * 0.5
    templates = cluster_templates(max_cells)

    def is_center_cell(ix, iy):
        return (
            abs((ix + 0.5) * PILLAR_SQUARE_SIZE - center_x) < spawn_half + PILLAR_SQUARE_SIZE * 0.5
            and abs((iy + 0.5) * PILLAR_SQUARE_SIZE - center_y) < spawn_half + PILLAR_SQUARE_SIZE * 0.5
        )

    def is_perimeter_cell(ix, iy):
        return (
            ix < PERIMETER_FLAT_CELLS
            or iy < PERIMETER_FLAT_CELLS
            or ix >= cells_x - PERIMETER_FLAT_CELLS
            or iy >= cells_y - PERIMETER_FLAT_CELLS
        )

    candidate_origins = [
        (ix, iy) for ix in range(cells_x) for iy in range(cells_y)
        if not is_center_cell(ix, iy)
    ]
    for _ in range(max(1000, len(candidate_origins) * 20)):
        if int(occupied.sum()) >= target_cells:
            break
        origin_x, origin_y = candidate_origins[int(rng.integers(len(candidate_origins)))]
        template = templates[int(rng.integers(len(templates)))]
        cells = [(origin_x + dx, origin_y + dy) for dx, dy in template]
        if any(ix < 0 or iy < 0 or ix >= cells_x or iy >= cells_y for ix, iy in cells):
            continue
        if any(is_center_cell(ix, iy) or is_perimeter_cell(ix, iy) for ix, iy in cells):
            continue
        if any(occupied[ix, iy] for ix, iy in cells):
            continue
        cell_set = set(cells)
        touches_existing = any(
            occupied[nx, ny]
            for ix, iy in cells
            for nx in range(max(0, ix - 1), min(cells_x, ix + 2))
            for ny in range(max(0, iy - 1), min(cells_y, iy + 2))
            if (nx, ny) not in cell_set
        )
        if touches_existing:
            continue
        occupied[tuple(np.asarray(cells).T)] = True
    if int(occupied.sum()) < target_cells:
        raise RuntimeError(f"placed {int(occupied.sum())} of {target_cells} target cells")
    return occupied, target_cells


def cover_rectangles(layout):
    """Cover the occupied cells with non-overlapping axis-aligned rectangles."""
    covered = np.zeros_like(layout)
    cells_x, cells_y = layout.shape
    rectangles = []
    for ix in range(cells_x):
        for iy in range(cells_y):
            if not layout[ix, iy] or covered[ix, iy]:
                continue
            width = 0
            while ix + width < cells_x and layout[ix + width, iy] and not covered[ix + width, iy]:
                width += 1
            height = 1
            while iy + height < cells_y and all(
                layout[ix + dx, iy + height] and not covered[ix + dx, iy + height]
                for dx in range(width)
            ):
                height += 1
            covered[ix:ix + width, iy:iy + height] = True
            rectangles.append((ix, iy, width, height))
    return rectangles


def list_to_str(values):
    return " ".join(str(value) for value in values)


def build_xml(rectangles, cells, target_cells):
    half = PILLAR_SQUARE_SIZE * 0.5
    lines = [
        '<mujoco model="go2 obstacle field">',
        '  <include file="go2.xml" />',
        '',
        '  <!-- Highest-difficulty obstacle terrain replicated from the go2_rl_lab',
        '       obstacle_toward task (difficulty 1.0): 9 m x 9 m tile of 0.5 m square,',
        '       1.0 m high pillars with max 2x2-cell clusters, 15% obstacle area ratio,',
        '       flat centre spawn area and one-cell flat perimeter. Regenerate with',
        '       terrain_tool/gen_obstacles.py; the tile is centred at the origin and',
        f'       uses {cells} pillar cells (target {target_cells}). -->',
        '  <statistic center="0 0 0.5" extent="7" />',
        '',
        '  <visual>',
        '    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0" />',
        '    <rgba haze="0.15 0.25 0.35 1" />',
        '    <global azimuth="-130" elevation="-20" />',
        '  </visual>',
        '',
        '  <asset>',
        '    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072" />',
        '    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300" />',
        '    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2" />',
        f'    <material name="obstacle_ground" rgba="{GROUND_COLOR}" specular="0" />',
        f'    <material name="obstacle_pillar" rgba="{PILLAR_COLOR}" specular="0.2" shininess="0.15" />',
        '  </asset>',
        '',
        '  <worldbody>',
        '    <light pos="0 0 1.5" dir="0 0 -1" directional="true" />',
        '    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane" />',
        f'    <geom name="obstacle_pad" pos="0 0 0.0005" type="box" size="{TERRAIN_SIZE[0] * 0.5:g} {TERRAIN_SIZE[1] * 0.5:g} 0.0005" contype="0" conaffinity="0" group="1" material="obstacle_ground" />',
    ]
    for index, (ix, iy, width, height) in enumerate(rectangles):
        x0 = ix * PILLAR_SQUARE_SIZE - TERRAIN_SIZE[0] * 0.5
        y0 = iy * PILLAR_SQUARE_SIZE - TERRAIN_SIZE[1] * 0.5
        pos = [x0 + width * half, y0 + height * half, PILLAR_HEIGHT * 0.5]
        size = [width * half, height * half, PILLAR_HEIGHT * 0.5]
        lines.append(
            f'    <geom name="pillar_{index:02d}" pos="{list_to_str(pos)}" type="box" '
            f'size="{list_to_str(size)}" quat="1.0 0.0 0.0 0.0" material="obstacle_pillar" />'
        )
    lines.extend(['  </worldbody>', '</mujoco>', ''])
    return "\n".join(lines)


def main():
    layout, target_cells = make_layout(DIFFICULTY, np.random.default_rng(SEED))
    rectangles = cover_rectangles(layout)
    with open(OUTPUT_PATH, "w") as f:
        f.write(build_xml(rectangles, int(layout.sum()), target_cells))
    print(f"tile:        {TERRAIN_SIZE[0]:g} x {TERRAIN_SIZE[1]:g} m, cell {PILLAR_SQUARE_SIZE:g} m")
    print(f"difficulty:  {DIFFICULTY:.2f}, seed {SEED}, pillar height {PILLAR_HEIGHT:g} m")
    print(f"cells:       {int(layout.sum())} occupied of {layout.size} (target {target_cells})")
    print(f"geoms:       {len(rectangles)} merged pillar boxes")
    print(f"written:     {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
