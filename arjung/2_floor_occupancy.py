import genesis as gs
import numpy as np

gs.init(backend=gs.cpu) #change to .gpu depending on environment

# ────────────────────────────────────────────────────────────
# 2-Floor Occupation Grid
# ────────────────────────────────────────────────────────────

# ── Define the occupation grid ──────────────────────
# Both floors share the same footprint (10 × 10 cells)
CELL_SIZE   = 0.5   # metres per grid cell
WALL_H      = 0.4   # wall height per floor
FLOOR_T     = 0.05  # floor slab thickness
FLOOR_GAP   = WALL_H * 4  # vertical offset between floors

# Occupation grids
MAZE_FLOOR_0 = np.array([
    [1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,1,0,0,0,0,1],
    [1,0,1,0,1,0,1,1,0,1],
    [1,0,1,0,0,0,0,1,0,1],
    [1,0,1,1,1,1,0,1,0,1],
    [1,0,0,0,0,1,0,0,0,1],
    [1,1,1,0,1,1,1,1,0,1],
    [1,0,0,0,1,0,0,0,0,1],
    [1,0,1,1,1,0,1,1,0,1],
    [1,1,1,1,1,1,1,1,1,1],
], dtype=int)

MAZE_FLOOR_1 = np.array([
    [1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,1,0,1],
    [1,0,1,1,1,0,0,1,0,1],
    [1,0,0,0,1,0,1,0,0,1],
    [1,1,1,0,1,0,1,0,1,1],
    [1,0,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,1,1,1,0,1],
    [1,0,0,0,0,0,0,1,0,1],
    [1,1,1,0,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1],
], dtype=int)

ROWS, COLS = MAZE_FLOOR_0.shape

# ── Scene ──────────────────────────────────

def build_Env(show_viewer=False):
    scene = gs.Scene(show_viewer=show_viewer)

    def add_box(scene, pos, size, color=(0.6, 0.6, 0.6, 1.0)):
        """Add a static URDF-free box morph."""
        scene.add_entity(
            morph=gs.morphs.Box(
                size=size,
                pos=pos,
                euler=(0, 0, 0),
                fixed=True,
            ),
            surface=gs.surfaces.Default(color=color),
        )

    # ── Build flooring ────────────────────────────────
    slab_w = COLS * CELL_SIZE
    slab_d = ROWS * CELL_SIZE
    cx = slab_w / 2
    cy = slab_d / 2

    # Floor 0 (ground level)
    add_box(scene,
            pos=(cx, 0, cy),
            size=(slab_w, FLOOR_T, slab_d),
            color=(0.85, 0.75, 0.60, 1.0))  # warm concrete

    # Floor 1 (2nd Floor)
    floor1_z = FLOOR_GAP
    add_box(scene,
            pos=(cx, floor1_z, cy),
            size=(slab_w, FLOOR_T, slab_d),
            color=(0.75, 0.85, 0.90, 1.0)) #teal

    # ── Generate walls from occupation grid ──────────────
    def build_walls(scene, maze, floor_y, wall_color):
        """Iterate over the occupation grid and place wall boxes."""
        for r in range(ROWS):
            for c in range(COLS):
                if maze[r, c] == 1:
                    wx = (c + 0.5) * CELL_SIZE #ensure observation uses same mapping
                    wz = (r + 0.5) * CELL_SIZE
                    wy = floor_y + WALL_H / 2
                    add_box(scene,
                            pos=(wx, wy, wz),
                            size=(CELL_SIZE, WALL_H, CELL_SIZE),
                            color=wall_color)

    build_walls(scene, MAZE_FLOOR_0, 0.0,
                wall_color=(0.30, 0.30, 0.35, 1.0))   # dark charcoal
    build_walls(scene, MAZE_FLOOR_1, floor1_z,
                wall_color=(0.25, 0.40, 0.55, 1.0))   # steel blue
    
    scene.build()
    return scene