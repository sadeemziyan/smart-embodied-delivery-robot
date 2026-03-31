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
                # euler=(0, 0, 0),
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
            pos=(cx, cy, 0),
            size=(slab_w, slab_d, FLOOR_T),
            color=(0.85, 0.75, 0.60, 1.0))  # warm concrete

    # Floor 1 (2nd Floor)
    floor1_z = FLOOR_GAP
    add_box(scene,
            pos=(cx, cy, floor1_z),
            size=(slab_w, slab_d, FLOOR_T),
            color=(0.75, 0.85, 0.90, 1.0)) #teal

    # ── Generate walls from occupation grid ──────────────
    def build_walls(scene, maze, floor_z, wall_color):
        for r in range(ROWS):
            c = 0
            while c < COLS:
                if maze[r, c] == 1:
                    run_start = c
                    while c < COLS and maze[r, c] == 1:
                        c += 1
                    run_end = c  # exclusive

                    run_length = run_end - run_start
                    wx = (run_start + run_length / 2) * CELL_SIZE
                    wy = (r + 0.5) * CELL_SIZE
                    wz = floor_z + WALL_H / 2

                    add_box(scene,
                            pos=(wx, wy, wz),
                            size=(run_length * CELL_SIZE, CELL_SIZE, WALL_H),
                            color=wall_color)
                else:
                    c += 1
                    
    build_walls(scene, MAZE_FLOOR_0, 0.0,
                wall_color=(0.30, 0.30, 0.35, 1.0))   # dark charcoal
    build_walls(scene, MAZE_FLOOR_1, floor1_z,
                wall_color=(0.25, 0.40, 0.55, 1.0))   # steel blue
    
    scene.build()
    return scene

if __name__ == "__main__":
    scene = build_Env(show_viewer=True)

    for _ in range(5000):
        scene.step()