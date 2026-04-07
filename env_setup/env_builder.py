import genesis as gs
from mazes import CELL_SIZE, WALL_H, FLOOR_T, FLOOR_GAP, MAZE_FLOOR_0, MAZE_FLOOR_1

def build_Env(maze_floor_0, maze_floor_1=None, show_viewer=False, robot_start=None):
    ROWS, COLS = maze_floor_0.shape

    scene = gs.Scene(show_viewer=show_viewer)

    def add_box(pos, size, color=(0.6, 0.6, 0.6, 1.0), fixed = True):
        entity = scene.add_entity(
            morph=gs.morphs.Box(size=size, pos=pos, fixed=True),
            surface=gs.surfaces.Default(color=color),
        )
        return entity

    slab_w = COLS * CELL_SIZE
    slab_d = ROWS * CELL_SIZE
    cx = slab_w / 2
    cy = slab_d / 2

    add_box(pos=(cx, cy, 0),
            size=(slab_w, slab_d, FLOOR_T),
            color=(0.85, 0.75, 0.60, 1.0))

    floor1_z = FLOOR_GAP
    if maze_floor_1 is not None:
        add_box(pos=(cx, cy, floor1_z),
                size=(slab_w, slab_d, FLOOR_T),
                color=(0.75, 0.85, 0.90, 1.0))

    def build_walls(maze, floor_z, wall_color):
        for r in range(ROWS):
            c = 0
            while c < COLS:
                if maze[r, c] == 1:
                    run_start = c
                    while c < COLS and maze[r, c] == 1:
                        c += 1
                    run_end = c

                    run_length = run_end - run_start
                    wx = (run_start + run_length / 2) * CELL_SIZE
                    wy = (r + 0.5) * CELL_SIZE
                    wz = floor_z + WALL_H / 2

                    add_box(pos=(wx, wy, wz),
                            size=(run_length * CELL_SIZE, CELL_SIZE, WALL_H),
                            color=wall_color)
                else:
                    c += 1

    build_walls(maze_floor_0, 0.0, wall_color=(0.30, 0.30, 0.35, 1.0))
    if maze_floor_1 is not None:
        build_walls(maze_floor_1, floor1_z, wall_color=(0.25, 0.40, 0.55, 1.0))

    robot = None
    if robot_start is not None:
        robot = add_box(
            pos=robot_start,
            size=(0.1, 0.1, 0.1),
            color=(0.10, 0.80, 0.10, 1.0),
            fixed=False,
        )
 
    scene.build()
    return scene, robot