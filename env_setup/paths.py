from mazes import MAZE_FLOOR_0_occ, MAZE_FLOOR_1_occ
from astar import find_path

FLOOR_1_SRC  = (1, 1)   # top-left open cell
FLOOR_1_DEST = (8, 8)   # bottom-right open cell

PATH_FLOOR_1 = find_path(MAZE_FLOOR_1_occ, FLOOR_1_SRC, FLOOR_1_DEST)

FLOOR_0_SRC  = (1, 1)
FLOOR_0_DEST = (8, 8)

PATH_FLOOR_0 = find_path(MAZE_FLOOR_0_occ, FLOOR_0_SRC, FLOOR_0_DEST)