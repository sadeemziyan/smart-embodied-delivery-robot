import heapq
import numpy as np
from mazes import CELL_SIZE

class _Cell:

    def __init__(self):
        self.parent_i = 0  # Parent cell's row index
        self.parent_j = 0  # Parent cell's column index
        self.f = float('inf')  # Total cost of the cell (g + h)
        self.g = float('inf')  # Cost from start to this cell
        self.h = 0  # Heuristic cost from this cell to destination

def is_valid(grid, row, col):
    rows = len(grid)
    cols = len(grid[0])
    return 0 <= row < rows and 0 <= col < cols

def _is_open(maze, row, col):
    return maze[row, col] == 1

def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

def _heuristic(row, col, dest: tuple):
    return abs(row - dest[0]) + abs(col - dest[1])

#returns reversed path from destination to source
def _trace_path(cell_details, dest: tuple):
    path = []
    r, c = dest
    while not (cell_details[r][c].parent_i == r and cell_details[r][c].parent_j == c):
        path.append((r, c))
        pr = cell_details[r][c].parent_i
        pc = cell_details[r][c].parent_j
        r, c = pr, pc
    path.append((r, c))
    path.reverse()
    return path


def _cells_to_world(cell_path: list):
    return [((col + 0.5) * CELL_SIZE, (row + 0.5) * CELL_SIZE)
            for row, col in cell_path]


def _a_star(maze, src: tuple, dest: tuple):
    rows, cols = maze.shape

    if not (is_valid(*src, rows, cols) and is_valid(*dest, rows, cols)):
        print("Source or destination is invalid")
        return []
    if not (_is_open(maze, *src) and _is_open(maze, *dest)):
        print("Source or the destination is blocked")
        return []
    if src == dest:
        print("We are already at the destination")
        return [src]

    closed = [[False] * cols for _ in range(rows)]
    details = [[_Cell() for _ in range(cols)] for _ in range(rows)]

    si, sj = src
    details[si][sj].f = 0.0
    details[si][sj].g = 0.0
    details[si][sj].h = 0.0
    details[si][sj].parent_i = si
    details[si][sj].parent_j = sj

    open_heap = [(0.0, si, sj)]

    DIRS = [(0, 1), (0, -1), (1, 0), (-1, 0)]

    while open_heap:
        _, i, j = heapq.heappop(open_heap)

        if closed[i][j]:
            continue
        closed[i][j] = True

        for di, dj in DIRS:
            ni, nj = i + di, j + dj

            if not _is_valid(ni, nj, rows, cols):
                continue
            if not _is_open(maze, ni, nj):
                continue
            if closed[ni][nj]:
                continue

            if (ni, nj) == dest:
                details[ni][nj].parent_i = i
                details[ni][nj].parent_j = j
                return _trace_path(details, dest)

            g_new = details[i][j].g + 1.0
            h_new = _heuristic(ni, nj, dest)
            f_new = g_new + h_new

            if details[ni][nj].f > f_new:
                details[ni][nj].f = f_new
                details[ni][nj].g = g_new
                details[ni][nj].h = h_new
                details[ni][nj].parent_i = i
                details[ni][nj].parent_j = j
                heapq.heappush(open_heap, (f_new, ni, nj))

    return []

def find_path(maze: np.ndarray, src_cell: tuple, dest_cell: tuple):
    cell_path = _a_star(maze, src_cell, dest_cell)
    if not cell_path:
        return []
    world_path = _cells_to_world(cell_path)
    return world_path