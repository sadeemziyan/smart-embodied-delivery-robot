import genesis as gs
from mazes import MAZE_FLOOR_0, MAZE_FLOOR_1
from env_builder import build_Env

gs.init(backend=gs.cpu)

scene = build_Env(MAZE_FLOOR_0, MAZE_FLOOR_1, show_viewer=True)

for _ in range(5000):
    scene.step()