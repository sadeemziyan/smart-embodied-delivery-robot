import genesis as gs
from mazes import MAZE_FLOOR_0, MAZE_FLOOR_1, FLOOR_GAP
from paths import PATH_FLOOR_1
from env_builder import build_Env
from robot_control import RobotController

def main():
    gs.init(backend=gs.cpu)

    robot_z = FLOOR_GAP + 0.051

    scene, robot_entity, spawn_pos, spawn_heading = build_Env(
        MAZE_FLOOR_0,
        MAZE_FLOOR_1,
        show_viewer=True,
    )

    controller = RobotController(robot_entity, PATH_FLOOR_1, robot_z, spawn_pos, spawn_heading)

    while not controller.done:
        controller.update()
        scene.step()

if __name__ == "__main__":
    main()