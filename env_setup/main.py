import genesis as gs
from mazes import MAZE_FLOOR_0, MAZE_FLOOR_1, FLOOR_GAP
from paths import PATH_FLOOR_1
from env_builder import build_Env
from robot_control import RobotController

def main():
    gs.init(backend=gs.cpu)

    start_xy = PATH_FLOOR_1[0] #first waypoint
    robot_z  = FLOOR_GAP + 0.051  #spawn above floor

    scene, robot_entity = build_Env(
        MAZE_FLOOR_0,
        MAZE_FLOOR_1,
        show_viewer=True,
        robot_start=(start_xy[0], start_xy[1], robot_z),
    )

    controller = RobotController(robot_entity, PATH_FLOOR_1, robot_z)

    while not controller.done:
        controller.update()
        scene.step()

if __name__ == "__main__":
    main()