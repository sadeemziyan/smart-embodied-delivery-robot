import numpy as np
from pure_pursuit import pure_pursuit_step, ROBOT_SPEED


class RobotController:
    def __init__(self, entity, waypoints, start_z):
        self.entity    = entity
        self.waypoints = waypoints
        self.pos       = np.array(waypoints[0], dtype=float)
        self.z         = start_z
        self.heading   = 0.0
        self.done      = False

    def update(self):
        if self.done:
            return True

        self.heading = pure_pursuit_step(
            self.waypoints, self.pos, self.heading
        )

        vx = ROBOT_SPEED * np.cos(self.heading)
        vy = ROBOT_SPEED * np.sin(self.heading)
        #degrees of freedom velocity, only move across x and y
        self.entity.set_dofs_velocity([vx, vy, 0.0, 0.0, 0.0, 0.0])

        sim_pos = self.entity.get_pos()
        self.pos = np.array([float(sim_pos[0]), float(sim_pos[1])], dtype=float)

        #if distance from dest<.1, done
        if np.linalg.norm(self.pos - np.array(self.waypoints[-1])) < 0.1:
            self.entity.set_dofs_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.done = True

        return self.done