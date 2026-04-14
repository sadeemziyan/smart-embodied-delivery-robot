import numpy as np
from pure_pursuit import pure_pursuit_step

class RobotController:
    def __init__(self, entity, waypoints, start_z, start_pos, start_heading, dt=1/60):
        self.entity = entity
        self.waypoints = waypoints
        self.pos = np.array(start_pos, dtype=float)
        self.z = start_z
        self.heading = start_heading
        self.dt = dt
        self.done = False

    def update(self):
        if not self.done:
            self.pos, self.heading = pure_pursuit_step(
                self.waypoints, self.pos, self.heading, self.dt
            )

            if np.linalg.norm(self.pos - np.array(self.waypoints[-1])) < 0.1:
                self.done = True

            self.entity.set_pos((float(self.pos[0]), float(self.pos[1]), self.z))

        return self.done