"""
Navigator component that ties together NavigationMap, AStarPlanner, and PurePursuitController.
"""

import math
from typing import Tuple, List, Optional

from navigation.map import NavigationMap
from navigation.planner import AStarPlanner
from navigation.controller import PurePursuitController


class Navigator:
    """
    Component 3 of 3: Navigator ties together the NavigationMap, AStarPlanner, and PurePursuitController.

    Args:
        nav_map: NavigationMap instance
        planner: AStarPlanner instance
        controller: PurePursuitController instance
        replan_interval: Steps between replanning (default 100)
    """

    def __init__(self, nav_map: NavigationMap, planner: AStarPlanner,
                 controller: PurePursuitController, replan_interval: int = 100):
        self.nav_map = nav_map
        self.planner = planner
        self.controller = controller
        self.replan_interval = replan_interval

        self.current_goal_world: Optional[Tuple[float, float]] = None
        self.current_plan: List[Tuple[float, float]] = []
        self.step_count: int = 0

    def reset(self, start_world: Tuple[float, float],
              goal_world: Tuple[float, float]) -> None:
        """
        Reset the navigator with new start and goal positions.

        Args:
            start_world: (x, y) tuple in world coordinates
            goal_world: (x, y) tuple in world coordinates
        """
        self.current_goal_world = goal_world
        self.step_count = 0

        # Plan initial path
        path = self.planner.plan(start_world, goal_world)
        if not path:
            raise RuntimeError(f"[Navigator] No path found from {start_world} to {goal_world}")

        self.current_plan = path
        self.controller.reset(path)

        # Print start, goal, number of waypoints
        print(f"[Navigator] Start: {start_world}, Goal: {goal_world}")
        print(f"[Navigator] Planned path with {len(path)} waypoints")

    def step(self, wx: float, wy: float, yaw: float) -> Tuple[float, float, float, bool]:
        """
        Compute velocity commands for current state with periodic replanning.

        Args:
            wx: Current robot position x (meters)
            wy: Current robot position y (meters)
            yaw: Current robot orientation angle (radians)
        Returns:
            Tuple of (vx, vy, yaw_rate, done)
        """
        self.step_count += 1

        # Replan periodically from current pose to goal
        if self.step_count % self.replan_interval == 0:
            new_path = self.planner.plan((wx, wy), self.current_goal_world)
            if new_path:
                self.current_plan = new_path
                self.controller.reset(new_path)

            dist = math.hypot(wx - self.current_goal_world[0],
                              wy - self.current_goal_world[1])
            if new_path:
                print(f"[Navigator] Replanned at step {self.step_count}: "
                      f"{len(new_path)} waypoints, distance to goal: {dist:.2f}m")
            else:
                print(f"[Navigator] Replan failed at step {self.step_count} — keeping existing path")

        # Log progress
        if self.step_count % self.replan_interval == 0:
            dist = math.hypot(wx - self.current_goal_world[0],
                              wy - self.current_goal_world[1])
            print(f"[Navigator] Step {self.step_count}: "
                  f"pos=({wx:.2f},{wy:.2f}), yaw={yaw:.2f}, "
                  f"distance to goal={dist:.2f}m, "
                  f"waypoint index={self.controller.current_waypoint_index}")

        # Get velocity commands from controller
        return self.controller.step(wx, wy, yaw)
