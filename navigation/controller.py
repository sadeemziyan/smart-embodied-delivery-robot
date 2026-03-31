"""
Pure pursuit controller module for the delivery robot navigation stack.

This module implements the PurePursuitController class that converts
a list of waypoints into velocity commands (linear, lateral) while
respecting maximum velocity and curvature limits.
"""

import math
from typing import List, Tuple


class PurePursuitController:
    """
    Pure pursuit controller for robot trajectory tracking.

    Configurable parameters allow tuning of response aggressiveness,
    speed limits, and goal acceptance criteria.
    """

    def __init__(
        self,
        lookahead_distance: float = 0.5,
        max_linear_vel: float = 0.5,
        max_angular_vel: float = 1.0,
        goal_threshold: float = 0.3
    ):
        """
        Initialize controller parameters.

        Args:
            lookahead_distance: Distance ahead to look for next target point
            max_linear_vel: Maximum linear speed (m/s)
            max_angular_vel: Maximum angular speed (rad/s)
            goal_threshold: Distance to goal considered as reached
        """
        self.lookahead_distance = lookahead_distance
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.goal_threshold = goal_threshold
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint_index: int = 0

    def reset(self, waypoints: List[Tuple[float, float]]) -> None:
        """
        Reset the controller with a new set of waypoints.

        Args:
            waypoints: List of (x, y) world coordinates
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0

    def step(self, current_x: float, current_y: float, current_yaw: float) -> Tuple[float, float, float, bool]:
        """
        Compute velocity commands for current state.

        Args:
            current_x: Current robot position x (meters)
            current_y: Current robot position y (meters)
            current_yaw: Current robot orientation angle (radians)
        Returns:
            Tuple of (linear_velocity, lateral_velocity, angular_velocity, done_flag)
        """
        if not self.waypoints:
            return (0.0, 0.0, 0.0, True)

        # Skip waypoints that have effectively already been reached, including the
        # first point after a replan which is often the robot's current cell center.
        while self.current_waypoint_index < len(self.waypoints) - 1:
            target_x, target_y = self.waypoints[self.current_waypoint_index]
            dist_to_current = math.hypot(target_x - current_x, target_y - current_y)
            if dist_to_current > self.lookahead_distance:
                break
            self.current_waypoint_index += 1

        # Current target waypoint
        target_x, target_y = self.waypoints[self.current_waypoint_index]
        dist_to_target = math.hypot(target_x - current_x, target_y - current_y)

        # Done: at last waypoint and within goal threshold
        is_last = (self.current_waypoint_index == len(self.waypoints) - 1)
        if is_last and dist_to_target <= self.goal_threshold:
            return (0.0, 0.0, 0.0, True)

        # Heading error
        target_heading = math.atan2(target_y - current_y, target_x - current_x)
        heading_error = target_heading - current_yaw
        # Normalize to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Velocity commands per spec
        vx = self.max_linear_vel * math.cos(heading_error)
        vy = 0.0  # Go2 policy uses yaw steering, not lateral motion
        yaw_rate = float(max(-self.max_angular_vel,
                             min(self.max_angular_vel,
                                 1.5 * heading_error)))

        return (float(vx), float(vy), float(yaw_rate), False)
