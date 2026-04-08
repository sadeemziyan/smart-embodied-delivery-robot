import numpy as np

LOOKAHEAD_DIST = 0.5  # metres
ROBOT_SPEED    = 0.5  # metres per second

def find_lookahead_point(waypoints, robot_pos, lookahead_dist):
    """
    Find the next target point on the path that is
    lookahead_dist away from the robot.
    """
    for i in range(len(waypoints) - 1, -1, -1):
        point = np.array(waypoints[i])
        dist = np.linalg.norm(point - robot_pos)
        if dist <= lookahead_dist:
            return point
    return np.array(waypoints[-1])

def pure_pursuit_step(waypoints, robot_pos, robot_heading, dt):
    """
    Given current robot state, return the new position and heading
    after one timestep.

    waypoints    : list of (x, y) tuples
    robot_pos    : np.array([x, y])
    robot_heading: angle in radians (0 = facing positive x axis)
    dt           : timestep in seconds

    returns: new_pos, new_heading
    """
    lookahead_point = find_lookahead_point(waypoints, robot_pos, LOOKAHEAD_DIST)

    # vector from robot to lookahead point
    direction = lookahead_point - robot_pos
    distance  = np.linalg.norm(direction)

    # if close enough to the last waypoint, stop
    last_waypoint = np.array(waypoints[-1])
    if np.linalg.norm(robot_pos - last_waypoint) < 0.1:
        return robot_pos, robot_heading

    # calculate the angle toward the lookahead point
    target_angle = np.arctan2(direction[1], direction[0])

    # gradually turn robot toward target angle
    angle_diff = target_angle - robot_heading
    # normalize to [-pi, pi]
    angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
    new_heading = robot_heading + angle_diff * 0.3

    # move robot forward in the new heading direction
    new_pos = robot_pos + ROBOT_SPEED * dt * np.array([
        np.cos(new_heading),
        np.sin(new_heading)
    ])

    return new_pos, new_heading