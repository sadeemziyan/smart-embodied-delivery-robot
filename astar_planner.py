import heapq
import math
import torch
import numpy as np
import genesis as gs


class AStarPlanner:
    """
    A* planner on a 2D grid. Outputs a sequence of (x, y) waypoints.
    RL policy then tracks these waypoints at the joint control.
    """

    def __init__(self, grid_resolution=0.2, grid_size=(50, 50)):
        self.grid_resolution = grid_resolution
        self.grid_width, self.grid_height = grid_size
        # Occupancy grid: 0 = free, 1 = obstacle
        self.grid = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)

    def world_to_grid(self, x, y):
        """Convert world coordinates (meters) to grid indices."""
        gx = int((x + self.grid_width * self.grid_resolution / 2) / self.grid_resolution)
        gy = int((y + self.grid_height * self.grid_resolution / 2) / self.grid_resolution)
        gx = max(0, min(self.grid_width - 1, gx))
        gy = max(0, min(self.grid_height - 1, gy))
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Convert grid indices back to world coordinates (meters)."""
        x = gx * self.grid_resolution - self.grid_width * self.grid_resolution / 2
        y = gy * self.grid_resolution - self.grid_height * self.grid_resolution / 2
        return x, y

    def add_obstacle(self, x, y, radius=0.3):
        """Mark a circular obstacle in the grid."""
        r_cells = int(radius / self.grid_resolution) + 1
        gx, gy = self.world_to_grid(x, y)
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                if dx * dx + dy * dy <= r_cells * r_cells:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                        self.grid[ny, nx] = 1

    def _heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def plan(self, start_world, goal_world):
        """
        Run A* from start to goal in world coordinates.
        Returns: List of (x, y) world-coordinate waypoints, or [] if no path found.
        """
        start = self.world_to_grid(*start_world)
        goal = self.world_to_grid(*goal_world)

        if self.grid[start[1], start[0]] or self.grid[goal[1], goal[0]]:
            return []  # start/goal in obstacle

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: self._heuristic(start, goal)}

        # 8-connected grid
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(self.grid_to_world(*current))
                    current = came_from[current]
                path.append(self.grid_to_world(*start))
                path.reverse()
                return path

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if not (0 <= neighbor[0] < self.grid_width and
                        0 <= neighbor[1] < self.grid_height):
                    continue
                if self.grid[neighbor[1], neighbor[0]]:
                    continue

                move_cost = math.sqrt(dx * dx + dy * dy)
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

    def smooth_path(self, path, smoothing_factor=0.5):
        """Simple path smoothing via gradient descent."""
        if len(path) <= 2:
            return path
        smoothed = [list(p) for p in path]
        tolerance = 0.001
        change = tolerance + 1
        while change > tolerance:
            change = 0.0
            for i in range(1, len(smoothed) - 1):
                for j in range(2):
                    old = smoothed[i][j]
                    smoothed[i][j] += smoothing_factor * (path[i][j] - smoothed[i][j])
                    smoothed[i][j] += smoothing_factor * (
                        smoothed[i - 1][j] + smoothed[i + 1][j] - 2 * smoothed[i][j]
                    )
                    change += abs(old - smoothed[i][j])
        return [tuple(p) for p in smoothed]


class WaypointManager:
    """
    Advances to the next waypoint when the robot is close enough.
    """

    def __init__(self, num_envs, waypoint_reach_threshold=0.3, device="cpu"):
        self.num_envs = num_envs
        self.threshold = waypoint_reach_threshold
        self.device = device

        # Current active waypoint index per env
        self.waypoint_idx = torch.zeros(num_envs, dtype=torch.long, device=device)
        # Waypoints: [num_envs, max_waypoints, 2]
        self.max_waypoints = 20
        self.waypoints = torch.zeros(
            (num_envs, self.max_waypoints, 2), dtype=torch.float32, device=device
        )
        self.num_waypoints = torch.zeros(num_envs, dtype=torch.long, device=device)

    def set_plan(self, env_idx, waypoints_xy):
        """Set the A* plan for a given environment."""
        n = min(len(waypoints_xy), self.max_waypoints)
        self.waypoints[env_idx, :n] = torch.tensor(
            waypoints_xy[:n], dtype=torch.float32, device=self.device
        )
        self.num_waypoints[env_idx] = n
        self.waypoint_idx[env_idx] = 0

    def get_current_waypoint(self, env_idx=None):
        """Return the current target waypoint for each env: [num_envs, 2]."""
        if env_idx is None:
            idx = self.waypoint_idx.clamp(max=self.max_waypoints - 1)
            return self.waypoints[torch.arange(self.num_envs, device=self.device), idx]
        else:
            idx = self.waypoint_idx[env_idx].clamp(max=self.max_waypoints - 1)
            return self.waypoints[env_idx, idx]

    def update(self, base_pos_xy):
        """Advance waypoint index for envs that have reached current waypoint."""
        current_wp = self.get_current_waypoint()  # [num_envs, 2]
        dist = torch.norm(base_pos_xy - current_wp, dim=-1)  # [num_envs]
        reached = dist < self.threshold
        not_at_end = self.waypoint_idx < (self.num_waypoints - 1)
        advance = reached & not_at_end
        self.waypoint_idx += advance.long()

    def is_done(self):
        """Return bool mask of envs that reached final waypoint."""
        return self.waypoint_idx >= (self.num_waypoints - 1)

    def reset(self, envs_idx=None):
        if envs_idx is None:
            self.waypoint_idx.zero_()
        else:
            self.waypoint_idx.masked_fill_(envs_idx, 0)
