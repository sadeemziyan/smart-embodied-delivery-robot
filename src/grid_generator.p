"""
Grid Generator Module
Team A: Environment Generation

This module generates procedural occupancy grids for robot navigation.
Uses "number of islands" style connectivity analysis to ensure solvability.
"""

import numpy as np
from typing import Tuple
from collections import deque


class GridGenerator:
    """Generates procedural occupancy grids for navigation environments."""
    
    def __init__(self, width: int = 15, height: int = 15, cell_size: float = 0.35):
        """
        Initialize the grid generator.
        
        Args:
            width: Grid width in cells
            height: Grid height in cells
            cell_size: Physical size of each cell in meters
        """
        self.width = width
        self.height = height
        self.cell_size = cell_size
        
    def generate(self, 
                 obstacle_density: float = 0.1,
                 num_stairs: int = 3,
                 seed: int = None) -> dict:
        """
        Generate a new occupancy grid.
        
        Args:
            obstacle_density: Probability of a cell being an obstacle (0-1)
            num_stairs: Number of stair locations to generate
            seed: Random seed for reproducibility
            
        Returns:
            dict containing:
                - 'grid': 2D numpy array (0=free, 1=wall, 3=stair)
                - 'height_map': 2D numpy array of elevations
                - 'start_pos': (x, y) starting position
                - 'goal_pos': (x, y) goal position
                - 'metadata': Additional info
        """
        if seed is not None:
            np.random.seed(seed)
            
        # Initialize empty grid
        grid = np.zeros((self.width, self.height), dtype=np.int32)
        
        # Add random walls
        for i in range(self.width):
            for j in range(self.height):
                if np.random.rand() < obstacle_density:
                    grid[i, j] = 1  # Wall
                    
        # Add stair markers
        for _ in range(num_stairs):
            x = np.random.randint(2, self.width - 2)
            y = np.random.randint(2, self.height - 2)
            grid[x, y] = 3  # Stair
            
        # Generate height map
        height_map = self._generate_height_map()
        
        # Sample start and goal positions
        start_pos, goal_pos = self._sample_start_goal(grid)
        
        # Validate connectivity (BFS)
        if not self._is_connected(grid, start_pos, goal_pos):
            # Try to fix connectivity by clearing a path
            grid = self._ensure_connectivity(grid, start_pos, goal_pos)
            
        metadata = {
            'solvable': self._is_connected(grid, start_pos, goal_pos),
            'num_obstacles': np.sum(grid == 1),
            'num_stairs': np.sum(grid == 3),
            'obstacle_density': np.mean(grid == 1),
        }
        
        return {
            'grid': grid,
            'height_map': height_map,
            'start_pos': start_pos,
            'goal_pos': goal_pos,
            'metadata': metadata,
        }
    
    def _generate_height_map(self) -> np.ndarray:
        """
        Generate procedural elevation map using simple noise.
        
        Returns:
            2D array of heights
        """
        height_map = np.zeros((self.width, self.height))
        
        # Simple sinusoidal terrain
        for i in range(self.width):
            for j in range(self.height):
                height_map[i, j] = (
                    np.sin(i * 0.4) + np.cos(j * 0.4)
                ) * 0.05
                
        # Add elevation zones
        num_zones = 2
        for _ in range(num_zones):
            cx = np.random.randint(4, self.width - 4)
            cy = np.random.randint(4, self.height - 4)
            max_elevation = 0.6
            
            for i in range(self.width):
                for j in range(self.height):
                    dist = np.sqrt((i - cx)**2 + (j - cy)**2)
                    if dist < 3:
                        height_map[i, j] += max_elevation * (1 - dist / 3)
                        
        return height_map
    
    def _sample_start_goal(self, grid: np.ndarray) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        """
        Sample valid start and goal positions in free space.
        
        Args:
            grid: Occupancy grid
            
        Returns:
            (start_pos, goal_pos) as (x, y) tuples
        """
        free_cells = np.argwhere(grid == 0)
        
        if len(free_cells) < 2:
            # Fallback: force some free space
            grid[1, 1] = 0
            grid[self.width - 2, self.height - 2] = 0
            free_cells = np.argwhere(grid == 0)
            
        # Sample two different free cells
        indices = np.random.choice(len(free_cells), size=2, replace=False)
        start_pos = tuple(free_cells[indices[0]])
        goal_pos = tuple(free_cells[indices[1]])
        
        return start_pos, goal_pos
    
    def _is_connected(self, grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> bool:
        """
        Check if start and goal are in the same connected component using BFS.
        This is the "number of islands" style connectivity check.
        
        Args:
            grid: Occupancy grid
            start: Start position (x, y)
            goal: Goal position (x, y)
            
        Returns:
            True if connected, False otherwise
        """
        if grid[start] != 0 or grid[goal] != 0:
            return False
            
        visited = set()
        queue = deque([start])
        visited.add(start)
        
        while queue:
            x, y = queue.popleft()
            
            if (x, y) == goal:
                return True
                
            # Check 4 neighbors
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                
                if (0 <= nx < self.width and 
                    0 <= ny < self.height and
                    (nx, ny) not in visited and
                    grid[nx, ny] == 0):  # Only free space
                    
                    visited.add((nx, ny))
                    queue.append((nx, ny))
                    
        return False
    
    def _ensure_connectivity(self, grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> np.ndarray:
        """
        Ensure connectivity by clearing obstacles along a simple path.
        
        Args:
            grid: Occupancy grid
            start: Start position
            goal: Goal position
            
        Returns:
            Modified grid with guaranteed connectivity
        """
        # Simple approach: clear a Manhattan path
        x, y = start
        gx, gy = goal
        
        # Move horizontally
        while x != gx:
            grid[x, y] = 0
            x += 1 if gx > x else -1
            
        # Move vertically
        while y != gy:
            grid[x, y] = 0
            y += 1 if gy > y else -1
            
        return grid
