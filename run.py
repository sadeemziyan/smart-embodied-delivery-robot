#!/usr/bin/env python3
"""
Main simulation entrypoint for the delivery robot navigation stack.

Runs through four phases:
1. Build the occupancy grid map
2. Run A* path planning
3. Execute pure pursuit control in a mock simulation
4. (Optional) Genesis physics simulation if installed

Usage: python run.py [--seed SEED] [--density DENSITY]
"""

import argparse
import sys
import os
import numpy as np
from typing import Tuple, List

# Local imports
from grid_generator import GridGenerator
from navigation.map import NavigationMap
from navigation.planner import AStarPlanner
from navigation.controller import PurePursuitController
from navigation.navigator import Navigator
from navigation.ros_interface import NavigatorNode


def configure_console_output() -> None:
    """Prefer UTF-8 streams so Genesis logs do not crash on Windows cp1252 consoles."""
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if stream is None or not hasattr(stream, "reconfigure"):
            continue
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass


def parse_args():
    parser = argparse.ArgumentParser(description="Run navigation simulation")
    parser.add_argument('--seed', type=int, default=42, help='Random seed for grid generation')
    parser.add_argument('--density', type=float, default=0.12, help='Obstacle density (0-1)')
    parser.add_argument('--width', type=int, default=20, help='Grid width in cells')
    parser.add_argument('--height', type=int, default=20, help='Grid height in cells')
    parser.add_argument('--cell_size', type=float, default=0.5, help='Cell size in meters')
    parser.add_argument('--robot_radius', type=float, default=0.4, help='Robot radius in meters')
    return parser.parse_args()


def phase1_build_map(args) -> Tuple[NavigationMap, Tuple[float, float], Tuple[float, float]]:
    """PHASE 1 — Build the occupancy grid map."""
    print("\n=== PHASE 1: Build the map ===")
    gen = GridGenerator(width=args.width, height=args.height, cell_size=args.cell_size)
    nav_map = NavigationMap(robot_radius_m=args.robot_radius)
    nav_map.build(gen, obstacle_density=args.density)

    start_world = nav_map.get_simulated_start_world()
    goal_world = nav_map.get_simulated_goal_world()

    world_size = nav_map.get_world_size()
    print(f"[run] Grid dimensions: {nav_map.width_cells}x{nav_map.height_cells} cells")
    print(f"[run] Cell size: {nav_map.cell_size_m:.3f} m")
    print(f"[run] World size: {world_size[0]:.2f} x {world_size[1]:.2f} m")
    print(f"[run] Start: {start_world}")
    print(f"[run] Goal:  {goal_world}")

    # Save map_debug.png as required by validation
    nav_map.verify_alignment(start_world, goal_world)
    print("[run] Saved map_debug.png")

    return nav_map, start_world, goal_world


def phase2_planner(nav_map: NavigationMap, start_world: Tuple[float, float], goal_world: Tuple[float, float]) -> List[Tuple[float, float]]:
    """PHASE 2 — Run A* planner to generate path."""
    print("\n=== PHASE 2: A* path planning ===")
    planner = AStarPlanner(nav_map)
    path = planner.plan(start_world, goal_world)

    if not path:
        print(f"[run] ERROR: No path found from {start_world} to {goal_world}")
        sys.exit(1)

    # Save visualization
    planner.visualize(path, 'planner_debug.png')
    print(f"[run] Planned path: {len(path)} waypoints, length = {calculate_path_length(path):.2f} m")
    print("[run] Saved planner_debug.png")
    return path


def phase3_mock_simulation(nav_map: NavigationMap, planner: AStarPlanner,
                           start_world: Tuple[float, float], goal_world: Tuple[float, float]):
    """PHASE 3 — Run pure pursuit controller in a mock simulation loop."""
    print("\n=== PHASE 3: Mock simulation (pure pursuit) ===")
    controller = PurePursuitController()
    navigator = Navigator(nav_map, planner, controller)
    navigator.reset(start_world, goal_world)

    # Mock state variables
    current_x, current_y = start_world
    current_yaw = 0.0  # face along X axis
    timestep = 0.1  # seconds

    max_steps = 2000
    step = 0
    goal_reached = False

    while step < max_steps:
        forward_vel, lateral_vel, yaw_rate, done = navigator.step(current_x, current_y, current_yaw)

        # The controller outputs body-frame commands, so rotate them into world frame.
        world_vx = forward_vel * np.cos(current_yaw) - lateral_vel * np.sin(current_yaw)
        world_vy = forward_vel * np.sin(current_yaw) + lateral_vel * np.cos(current_yaw)

        current_x += world_vx * timestep
        current_y += world_vy * timestep
        current_yaw += yaw_rate * timestep

        if done:
            goal_reached = True
            break

        step += 1

    print(f"[run] Simulation ended after {step+1} steps")
    if goal_reached:
        print(f"[run] SUCCESS: Goal reached at step {step+1}")
    else:
        print(f"[run] WARNING: Step limit reached without reaching goal")

    return goal_reached


def phase4_genesis(nav_map: NavigationMap, start_world: Tuple[float, float], goal_world: Tuple[float, float]):
    """PHASE 4 — Run in Genesis physics simulator if available."""
    print("\n=== PHASE 4: Genesis integration ===")

    try:
        import genesis as gs
        gs.init(backend=gs.cpu, logging_level="warning")

        print("[run] Genesis detected — initializing scene...")

        # Initialize scene with a camera aimed at the robot start area.
        scene = gs.Scene(
            show_viewer=True,
            sim_options=gs.options.SimOptions(dt=0.01),
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(start_world[0] + 2.5, start_world[1] - 2.5, 2.0),
                camera_lookat=(start_world[0], start_world[1], 0.5),
                camera_fov=40,
                max_FPS=60,
            ),
        )

        # Add ground plane
        scene.add_entity(gs.morphs.Plane())

        # Add wall entities for each wall cell in the raw grid
        raw_grid = nav_map.get_raw_grid()
        width_cells = nav_map.width_cells
        height_cells = nav_map.height_cells
        cell_size = nav_map.cell_size_m

        wall_height = 1.0  # meters
        wall_thickness = cell_size

        wall_count = 0
        for i in range(width_cells):
            for j in range(height_cells):
                if raw_grid[i, j] == 1:  # wall cell
                    wx, wy = nav_map.grid_to_world(i, j)
                    scene.add_entity(
                        gs.morphs.Box(
                            size=(wall_thickness, wall_thickness, wall_height),
                            pos=(wx, wy, wall_height / 2)
                        )
                    )
                    wall_count += 1
        print(f"[run] Added {wall_count} wall entities")

        # Load Go2 robot from the same Genesis asset path used by go2_nav_env.py.
        go2_urdf_path = "urdf/go2/urdf/go2.urdf"
        try:
            robot = scene.add_entity(
                gs.morphs.URDF(
                    file=go2_urdf_path,
                    pos=(start_world[0], start_world[1], 0.5),
                    euler=(0, 0, 0),
                    fixed=False
                )
            )
            print(f"[run] Go2 robot loaded from {go2_urdf_path}")
        except Exception as e:
            bundled_go2_urdf = os.path.join(
                os.path.dirname(__file__),
                "Genesis",
                "genesis",
                "assets",
                "urdf",
                "go2",
                "urdf",
                "go2.urdf",
            )
            print(f"[run] Genesis asset path failed: {e}")
            print(f"[run] Retrying Go2 load from bundled asset: {bundled_go2_urdf}")
            try:
                robot = scene.add_entity(
                    gs.morphs.URDF(
                        file=bundled_go2_urdf,
                        pos=(start_world[0], start_world[1], 0.5),
                        euler=(0, 0, 0),
                        fixed=False
                    )
                )
                print(f"[run] Go2 robot loaded from {bundled_go2_urdf}")
            except Exception as nested_e:
                print(f"[run] Failed to load Go2: {nested_e}")
                print("[run] Skipping Genesis simulation")
                return

        scene.build()

        # Set up navigation components
        planner = AStarPlanner(nav_map)
        controller = PurePursuitController()
        navigator = Navigator(nav_map, planner, controller)
        navigator.reset(start_world, goal_world)

        print("[run] Starting Genesis simulation loop...")
        steps = 3000
        for sim_step in range(steps):
            # Get robot pose
            pos = robot.get_pos()
            quat = robot.get_quat()  # (w, x, y, z)
            wx, wy, wz = float(pos[0]), float(pos[1]), float(pos[2])
            yaw = NavigatorNode._quaternion_to_yaw(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))

            # Step navigator
            forward_vel, lateral_vel, yaw_rate, done = navigator.step(wx, wy, yaw)
            world_vx = forward_vel * np.cos(yaw) - lateral_vel * np.sin(yaw)
            world_vy = forward_vel * np.sin(yaw) + lateral_vel * np.cos(yaw)

            # For a floating-base robot, the first six local DoFs drive the base.
            robot.set_dofs_velocity(
                [world_vx, world_vy, 0.0, 0.0, 0.0, yaw_rate],
                dofs_idx_local=slice(0, 6),
            )
            scene.step()

            if sim_step % 100 == 0:
                print(
                    f"[run] Step {sim_step}: pos=({wx:.2f}, {wy:.2f}), "
                    f"yaw={yaw:.2f}, cmd=({world_vx:.2f}, {world_vy:.2f}, {yaw_rate:.2f})"
                )

            if done:
                print(f"[run] Goal reached at step {sim_step}!")
                break

        print("[run] Genesis simulation complete")
        if scene.viewer is not None:
            input("[run] Press Enter to close Genesis viewer...")

    except ImportError as e:
        print(f"[run] Genesis not installed — skipping Phase 4")
        print(f"[run] Error: {e}")
        print("[run] To install: pip install genesis-world")
    except Exception as e:
        print(f"[run] Genesis phase 4 failed: {e}")
        print("[run] Skipping phase 4")


def calculate_path_length(path: List[Tuple[float, float]]) -> float:
    """Calculate total Euclidean length of a path."""
    length = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        length += np.sqrt(dx*dx + dy*dy)
    return length


def main():
    configure_console_output()
    args = parse_args()

    # PHASE 1: Build map
    nav_map, start_world, goal_world = phase1_build_map(args)

    # PHASE 2: Plan path
    planner = AStarPlanner(nav_map)
    path = phase2_planner(nav_map, start_world, goal_world)

    # PHASE 3: Mock simulation
    phase3_mock_simulation(nav_map, planner, start_world, goal_world)

    # PHASE 4: Genesis (optional)
    phase4_genesis(nav_map, start_world, goal_world)

    print("\n=== All phases completed ===")
    return 0


if __name__ == '__main__':
    sys.exit(main())
