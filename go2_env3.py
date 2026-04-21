import math
import torch

import genesis as gs
from genesis.utils.geom import quat_to_xyz, transform_by_quat, inv_quat, transform_quat_by_quat

from astar_planner import AStarPlanner, WaypointManager


def gs_rand(lower, upper, batch_shape):
    assert lower.shape == upper.shape
    return (upper - lower) * torch.rand(
        size=(*batch_shape, *lower.shape), dtype=gs.tc_float, device=gs.device
    ) + lower


class Go2Env:
    def __init__(self, num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg, show_viewer=False):
        self.num_envs = num_envs
        self.num_obs = obs_cfg["num_obs"]           
        self.num_privileged_obs = None
        self.num_actions = env_cfg["num_actions"]
        self.num_commands = command_cfg["num_commands"]
        self.device = gs.device

        self.simulate_action_latency = True
        self.dt = 0.02                              
        self.max_episode_length = math.ceil(env_cfg["episode_length_s"] / self.dt)

        self.env_cfg = env_cfg
        self.obs_cfg = obs_cfg
        self.reward_cfg = reward_cfg
        self.command_cfg = command_cfg

        self.obs_scales = obs_cfg["obs_scales"]
        self.reward_scales = reward_cfg["reward_scales"]

        # ------------------------------------------------------------------ #
        # A* planner (shared grid, per-env waypoint manager)
        # ------------------------------------------------------------------ #
        self.astar = AStarPlanner(
            grid_resolution=env_cfg.get("astar_grid_resolution", 0.2),
            grid_size=env_cfg.get("astar_grid_size", (50, 50)),
        )
        # Optionally seed some static obstacles from config
        for obs_pos in env_cfg.get("obstacles", []):
            self.astar.add_obstacle(obs_pos[0], obs_pos[1], radius=obs_pos[2] if len(obs_pos) > 2 else 0.3)

        self.waypoint_manager = WaypointManager(
            num_envs=num_envs,
            waypoint_reach_threshold=env_cfg.get("waypoint_reach_threshold", 0.3),
            device=gs.device,
        )

        # Goal positions per env [num_envs, 2] (x, y in world frame)
        self.goals = torch.zeros((num_envs, 2), dtype=gs.tc_float, device=gs.device)

        # ------------------------------------------------------------------ #
        # Genesis scene
        # ------------------------------------------------------------------ #
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=self.dt, substeps=2),
            rigid_options=gs.options.RigidOptions(
                enable_self_collision=False,
                tolerance=1e-5,
                max_collision_pairs=20,
            ),
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(2.0, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
                max_FPS=int(1.0 / self.dt),
            ),
            vis_options=gs.options.VisOptions(rendered_envs_idx=[0]),
            show_viewer=show_viewer,
        )

        self.scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file="urdf/go2/urdf/go2.urdf",
                pos=self.env_cfg["base_init_pos"],
                quat=self.env_cfg["base_init_quat"],
            )
        )
        self.scene.build(n_envs=num_envs)

        self.motors_dof_idx = torch.tensor(
            [self.robot.get_joint(name).dof_start for name in self.env_cfg["joint_names"]],
            dtype=gs.tc_int,
            device=gs.device,
        )
        self.actions_dof_idx = torch.argsort(self.motors_dof_idx)

        # PD gains
        self.robot.set_dofs_kp([self.env_cfg["kp"]] * self.num_actions, self.motors_dof_idx)
        self.robot.set_dofs_kv([self.env_cfg["kd"]] * self.num_actions, self.motors_dof_idx)

        # Gravity vector
        self.global_gravity = torch.tensor([0.0, 0.0, -1.0], dtype=gs.tc_float, device=gs.device)

        # Initial state tensors
        self.init_base_pos  = torch.tensor(self.env_cfg["base_init_pos"],  dtype=gs.tc_float, device=gs.device)
        self.init_base_quat = torch.tensor(self.env_cfg["base_init_quat"], dtype=gs.tc_float, device=gs.device)
        self.inv_base_init_quat = inv_quat(self.init_base_quat)
        self.init_dof_pos = torch.tensor(
            [self.env_cfg["default_joint_angles"][joint.name] for joint in self.robot.joints[1:]],
            dtype=gs.tc_float, device=gs.device,
        )
        self.init_qpos = torch.concatenate((self.init_base_pos, self.init_base_quat, self.init_dof_pos))
        self.init_projected_gravity = transform_by_quat(self.global_gravity, self.inv_base_init_quat)

        # Runtime buffers
        self.base_lin_vel       = torch.empty((num_envs, 3),                dtype=gs.tc_float, device=gs.device)
        self.base_ang_vel       = torch.empty((num_envs, 3),                dtype=gs.tc_float, device=gs.device)
        self.projected_gravity  = torch.empty((num_envs, 3),                dtype=gs.tc_float, device=gs.device)
        self.obs_buf            = torch.empty((num_envs, self.num_obs),      dtype=gs.tc_float, device=gs.device)
        self.rew_buf            = torch.empty((num_envs,),                   dtype=gs.tc_float, device=gs.device)
        self.reset_buf          = torch.ones ((num_envs,),                   dtype=gs.tc_bool,  device=gs.device)
        self.episode_length_buf = torch.empty((num_envs,),                   dtype=gs.tc_int,   device=gs.device)
        self.commands           = torch.empty((num_envs, self.num_commands), dtype=gs.tc_float, device=gs.device)
        self.commands_scale = torch.tensor(
            [self.obs_scales["lin_vel"], self.obs_scales["lin_vel"], self.obs_scales["ang_vel"]],
            device=gs.device, dtype=gs.tc_float,
        )
        self.commands_limits = [
            torch.tensor(values, dtype=gs.tc_float, device=gs.device)
            for values in zip(
                self.command_cfg["lin_vel_x_range"],
                self.command_cfg["lin_vel_y_range"],
                self.command_cfg["ang_vel_range"],
            )
        ]
        self.actions      = torch.zeros((num_envs, self.num_actions), dtype=gs.tc_float, device=gs.device)
        self.last_actions = torch.zeros_like(self.actions)
        self.dof_pos      = torch.empty_like(self.actions)
        self.dof_vel      = torch.empty_like(self.actions)
        self.last_dof_vel = torch.zeros_like(self.actions)
        self.base_pos     = torch.empty((num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.base_quat    = torch.empty((num_envs, 4), dtype=gs.tc_float, device=gs.device)
        self.default_dof_pos = torch.tensor(
            [self.env_cfg["default_joint_angles"][name] for name in self.env_cfg["joint_names"]],
            dtype=gs.tc_float, device=gs.device,
        )
        self.extras = {"observations": {}}

        # Reward bookkeeping
        self.reward_functions, self.episode_sums = {}, {}
        for name in self.reward_scales.keys():
            self.reward_scales[name] *= self.dt
            self.reward_functions[name] = getattr(self, "_reward_" + name)
            self.episode_sums[name] = torch.zeros((num_envs,), dtype=gs.tc_float, device=gs.device)

    # ---------------------------------------------------------------------- #
    # A* helpers
    # ---------------------------------------------------------------------- #
    def _plan_astar(self, env_idx):
        """Run A* for a single environment and load waypoints into the manager."""
        start_xy = (
            float(self.base_pos[env_idx, 0].cpu()),
            float(self.base_pos[env_idx, 1].cpu()),
        )
        goal_xy = (
            float(self.goals[env_idx, 0].cpu()),
            float(self.goals[env_idx, 1].cpu()),
        )
        raw_path = self.astar.plan(start_xy, goal_xy)
        if len(raw_path) < 2:
            # Fallback: straight line to goal if no path found
            raw_path = [start_xy, goal_xy]
        smoothed = self.astar.smooth_path(raw_path)
        self.waypoint_manager.set_plan(env_idx, smoothed)

    def _sample_goals(self, envs_idx=None):
        """Sample random goal positions for each env."""
        goal_range = self.env_cfg.get("goal_range", 3.0)
        if envs_idx is None:
            self.goals = (torch.rand((self.num_envs, 2), dtype=gs.tc_float, device=gs.device) - 0.5) * 2 * goal_range
        else:
            new_goals = (torch.rand((self.num_envs, 2), dtype=gs.tc_float, device=gs.device) - 0.5) * 2 * goal_range
            self.goals = torch.where(envs_idx[:, None], new_goals, self.goals)

    def _get_waypoint_vec(self):
        """
        Return relative vector from robot base to current waypoint, in world XY.
        Shape: [num_envs, 2]
        """
        current_wp = self.waypoint_manager.get_current_waypoint()   # [num_envs, 2]
        robot_xy   = self.base_pos[:, :2]                            # [num_envs, 2]
        return current_wp - robot_xy

    # ---------------------------------------------------------------------- #
    # Core step / observation / reset
    # ---------------------------------------------------------------------- #
    def step(self, actions):
        self.actions = torch.clip(actions, -self.env_cfg["clip_actions"], self.env_cfg["clip_actions"])
        exec_actions = self.last_actions if self.simulate_action_latency else self.actions
        target_dof_pos = exec_actions * self.env_cfg["action_scale"] + self.default_dof_pos
        self.robot.control_dofs_position(target_dof_pos[:, self.actions_dof_idx], slice(6, 18))
        self.scene.step()

        # Update state buffers
        self.episode_length_buf += 1
        self.base_pos  = self.robot.get_pos()
        self.base_quat = self.robot.get_quat()
        self.base_euler = quat_to_xyz(
            transform_quat_by_quat(self.inv_base_init_quat, self.base_quat), rpy=True, degrees=True
        )
        inv_base_quat       = inv_quat(self.base_quat)
        self.base_lin_vel   = transform_by_quat(self.robot.get_vel(), inv_base_quat)
        self.base_ang_vel   = transform_by_quat(self.robot.get_ang(), inv_base_quat)
        self.projected_gravity = transform_by_quat(self.global_gravity, inv_base_quat)
        self.dof_pos = self.robot.get_dofs_position(self.motors_dof_idx)
        self.dof_vel = self.robot.get_dofs_velocity(self.motors_dof_idx)

        # Waypoints for envs that reached current target
        self.waypoint_manager.update(self.base_pos[:, :2])

        # Compute rewards
        self.rew_buf.zero_()
        for name, reward_func in self.reward_functions.items():
            rew = reward_func() * self.reward_scales[name]
            self.rew_buf += rew
            self.episode_sums[name] += rew

        # Resample velocity commands (kept for API compatibility)
        self._resample_commands(self.episode_length_buf % int(self.env_cfg["resampling_time_s"] / self.dt) == 0)

        # Termination
        self.reset_buf  = self.episode_length_buf > self.max_episode_length
        self.reset_buf |= torch.abs(self.base_euler[:, 1]) > self.env_cfg["termination_if_pitch_greater_than"]
        self.reset_buf |= torch.abs(self.base_euler[:, 0]) > self.env_cfg["termination_if_roll_greater_than"]
        # Also terminate if base drops below a threshold (robot fell)
        self.reset_buf |= self.base_pos[:, 2] < self.env_cfg.get("termination_if_height_lower_than", 0.15)

        self.extras["time_outs"] = (self.episode_length_buf > self.max_episode_length).to(dtype=gs.tc_float)

        self._reset_idx(self.reset_buf)
        self._update_observation()

        self.last_actions.copy_(self.actions)
        self.last_dof_vel.copy_(self.dof_vel)

        self.extras["observations"]["critic"] = self.obs_buf
        return self.obs_buf, self.rew_buf, self.reset_buf, self.extras

    def get_observations(self):
        self.extras["observations"]["critic"] = self.obs_buf
        return self.obs_buf, self.extras

    def get_privileged_observations(self):
        return None

    def _resample_commands(self, envs_idx):
        self.commands = gs_rand(*self.commands_limits, (self.num_envs,))

    def _reset_idx(self, envs_idx=None):
        if envs_idx is None or (isinstance(envs_idx, torch.Tensor) and envs_idx.any()):
            # Reset physics state
            self.robot.set_qpos(self.init_qpos, envs_idx=envs_idx, zero_velocity=True, skip_forward=True)

            if envs_idx is None:
                self.base_pos.copy_(self.init_base_pos)
                self.base_quat.copy_(self.init_base_quat)
                self.projected_gravity.copy_(self.init_projected_gravity)
                self.dof_pos.copy_(self.init_dof_pos)
                self.base_lin_vel.zero_()
                self.base_ang_vel.zero_()
                self.dof_vel.zero_()
                self.actions.zero_()
                self.last_actions.zero_()
                self.last_dof_vel.zero_()
                self.episode_length_buf.zero_()
                self.reset_buf.fill_(True)
            else:
                torch.where(envs_idx[:, None], self.init_base_pos,        self.base_pos,        out=self.base_pos)
                torch.where(envs_idx[:, None], self.init_base_quat,       self.base_quat,       out=self.base_quat)
                torch.where(envs_idx[:, None], self.init_projected_gravity, self.projected_gravity, out=self.projected_gravity)
                torch.where(envs_idx[:, None], self.init_dof_pos,         self.dof_pos,         out=self.dof_pos)
                self.base_lin_vel.masked_fill_(envs_idx[:, None], 0.0)
                self.base_ang_vel.masked_fill_(envs_idx[:, None], 0.0)
                self.dof_vel.masked_fill_(envs_idx[:, None], 0.0)
                self.actions.masked_fill_(envs_idx[:, None], 0.0)
                self.last_actions.masked_fill_(envs_idx[:, None], 0.0)
                self.last_dof_vel.masked_fill_(envs_idx[:, None], 0.0)
                self.episode_length_buf.masked_fill_(envs_idx, 0)
                self.reset_buf.masked_fill_(envs_idx, True)

            # Sample new goals and re-plan A* for reset envs
            self._sample_goals(envs_idx)
            reset_indices = (
                range(self.num_envs) if envs_idx is None
                else envs_idx.nonzero(as_tuple=False).squeeze(-1).tolist()
            )
            if isinstance(reset_indices, range):
                reset_indices = list(reset_indices)
            for i in reset_indices:
                self._plan_astar(i)

            self.waypoint_manager.reset(envs_idx)

            # Episode logging
            n_envs = envs_idx.sum() if envs_idx is not None else self.num_envs
            self.extras["episode"] = {}
            for key, value in self.episode_sums.items():
                if envs_idx is None:
                    mean = value.mean()
                else:
                    mean = torch.where(n_envs > 0, value[envs_idx].sum() / n_envs, 0.0)
                self.extras["episode"]["rew_" + key] = mean / self.env_cfg["episode_length_s"]
                if envs_idx is None:
                    value.zero_()
                else:
                    value.masked_fill_(envs_idx, 0.0)

            self._resample_commands(envs_idx)

    def _update_observation(self):
        """
        Observation vector (47-dim):
          [0:3]   base angular velocity (scaled)
          [3:6]   projected gravity
          [6:9]   velocity commands (lin_x, lin_y, ang) — kept for API compat
          [9:21]  DOF positions (relative to default)
          [21:33] DOF velocities
          [33:45] last actions
          [45:47] relative waypoint vector (dx, dy) in world frame
        """
        wp_vec = self._get_waypoint_vec() * self.obs_scales.get("waypoint", 1.0)  # [num_envs, 2]

        self.obs_buf = torch.cat(
            [
                self.base_ang_vel  * self.obs_scales["ang_vel"],                    # 3
                self.projected_gravity,                                              # 3
                self.commands      * self.commands_scale,                            # 3
                (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"], # 12
                self.dof_vel       * self.obs_scales["dof_vel"],                    # 12
                self.actions,                                                        # 12
                wp_vec,                                                              # 2  ← A* waypoint
            ],
            dim=-1,
        )

    def reset(self):
        self._reset_idx(None)
        self._update_observation()
        return self.obs_buf, None

    # ---------------------------------------------------------------------- #
    # Reward functions
    # ---------------------------------------------------------------------- #

    def _reward_tracking_lin_vel(self):
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_tracking_ang_vel(self):
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_lin_vel_z(self):
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_action_rate(self):
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    def _reward_similar_to_default(self):
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1)

    def _reward_base_height(self):
        return torch.square(self.base_pos[:, 2] - self.reward_cfg["base_height_target"])

    # New A*-specific reward functions 

    def _reward_waypoint_tracking(self):
        """
        Goal Tracking: reward proximity to current A* waypoint.
        Uses an exponential kernel so reward is highest when the robot is closest.
        """
        wp_vec = self._get_waypoint_vec()                       # [N, 2]
        dist   = torch.norm(wp_vec, dim=-1)                     # [N]
        sigma  = self.reward_cfg.get("waypoint_tracking_sigma", 0.5)
        return torch.exp(-dist / sigma)

    def _reward_goal_progress(self):
        """
        Bonus reward each time the robot advances to a new waypoint.
        This is approximated by checking how close to the current waypoint we are.
        """
        wp_vec = self._get_waypoint_vec()
        dist   = torch.norm(wp_vec, dim=-1)
        # Binary bonus when inside reach threshold
        reached = (dist < self.waypoint_manager.threshold).to(dtype=gs.tc_float)
        return reached

    def _reward_energy_efficiency(self):
        """
        Energy Efficiency: penalize high torque / power usage.
        Proxy: sum of squared joint velocities * joint position changes.
        """
        return torch.sum(torch.square(self.dof_vel) * torch.square(self.actions), dim=1)

    def _reward_stability(self):
        """
        Stability: penalize large body orientation deviations.
        Uses the projected gravity to measure tilt from upright.
        Perfect upright: projected_gravity = [0, 0, -1] → deviation = 0.
        """
        # |projected_gravity + [0,0,1]|^2 ≈ 0 when upright
        upright = torch.tensor([0.0, 0.0, -1.0], device=gs.device, dtype=gs.tc_float)
        tilt = torch.sum(torch.square(self.projected_gravity - upright), dim=1)
        return tilt

    def _reward_forward_progress(self):
        """
        Encourage the robot to move toward the final goal (not just current waypoint).
        Uses the dot product between base linear velocity and goal direction.
        """
        goal_vec  = self.goals - self.base_pos[:, :2]            
        goal_dist = torch.norm(goal_vec, dim=-1, keepdim=True).clamp(min=1e-3)
        goal_dir  = goal_vec / goal_dist                          
        forward_vel = torch.sum(self.base_lin_vel[:, :2] * goal_dir, dim=-1)
        return forward_vel.clamp(min=0.0)
