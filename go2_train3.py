"""
go2_train.py — Training script for A* + RL hybrid locomotion.

High-level A* plans waypoint paths; low-level RL policy tracks them via PPO.

Observation (47-dim):
  ang_vel(3) | projected_gravity(3) | cmd(3) | dof_pos(12) | dof_vel(12) | actions(12) | wp_vec(2)

Rewards:
  + waypoint_tracking  : exp(-dist/sigma) for reaching A* waypoints
  + goal_progress      : binary bonus when within reach threshold
  + forward_progress   : velocity alignment with final goal direction
  - energy_efficiency  : penalize high torque * velocity
  - stability          : penalize body tilt
  - lin_vel_z          : penalize vertical body oscillation
  - action_rate        : penalize jerky actions
  - similar_to_default : encourage nominal joint pose
"""

import argparse
import os
import pickle
import shutil
from importlib import metadata

try:
    try:
        if metadata.version("rsl-rl"):
            raise ImportError
    except metadata.PackageNotFoundError:
        if metadata.version("rsl-rl-lib") != "2.2.4":
            raise ImportError
except (metadata.PackageNotFoundError, ImportError) as e:
    raise ImportError("Please uninstall 'rsl_rl' and install 'rsl-rl-lib==2.2.4'.") from e

from rsl_rl.runners import OnPolicyRunner
import genesis as gs
from go2_env import Go2Env


def get_train_cfg(exp_name, max_iterations):
    return {
        "algorithm": {
            "class_name": "PPO",
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 0.001,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
        },
        "init_member_classes": {},
        "policy": {
            "activation": "elu",
            "actor_hidden_dims": [512, 256, 128],
            "critic_hidden_dims": [512, 256, 128],
            "init_noise_std": 1.0,
            "class_name": "ActorCritic",
        },
        "runner": {
            "checkpoint": -1,
            "experiment_name": exp_name,
            "load_run": -1,
            "log_interval": 1,
            "max_iterations": max_iterations,
            "record_interval": -1,
            "resume": False,
            "resume_path": None,
            "run_name": "",
        },
        "runner_class_name": "OnPolicyRunner",
        "num_steps_per_env": 24,
        "save_interval": 100,
        "empirical_normalization": None,
        "seed": 1,
    }


def get_cfgs():
    env_cfg = {
        "num_actions": 12,
        # Joint defaults
        "default_joint_angles": {
            "FL_hip_joint": 0.0,
            "FR_hip_joint": 0.0,
            "RL_hip_joint": 0.0,
            "RR_hip_joint": 0.0,
            "FL_thigh_joint": 0.8,
            "FR_thigh_joint": 0.8,
            "RL_thigh_joint": 1.0,
            "RR_thigh_joint": 1.0,
            "FL_calf_joint": -1.5,
            "FR_calf_joint": -1.5,
            "RL_calf_joint": -1.5,
            "RR_calf_joint": -1.5,
        },
        "joint_names": [
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        ],
        # PD control
        "kp": 20.0,
        "kd": 0.5,
        # Termination thresholds
        "termination_if_roll_greater_than": 10,   # degrees
        "termination_if_pitch_greater_than": 10,
        "termination_if_height_lower_than": 0.15, # robot collapsed
        # Initial pose
        "base_init_pos": [0.0, 0.0, 0.42],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],
        # Episode
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        "action_scale": 0.25,
        "simulate_action_latency": True,
        "clip_actions": 100.0,
        # A* planner settings
        "astar_grid_resolution": 0.2,            # meters/cell
        "astar_grid_size": (50, 50),             # grid cells (10m × 10m world)
        "waypoint_reach_threshold": 0.3,         # meters: when to advance waypoint
        "goal_range": 3.0,                       # random goals within ±3 m
        # Optional static obstacles: list of [x, y, radius]
        "obstacles": [
            # Example: [1.0, 0.5, 0.4],
        ],
    }

    obs_cfg = {
        # 3 (ang_vel) + 3 (gravity) + 3 (cmd) + 12 (dof_pos) + 12 (dof_vel) + 12 (actions) + 2 (wp_vec)
        "num_obs": 47,
        "obs_scales": {
            "lin_vel":  2.0,
            "ang_vel":  0.25,
            "dof_pos":  1.0,
            "dof_vel":  0.05,
            "waypoint": 1.0,   # scale for relative waypoint vector
        },
    }

    reward_cfg = {
        # Shared sigma for velocity-tracking rewards
        "tracking_sigma": 0.25,
        # A*-specific
        "waypoint_tracking_sigma": 0.5,   # controls sharpness of waypoint reward
        # Targets
        "base_height_target": 0.3,
        "feet_height_target": 0.075,
        # Reward scales (positive = reward, negative = penalty; all multiplied by dt internally)
        "reward_scales": {
            # A* trajectory rewards
            "waypoint_tracking":  2.0,    # primary: follow A* path
            "goal_progress":      1.0,    # bonus for reaching each waypoint
            "forward_progress":   0.5,    # align velocity with final goal
            # Stability / safety penalties
            "stability":         -2.0,    # penalize body tilt
            "lin_vel_z":         -1.0,    # penalize vertical bouncing
            # Efficiency penalties
            "energy_efficiency": -0.001,  # penalize torque × velocity
            "action_rate":       -0.005,  # penalize jerky commands
            "similar_to_default": -0.05,  # soft regularization toward default pose
        },
    }

    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0.5, 0.5],
        "lin_vel_y_range": [0, 0],
        "ang_vel_range":   [0, 0],
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name",      type=str, default="go2-astar-rl")
    parser.add_argument("-B", "--num_envs",       type=int, default=4096)
    parser.add_argument("--max_iterations",       type=int, default=300)
    args = parser.parse_args()

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs()
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)

    if os.path.exists(log_dir):
        shutil.rmtree(log_dir)
    os.makedirs(log_dir, exist_ok=True)

    pickle.dump(
        [env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg],
        open(f"{log_dir}/cfgs.pkl", "wb"),
    )

    gs.init(
        backend=gs.gpu,
        precision="32",
        logging_level="warning",
        seed=train_cfg["seed"],
        performance_mode=True,
    )

    env = Go2Env(
        num_envs=args.num_envs,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)
    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()

"""
# Training (A* + RL)
python examples/locomotion/go2_train.py -e go2-astar-rl -B 4096 --max_iterations 300

# With custom obstacles
#   Edit env_cfg["obstacles"] in get_cfgs() before running.
"""
