"""
go2_eval.py — Evaluation script for the A* + RL hybrid locomotion policy.

Loads a trained checkpoint and runs the policy with live A* replanning.
Press Ctrl+C to stop. The viewer shows the first environment.
"""

import argparse
import os
import pickle
from importlib import metadata

import torch

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="go2-astar-rl")
    parser.add_argument("--ckpt",           type=int, default=100)
    args = parser.parse_args()

    gs.init(backend=gs.cpu)

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(
        open(f"{log_dir}/cfgs.pkl", "rb")
    )

    # Disable all reward scales during eval (policy is deterministic)
    reward_cfg["reward_scales"] = {}

    env = Go2Env(
        num_envs=1,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        show_viewer=True,
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)
    resume_path = os.path.join(log_dir, f"model_{args.ckpt}.pt")
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=gs.device)

    obs, _ = env.reset()
    step = 0
    with torch.no_grad():
        while True:
            actions = policy(obs)
            obs, rews, dones, infos = env.step(actions)
            step += 1

            # Print A* progress every 100 steps
            if step % 100 == 0:
                wp_idx  = env.waypoint_manager.waypoint_idx[0].item()
                n_wp    = env.waypoint_manager.num_waypoints[0].item()
                wp_vec  = env.waypoint_manager.get_current_waypoint()[0]
                goal    = env.goals[0]
                pos     = env.base_pos[0]
                dist_to_goal = torch.norm(pos[:2] - goal).item()
                print(
                    f"[step {step:5d}] "
                    f"waypoint {wp_idx}/{n_wp} | "
                    f"next wp: ({wp_vec[0]:.2f}, {wp_vec[1]:.2f}) | "
                    f"goal: ({goal[0]:.2f}, {goal[1]:.2f}) | "
                    f"dist_to_goal: {dist_to_goal:.2f} m | "
                    f"rew: {rews[0].item():.3f}"
                )


if __name__ == "__main__":
    main()

"""
# Evaluation
python examples/locomotion/go2_eval.py -e go2-astar-rl --ckpt 100
python examples/locomotion/go2_eval.py -e go2-astar-rl --ckpt 300
"""
