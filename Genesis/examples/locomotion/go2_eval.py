import argparse
import os
import pickle
from glob import glob
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


def find_latest_checkpoint(log_dir: str):
    model_paths = glob(os.path.join(log_dir, "model_*.pt"))
    checkpoints = []
    for model_path in model_paths:
        stem = os.path.splitext(os.path.basename(model_path))[0]
        try:
            checkpoints.append((int(stem.split("_")[-1]), model_path))
        except ValueError:
            continue
    if not checkpoints:
        return None, None
    return max(checkpoints, key=lambda item: item[0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="go2-walking")
    parser.add_argument("--ckpt", type=int, default=-1)
    parser.add_argument("--max_steps", type=int, default=3000)
    parser.add_argument("--print_every", type=int, default=100)
    args = parser.parse_args()

    gs.init(backend=gs.cpu)

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(open(f"logs/{args.exp_name}/cfgs.pkl", "rb"))
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
    if args.ckpt >= 0:
        ckpt_num = args.ckpt
        resume_path = os.path.join(log_dir, f"model_{ckpt_num}.pt")
    else:
        ckpt_num, resume_path = find_latest_checkpoint(log_dir)
        if resume_path is None:
            raise FileNotFoundError(f"No checkpoint files found in {log_dir}")
        print(f"[go2_eval] Using latest checkpoint: model_{ckpt_num}.pt")
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=gs.device)

    obs, _ = env.reset()
    start_pos = None
    with torch.no_grad():
        for step in range(args.max_steps):
            actions = policy(obs)
            obs, rews, dones, infos = env.step(actions)
            pos = env.robot.get_pos()
            if start_pos is None:
                start_pos = pos.clone()
            if step % args.print_every == 0:
                dx = float(pos[0, 0] - start_pos[0, 0])
                dy = float(pos[0, 1] - start_pos[0, 1])
                print(f"[go2_eval] step={step:4d} pos=({float(pos[0,0]):.2f},{float(pos[0,1]):.2f}) delta=({dx:.2f},{dy:.2f})")
        input("[go2_eval] Evaluation finished. Press Enter to close the viewer...")


if __name__ == "__main__":
    main()

"""
# evaluation
python examples/locomotion/go2_eval.py -e go2-walking -v --ckpt 100
"""
