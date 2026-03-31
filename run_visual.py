"""
Delivery robot visual demo.
Run: python run_visual.py

Shows Unitree Go2 navigating through a procedurally generated
building using A* path planning and pure pursuit control,
rendered live in the Genesis physics viewer.
"""

import sys
import os
import pickle
from glob import glob
import torch
import numpy as np
import genesis as gs

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
GENESIS_CANDIDATES = [
    os.path.join(BASE_DIR, 'Genesis'),
    os.path.join(BASE_DIR, '..', 'Genesis'),
]
GENESIS_PATH = next(
    (os.path.abspath(path) for path in GENESIS_CANDIDATES if os.path.exists(path)),
    None,
)
if GENESIS_PATH is None:
    checked_paths = ', '.join(os.path.abspath(path) for path in GENESIS_CANDIDATES)
    raise FileNotFoundError(f'Genesis repo not found. Checked: {checked_paths}')

sys.path.insert(0, GENESIS_PATH)
sys.path.insert(0, os.path.join(GENESIS_PATH, 'examples', 'locomotion'))

from grid_generator import GridGenerator
from navigation.map import NavigationMap
from navigation.planner import AStarPlanner
from navigation.controller import PurePursuitController
from navigation.navigator import Navigator
from go2_nav_env import Go2NavEnv

CHECKPOINT_CANDIDATES = [
    os.path.join(BASE_DIR, 'logs', 'go2-walking'),
    os.path.join(GENESIS_PATH, 'logs', 'go2-walking'),
]
CHECKPOINT_NUM  = 100


def load_policy(env, checkpoint_path: str, ckpt_num: int, device: str):
    """
    Load trained PPO locomotion policy from checkpoint.
    Reads train_cfg from go2_train.py to match training config exactly.
    """
    from rsl_rl.runners import OnPolicyRunner
    cfgs_path = os.path.join(checkpoint_path, 'cfgs.pkl')
    if not os.path.exists(cfgs_path):
        raise FileNotFoundError(f'Missing training config snapshot: {cfgs_path}')

    _, _, _, _, train_cfg = pickle.load(open(cfgs_path, 'rb'))
    runner = OnPolicyRunner(env, train_cfg, checkpoint_path, device=device)
    ckpt_file = os.path.join(checkpoint_path, f'model_{ckpt_num}.pt')
    runner.load(ckpt_file)
    return runner.get_inference_policy(device=device)


def find_latest_checkpoint(checkpoint_path: str):
    """Return the highest-numbered checkpoint in a Genesis log directory."""
    model_paths = glob(os.path.join(checkpoint_path, 'model_*.pt'))
    checkpoints = []
    for model_path in model_paths:
        stem = os.path.splitext(os.path.basename(model_path))[0]
        try:
            checkpoints.append((int(stem.split('_')[-1]), model_path))
        except ValueError:
            continue
    if not checkpoints:
        return None, None

    return max(checkpoints, key=lambda item: item[0])


def resolve_checkpoint_path():
    """Prefer the first checkpoint directory that already has training artifacts."""
    for checkpoint_path in CHECKPOINT_CANDIDATES:
        if os.path.exists(os.path.join(checkpoint_path, 'cfgs.pkl')):
            return checkpoint_path
    return CHECKPOINT_CANDIDATES[0]


def main() -> None:
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f'[run_visual] Device: {device}')

    # ------------------------------------------------------------ #
    # Phase 1 — map
    # ------------------------------------------------------------ #
    print('\n[run_visual] Phase 1: Building navigation map...')
    gen     = GridGenerator(width=20, height=20, cell_size=0.5)
    nav_map = NavigationMap(robot_radius_m=0.5)
    nav_map.build(gen)

    start_world = nav_map.get_simulated_start_world()
    goal_world  = nav_map.get_simulated_goal_world()
    print(f'[run_visual] Start: {start_world}')
    print(f'[run_visual] Goal:  {goal_world}')

    # ------------------------------------------------------------ #
    # Phase 2 — plan
    # ------------------------------------------------------------ #
    print('\n[run_visual] Phase 2: Planning A* path...')
    planner = AStarPlanner(nav_map)
    path    = planner.plan(start_world, goal_world)
    if not path:
        print('[run_visual] No path found — exiting')
        sys.exit(1)
    planner.visualize(path)
    print(f'[run_visual] {len(path)} waypoints, saved planner_debug.png')

    # ------------------------------------------------------------ #
    # Phase 3 — boot Genesis
    # ------------------------------------------------------------ #
    print('\n[run_visual] Phase 3: Booting Genesis...')
    import importlib.util
    train_script = os.path.join(
        GENESIS_PATH, 'examples', 'locomotion', 'go2_train.py'
    )
    spec = importlib.util.spec_from_file_location('go2_train', train_script)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    env_cfg, obs_cfg, reward_cfg, command_cfg = mod.get_cfgs()
    gs_backend = gs.gpu if device == 'cuda' else gs.cpu
    gs.init(backend=gs_backend, precision='32', logging_level='warning')
    env = Go2NavEnv(
        num_envs=1,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        show_viewer=True,
        nav_map=nav_map,
    )

    # ------------------------------------------------------------ #
    # Phase 4 — load policy
    # ------------------------------------------------------------ #
    print('\n[run_visual] Phase 4: Loading locomotion policy...')
    checkpoint_path = resolve_checkpoint_path()
    requested_ckpt = os.path.join(checkpoint_path, f'model_{CHECKPOINT_NUM}.pt')
    latest_ckpt_num, latest_ckpt_path = find_latest_checkpoint(checkpoint_path)
    if not os.path.exists(requested_ckpt):
        if latest_ckpt_path is None:
            print('[run_visual] No checkpoint found. Train first:')
            print(f'    cd {GENESIS_PATH}')
            print('    python examples\\locomotion\\go2_train.py')
            print(f'[run_visual] Expected checkpoint path: {requested_ckpt}')
            print(f'[run_visual] Expected config snapshot: {os.path.join(checkpoint_path, "cfgs.pkl")}')
            sys.exit(1)

        print(f'[run_visual] Requested checkpoint missing: {requested_ckpt}')
        print(f'[run_visual] Falling back to latest available checkpoint: {latest_ckpt_path}')
        ckpt_num = latest_ckpt_num
    else:
        ckpt_num = CHECKPOINT_NUM

    policy = load_policy(env, checkpoint_path, ckpt_num, device)
    print('[run_visual] Policy loaded')

    # ------------------------------------------------------------ #
    # Phase 5 — navigator
    # ------------------------------------------------------------ #
    print('\n[run_visual] Phase 5: Setting up navigator...')
    controller = PurePursuitController(
        lookahead_distance = 1.0,
        max_linear_vel     = 0.5,
        max_angular_vel    = 0.8,
        goal_threshold     = 0.5,
    )
    navigator = Navigator(nav_map, planner, controller,
                          replan_interval=100)
    obs, _  = env.reset()
    smoke_test_command = (0.5, 0.0, 0.0)
    env.set_nav_command(*smoke_test_command)
    navigator.reset(start_world, goal_world)

    # ------------------------------------------------------------ #
    # Phase 6 — sim loop
    # ------------------------------------------------------------ #
    print('\n[run_visual] Phase 6: Running — watch the viewer window')
    max_steps = 10000
    step      = 0
    nav_done  = False

    while step < max_steps and not nav_done:
        wx, wy, yaw           = env.get_robot_world_pose()
        vx, vy, yr = smoke_test_command
        env.set_nav_command(vx, vy, yr)

        with torch.no_grad():
            actions = policy(obs)
        obs, _, done, _ = env.step(actions)

        if done.any():
            print(f'[run_visual] Robot fell at step {step} — resetting')
            obs, _ = env.reset()
            env.set_nav_command(*smoke_test_command)
            navigator.reset(start_world, goal_world)

        if step % 200 == 0:
            dist = np.hypot(wx - goal_world[0], wy - goal_world[1])
            print(f'[run_visual] step={step:5d} | '
                  f'pos=({wx:.2f},{wy:.2f}) | '
                  f'cmd=vx={vx:.2f} yr={yr:.2f} | '
                  f'dist={dist:.2f}m')
        step += 1

    print(f'\n[run_visual] Smoke test finished after {step} steps')


if __name__ == '__main__':
    main()
