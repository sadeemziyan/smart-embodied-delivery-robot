# smart-embodied-delivery-robot

Smart Embodied Delivery Robot - Genesis Simulation

## Project Overview

This repo contains a grid-based delivery robot navigation stack plus a vendored Genesis checkout for Go2 locomotion experiments and visualization.

## Main Project Files

- `run.py`: map generation, A* planning, and mock/Genesis integration entry point
- `run_visual.py`: visual runner that integrates navigation with the Genesis Go2 environment
- `go2_nav_env.py`: custom Genesis-based Go2 navigation environment
- `navigation/`: map, planner, controller, and navigator modules
- `Genesis/`: vendored Genesis source with local compatibility fixes used by this project

## Handoff Notes

- This repo intentionally excludes local logs, checkpoints, debug images, caches, and tool state.
- Genesis is included as source code, not as a submodule, so collaborators can run the exact patched version in this repo.
- The current machine was CPU-only during development, so GPU collaborators should be able to train and evaluate much faster.

## Suggested Setup

Install Python dependencies from the project root:

```powershell
python -m pip install -r requirements.txt
python -m pip install tensorboard rsl-rl-lib==2.2.4
```

## Training / Eval

Train the official Genesis locomotion example:

```powershell
python .\Genesis\examples\locomotion\go2_train.py -B 32 --max_iterations 30
```

Evaluate the latest checkpoint:

```powershell
python .\Genesis\examples\locomotion\go2_eval.py --ckpt -1 --max_steps 1500 --print_every 100
```

Run the project visual integration:

```powershell
python .\run_visual.py
```
