#!/usr/bin/env python3
"""
Standalone Genesis demo that makes the Go2 visibly move its legs.

This ignores navigation and RL entirely and drives the leg joints directly so
the viewer shows clear quadruped motion right away.
"""

import math
import os
import sys

import numpy as np
import torch


def configure_console_output() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if stream is None or not hasattr(stream, "reconfigure"):
            continue
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
GENESIS_CANDIDATES = [
    os.path.join(BASE_DIR, "Genesis"),
    os.path.join(BASE_DIR, "..", "Genesis"),
]
GENESIS_PATH = next(
    (os.path.abspath(path) for path in GENESIS_CANDIDATES if os.path.exists(path)),
    None,
)
if GENESIS_PATH is None:
    checked_paths = ", ".join(os.path.abspath(path) for path in GENESIS_CANDIDATES)
    raise FileNotFoundError(f"Genesis repo not found. Checked: {checked_paths}")

sys.path.insert(0, GENESIS_PATH)

import genesis as gs
JOINT_NAMES = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]

DEFAULT_JOINT_ANGLES = np.array(
    [
        0.0,
        0.8,
        -1.5,
        0.0,
        0.8,
        -1.5,
        0.0,
        1.0,
        -1.5,
        0.0,
        1.0,
        -1.5,
    ],
    dtype=np.float32,
)


def make_joint_targets(t: float) -> np.ndarray:
    """Simple trot-like oscillation for the 12 Go2 joints."""
    freq_hz = 1.2
    phase = 2.0 * math.pi * freq_hz * t
    phase_a = math.sin(phase)
    phase_b = math.sin(phase + math.pi)

    targets = DEFAULT_JOINT_ANGLES.copy()

    leg_phases = {
        "FR": phase_a,
        "FL": phase_b,
        "RR": phase_b,
        "RL": phase_a,
    }

    leg_offsets = {
        "FR": 0,
        "FL": 3,
        "RR": 6,
        "RL": 9,
    }

    for leg, offset in leg_offsets.items():
        s = leg_phases[leg]
        targets[offset + 0] += 0.18 * s
        targets[offset + 1] += 0.40 * s
        targets[offset + 2] -= 0.65 * s

    return targets


def main() -> int:
    configure_console_output()

    backend = gs.gpu if torch.cuda.is_available() else gs.cpu
    gs.init(backend=backend, precision="32", logging_level="warning")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=0.02),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(1.8, -1.6, 1.2),
            camera_lookat=(0.0, 0.0, 0.30),
            camera_fov=45,
            max_FPS=60,
        ),
        show_viewer=True,
    )

    scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))

    robot = scene.add_entity(
        gs.morphs.URDF(
            file="urdf/go2/urdf/go2.urdf",
            pos=(0.0, 0.0, 0.42),
            quat=(1.0, 0.0, 0.0, 0.0),
            fixed=True,
        )
    )

    scene.build()

    motors_dof_idx = [robot.get_joint(name).dofs_idx_local[0] for name in JOINT_NAMES]

    robot.set_dofs_kp([40.0] * len(motors_dof_idx), motors_dof_idx)
    robot.set_dofs_kv([2.0] * len(motors_dof_idx), motors_dof_idx)
    robot.set_dofs_position(DEFAULT_JOINT_ANGLES, motors_dof_idx)

    print("[go2_demo] Running grounded leg-motion demo...")

    steps = 4000
    dt = 0.02

    for step in range(steps):
        t = step * dt

        joint_targets = make_joint_targets(t)
        robot.control_dofs_position(joint_targets, motors_dof_idx)
        scene.step()

        if step % 200 == 0:
            print(
                f"[go2_demo] step={step:4d} "
                f"hip={joint_targets[0]:.2f} "
                f"thigh={joint_targets[1]:.2f} "
                f"calf={joint_targets[2]:.2f}"
            )

    input("[go2_demo] Demo finished. Press Enter to close the viewer...")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
