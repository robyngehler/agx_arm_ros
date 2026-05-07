from __future__ import annotations

from pathlib import Path

from trajectory_msgs.msg import JointTrajectory

from .trajectory_io import load_recorded_trajectory, recorded_to_joint_trajectory


def load_joint_trajectory(file_path: str | Path) -> JointTrajectory:
    return recorded_to_joint_trajectory(load_recorded_trajectory(file_path))