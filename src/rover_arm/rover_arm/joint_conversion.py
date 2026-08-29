"""Pure radians<->motor-steps conversion for the 5-axis arm, kept free
of ROS dependencies so it's testable without rclpy - same pure-logic/
thin-IO split used throughout this workspace (e.g. rover_base/odometry.py).

Calibration source: rover_arm/config/arm_topology.yaml's
steps_per_joint_rev (200 full steps * 1/16 microstepping * 5:1 gear =
16000, per joint - see that file's own comment for the reasoning).
That parameter existed as declared-but-unused placeholder data before
this module gave it something to actually do: converting MoveIt's
planned trajectories (radians, relative to each joint's homed zero -
see arm_mega2.ino's homing sequence) into the joint_target_steps this
rover's firmware has always expected.
"""

from __future__ import annotations

import math
from typing import List


def radians_to_steps(radians: float, steps_per_rev: int) -> int:
    """Converts an absolute joint angle in radians to the nearest whole
    motor step, relative to the homed zero position.
    """
    return int(round(radians / (2.0 * math.pi) * steps_per_rev))


def steps_to_radians(steps: int, steps_per_rev: int) -> float:
    """Inverse of radians_to_steps."""
    return (steps / steps_per_rev) * 2.0 * math.pi


def joint_radians_to_steps(joint_radians: List[float], steps_per_joint_rev: List[int]) -> List[int]:
    """Vector form of radians_to_steps - one steps_per_rev value per
    joint, since each joint's gearing/microstepping may differ.
    """
    if len(joint_radians) != len(steps_per_joint_rev):
        raise ValueError(
            f"joint_radians has {len(joint_radians)} entries but "
            f"steps_per_joint_rev has {len(steps_per_joint_rev)}"
        )
    return [radians_to_steps(r, s) for r, s in zip(joint_radians, steps_per_joint_rev)]


def joint_steps_to_radians(joint_steps: List[int], steps_per_joint_rev: List[int]) -> List[float]:
    """Inverse of joint_radians_to_steps."""
    if len(joint_steps) != len(steps_per_joint_rev):
        raise ValueError(
            f"joint_steps has {len(joint_steps)} entries but "
            f"steps_per_joint_rev has {len(steps_per_joint_rev)}"
        )
    return [steps_to_radians(s, spr) for s, spr in zip(joint_steps, steps_per_joint_rev)]


def reorder_by_name(names: List[str], values: List[float], canonical_order: List[str]) -> List[float]:
    """Reorders `values` (given in the order of `names`) into
    `canonical_order`.

    Exists because trajectory_msgs/JointTrajectory doesn't guarantee
    joint_names arrives in any particular order - MoveIt sends them in
    whatever order the planning group was defined in the SRDF, which
    need not match this rover's own joint_names order
    (rover_arm/config/arm_topology.yaml, which is also the order
    firmware/arm_mega2.ino expects joint_target_steps in). Getting
    this wrong would silently send a correct-looking but wrong-joint
    command - same shape, scrambled meaning - so it's covered by its
    own dedicated tests rather than assumed correct by inspection.
    """
    if len(names) != len(values):
        raise ValueError(f"names has {len(names)} entries but values has {len(values)}")
    if set(names) != set(canonical_order):
        raise ValueError(f"names {sorted(names)} does not match canonical_order {sorted(canonical_order)}")
    index_by_name = {name: i for i, name in enumerate(names)}
    return [values[index_by_name[name]] for name in canonical_order]
