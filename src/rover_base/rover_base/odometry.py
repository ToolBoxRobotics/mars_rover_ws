"""Wheel odometry from the base's two fixed middle wheels (ML, MR).

Only ML/MR are used, deliberately, not all six wheels: they're the
only pair whose rolling direction never changes with the corner
steering angle (FL/FR/RL/RR all point wherever ACKERMANN/POINT_TURN
currently steered them), so they're the only pair standard
differential-drive odometry math applies to directly. This is exactly
the plan flagged in README.md's former "no wheel odometry yet" gap.

This math is exact for a rigid, zero-slip vehicle in both of the
base's drive modes (ACKERMANN and POINT_TURN): ML/MR's rolling axis is
always the vehicle's own forward axis in either one, so a wheel
encoder fully observes the resulting motion - same zero-slip caveat as
the forward kinematics in kinematics.py, no mode-specific gaps.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class OdometryConfig:
    wheel_radius_m: float
    track_m: float
    encoder_ticks_per_rev: int


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float  # radians, normalized to (-pi, pi]


@dataclass(frozen=True)
class OdometryUpdate:
    pose: Pose2D
    linear_x_mps: float
    angular_z_radps: float


def ticks_to_distance(ticks: int, cfg: OdometryConfig) -> float:
    """Arc length (meters) a wheel has rolled for a given tick delta.
    Sign follows the tick sign - a wheel spinning "backward" per the
    base's own encoder convention yields a negative distance.
    """
    wheel_circumference = 2.0 * math.pi * cfg.wheel_radius_m
    return (ticks / cfg.encoder_ticks_per_rev) * wheel_circumference


def normalize_angle(theta: float) -> float:
    """Wraps an angle to (-pi, pi]."""
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta <= -math.pi:
        theta += 2.0 * math.pi
    return theta


def integrate_odometry(
    pose: Pose2D,
    ml_delta_ticks: int,
    mr_delta_ticks: int,
    dt_sec: float,
    cfg: OdometryConfig,
) -> OdometryUpdate:
    """Advances `pose` by one encoder update using the standard exact
    differential-drive integration (midpoint heading, not just the
    starting heading, for better accuracy over a finite time step).

    Sign convention matches rover_base.kinematics.twist_to_wheel_commands:
    ML sits at +y (left), MR at -y (right); a positive angular_z (left/
    CCW turn) speeds MR up and slows ML down, so MR rolling further
    than ML in a given step means the vehicle turned left (positive
    delta_theta) - exactly mirroring how the forward kinematics
    produces that same motion from a commanded twist.
    """
    delta_left = ticks_to_distance(ml_delta_ticks, cfg)
    delta_right = ticks_to_distance(mr_delta_ticks, cfg)

    delta_center = (delta_left + delta_right) / 2.0
    delta_theta = (delta_right - delta_left) / cfg.track_m

    mid_theta = pose.theta + delta_theta / 2.0
    new_pose = Pose2D(
        x=pose.x + delta_center * math.cos(mid_theta),
        y=pose.y + delta_center * math.sin(mid_theta),
        theta=normalize_angle(pose.theta + delta_theta),
    )

    linear_x = delta_center / dt_sec if dt_sec > 0 else 0.0
    angular_z = delta_theta / dt_sec if dt_sec > 0 else 0.0

    return OdometryUpdate(pose=new_pose, linear_x_mps=linear_x, angular_z_radps=angular_z)


def quaternion_from_yaw(theta: float):
    """Pure-yaw Euler-to-quaternion. Returns (x, y, z, w)."""
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


def diagonal_covariance(diagonal_values):
    """Builds a flat, row-major 6x6 covariance matrix (as ROS message
    covariance fields expect) with the given 6 values on the diagonal
    and zero everywhere else. Used by odometry_node.py to publish
    explicit covariance rather than the message-default all-zeros -
    see that module's docstring for why an ambiguous zero covariance
    matters for this rover specifically.
    """
    matrix = [0.0] * 36
    for i, value in enumerate(diagonal_values):
        matrix[i * 6 + i] = value
    return matrix
