import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from rover_base.odometry import (
    OdometryConfig,
    Pose2D,
    diagonal_covariance,
    integrate_odometry,
    normalize_angle,
    quaternion_from_yaw,
    ticks_to_distance,
)

CFG = OdometryConfig(wheel_radius_m=0.075, track_m=0.46, encoder_ticks_per_rev=663)
ORIGIN = Pose2D(0.0, 0.0, 0.0)


def test_ticks_to_distance_one_full_revolution():
    distance = ticks_to_distance(663, CFG)
    assert distance == pytest.approx(2.0 * math.pi * 0.075)


def test_ticks_to_distance_negative_ticks_negative_distance():
    assert ticks_to_distance(-663, CFG) == pytest.approx(-2.0 * math.pi * 0.075)


def test_ticks_to_distance_zero_is_zero():
    assert ticks_to_distance(0, CFG) == 0.0


# -- straight-line motion ---------------------------------------------------
def test_straight_line_equal_deltas_moves_forward_no_rotation():
    update = integrate_odometry(ORIGIN, ml_delta_ticks=100, mr_delta_ticks=100, dt_sec=1.0, cfg=CFG)
    expected_distance = ticks_to_distance(100, CFG)
    assert update.pose.x == pytest.approx(expected_distance)
    assert update.pose.y == pytest.approx(0.0, abs=1e-9)
    assert update.pose.theta == pytest.approx(0.0)
    assert update.linear_x_mps == pytest.approx(expected_distance)  # dt=1
    assert update.angular_z_radps == pytest.approx(0.0)


def test_straight_line_from_nonzero_heading_moves_along_that_heading():
    start = Pose2D(x=1.0, y=2.0, theta=math.pi / 2.0)  # facing +y
    update = integrate_odometry(start, ml_delta_ticks=100, mr_delta_ticks=100, dt_sec=1.0, cfg=CFG)
    expected_distance = ticks_to_distance(100, CFG)
    assert update.pose.x == pytest.approx(1.0, abs=1e-9)  # cos(90deg)=0, no x change
    assert update.pose.y == pytest.approx(2.0 + expected_distance)
    assert update.pose.theta == pytest.approx(math.pi / 2.0)


# -- pure rotation -----------------------------------------------------------
def test_pure_rotation_matches_kinematics_sign_convention():
    # Mirrors twist_to_wheel_commands(0, +w, geo): positive angular_z
    # (left/CCW turn) drives ML backward and MR forward. Feed the same
    # sign pattern in as encoder deltas and confirm delta_theta comes
    # out positive (a left turn), not negative.
    update = integrate_odometry(ORIGIN, ml_delta_ticks=-50, mr_delta_ticks=50, dt_sec=1.0, cfg=CFG)
    assert update.angular_z_radps > 0
    # Symmetric pivot: net translation should be zero (delta_center = 0).
    assert update.pose.x == pytest.approx(0.0, abs=1e-9)
    assert update.pose.y == pytest.approx(0.0, abs=1e-9)


def test_pure_rotation_opposite_sign_turns_right():
    update = integrate_odometry(ORIGIN, ml_delta_ticks=50, mr_delta_ticks=-50, dt_sec=1.0, cfg=CFG)
    assert update.angular_z_radps < 0


def test_pure_rotation_magnitude_matches_hand_computed_value():
    delta_left = ticks_to_distance(-50, CFG)
    delta_right = ticks_to_distance(50, CFG)
    expected_delta_theta = (delta_right - delta_left) / CFG.track_m
    update = integrate_odometry(ORIGIN, ml_delta_ticks=-50, mr_delta_ticks=50, dt_sec=1.0, cfg=CFG)
    assert update.pose.theta == pytest.approx(expected_delta_theta)
    assert update.angular_z_radps == pytest.approx(expected_delta_theta)  # dt=1


# -- combined arc -------------------------------------------------------------
def test_arc_motion_matches_hand_computed_values():
    ml, mr = 80, 120
    delta_left = ticks_to_distance(ml, CFG)
    delta_right = ticks_to_distance(mr, CFG)
    delta_center = (delta_left + delta_right) / 2.0
    delta_theta = (delta_right - delta_left) / CFG.track_m
    mid_theta = 0.0 + delta_theta / 2.0
    expected_x = delta_center * math.cos(mid_theta)
    expected_y = delta_center * math.sin(mid_theta)

    update = integrate_odometry(ORIGIN, ml_delta_ticks=ml, mr_delta_ticks=mr, dt_sec=0.5, cfg=CFG)
    assert update.pose.x == pytest.approx(expected_x)
    assert update.pose.y == pytest.approx(expected_y)
    assert update.pose.theta == pytest.approx(delta_theta)
    assert update.linear_x_mps == pytest.approx(delta_center / 0.5)
    assert update.angular_z_radps == pytest.approx(delta_theta / 0.5)


def test_zero_dt_does_not_divide_by_zero():
    update = integrate_odometry(ORIGIN, ml_delta_ticks=10, mr_delta_ticks=10, dt_sec=0.0, cfg=CFG)
    assert update.linear_x_mps == 0.0
    assert update.angular_z_radps == 0.0
    # Position still integrates correctly even though dt is zero (dt
    # only affects the reported velocity, not the position delta).
    assert update.pose.x == pytest.approx(ticks_to_distance(10, CFG))


# -- angle normalization -------------------------------------------------------
def test_normalize_angle_wraps_above_pi():
    assert normalize_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)


def test_normalize_angle_wraps_below_negative_pi():
    assert normalize_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)


def test_normalize_angle_leaves_in_range_values_untouched():
    assert normalize_angle(1.0) == pytest.approx(1.0)
    assert normalize_angle(-1.0) == pytest.approx(-1.0)
    assert normalize_angle(math.pi) == pytest.approx(math.pi)


def test_integration_accumulates_and_wraps_heading_over_many_updates():
    pose = ORIGIN
    # Enough full-track-differential steps to spin past +pi at least once.
    for _ in range(20):
        update = integrate_odometry(pose, ml_delta_ticks=-50, mr_delta_ticks=50, dt_sec=1.0, cfg=CFG)
        pose = update.pose
    assert -math.pi < pose.theta <= math.pi


# -- quaternion ---------------------------------------------------------------
def test_quaternion_from_yaw_zero_is_identity():
    assert quaternion_from_yaw(0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quaternion_from_yaw_quarter_turn():
    x, y, z, w = quaternion_from_yaw(math.pi / 2.0)
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert z == pytest.approx(math.sqrt(2) / 2)
    assert w == pytest.approx(math.sqrt(2) / 2)


def test_quaternion_from_yaw_half_turn():
    x, y, z, w = quaternion_from_yaw(math.pi)
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert z == pytest.approx(1.0)
    assert w == pytest.approx(0.0, abs=1e-9)


def test_quaternion_from_yaw_is_always_normalized():
    for theta in (0.3, -1.7, 2.9, -3.0):
        x, y, z, w = quaternion_from_yaw(theta)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        assert norm == pytest.approx(1.0)


# -- diagonal_covariance -------------------------------------------------
def test_diagonal_covariance_places_values_on_the_diagonal():
    matrix = diagonal_covariance([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    assert len(matrix) == 36
    for i, expected in enumerate([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]):
        assert matrix[i * 6 + i] == expected


def test_diagonal_covariance_off_diagonal_is_zero():
    matrix = diagonal_covariance([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    off_diagonal_sum = sum(v for i, v in enumerate(matrix) if i % 6 != i // 6)
    assert off_diagonal_sum == 0.0


def test_diagonal_covariance_all_zero_input_gives_all_zero_matrix():
    matrix = diagonal_covariance([0.0] * 6)
    assert matrix == [0.0] * 36
