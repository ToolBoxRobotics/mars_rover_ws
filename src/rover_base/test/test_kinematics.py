import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rover_base.kinematics import (
    BaseGeometry,
    point_turn_wheel_commands,
    stop_wheel_commands,
    twist_to_wheel_commands,
)

GEO = BaseGeometry(
    wheelbase_front_m=0.30,
    wheelbase_rear_m=0.30,
    track_m=0.46,
    wheel_radius_m=0.075,
    max_wheel_rpm=83.0,
    max_steer_deg=60.0,
)


def test_straight_ahead_has_zero_steer_and_equal_wheel_speeds():
    throttle, steer = twist_to_wheel_commands(0.3, 0.0, GEO)
    assert steer == (0, 0, 0, 0)
    assert len(set(throttle)) == 1  # all six wheels identical
    assert throttle[0] > 0


def test_stationary_has_zero_throttle():
    throttle, steer = twist_to_wheel_commands(0.0, 0.0, GEO)
    assert throttle == (0, 0, 0, 0, 0, 0)
    assert steer == (0, 0, 0, 0)


def test_left_turn_slows_left_wheels_speeds_up_right_wheels():
    throttle, _steer = twist_to_wheel_commands(0.3, 0.5, GEO)
    fl, fr, ml, mr, rl, rr = throttle
    # y>0 wheels are FL, ML, RL (left side); angular_z>0 = turning left => left side slower
    assert fl < fr
    assert ml < mr
    assert rl < rr


def test_point_turn_and_arced_turn_mirror_front_and_rear():
    # For this ICR-tangent formula, the front corner pair and the rear
    # corner pair always steer as mirror images of each other (front
    # positive <-> rear negative on the same side), both for a true
    # in-place pivot (R=0) and for a finite-radius arced turn. Left and
    # right are NOT mirrors of each other - both front wheels lean the
    # same rotational direction for a pivot, which is the physically
    # correct tangent-to-center behavior.
    for angular_z in (0.8, 0.5):
        _throttle, steer = twist_to_wheel_commands(0.0 if angular_z == 0.8 else 0.3, angular_z, GEO)
        fl, fr, rl, rr = steer
        assert fl == -rl
        assert fr == -rr
        assert fl != 0


def test_steer_angle_clamped_to_hardware_limit():
    # A very tight radius should saturate at the configured servo limit.
    _throttle, steer = twist_to_wheel_commands(0.05, 1.5, GEO)
    for s in steer:
        assert abs(s) <= int(GEO.max_steer_deg * 10)


def test_output_shapes():
    throttle, steer = twist_to_wheel_commands(0.1, 0.1, GEO)
    assert len(throttle) == 6
    assert len(steer) == 4
    assert all(isinstance(t, int) for t in throttle)
    assert all(isinstance(s, int) for s in steer)


# -- POINT_TURN mode ------------------------------------------------------

def test_point_turn_matches_ackermann_with_zero_forward_component():
    for angular_z in (0.3, -0.9, 1.5):
        expected = twist_to_wheel_commands(0.0, angular_z, GEO)
        assert point_turn_wheel_commands(angular_z, GEO) == expected


def test_point_turn_produces_nonzero_fanned_steering():
    throttle, steer = point_turn_wheel_commands(0.7, GEO)
    assert any(s != 0 for s in steer)
    assert any(t != 0 for t in throttle)


def test_point_turn_zero_rate_is_fully_neutral():
    throttle, steer = point_turn_wheel_commands(0.0, GEO)
    assert throttle == (0, 0, 0, 0, 0, 0)
    assert steer == (0, 0, 0, 0)


# -- STOP mode ---------------------------------------------------------

def test_stop_is_always_fully_zero():
    throttle, steer = stop_wheel_commands()
    assert throttle == (0, 0, 0, 0, 0, 0)
    assert steer == (0, 0, 0, 0)
