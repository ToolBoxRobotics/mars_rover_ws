import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from rover_arm.joint_conversion import (
    joint_radians_to_steps,
    joint_steps_to_radians,
    radians_to_steps,
    reorder_by_name,
    steps_to_radians,
)

STEPS_PER_REV = 16000  # matches arm_topology.yaml's steps_per_joint_rev


def test_zero_radians_is_zero_steps():
    assert radians_to_steps(0.0, STEPS_PER_REV) == 0


def test_full_revolution_equals_steps_per_rev():
    assert radians_to_steps(2 * math.pi, STEPS_PER_REV) == STEPS_PER_REV


def test_half_revolution_equals_half_steps_per_rev():
    assert radians_to_steps(math.pi, STEPS_PER_REV) == STEPS_PER_REV // 2


def test_negative_radians_gives_negative_steps():
    assert radians_to_steps(-math.pi, STEPS_PER_REV) == -(STEPS_PER_REV // 2)


def test_rounds_to_nearest_step():
    # A fraction of a step past 0.5 should round up, not truncate towards zero.
    tiny_angle = (0.6 / STEPS_PER_REV) * 2 * math.pi
    assert radians_to_steps(tiny_angle, STEPS_PER_REV) == 1


def test_steps_to_radians_is_inverse_of_radians_to_steps_at_exact_steps():
    # Exact step counts round-trip exactly; fractional radians may not
    # (expected - see the rounding test above), so only assert
    # round-trip fidelity starting from an integer step count.
    for steps in (-8000, -1, 0, 1, 4000, 16000):
        radians = steps_to_radians(steps, STEPS_PER_REV)
        assert radians_to_steps(radians, STEPS_PER_REV) == steps


def test_joint_vector_conversion_matches_scalar_per_joint():
    steps_per_joint = [16000, 16000, 16000, 16000, 16000]
    radians = [0.0, math.pi / 2, -math.pi / 2, math.pi, -math.pi]
    steps = joint_radians_to_steps(radians, steps_per_joint)
    expected = [radians_to_steps(r, s) for r, s in zip(radians, steps_per_joint)]
    assert steps == expected


def test_joint_vector_conversion_round_trips():
    steps_per_joint = [16000, 16000, 16000, 16000, 16000]
    steps = [1000, -2000, 0, 8000, -8000]
    radians = joint_steps_to_radians(steps, steps_per_joint)
    assert joint_radians_to_steps(radians, steps_per_joint) == steps


def test_mismatched_length_raises():
    with pytest.raises(ValueError):
        joint_radians_to_steps([0.0, 0.0], [16000, 16000, 16000])
    with pytest.raises(ValueError):
        joint_steps_to_radians([0, 0, 0], [16000, 16000])


def test_different_steps_per_joint_handled_independently():
    # A joint with finer microstepping/gearing than another shouldn't
    # affect its neighbor's conversion.
    steps_per_joint = [16000, 32000, 16000, 16000, 16000]
    radians = [math.pi, math.pi, math.pi, math.pi, math.pi]
    steps = joint_radians_to_steps(radians, steps_per_joint)
    assert steps[1] == 2 * steps[0]


# -- reorder_by_name ---------------------------------------------------
CANONICAL = ["shoulder_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll"]


def test_reorder_by_name_identity_when_already_in_order():
    values = [1.0, 2.0, 3.0, 4.0, 5.0]
    assert reorder_by_name(CANONICAL, values, CANONICAL) == values


def test_reorder_by_name_handles_scrambled_order():
    scrambled_names = ["wrist_roll", "elbow_pitch", "shoulder_yaw", "wrist_pitch", "shoulder_pitch"]
    scrambled_values = [50.0, 30.0, 10.0, 40.0, 20.0]  # value i corresponds to scrambled_names[i]
    result = reorder_by_name(scrambled_names, scrambled_values, CANONICAL)
    # CANONICAL order is shoulder_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll
    assert result == [10.0, 20.0, 30.0, 40.0, 50.0]


def test_reorder_by_name_mismatched_length_raises():
    with pytest.raises(ValueError):
        reorder_by_name(CANONICAL, [1.0, 2.0], CANONICAL)


def test_reorder_by_name_unknown_joint_name_raises():
    bad_names = CANONICAL[:-1] + ["not_a_real_joint"]
    with pytest.raises(ValueError):
        reorder_by_name(bad_names, [0.0] * 5, CANONICAL)


def test_reorder_by_name_missing_joint_raises():
    # Only 4 of the 5 canonical joints present - a partial trajectory
    # goal should be rejected, not silently accepted with a gap.
    with pytest.raises(ValueError):
        reorder_by_name(CANONICAL[:-1], [0.0] * 4, CANONICAL)
