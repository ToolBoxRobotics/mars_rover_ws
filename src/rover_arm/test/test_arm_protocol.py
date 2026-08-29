import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "rover_protocol"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from rover_protocol.framing import RoverFrameError, decode_frame
from rover_arm import arm_protocol


def test_encode_joint_command_roundtrip_enabled():
    targets = [100, -200, 300, -400, 500]
    frame = arm_protocol.encode_joint_command(targets, enable=True)
    msg_type, fields = decode_frame(frame)
    assert msg_type == "A"
    assert fields == targets + [1]


def test_encode_joint_command_roundtrip_disabled():
    targets = [0, 0, 0, 0, 0]
    frame = arm_protocol.encode_joint_command(targets, enable=False)
    _msg_type, fields = decode_frame(frame)
    assert fields[-1] == 0


def test_encode_joint_command_wrong_length_raises():
    with pytest.raises(RoverFrameError):
        arm_protocol.encode_joint_command([1, 2, 3], enable=True)


def test_encode_home_request_defaults_to_all_joints():
    frame = arm_protocol.encode_home_request()
    msg_type, fields = decode_frame(frame)
    assert msg_type == "Z"
    assert fields == [-1]


def test_encode_home_request_none_is_same_as_default():
    frame_default = arm_protocol.encode_home_request()
    frame_none = arm_protocol.encode_home_request(None)
    assert decode_frame(frame_default) == decode_frame(frame_none)


def test_encode_home_request_specific_joint():
    for joint_index in range(5):
        frame = arm_protocol.encode_home_request(joint_index)
        msg_type, fields = decode_frame(frame)
        assert msg_type == "Z"
        assert fields == [joint_index]


def test_encode_home_request_rejects_out_of_range():
    with pytest.raises(RoverFrameError):
        arm_protocol.encode_home_request(5)
    with pytest.raises(RoverFrameError):
        arm_protocol.encode_home_request(-2)


def test_encode_preset_request_valid():
    for preset in (arm_protocol.PRESET_INITIAL, arm_protocol.PRESET_TRANSPORT, arm_protocol.PRESET_SERVICE):
        frame = arm_protocol.encode_preset_request(preset)
        msg_type, fields = decode_frame(frame)
        assert msg_type == "P"
        assert fields == [preset]


def test_encode_preset_request_rejects_unknown():
    with pytest.raises(RoverFrameError):
        arm_protocol.encode_preset_request(3)
    with pytest.raises(RoverFrameError):
        arm_protocol.encode_preset_request(-1)


def test_encode_emergency_stop_engage():
    frame = arm_protocol.encode_emergency_stop(True)
    msg_type, fields = decode_frame(frame)
    assert msg_type == "X"
    assert fields == [1]


def test_encode_emergency_stop_clear():
    frame = arm_protocol.encode_emergency_stop(False)
    _msg_type, fields = decode_frame(frame)
    assert fields == [0]


def test_parse_joint_state_valid():
    fields = [10, 20, 30, 40, 50, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 12600, 235, 65, 0]
    positions, limits, joint_homed, voltage_mv, temperature_deci_c, fan_duty_percent, estop_active = (
        arm_protocol.parse_joint_state(fields)
    )
    assert positions == [10, 20, 30, 40, 50]
    assert limits == [False, True, False, False, True]
    assert joint_homed == [True, True, False, True, True]
    assert voltage_mv == 12600
    assert temperature_deci_c == 235
    assert fan_duty_percent == 65
    assert estop_active is False


def test_parse_joint_state_estop_active_true():
    fields = [10, 20, 30, 40, 50, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 12600, 235, 65, 1]
    *_rest, estop_active = arm_protocol.parse_joint_state(fields)
    assert estop_active is True


def test_parse_joint_state_per_joint_homed_can_be_partial():
    # The whole point of per-joint homing: some joints homed, others
    # not, at the same time - must not collapse into a single bool.
    fields = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 12000, -9999, 0, 0]
    _, _, joint_homed, _, _, _, _ = arm_protocol.parse_joint_state(fields)
    assert joint_homed == [True, False, False, False, False]


def test_parse_joint_state_wrong_length_raises():
    with pytest.raises(RoverFrameError):
        arm_protocol.parse_joint_state([1, 2, 3])
    with pytest.raises(RoverFrameError):
        # Old 12-field format from before per-joint homing - must not
        # be silently accepted as if still valid.
        arm_protocol.parse_joint_state([10, 20, 30, 40, 50, 0, 1, 0, 0, 1, 1, 12600])
    with pytest.raises(RoverFrameError):
        # Old 16-field format from before temperature was added - same
        # reasoning, must not be silently accepted either.
        arm_protocol.parse_joint_state([10, 20, 30, 40, 50, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 12600])
    with pytest.raises(RoverFrameError):
        # Old 17-field format from before fan_duty_percent was added.
        arm_protocol.parse_joint_state([10, 20, 30, 40, 50, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 12600, 235])
    with pytest.raises(RoverFrameError):
        # Old 18-field format from before estop_active was added - must
        # not be silently accepted either.
        arm_protocol.parse_joint_state([10, 20, 30, 40, 50, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 12600, 235, 65])


# -- regression: numpy.int32 elements from a ROS array field --------------
# Real crash on real hardware: rclpy backs fixed-size numeric array
# message fields (ArmCommand's `int32[5] joint_target_steps`) with
# numpy.ndarray internally - even a plain Python list assigned to the
# field gets stored this way - so elements read back out are
# numpy.int32, not Python int. isinstance(numpy.int32(0), int) is
# False, which used to reach all the way to rover_protocol.framing's
# strict int-only check and crash arm_bridge_node.py on its very first
# control cycle. Reproduces the exact failure mode with real numpy
# (not a mock) rather than trusting the fix by inspection alone.
def test_encode_joint_command_accepts_numpy_int32_elements():
    numpy = pytest.importorskip("numpy")
    targets = numpy.array([10, -20, 30, -40, 50], dtype=numpy.int32)
    # Confirms the test actually exercises the reported failure mode,
    # not a no-op - if this assertion ever fails, numpy's own behavior
    # changed and this test needs re-thinking, not the fix.
    assert not isinstance(targets[0], int)

    frame = arm_protocol.encode_joint_command(targets, enable=True)
    msg_type, fields = decode_frame(frame)
    assert msg_type == "A"
    assert fields == [10, -20, 30, -40, 50, 1]
