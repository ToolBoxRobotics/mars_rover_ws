import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "rover_protocol"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from rover_protocol.framing import RoverFrameError, decode_frame
from rover_base import base_protocol


def test_encode_drive_roundtrip():
    wheels = [1000, -1000, 500, -500, 0, 250]
    steer = [150, -150, 0, -300]
    frame = base_protocol.encode_drive(wheels, steer)
    msg_type, fields = decode_frame(frame)
    assert msg_type == "D"
    assert fields == wheels + steer


def test_encode_drive_wrong_wheel_count_raises():
    with pytest.raises(RoverFrameError):
        base_protocol.encode_drive([1, 2, 3], [0, 0, 0, 0])


def test_encode_drive_wrong_steer_count_raises():
    with pytest.raises(RoverFrameError):
        base_protocol.encode_drive([0, 0, 0, 0, 0, 0], [0, 0])


def test_encode_heartbeat():
    frame = base_protocol.encode_heartbeat()
    msg_type, fields = decode_frame(frame)
    assert msg_type == "H"
    assert fields == []


def test_parse_encoder_state_valid():
    fields = [100, 200, 12600, 12400, 235, 65]  # ML, MR, drive_voltage_mv, steering_voltage_mv, temperature_deci_c (23.5 deg C), fan_duty_percent
    ticks, drive_voltage_mv, steering_voltage_mv, temperature_deci_c, fan_duty_percent = (
        base_protocol.parse_encoder_state(fields)
    )
    assert ticks == [100, 200]
    assert drive_voltage_mv == 12600
    assert steering_voltage_mv == 12400
    assert temperature_deci_c == 235
    assert fan_duty_percent == 65


def test_parse_encoder_state_sensor_not_found_sentinel():
    ticks, drive_voltage_mv, steering_voltage_mv, temperature_deci_c, _fan_duty_percent = (
        base_protocol.parse_encoder_state([0, 0, 12000, 11900, -9999, 30])
    )
    assert temperature_deci_c == -9999


def test_parse_encoder_state_fan_off_when_not_running():
    # 0 is a real, expected duty cycle (the thermostat hasn't decided
    # to run the fan), not a sentinel or an error.
    *_rest, fan_duty_percent = base_protocol.parse_encoder_state([0, 0, 12000, 11900, 200, 0])
    assert fan_duty_percent == 0


def test_parse_encoder_state_wrong_length_raises():
    with pytest.raises(RoverFrameError):
        base_protocol.parse_encoder_state([1, 2, 3, 4, 5, 6, 7])
    with pytest.raises(RoverFrameError):
        # Old 5-field format from before the second (steering) voltage
        # sensor was added - must not be silently accepted as if
        # still valid.
        base_protocol.parse_encoder_state([1, 2, 3, 4, 5])
    with pytest.raises(RoverFrameError):
        # Older still: the 4-field format from before fan_duty_percent
        # was added at all.
        base_protocol.parse_encoder_state([1, 2, 3, 4])


# -- regression: numpy.int32 elements (see rover_arm/test/test_arm_protocol.py
# for the full story - this rover's actual crash was in ArmCommand's
# array field specifically, not here, since base_bridge_node always
# computes fresh Python ints via kinematics.py rather than reading an
# array field off a ROS message. Tested anyway for consistency, since
# encode_drive picked up the same defensive int() cast as
# encode_joint_command did. --------------------------------------------
def test_encode_drive_accepts_numpy_int32_elements():
    numpy = pytest.importorskip("numpy")
    throttle = numpy.array([100, -100, 0, 0, 100, -100], dtype=numpy.int32)
    steer = numpy.array([50, -50, 50, -50], dtype=numpy.int32)
    assert not isinstance(throttle[0], int)

    frame = base_protocol.encode_drive(throttle, steer)
    msg_type, fields = decode_frame(frame)
    assert msg_type == "D"
    assert fields == [100, -100, 0, 0, 100, -100, 50, -50, 50, -50]
