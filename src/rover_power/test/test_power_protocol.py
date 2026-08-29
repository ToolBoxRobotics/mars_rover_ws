import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "rover_protocol"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from rover_protocol.framing import RoverFrameError, encode_frame
from rover_power import power_protocol


def test_decode_line_roundtrip():
    frame = encode_frame("S", [25200, 4500, 25100, 4200, 235, 65])
    msg_type, fields = power_protocol.decode_line(frame)
    assert msg_type == "S"
    assert fields == [25200, 4500, 25100, 4200, 235, 65]


def test_parse_power_state_valid():
    battery1_mv, battery1_ma, battery2_mv, battery2_ma, computer_temperature_deci_c, fan_duty_percent = (
        power_protocol.parse_power_state([25200, 4500, 25100, 4200, 235, 65])
    )
    assert battery1_mv == 25200
    assert battery1_ma == 4500
    assert battery2_mv == 25100
    assert battery2_ma == 4200
    assert computer_temperature_deci_c == 235
    assert fan_duty_percent == 65


def test_parse_power_state_sensor_not_found_sentinel():
    *_rest, computer_temperature_deci_c, _fan_duty_percent = power_protocol.parse_power_state(
        [24000, 0, 24000, 0, -9999, 30]
    )
    assert computer_temperature_deci_c == -9999


def test_parse_power_state_fan_off_when_not_running():
    # 0 is a real, expected duty cycle (the thermostat hasn't decided
    # to run the fan), not a sentinel or an error.
    *_rest, fan_duty_percent = power_protocol.parse_power_state([24000, 0, 24000, 0, 200, 0])
    assert fan_duty_percent == 0


def test_parse_power_state_negative_current_allowed_per_battery():
    # Each INA226 is bidirectional in principle - a small negative
    # reading near true-zero load (sensor noise) must not be rejected
    # as invalid, independently for each battery.
    battery1_mv, battery1_ma, battery2_mv, battery2_ma, _, _ = power_protocol.parse_power_state(
        [24000, -50, 24000, 30, 200, 0]
    )
    assert battery1_ma == -50
    assert battery2_ma == 30


def test_parse_power_state_wrong_length_raises():
    with pytest.raises(RoverFrameError):
        power_protocol.parse_power_state([1, 2, 3])
    with pytest.raises(RoverFrameError):
        power_protocol.parse_power_state([1, 2, 3, 4, 5, 6, 7])
    with pytest.raises(RoverFrameError):
        # Old 5-field format from before per-battery current replaced
        # the single shared current field - must not be silently
        # accepted as if still valid.
        power_protocol.parse_power_state([24000, 24000, 0, 200, 0])
