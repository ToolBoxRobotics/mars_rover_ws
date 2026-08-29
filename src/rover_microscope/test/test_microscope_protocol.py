import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "rover_protocol"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from rover_protocol.framing import RoverFrameError, decode_frame
from rover_microscope import microscope_protocol


def test_encode_microscope_command_roundtrip():
    frame = microscope_protocol.encode_microscope_command(1200, 180, True, True)
    msg_type, fields = decode_frame(frame)
    assert msg_type == "C"
    assert fields == [1200, 180, 1, 1]


def test_encode_microscope_command_cover_closed():
    frame = microscope_protocol.encode_microscope_command(0, 0, False, True)
    _msg_type, fields = decode_frame(frame)
    assert fields[2] == 0


def test_encode_microscope_command_driver_disabled():
    frame = microscope_protocol.encode_microscope_command(0, 0, False, False)
    _msg_type, fields = decode_frame(frame)
    assert fields[3] == 0


def test_encode_microscope_command_rejects_bad_led_pwm():
    with pytest.raises(RoverFrameError):
        microscope_protocol.encode_microscope_command(0, 300, False, False)
    with pytest.raises(RoverFrameError):
        microscope_protocol.encode_microscope_command(0, -1, False, False)


def test_parse_microscope_state_valid():
    focus, led, cover, homed, driver_enabled, temperature_deci_c, fan_duty_percent = (
        microscope_protocol.parse_microscope_state([500, 128, 1, 1, 1, 235, 65])
    )
    assert (focus, led, cover, homed, driver_enabled) == (500, 128, True, True, True)
    assert temperature_deci_c == 235
    assert fan_duty_percent == 65


def test_parse_microscope_state_driver_disabled():
    _, _, _, _, driver_enabled, _, _ = microscope_protocol.parse_microscope_state(
        [0, 0, 0, 1, 0, -9999, 0]
    )
    assert driver_enabled is False


def test_parse_microscope_state_sensor_not_found_sentinel():
    *_rest, temperature_deci_c, _fan_duty_percent = microscope_protocol.parse_microscope_state(
        [0, 0, 0, 1, 0, -9999, 0]
    )
    assert temperature_deci_c == -9999


def test_parse_microscope_state_fan_off_when_not_running():
    # 0 is a real, expected duty cycle (the thermostat hasn't decided
    # to run the fan), not a sentinel or an error.
    *_rest, fan_duty_percent = microscope_protocol.parse_microscope_state([0, 0, 0, 1, 0, 200, 0])
    assert fan_duty_percent == 0


def test_parse_microscope_state_wrong_length_raises():
    with pytest.raises(RoverFrameError):
        microscope_protocol.parse_microscope_state([1, 2, 3])
    with pytest.raises(RoverFrameError):
        # Old 4-field format from before driver_enable was added - must
        # not be silently accepted as if still valid.
        microscope_protocol.parse_microscope_state([500, 128, 1, 1])
    with pytest.raises(RoverFrameError):
        # Old 5-field format from before temperature/fan were added.
        microscope_protocol.parse_microscope_state([500, 128, 1, 1, 1])
