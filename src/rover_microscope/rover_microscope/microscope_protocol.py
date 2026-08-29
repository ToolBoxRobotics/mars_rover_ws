"""Microscope (Uno #4) serial message layer, built on rover_protocol.framing.

Frame types
-----------
  'C' (host -> Uno4): actuator command
      fields = [focus_target_steps, led_pwm, cover_open, driver_enable]
      focus_target_steps = combined focus/zoom stepper target, steps
                            from power-on zero (no calibration switch
                            on this axis - see package README)
      led_pwm             = 0..255 dimmable ring-light brightness
      cover_open           = 1 to slide the SG90 lens cover open, 0 closed
      driver_enable         = 1 to energize the DRV8825, 0 to de-energize
                            it (free-spin). Starts disabled at boot -
                            a command has to explicitly enable it.

  'S' (Uno4 -> host): actuator state
      fields = [focus_position_steps, led_pwm, cover_open, homed,
                driver_enabled, temperature_deci_c, fan_duty_percent]
      homed is always 1 once setup() completes - kept for symmetry
      with the arm/mast state messages and in case a calibration
      switch is added to this axis later.
      temperature_deci_c: board/enclosure temperature in tenths of a
                degree Celsius, from a DS18B20 on 1-Wire (Uno pin 11) -
                -9999 ("sensor not found") if the sensor didn't
                respond on its most recent read - checked fresh every
                cycle, not just once at boot; see base_mega1.ino's own
                header comment for the pull-up requirement this
                depends on. Added later than the other four boards'
                own copies of this field - this board was deliberately
                excluded from that original session, then added back
                once fan_duty_percent (below) needed a temperature
                input.
      fan_duty_percent: 0-100, cooling fan PWM duty cycle - entirely
                automatic, a function of temperature_deci_c (see
                microscope_uno4.ino's updateFanControl()), never
                operator-commanded, so there's no corresponding field
                on 'C' above. 0 means the fan is currently off, not
                necessarily broken.
"""

from __future__ import annotations

from typing import List, Tuple

from rover_protocol.framing import RoverFrameError, decode_frame, encode_frame


def encode_microscope_command(
    focus_target_steps: int, led_pwm: int, cover_open: bool, driver_enable: bool
) -> str:
    if not (0 <= led_pwm <= 255):
        raise RoverFrameError(f"led_pwm must be 0..255, got {led_pwm}")
    return encode_frame(
        "C",
        [
            int(focus_target_steps),
            int(led_pwm),
            1 if cover_open else 0,
            1 if driver_enable else 0,
        ],
    )


def decode_line(line) -> Tuple[str, List[int]]:
    return decode_frame(line)


def parse_microscope_state(fields: List[int]):
    """Returns (focus_position_steps, led_pwm, cover_open, homed, driver_enabled, temperature_deci_c, fan_duty_percent)."""
    if len(fields) != 7:
        raise RoverFrameError(f"'S' frame expected 7 fields, got {len(fields)}")
    focus_position, led_pwm, cover_open, homed, driver_enabled, temperature_deci_c, fan_duty_percent = fields
    return (
        focus_position,
        led_pwm,
        bool(cover_open),
        bool(homed),
        bool(driver_enabled),
        temperature_deci_c,
        fan_duty_percent,
    )
