"""Power/environmental monitoring board (Uno #6) serial message
layer, built on rover_protocol.framing.

Structurally simpler than every other *_protocol.py in this project:
there is no host -> Uno6 command frame at all, only the one frame
this board sends on its own. See power_uno6.ino's own header comment
for why (this board commands nothing - it's a pure telemetry source
plus one automatic, temperature-driven fan).

Frame types
-----------
  'S' (Uno6 -> host): power/environmental state
      fields = [battery1_mv, battery1_ma, battery2_mv, battery2_ma,
                computer_temperature_deci_c, fan_duty_percent]
      battery1_mv/battery1_ma, battery2_mv/battery2_ma: each battery's
                  own voltage (millivolts) and current (milliamps,
                  signed), from that battery's own INA226 sitting
                  behind a TCA9548A I2C multiplexer (battery 1 on mux
                  channel 0, battery 2 on channel 1) - see
                  power_uno6.ino's readBattery() for the read itself,
                  and its own header comment's I2C ADDRESSING and
                  SHUNT RESISTOR WARNING sections for the reasoning
                  behind the mux (both units share one default I2C
                  address, which is exactly what the mux makes
                  workable) and a real, must-verify caveat about
                  current-reading accuracy specifically.
      computer_temperature_deci_c: the onboard computer's temperature
                  (not this board's own enclosure) in tenths of a
                  degree Celsius, from a DS18B20 on 1-Wire (Uno pin
                  D2) - -9999 ("sensor not found") if the sensor
                  didn't respond on its most recent read - checked
                  fresh every cycle, not just once at boot; see
                  base_mega1.ino's own header comment for the pull-up
                  requirement this depends on.
      fan_duty_percent: 0-100, the computer's cooling fan PWM duty
                  cycle - entirely automatic, a function of
                  computer_temperature_deci_c (see power_uno6.ino's
                  updateFanControl()), never operator-commanded - 0
                  means the fan is currently off, not necessarily
                  broken.
"""

from __future__ import annotations

from typing import List, Tuple

from rover_protocol.framing import RoverFrameError, decode_frame


def decode_line(line) -> Tuple[str, List[int]]:
    return decode_frame(line)


def parse_power_state(fields: List[int]):
    """Returns (battery1_mv, battery1_ma, battery2_mv, battery2_ma, computer_temperature_deci_c, fan_duty_percent)."""
    if len(fields) != 6:
        raise RoverFrameError(f"'S' frame expected 6 fields, got {len(fields)}")
    battery1_mv, battery1_ma, battery2_mv, battery2_ma, computer_temperature_deci_c, fan_duty_percent = fields
    return battery1_mv, battery1_ma, battery2_mv, battery2_ma, computer_temperature_deci_c, fan_duty_percent
