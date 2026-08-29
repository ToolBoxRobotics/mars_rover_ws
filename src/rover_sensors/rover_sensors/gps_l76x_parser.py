"""Thin, testable wrapper around NMEA 0183 sentence parsing for the
Waveshare L76X GPS module, which (like the BNO086 in RVC mode) is
wired straight to a USB-serial adapter and needs no Arduino: it
free-runs, pushing standard NMEA sentences at its configured baud
rate (9600 by default; can be raised with a PMTK config command if
the module is reconfigured).

Only GGA (fix data: lat/lon/altitude/quality/satellite count) and RMC
(recommended minimum: adds ground speed and course) are consumed;
everything else the module emits (GSV, GSA, VTG, ...) is ignored.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Union

import pynmea2

# GGA fix quality codes (NMEA 0183 standard)
FIX_QUALITY_INVALID = 0
FIX_QUALITY_GPS = 1
FIX_QUALITY_DGPS = 2


@dataclass(frozen=True)
class GgaFix:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    fix_quality: int
    num_satellites: int
    hdop: float


@dataclass(frozen=True)
class RmcVelocity:
    speed_mps: float
    course_deg: float
    valid: bool


def parse_sentence(line: str) -> Optional[Union[GgaFix, RmcVelocity]]:
    """Parse one raw NMEA line. Returns a GgaFix, an RmcVelocity, or
    None for any sentence type we don't care about or that fails to
    parse (checksum error, truncated line, etc. - all expected,
    recoverable noise on a live serial link).
    """
    line = line.strip()
    if not line:
        return None
    try:
        msg = pynmea2.parse(line)
    except (pynmea2.ParseError, pynmea2.ChecksumError):
        return None

    sentence_type = getattr(msg, "sentence_type", "")

    if sentence_type == "GGA":
        try:
            if msg.latitude is None or msg.longitude is None:
                return None
            return GgaFix(
                latitude_deg=float(msg.latitude),
                longitude_deg=float(msg.longitude),
                altitude_m=float(msg.altitude) if msg.altitude not in (None, "") else 0.0,
                fix_quality=int(msg.gps_qual) if msg.gps_qual not in (None, "") else FIX_QUALITY_INVALID,
                num_satellites=int(msg.num_sats) if msg.num_sats not in (None, "") else 0,
                hdop=float(msg.horizontal_dil) if msg.horizontal_dil not in (None, "") else 0.0,
            )
        except (TypeError, ValueError):
            return None

    if sentence_type == "RMC":
        try:
            valid = getattr(msg, "status", "V") == "A"
            speed_knots = float(msg.spd_over_grnd) if msg.spd_over_grnd not in (None, "") else 0.0
            course = float(msg.true_course) if msg.true_course not in (None, "") else 0.0
            return RmcVelocity(speed_mps=speed_knots * 0.514444, course_deg=course, valid=valid)
        except (TypeError, ValueError):
            return None

    return None
