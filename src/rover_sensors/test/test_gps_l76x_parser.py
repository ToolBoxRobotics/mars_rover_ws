import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rover_sensors.gps_l76x_parser import GgaFix, RmcVelocity, parse_sentence

# Canonical textbook NMEA0183 example sentences (identical fix location
# used across most NMEA references), used here as independently-known
# golden values rather than something we generated ourselves.
GGA_EXAMPLE = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
RMC_EXAMPLE = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"


def test_gga_parses_position_and_quality():
    result = parse_sentence(GGA_EXAMPLE)
    assert isinstance(result, GgaFix)
    assert abs(result.latitude_deg - 48.1173) < 1e-4
    assert abs(result.longitude_deg - 11.51667) < 1e-4
    assert abs(result.altitude_m - 545.4) < 1e-6
    assert result.fix_quality == 1
    assert result.num_satellites == 8


def test_rmc_parses_speed_and_course_and_converts_knots_to_mps():
    result = parse_sentence(RMC_EXAMPLE)
    assert isinstance(result, RmcVelocity)
    assert result.valid is True
    # 22.4 knots * 0.514444 m/s/knot
    assert abs(result.speed_mps - (22.4 * 0.514444)) < 1e-6
    assert abs(result.course_deg - 84.4) < 1e-6


def test_rmc_invalid_status_flag_reported():
    invalid_line = RMC_EXAMPLE.replace(",A,", ",V,")
    # status flag change breaks the original checksum; rebuild it.
    body = invalid_line[1 : invalid_line.index("*")]
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    fixed = f"${body}*{checksum:02X}"
    result = parse_sentence(fixed)
    assert isinstance(result, RmcVelocity)
    assert result.valid is False


def test_unrelated_sentence_type_returns_none():
    gsv = "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74"
    assert parse_sentence(gsv) is None


def test_garbage_line_returns_none():
    assert parse_sentence("not-an-nmea-sentence-at-all") is None


def test_bad_checksum_returns_none():
    corrupted = GGA_EXAMPLE[:-2] + "00"
    assert parse_sentence(corrupted) is None


def test_empty_line_returns_none():
    assert parse_sentence("") is None
    assert parse_sentence("   ") is None
