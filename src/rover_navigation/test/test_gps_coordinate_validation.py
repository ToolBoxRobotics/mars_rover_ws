import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from gps_coordinate_validation import validate_coordinates


def test_valid_coordinates_return_none():
    assert validate_coordinates(42.4373, -86.9436) is None


def test_zero_is_valid():
    assert validate_coordinates(0.0, 0.0) is None


def test_boundary_values_are_valid():
    assert validate_coordinates(90.0, 180.0) is None
    assert validate_coordinates(-90.0, -180.0) is None


def test_latitude_out_of_range():
    assert validate_coordinates(90.1, 0.0) is not None
    assert validate_coordinates(-90.1, 0.0) is not None


def test_longitude_out_of_range():
    assert validate_coordinates(0.0, 180.1) is not None
    assert validate_coordinates(0.0, -180.1) is not None


def test_error_message_mentions_which_field_is_bad():
    lat_error = validate_coordinates(200.0, 0.0)
    assert "latitude" in lat_error
    lon_error = validate_coordinates(0.0, 200.0)
    assert "longitude" in lon_error
