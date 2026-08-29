"""Pure GPS coordinate validation for gps_goal.py, kept free of any
ROS dependency (rclpy, nav2_msgs, robot_localization) so it can be
unit tested without a ROS 2 install - same pure-logic/thin-IO split
used throughout this workspace (e.g. tools/udev_device_id.py).
"""

from __future__ import annotations

from typing import Optional


def validate_coordinates(latitude: float, longitude: float) -> Optional[str]:
    """Returns None if (latitude, longitude) is a valid WGS84
    coordinate, or a human-readable error string if not.
    """
    if not (-90.0 <= latitude <= 90.0):
        return f"latitude {latitude} out of range [-90, 90]"
    if not (-180.0 <= longitude <= 180.0):
        return f"longitude {longitude} out of range [-180, 180]"
    return None
