"""4-corner-steering kinematics for the 6-wheel rocker-bogie base.

Only the front and rear wheel pairs steer (indices FL, FR, RL, RR);
the two middle wheels (ML, MR) are fixed and drive-only, exactly like
Opportunity / Curiosity / Perseverance.

Three selectable drive modes (see rover_msgs/DriveMode), each with its
own function below:

  ACKERMANN (twist_to_wheel_commands)  - normal driving; Twist.linear.x
      + Twist.angular.z, exact ICR-tangent corner steering
  POINT_TURN (point_turn_wheel_commands) - rotate about the rover's own
      center; Twist.angular.z only, forward component forced to zero
  STOP (stop_wheel_commands) - unconditional zero throttle and centered
      steering, ignoring Twist entirely

Steering angle in ACKERMANN/POINT_TURN uses an exact instant-center-of-
rotation (ICR) geometric construction: for a desired body twist (v, w),
the ICR sits at (0, R) in the vehicle frame (+x forward, +y left) with
R = v / w. Each corner wheel is pointed tangent to the circle traced
around that ICR, which for a wheel at (x_w, y_w) works out to the clean
closed form

    steer_angle = atan2(x_w, R - y_w)

This single formula naturally produces the expected mirrored front/rear
steering for arced turns and the correct "point turn" fan-out when
v == 0. It is exact for a rigid, zero-slip vehicle.

Wheel throttle in ACKERMANN/POINT_TURN uses the simpler, standard
differential/skid-steer mixing (v -+ w * track/2 per side) rather than
an exact zero-slip speed distribution. The middle wheels already
tolerate slip during point turns on this class of rover, so the extra
complexity of an exact per-wheel speed model is not justified for the
base scaffold; revisit if wheel odometry drift proves significant on
the real hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

# Wheel order used everywhere in rover_base: FL, FR, ML, MR, RL, RR
WHEEL_NAMES = ("FL", "FR", "ML", "MR", "RL", "RR")
# Steering corners (subset of the above, order must match BaseCommand.steer_decideg)
STEER_NAMES = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class BaseGeometry:
    wheelbase_front_m: float
    wheelbase_rear_m: float
    track_m: float
    wheel_radius_m: float
    max_wheel_rpm: float
    max_steer_deg: float

    def wheel_positions(self):
        """Return {name: (x, y)} in the vehicle frame (+x fwd, +y left)."""
        lf, lr, t = self.wheelbase_front_m, self.wheelbase_rear_m, self.track_m
        return {
            "FL": (lf, t / 2.0),
            "FR": (lf, -t / 2.0),
            "ML": (0.0, t / 2.0),
            "MR": (0.0, -t / 2.0),
            "RL": (-lr, t / 2.0),
            "RR": (-lr, -t / 2.0),
        }

    @property
    def max_wheel_surface_mps(self) -> float:
        return self.max_wheel_rpm / 60.0 * 2.0 * math.pi * self.wheel_radius_m


ANGULAR_EPSILON = 1e-3  # rad/s below which we treat the path as straight


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def compute_corner_steer_deg(x_w: float, y_w: float, linear_x: float, angular_z: float) -> float:
    """Exact ICR-tangent steering angle (degrees) for a wheel at (x_w, y_w)."""
    if abs(angular_z) < ANGULAR_EPSILON:
        return 0.0
    turning_radius = linear_x / angular_z
    return math.degrees(math.atan2(x_w, turning_radius - y_w))


def twist_to_wheel_commands(
    linear_x: float,
    angular_z: float,
    geometry: BaseGeometry,
) -> Tuple[Tuple[int, int, int, int, int, int], Tuple[int, int, int, int, int]]:
    """ACKERMANN mode: convert a body twist into wheel throttle
    (-1000..1000) and corner steer angles in decidegrees (tenths of a
    degree), ready to hand to :func:`rover_base.base_protocol.encode_drive`.

    Returns (throttle_by_wheel_order, steer_deciDeg_by_steer_order) where
    wheel order is WHEEL_NAMES and steer order is STEER_NAMES.
    """
    positions = geometry.wheel_positions()
    max_speed = geometry.max_wheel_surface_mps

    throttles = []
    for name in WHEEL_NAMES:
        _x_w, y_w = positions[name]
        # Standard differential/skid mixing: left wheels (y>0) slow down
        # for a left (positive angular_z) turn, right wheels speed up.
        wheel_speed = linear_x - angular_z * y_w
        throttle_frac = 0.0 if max_speed <= 0 else wheel_speed / max_speed
        throttles.append(int(round(_clamp(throttle_frac, -1.0, 1.0) * 1000)))

    steer_decideg = []
    for name in STEER_NAMES:
        x_w, y_w = positions[name]
        angle_deg = compute_corner_steer_deg(x_w, y_w, linear_x, angular_z)
        angle_deg = _clamp(angle_deg, -geometry.max_steer_deg, geometry.max_steer_deg)
        steer_decideg.append(int(round(angle_deg * 10)))

    return tuple(throttles), tuple(steer_decideg)  # type: ignore[return-value]


def point_turn_wheel_commands(
    angular_z: float,
    geometry: BaseGeometry,
) -> Tuple[Tuple[int, int, int, int, int, int], Tuple[int, int, int, int, int]]:
    """POINT_TURN mode: rotate about the rover's own geometric center.

    Mathematically this is exactly ``twist_to_wheel_commands(0.0,
    angular_z, geometry)`` - the ICR-tangent formula already reduces to
    the correct "fan the corners out, spin left/right wheels opposite
    ways" point-turn geometry when linear_x is zero. This wrapper exists
    so callers (and DriveMode dispatch) get an explicit, named mode
    that can never accidentally pick up a nonzero forward component,
    rather than relying on every caller remembering to pass 0.0.

    Hardware caveat: a true zero-radius pivot needs corner angles well
    past +-90 degrees for wheels close to the vehicle's center (the
    ICR-tangent formula is exact for the vehicle geometry, not for the
    servos). With this rover's +-60 degree steering limit, the achieved
    motion is the closest available approximation, not a perfect pivot -
    see the "Explicit assumptions" section of README.md.
    """
    return twist_to_wheel_commands(0.0, angular_z, geometry)


def stop_wheel_commands() -> Tuple[Tuple[int, int, int, int, int, int], Tuple[int, int, int, int, int]]:
    """STOP mode: unconditional zero throttle and centered (0 deg)
    steering, regardless of any Twist input. Named explicitly (rather
    than inlined at call sites) so it reads the same way in the
    DriveMode dispatch as the other three modes.
    """
    return (0, 0, 0, 0, 0, 0), (0, 0, 0, 0)
