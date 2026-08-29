"""Publishes the Waveshare L76X GPS module (wired directly to USB, no
Arduino involved) as sensor_msgs/NavSatFix, plus ground speed/course
from RMC sentences as geometry_msgs/TwistStamped.
"""

from __future__ import annotations

import math

import rclpy
import serial
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .gps_l76x_parser import GgaFix, RmcVelocity, parse_sentence


class GpsL76xNode(Node):
    def __init__(self) -> None:
        super().__init__("gps_l76x_node")

        self.declare_parameter("serial_port", "/dev/rover/gps")
        self.declare_parameter("serial_baud", 9600)
        self.declare_parameter("frame_id", "gps_link")

        self._frame_id = self.get_parameter("frame_id").value
        self._serial = None
        self._line_buf = b""

        self._fix_pub = self.create_publisher(NavSatFix, "rover_sensors/gps/fix", 10)
        self._velocity_pub = self.create_publisher(TwistStamped, "rover_sensors/gps/velocity", 10)

        self._open_serial()
        self.create_timer(1.0 / 20.0, self._on_timer)

    def _open_serial(self) -> None:
        port = self.get_parameter("serial_port").value
        baud = int(self.get_parameter("serial_baud").value)
        try:
            self._serial = serial.Serial(port=port, baudrate=baud, timeout=0)
            self.get_logger().info(f"opened L76X GPS port {port} @ {baud} baud")
        except Exception as exc:
            self.get_logger().warn(f"could not open {port}: {exc}; will keep retrying")
            self._serial = None

    def _on_timer(self) -> None:
        if self._serial is None:
            self._open_serial()
            return

        try:
            chunk = self._serial.read(512)
        except Exception as exc:
            self.get_logger().warn(f"GPS serial read failed: {exc}; reopening")
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            return

        if not chunk:
            return

        self._line_buf += chunk
        while b"\n" in self._line_buf:
            raw_line, self._line_buf = self._line_buf.split(b"\n", 1)
            try:
                line = raw_line.decode("ascii", errors="ignore")
            except Exception:
                continue
            result = parse_sentence(line)
            if isinstance(result, GgaFix):
                self._publish_fix(result)
            elif isinstance(result, RmcVelocity):
                self._publish_velocity(result)

    def _publish_fix(self, fix: GgaFix) -> None:
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.latitude = fix.latitude_deg
        msg.longitude = fix.longitude_deg
        msg.altitude = fix.altitude_m
        msg.status.status = (
            NavSatStatus.STATUS_FIX if fix.fix_quality != 0 else NavSatStatus.STATUS_NO_FIX
        )
        msg.status.service = NavSatStatus.SERVICE_GPS
        # Rough covariance from HDOP; a real UERE-based model is a
        # reasonable future improvement once the module is bench-tested.
        variance = max(fix.hdop, 1.0) ** 2
        msg.position_covariance = [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance * 4.0]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self._fix_pub.publish(msg)

    def _publish_velocity(self, velocity: RmcVelocity) -> None:
        if not velocity.valid:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        course_rad = math.radians(velocity.course_deg)
        msg.twist.linear.x = velocity.speed_mps * math.cos(course_rad)
        msg.twist.linear.y = velocity.speed_mps * math.sin(course_rad)
        self._velocity_pub.publish(msg)

    def destroy_node(self) -> bool:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsL76xNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
