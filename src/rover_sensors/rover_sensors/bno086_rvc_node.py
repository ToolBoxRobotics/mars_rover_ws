"""Publishes the BNO086 IMU (SparkFun VR IMU Breakout - BNO086, Qwiic;
PS0/PS1 jumpers strapped for UART-RVC mode, wired via its UART edge
pins to a Waveshare "USB TO TTL (B)" CH343G converter - see
bno086_rvc_parser.py for why this rules out true I2C on this link)
as sensor_msgs/Imu.

No host microcontroller is involved for this sensor: it free-runs at
100 Hz over serial the instant it has power, so this node just opens
the port, feeds raw bytes through the StreamSync parser, and
republishes each decoded frame.

RVC mode does not report gyro rate, so angular_velocity is left at
zero with covariance[0] = -1 (REP 103 "data unavailable" convention)
rather than fabricated. Orientation and linear_acceleration ARE
provided by the sensor's own fusion and are published normally.
"""

from __future__ import annotations

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .bno086_rvc_parser import StreamSync, quaternion_from_yaw_pitch_roll

# REP 103: covariance[0] == -1 means "no estimate available, ignore this field".
_UNAVAILABLE_COVARIANCE = [-1.0] + [0.0] * 8


class Bno086RvcNode(Node):
    def __init__(self) -> None:
        super().__init__("bno086_rvc_node")

        self.declare_parameter("serial_port", "/dev/rover/imu")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("frame_id", "imu_link")

        self._frame_id = self.get_parameter("frame_id").value
        self._sync = StreamSync()
        self._serial = None

        self._imu_pub = self.create_publisher(Imu, "rover_sensors/imu/data", 20)

        self._open_serial()
        # The sensor pushes frames at 100 Hz unprompted; poll faster
        # than that so bytes don't pile up in the OS serial buffer.
        self.create_timer(1.0 / 200.0, self._on_timer)

    def _open_serial(self) -> None:
        port = self.get_parameter("serial_port").value
        baud = int(self.get_parameter("serial_baud").value)
        try:
            self._serial = serial.Serial(port=port, baudrate=baud, timeout=0)
            self.get_logger().info(f"opened BNO086 RVC port {port} @ {baud} baud")
        except Exception as exc:
            self.get_logger().warn(f"could not open {port}: {exc}; will keep retrying")
            self._serial = None

    def _on_timer(self) -> None:
        if self._serial is None:
            self._open_serial()
            return

        try:
            data = self._serial.read(256)
        except Exception as exc:
            self.get_logger().warn(f"BNO086 serial read failed: {exc}; reopening")
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            return

        if not data:
            return

        for reading in self._sync.feed(data):
            self._publish_imu(reading)

    def _publish_imu(self, reading) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        qx, qy, qz, qw = quaternion_from_yaw_pitch_roll(reading.yaw_deg, reading.pitch_deg, reading.roll_deg)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        # Orientation covariance unknown-but-valid: leave default zeros
        # (ROS convention: all-zero means "unknown but not invalid",
        # distinct from the -1 sentinel used for genuinely absent data).

        msg.angular_velocity_covariance = _UNAVAILABLE_COVARIANCE

        msg.linear_acceleration.x = reading.accel_x_mps2
        msg.linear_acceleration.y = reading.accel_y_mps2
        msg.linear_acceleration.z = reading.accel_z_mps2

        self._imu_pub.publish(msg)

    def destroy_node(self) -> bool:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Bno086RvcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
