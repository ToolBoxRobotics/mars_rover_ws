"""rover_power bridge node.

Reads ``rover_msgs/PowerState`` telemetry (two batteries' own voltage
and current, onboard computer temperature, and its automatic
fan's duty cycle) from the power/environmental monitoring Uno (#6)
and republishes it. There is nothing to subscribe to and nothing to
send - this is the one bridge node in the project with no command
message and no command-sending timer logic, because this board
commands nothing: it's a pure telemetry source plus one automatic,
temperature-driven fan the firmware runs entirely on its own. See
power_uno6.ino's own header comment for the full reasoning.

Every other bridge node's timer both drains incoming serial data AND
encodes/sends a command frame each tick, because the firmware on the
other end only replies to a command with a state frame - there's
nothing to reply to here, so the firmware sends its own state frame
proactively, on its own fixed interval (see power_uno6.ino's
kStateSendIntervalMs). This node's timer exists purely to poll for
that unprompted data and to publish BoardStatus at a steady rate,
matching every other board's own board_status cadence for the web
GUI's connection lamps - not to drive anything on the firmware side.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from rover_msgs.msg import BoardStatus, PowerState
from rover_protocol import SerialLink

from . import power_protocol


class PowerBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("rover_power_bridge")

        self.declare_parameter("serial_port", "/dev/rover/power")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("boot_grace_sec", 2.0)
        self.declare_parameter("poll_rate_hz", 10.0)

        self._link = SerialLink(
            port=self.get_parameter("serial_port").value,
            baud=int(self.get_parameter("serial_baud").value),
            timeout=0.05,
            boot_grace_sec=float(self.get_parameter("boot_grace_sec").value),
        )

        self._state_pub = self.create_publisher(PowerState, "rover_power/state", 10)
        self._status_pub = self.create_publisher(BoardStatus, "rover_power/board_status", 10)

        rate_hz = float(self.get_parameter("poll_rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f"rover_power bridge starting on {self._link.port} @ {self._link.baud} baud"
        )

    def _on_timer(self) -> None:
        self._drain_incoming()
        self._publish_status()

    def _drain_incoming(self, max_reads: int = 8) -> None:
        for _ in range(max_reads):
            result = self._link.read_decoded()
            if result is None:
                return
            msg_type, fields = result
            if msg_type == "S":
                self._publish_power_state(fields)

    def _publish_power_state(self, fields) -> None:
        try:
            battery1_mv, battery1_ma, battery2_mv, battery2_ma, computer_temperature_deci_c, fan_duty_percent = (
                power_protocol.parse_power_state(fields)
            )
        except Exception as exc:
            self.get_logger().warn(f"dropping malformed power state frame: {exc}")
            return

        state = PowerState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.battery1_voltage_mv = battery1_mv
        state.battery1_current_ma = battery1_ma
        state.battery2_voltage_mv = battery2_mv
        state.battery2_current_ma = battery2_ma
        state.computer_temperature_decic = computer_temperature_deci_c
        state.fan_duty_percent = fan_duty_percent
        self._state_pub.publish(state)

    def _publish_status(self) -> None:
        status = BoardStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.board_name = "power_uno6"
        status.port = self._link.port
        status.connected = self._link.connected
        status.rx_frame_count = self._link.rx_frame_count
        status.checksum_error_count = self._link.checksum_error_count
        status.reconnect_count = self._link.reconnect_count
        age = self._link.last_rx_age_sec()
        status.last_rx_age_sec = float(age) if age != float("inf") else -1.0
        self._status_pub.publish(status)

    def destroy_node(self) -> bool:
        self._link.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PowerBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
