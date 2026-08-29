"""rover_arm bridge node.

Subscribes to ``rover_msgs/ArmCommand`` (direct joint-space targets in
motor steps - this scaffold intentionally does not include an inverse
kinematics layer; add a MoveIt 2 config or a custom IK node upstream
of this bridge when task-space control is needed) and streams it to
the arm Mega (#2). Republishes joint positions and calibration-switch
state as ``rover_msgs/ArmState``.

Sends a one-shot homing request ('Z', all 5 joints) on startup if
``home_on_startup`` is true, since the arm's stepper joints have no
absolute encoders and need the calibration switches to establish a
zero position after every power-up.

Also exposes ``rover_arm/home_joint`` (``rover_msgs/srv/HomeJoint``)
for triggering homing on demand after startup - either a single joint
(0-4) or all five (-1) - used by the web GUI's per-joint calibration
buttons and its "calibrate all 5" button. Validates the joint index
locally and reports whether the request was successfully written to
the serial link; whether the firmware actually acts on it (e.g. it
silently ignores a new request while one is already in progress) is
downstream of what this service can directly confirm - the next
``ArmState`` message's ``joint_homed`` array is the actual source of
truth for whether/when homing completed.

Two more one-shot services, same "validate locally, report whether the
write succeeded, let ArmState be the actual source of truth for
whether it took effect" shape as home_joint above:
  ``rover_arm/emergency_stop`` (``rover_msgs/srv/EmergencyStop``) -
      engage/clear the arm's latching e-stop. Sent immediately, not
      queued behind the regular per-tick command below - see
      _on_emergency_stop() for why this bypasses the normal
      _last_command flow entirely rather than waiting for the next
      timer tick.
  ``rover_arm/arm_preset`` (``rover_msgs/srv/ArmPreset``) - move to one
      of three firmware-defined poses (initial/transport/service).
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from rover_msgs.msg import ArmCommand, ArmState, BoardStatus
from rover_msgs.srv import ArmPreset, EmergencyStop, HomeJoint
from rover_protocol import SerialLink

from . import arm_protocol

_JOINT_NAMES = ["shoulder_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll"]
# Index must match arm_protocol.py's own PRESET_INITIAL/PRESET_TRANSPORT/PRESET_SERVICE.
_PRESET_NAMES = ["initial", "transport", "service"]


class ArmBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("rover_arm_bridge")

        self.declare_parameter("serial_port", "/dev/rover/arm")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("boot_grace_sec", 2.0)
        self.declare_parameter("home_on_startup", True)

        self._command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)

        self._link = SerialLink(
            port=self.get_parameter("serial_port").value,
            baud=int(self.get_parameter("serial_baud").value),
            timeout=0.05,
            boot_grace_sec=float(self.get_parameter("boot_grace_sec").value),
        )

        self._last_command = ArmCommand()
        self._last_command.joint_target_steps = [0, 0, 0, 0, 0]
        self._last_command.enable = False
        self._last_command_time = 0.0
        self._sent_home_request = False

        self._state_pub = self.create_publisher(ArmState, "rover_arm/state", 10)
        self._status_pub = self.create_publisher(BoardStatus, "rover_arm/board_status", 10)
        self.create_subscription(ArmCommand, "rover_arm/command", self._on_command, 10)
        self.create_service(HomeJoint, "rover_arm/home_joint", self._on_home_joint)
        self.create_service(EmergencyStop, "rover_arm/emergency_stop", self._on_emergency_stop)
        self.create_service(ArmPreset, "rover_arm/arm_preset", self._on_arm_preset)

        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f"rover_arm bridge starting on {self._link.port} @ {self._link.baud} baud"
        )

    def _on_command(self, msg: ArmCommand) -> None:
        self._last_command = msg
        self._last_command_time = time.monotonic()

    def _on_home_joint(self, request: HomeJoint.Request, response: HomeJoint.Response) -> HomeJoint.Response:
        index = int(request.joint_index)
        if index < -1 or index > 4:
            response.accepted = False
            response.message = f"rejected: joint_index must be -1..4, got {index}"
            return response

        try:
            frame = arm_protocol.encode_home_request(index)
        except Exception as exc:
            response.accepted = False
            response.message = f"rejected: {exc}"
            return response

        if self._link.write_frame(frame):
            response.accepted = True
            if index == -1:
                response.message = "homing all 5 joints"
            else:
                response.message = f"homing joint {index} ({_JOINT_NAMES[index]})"
            self.get_logger().info(f"home_joint service: {response.message}")
        else:
            response.accepted = False
            response.message = "rejected: write to serial link failed (port not open?)"
        return response

    def _on_emergency_stop(
        self, request: EmergencyStop.Request, response: EmergencyStop.Response
    ) -> EmergencyStop.Response:
        # Written directly, immediately - not queued behind the
        # regular per-tick _on_timer() send the way a plain ArmCommand
        # would be. An emergency stop waiting for the next control-rate
        # tick (up to 1/control_rate_hz away) defeats the point of it
        # being immediate; home_joint above gets away with the queued
        # approach because homing isn't time-critical the same way.
        frame = arm_protocol.encode_emergency_stop(bool(request.engage))
        if self._link.write_frame(frame):
            response.accepted = True
            response.message = "emergency stop engaged" if request.engage else "emergency stop cleared"
            self.get_logger().info(f"emergency_stop service: {response.message}")
        else:
            response.accepted = False
            response.message = "rejected: write to serial link failed (port not open?)"
        return response

    def _on_arm_preset(self, request: ArmPreset.Request, response: ArmPreset.Response) -> ArmPreset.Response:
        preset = int(request.preset)
        try:
            frame = arm_protocol.encode_preset_request(preset)
        except Exception as exc:
            response.accepted = False
            response.message = f"rejected: {exc}"
            return response

        if self._link.write_frame(frame):
            response.accepted = True
            response.message = f"moving to {_PRESET_NAMES[preset]} position"
            self.get_logger().info(f"arm_preset service: {response.message}")
        else:
            response.accepted = False
            response.message = "rejected: write to serial link failed (port not open?)"
        return response

    def _on_timer(self) -> None:
        self._drain_incoming()

        if self.get_parameter("home_on_startup").value and not self._sent_home_request:
            if self._link.write_frame(arm_protocol.encode_home_request()):
                self._sent_home_request = True
                self.get_logger().info("sent homing request to arm Mega")
            return  # wait for the next tick before sending a joint command

        # joint_target_steps is a fixed-size int32[5] array field - rclpy
        # backs fixed-size numeric array fields with numpy.ndarray
        # internally (this applies even to a plain Python list assigned
        # to the field, e.g. this node's own __init__ default), so its
        # elements come out as numpy.int32, not Python int. isinstance(
        # numpy.int32(0), int) is False, which used to reach
        # encode_joint_command's strict int-only check and crash here.
        # Converting at this boundary - where a ROS message field's
        # value turns into a plain Python value this node reasons about -
        # rather than relying solely on the protocol layer's own cast.
        raw_targets = [int(t) for t in self._last_command.joint_target_steps]

        stale = (time.monotonic() - self._last_command_time) > self._command_timeout_sec
        if stale:
            # Hold last known position with drivers enabled rather than
            # forcing a jump; the base/mast bridges fail-safe to a full
            # stop, but an arm holding position under load is usually
            # safer than de-energizing mid-air.
            targets = raw_targets or [0] * 5
            enable = self._last_command.enable
        else:
            targets = raw_targets
            enable = self._last_command.enable

        # Sent unconditionally, even mid-homing - harmless now, though
        # it wasn't always: the firmware's own handleJointCommand()
        # used to apply enable unconditionally too, before its own
        # homed/homingInProgress gate, so a stale enable=false default
        # arriving here mid-homing could actually disable the drivers
        # startHoming() had just energized. Fixed on the firmware side
        # (setDriversEnabled() moved inside the gate) rather than by
        # adding coordination here - this node still doesn't need to
        # know or care whether a homing run is in progress when it
        # sends its own periodic frame, on its own, independent
        # (service-callback) timing from the homing service above.
        frame = arm_protocol.encode_joint_command(targets, enable)
        self._link.write_frame(frame)
        self._publish_status()

    def _drain_incoming(self, max_reads: int = 8) -> None:
        for _ in range(max_reads):
            result = self._link.read_decoded()
            if result is None:
                return
            msg_type, fields = result
            if msg_type == "S":
                self._publish_joint_state(fields)

    def _publish_joint_state(self, fields) -> None:
        try:
            positions, limits, joint_homed, voltage_mv, temperature_deci_c, fan_duty_percent, estop_active, drivers_enabled = (
                arm_protocol.parse_joint_state(fields)
            )
        except Exception as exc:
            self.get_logger().warn(f"dropping malformed arm state frame: {exc}")
            return

        state = ArmState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.joint_position_steps = positions
        state.limit_switch_triggered = limits
        state.joint_homed = joint_homed
        state.homed = all(joint_homed)
        state.supply_voltage_mv = voltage_mv
        state.board_temperature_decic = temperature_deci_c
        state.fan_duty_percent = fan_duty_percent
        state.estop_active = estop_active
        state.drivers_enabled = drivers_enabled
        self._state_pub.publish(state)

    def _publish_status(self) -> None:
        status = BoardStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.board_name = "arm_mega2"
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
    node = ArmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
