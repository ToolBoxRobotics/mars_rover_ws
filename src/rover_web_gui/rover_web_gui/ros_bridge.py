"""Runs one rclpy Node in a background thread and exposes it to the
FastAPI app (which lives on asyncio in the main thread) through a
plain, thread-safe Python interface: a lock-protected telemetry
snapshot dict, publisher wrapper methods, and service-call helpers.

This is the standard pattern for bridging ROS 2 and an asyncio web
framework - rclpy's executor and FastAPI/uvicorn's event loop are two
independent loops that don't share thread-affinity requirements for
simple publish/subscribe, as long as shared state is protected by a
lock (spin() itself must not be called concurrently from two threads,
which is why it owns its own dedicated thread here).
"""

from __future__ import annotations

import threading
from typing import Optional

from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu, NavSatFix
from std_srvs.srv import Trigger

from rover_msgs.msg import (
    AntennaCommand,
    AntennaState,
    ArmCommand,
    ArmState,
    BaseCommand,
    BaseState,
    BoardStatus,
    DriveMode,
    MastCommand,
    MastState,
    MicroscopeCommand,
    MicroscopeState,
    PowerState,
)
from rover_msgs.srv import ArmPreset, EmergencyStop, HomeJoint


class RosBridge:
    def __init__(self) -> None:
        self._node = Node("rover_web_gui")

        # Shared with the physical Xbox controller via
        # rover_teleop/config/drive_sensitivity.yaml (loaded through
        # the `/**` wildcard by both nodes' launch files) - single
        # source of truth for how fast either input device can command
        # the rover, instead of separately hardcoded values drifting
        # apart. Defaults here only matter if that file isn't loaded
        # (e.g. running web_gui_node standalone without its launch file).
        self._node.declare_parameter("max_linear_mps", 0.65)
        self._node.declare_parameter("max_angular_radps", 1.5)
        self._node.declare_parameter("deadzone", 0.12)
        # Same reasoning, from the shared rover_mast/config/mast_topology.yaml -
        # the mast's "TRANSPORT POSITION" button needs to know where
        # that actually is without hardcoding it separately in JS.
        self._node.declare_parameter("transport_head_yaw_deg", 0.0)
        self._node.declare_parameter("transport_head_pitch_deg", 0.0)
        # Same reasoning again, from the shared
        # rover_antenna/config/antenna_topology.yaml - the antenna
        # panel's azimuth/elevation sliders need their real operational
        # range rather than a hardcoded guess in JS.
        self._node.declare_parameter("min_azimuth_deg", 15.0)
        self._node.declare_parameter("max_azimuth_deg", 285.0)
        self._node.declare_parameter("min_elevation_deg", 0.0)
        self._node.declare_parameter("max_elevation_deg", 180.0)
        self._lock = threading.Lock()

        # Keep this in sync with get_snapshot()'s own return dict below,
        # AND with each _on_X_state() callback's own hand-built dict
        # further down - a key added to a ROS message but missed in one
        # of these three places is a real, already-happened bug twice
        # over now: once for antenna's telemetry never reaching the
        # frontend at all (this dict / get_snapshot()), and again for
        # every subsystem's board_temperature_decic silently never being
        # captured out of the incoming message in the first place (the
        # _on_X_state() callbacks - a bug get_snapshot() being correct
        # couldn't have caught, since there was nothing there yet to
        # pass through). Nothing enforces these three staying in sync;
        # this comment is the enforcement.
        self._state: dict = {
            "board_status": {},  # board_name -> dict
            "base": None,
            "drive_mode": DriveMode.ACKERMANN,
            "arm": None,
            "mast": None,
            "microscope": None,
            "antenna": None,
            "power": None,
            "imu": None,
            "gps_fix": None,
        }
        self._latest_microscope_jpeg: Optional[bytes] = None
        self._latest_main_camera_jpeg: Optional[bytes] = None

        self._cmd_vel_pub = self._node.create_publisher(Twist, "cmd_vel", 10)
        self._drive_mode_pub = self._node.create_publisher(DriveMode, "rover_base/drive_mode", 10)
        self._arm_pub = self._node.create_publisher(ArmCommand, "rover_arm/command", 10)
        self._mast_pub = self._node.create_publisher(MastCommand, "rover_mast/command", 10)
        self._microscope_pub = self._node.create_publisher(MicroscopeCommand, "rover_microscope/command", 10)
        self._antenna_pub = self._node.create_publisher(AntennaCommand, "rover_antenna/command", 10)

        self._node.create_subscription(BaseState, "rover_base/state", self._on_base_state, 10)
        self._node.create_subscription(BaseCommand, "rover_base/command_echo", self._on_base_command_echo, 10)
        self._node.create_subscription(ArmState, "rover_arm/state", self._on_arm_state, 10)
        self._node.create_subscription(MastState, "rover_mast/state", self._on_mast_state, 10)
        self._node.create_subscription(MicroscopeState, "rover_microscope/state", self._on_microscope_state, 10)
        self._node.create_subscription(AntennaState, "rover_antenna/state", self._on_antenna_state, 10)
        self._node.create_subscription(PowerState, "rover_power/state", self._on_power_state, 10)
        self._node.create_subscription(Imu, "rover_sensors/imu/data", self._on_imu, 10)
        self._node.create_subscription(NavSatFix, "rover_sensors/gps/fix", self._on_gps_fix, 10)

        for board_topic in (
            "rover_base/board_status",
            "rover_arm/board_status",
            "rover_mast/board_status",
            "rover_microscope/board_status",
            "rover_antenna/board_status",
            "rover_power/board_status",
        ):
            self._node.create_subscription(
                BoardStatus, board_topic, self._on_board_status, 10
            )

        self._node.create_subscription(
            CompressedImage, "rover_microscope/image/compressed", self._on_microscope_frame, 5
        )
        self._node.create_subscription(
            CompressedImage, "rover_sensors/main_camera/image/compressed", self._on_main_camera_frame, 5
        )

        self._snapshot_client = self._node.create_client(Trigger, "rover_microscope/take_snapshot")
        self._recording_client = self._node.create_client(Trigger, "rover_microscope/toggle_recording")
        self._home_joint_client = self._node.create_client(HomeJoint, "rover_arm/home_joint")
        self._emergency_stop_client = self._node.create_client(EmergencyStop, "rover_arm/emergency_stop")
        self._arm_preset_client = self._node.create_client(ArmPreset, "rover_arm/arm_preset")

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self._executor.shutdown()
        self._node.destroy_node()

    # -- subscription callbacks (run on the executor thread) -----------------
    def _on_base_state(self, msg: BaseState) -> None:
        with self._lock:
            self._state["base"] = {
                # int(x) for x in ... , not list(...) directly - msg.encoder_ticks
                # is a fixed-size int32[] array field, which rclpy backs with
                # numpy.ndarray; list() on that produces numpy.int32 elements,
                # not plain Python int. json.dumps() cannot serialize numpy.int32
                # at all - it raises TypeError, uncaught, inside
                # _telemetry_sender()'s own asyncio task in server.py, which
                # silently kills that task for every board at once, not just
                # this one. Same underlying bug class already fixed once in
                # arm_bridge_node.py's own _on_timer() - this is a second,
                # independent occurrence of it, in a different file, not a
                # recurrence of the same one.
                "encoder_ticks": [int(t) for t in msg.encoder_ticks],
                "encoder_delta_ticks": [int(t) for t in msg.encoder_delta_ticks],
                "drive_voltage_mv": msg.drive_voltage_mv,
                "steering_voltage_mv": msg.steering_voltage_mv,
                "board_temperature_decic": msg.board_temperature_decic,
                "fan_duty_percent": msg.fan_duty_percent,
            }

    def _on_base_command_echo(self, msg: BaseCommand) -> None:
        with self._lock:
            self._state["drive_mode"] = msg.drive_mode

    def _on_arm_state(self, msg: ArmState) -> None:
        with self._lock:
            self._state["arm"] = {
                # Same fix, same reasoning as _on_base_state's own comment
                # above - joint_position_steps is int32[5], limit_switch_
                # triggered/joint_homed are bool[5]; every one of these is a
                # fixed-size array field, so every one needs the same explicit
                # per-element cast (numpy.bool_ is exactly as unserializable
                # to json.dumps() as numpy.int32 - this isn't an int-specific
                # issue, it's a numpy-scalar-specific one).
                "joint_position_steps": [int(p) for p in msg.joint_position_steps],
                "limit_switch_triggered": [bool(t) for t in msg.limit_switch_triggered],
                "joint_homed": [bool(h) for h in msg.joint_homed],
                "homed": msg.homed,
                "supply_voltage_mv": msg.supply_voltage_mv,
                "board_temperature_decic": msg.board_temperature_decic,
                "fan_duty_percent": msg.fan_duty_percent,
                "estop_active": msg.estop_active,
            }

    def _on_mast_state(self, msg: MastState) -> None:
        with self._lock:
            self._state["mast"] = {
                "head_yaw_decideg": msg.head_yaw_decideg,
                "head_pitch_decideg": msg.head_pitch_decideg,
                "lift_state": msg.lift_state,
                "yaw_limit_triggered": msg.yaw_limit_triggered,
                "pitch_limit_triggered": msg.pitch_limit_triggered,
                "homed": msg.homed,
                "driver_enabled": msg.driver_enabled,
                "supply_voltage_mv": msg.supply_voltage_mv,
                "board_temperature_decic": msg.board_temperature_decic,
                "fan_duty_percent": msg.fan_duty_percent,
            }

    def _on_antenna_state(self, msg: AntennaState) -> None:
        with self._lock:
            self._state["antenna"] = {
                "azimuth_decideg": msg.azimuth_decideg,
                "elevation_decideg": msg.elevation_decideg,
                "azimuth_limit_triggered": msg.azimuth_limit_triggered,
                "elevation_limit_triggered": msg.elevation_limit_triggered,
                "homed": msg.homed,
                "driver_enabled": msg.driver_enabled,
                "supply_voltage_mv": msg.supply_voltage_mv,
                "board_temperature_decic": msg.board_temperature_decic,
                "fan_duty_percent": msg.fan_duty_percent,
            }

    def _on_power_state(self, msg: PowerState) -> None:
        with self._lock:
            self._state["power"] = {
                "battery1_voltage_mv": msg.battery1_voltage_mv,
                "battery1_current_ma": msg.battery1_current_ma,
                "battery2_voltage_mv": msg.battery2_voltage_mv,
                "battery2_current_ma": msg.battery2_current_ma,
                "computer_temperature_decic": msg.computer_temperature_decic,
                "fan_duty_percent": msg.fan_duty_percent,
            }

    def _on_microscope_state(self, msg: MicroscopeState) -> None:
        with self._lock:
            self._state["microscope"] = {
                "focus_position_steps": msg.focus_position_steps,
                "led_pwm": msg.led_pwm,
                "cover_open": msg.cover_open,
                "homed": msg.homed,
                "driver_enabled": msg.driver_enabled,
                "board_temperature_decic": msg.board_temperature_decic,
                "fan_duty_percent": msg.fan_duty_percent,
            }

    def _on_board_status(self, msg: BoardStatus) -> None:
        with self._lock:
            self._state["board_status"][msg.board_name] = {
                "connected": msg.connected,
                "rx_frame_count": msg.rx_frame_count,
                "checksum_error_count": msg.checksum_error_count,
                "reconnect_count": msg.reconnect_count,
                "last_rx_age_sec": msg.last_rx_age_sec,
            }

    def _on_imu(self, msg: Imu) -> None:
        with self._lock:
            self._state["imu"] = {
                "orientation": {
                    "x": msg.orientation.x,
                    "y": msg.orientation.y,
                    "z": msg.orientation.z,
                    "w": msg.orientation.w,
                },
                "linear_acceleration": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z,
                },
            }

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        with self._lock:
            self._state["gps_fix"] = {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "altitude": msg.altitude,
                "status": msg.status.status,
            }

    def _on_microscope_frame(self, msg: CompressedImage) -> None:
        with self._lock:
            self._latest_microscope_jpeg = bytes(msg.data)

    def _on_main_camera_frame(self, msg: CompressedImage) -> None:
        with self._lock:
            self._latest_main_camera_jpeg = bytes(msg.data)

    # -- read side, called from FastAPI's asyncio thread ---------------------
    def get_snapshot(self) -> dict:
        # Keep in sync with __init__'s self._state declaration above -
        # see that comment for why this matters.
        with self._lock:
            return {
                "board_status": dict(self._state["board_status"]),
                "base": self._state["base"],
                "drive_mode": self._state["drive_mode"],
                "arm": self._state["arm"],
                "mast": self._state["mast"],
                "microscope": self._state["microscope"],
                "antenna": self._state["antenna"],
                "power": self._state["power"],
                "imu": self._state["imu"],
                "gps_fix": self._state["gps_fix"],
            }

    def get_static_config(self) -> dict:
        """Values fixed at launch time (not live telemetry) that the
        frontend needs once on page load rather than hardcoded in JS,
        via GET /api/config - drive sensitivity for the virtual
        joystick, the mast's transport-position preset, and the
        antenna's azimuth/elevation range. All three come from shared
        yaml files also loaded by other nodes (rover_teleop's Xbox
        controller for the first, nothing else for the other two
        currently), so this can never silently drift from what those
        actually use. Kept as one flat dict rather than nested
        drive/mast/antenna sections - small enough that nesting would
        add structure without adding clarity, and it avoids touching
        the frontend's existing driveConfig.* reads for the keys that
        were already here.
        """
        return {
            "max_linear_mps": float(self._node.get_parameter("max_linear_mps").value),
            "max_angular_radps": float(self._node.get_parameter("max_angular_radps").value),
            "deadzone": float(self._node.get_parameter("deadzone").value),
            "transport_head_yaw_deg": float(self._node.get_parameter("transport_head_yaw_deg").value),
            "transport_head_pitch_deg": float(self._node.get_parameter("transport_head_pitch_deg").value),
            "min_azimuth_deg": float(self._node.get_parameter("min_azimuth_deg").value),
            "max_azimuth_deg": float(self._node.get_parameter("max_azimuth_deg").value),
            "min_elevation_deg": float(self._node.get_parameter("min_elevation_deg").value),
            "max_elevation_deg": float(self._node.get_parameter("max_elevation_deg").value),
        }

    def get_latest_microscope_jpeg(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_microscope_jpeg

    def get_latest_main_camera_jpeg(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_main_camera_jpeg

    # -- command side, called from FastAPI's asyncio thread ------------------
    def send_drive(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_vel_pub.publish(msg)

    def send_drive_mode(self, mode: int) -> None:
        self._drive_mode_pub.publish(DriveMode(mode=int(mode)))

    def send_arm(self, joint_target_steps, enable: bool) -> None:
        msg = ArmCommand()
        msg.joint_target_steps = [int(v) for v in joint_target_steps]
        msg.enable = bool(enable)
        self._arm_pub.publish(msg)

    def send_mast(self, head_yaw_decideg: int, head_pitch_decideg: int, lift_mode: int, driver_enable: bool) -> None:
        msg = MastCommand()
        msg.head_yaw_decideg = int(head_yaw_decideg)
        msg.head_pitch_decideg = int(head_pitch_decideg)
        msg.lift_mode = int(lift_mode)
        msg.driver_enable = bool(driver_enable)
        self._mast_pub.publish(msg)

    def send_antenna(self, azimuth_decideg: int, elevation_decideg: int, driver_enable: bool) -> None:
        msg = AntennaCommand()
        msg.azimuth_decideg = int(azimuth_decideg)
        msg.elevation_decideg = int(elevation_decideg)
        msg.driver_enable = bool(driver_enable)
        self._antenna_pub.publish(msg)

    def send_microscope(self, focus_target_steps: int, led_pwm: int, cover_open: bool, driver_enable: bool) -> None:
        msg = MicroscopeCommand()
        msg.focus_target_steps = int(focus_target_steps)
        msg.led_pwm = int(led_pwm)
        msg.cover_open = bool(cover_open)
        msg.driver_enable = bool(driver_enable)
        self._microscope_pub.publish(msg)

    def call_snapshot(self) -> dict:
        return self._call_trigger(self._snapshot_client)

    def call_toggle_recording(self) -> dict:
        return self._call_trigger(self._recording_client)

    def call_home_joint(self, joint_index: int) -> dict:
        """joint_index: -1 homes all 5 arm joints, 0-4 homes just that one."""
        return self._call_accepted_service(self._home_joint_client, HomeJoint.Request(joint_index=int(joint_index)))

    def call_emergency_stop(self, engage: bool) -> dict:
        """engage=True latches the arm's e-stop, False clears it."""
        return self._call_accepted_service(self._emergency_stop_client, EmergencyStop.Request(engage=bool(engage)))

    def call_arm_preset(self, preset: int) -> dict:
        """preset: 0=initial, 1=transport, 2=service - see arm_protocol.py's own PRESET_* constants (ROS-side, not directly importable here, so kept as a plain int at this layer)."""
        return self._call_accepted_service(self._arm_preset_client, ArmPreset.Request(preset=int(preset)))

    def _call_accepted_service(self, client, request) -> dict:
        """Shared by call_home_joint/call_emergency_stop/call_arm_preset -
        all three request types share the same {accepted, message}
        response shape (the "validate locally, report whether the
        write succeeded" pattern documented in arm_bridge_node.py's own
        module docstring), so the actual call/wait/extract logic only
        needs writing once.
        """
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                return {"accepted": False, "message": "service unavailable"}

        future = client.call_async(request)

        # Same threading.Event pattern as _call_trigger below - the node
        # is already being spun by the background MultiThreadedExecutor,
        # so this thread waits for the future rather than spinning it.
        done_event = threading.Event()
        future.add_done_callback(lambda _f: done_event.set())
        if not done_event.wait(timeout=3.0):
            return {"accepted": False, "message": "service call timed out"}

        result = future.result()
        if result is None:
            return {"accepted": False, "message": "service call returned no result"}
        return {"accepted": result.accepted, "message": result.message}

    def _call_trigger(self, client) -> dict:
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                return {"success": False, "message": "service unavailable"}

        future = client.call_async(Trigger.Request())

        # The node is already being spun continuously by the background
        # MultiThreadedExecutor (see start()), so we must NOT call
        # rclpy.spin_until_future_complete() here - that would spin the
        # same node concurrently from a second thread. Instead, wait for
        # the executor thread to resolve the future via a plain
        # threading.Event, which is safe to block on from this (FastAPI
        # worker) thread.
        done_event = threading.Event()
        future.add_done_callback(lambda _f: done_event.set())
        if not done_event.wait(timeout=3.0):
            return {"success": False, "message": "service call timed out"}

        result = future.result()
        if result is None:
            return {"success": False, "message": "service call returned no result"}
        return {"success": result.success, "message": result.message}
