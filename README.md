# Mars Rover — ROS 2 Humble Workspace

A 6-wheel, 4-corner-steering Mars exploration rover (Opportunity-style
rocker-bogie steering geometry) with a 5-axis arm, a microscope module
at the 3rd wrist joint, a full sensor suite, Xbox 360 teleop, and a
browser-based ground control console.

**This workspace was rebuilt from scratch** against a new, simplified
hardware spec. It intentionally does not carry over assumptions from
any earlier iteration of this project (e.g. the main perception camera
is now a plain USB/OpenCV webcam per the current spec, not the
previously-selected depth camera — see `rover_description`).

Host OS: Ubuntu 22.04.5 LTS · ROS 2 Humble.

Three documents, three different jobs: **this README** is the
engineering reference (architecture, design decisions, why things are
built the way they are); **`docs/INSTALL.md`** is the one-time
from-scratch build and bring-up guide; **`docs/USER_MANUAL.md`** is
the day-to-day operating guide for a rover that's already built and
running — start-up, driving, arm/mast/microscope controls, and what
the telemetry means.

## Hardware topology

| Board / device | Connects | Controls | ROS package |
|---|---|---|---|
| Arduino Mega #1 | `/dev/rover/base` | 6x drive motor (2x with encoders - see Navigation below), 3x DRI0002 Dual H-Bridge V1.4 (L298N) driver, 4x steering servo via PCA9685 (I2C, address 0x40 - factory default, deliberately unjumpered), 2x FZ0430 voltage sensor (drive rail on A0, steering rail on A1), DS18B20 temperature sensor (1-Wire, pin A4), cooling fan (MOSFET driver module, hardware PWM, pin 44, automatic/thermostatic) | `rover_base` |
| Arduino Mega #2 | `/dev/rover/arm` | 5x NEMA17 + EBA-17-M planetary gearbox (120:1); J1-J3 TB6600, J4-J5 A4988; 5x calibration switch (per-joint configurable homing direction/order/offset - see "Explicit assumptions"); FZ0430 voltage sensor (A0); DS18B20 temperature sensor (1-Wire, pin 20); cooling fan (MOSFET driver module, hardware PWM, pin 44, automatic/thermostatic); latching emergency stop and 3 predefined poses (initial/transport/service) | `rover_arm` |
| Arduino Uno #3 | `/dev/rover/mast` | 2x NEMA17 head yaw/pitch (TB6600 driver, + calibration switches), DC lift (HW-039 driver, erect/stow), 2x lift limit switch, FZ0430 voltage sensor (A0), DS18B20 temperature sensor (1-Wire, pin A4), cooling fan (MOSFET driver module, software PWM, pin A2, automatic/thermostatic) | `rover_mast` |
| Arduino Uno #4 | `/dev/rover/microscope` | DRV8825 + 24BYJ-48 focus/zoom stepper (4-wire bipolar wiring), dimmable LED, SG90 lens cover, DS18B20 temperature sensor (1-Wire, pin 11), cooling fan (MOSFET driver module, hardware PWM, pin 3, automatic/thermostatic) | `rover_microscope` |
| Arduino Uno #5 | `/dev/rover/antenna` | 2x NEMA17 + EBA-17-M planetary gearbox (120:1) + TB6600, high-gain-antenna gimbal (azimuth G1 + elevation G2), 2x calibration switch, FZ0430 voltage sensor (A0), DS18B20 temperature sensor (1-Wire, pin A4), cooling fan (MOSFET driver module, hardware PWM, pin 9, automatic/thermostatic) | `rover_antenna` |
| Arduino Uno #6 | `/dev/rover/power` | 2x INA226 voltage+current monitor (one per 24V battery, behind a TCA9548A I2C mux, A4/A5 - see "Explicit assumptions" for why the mux is needed and a real caveat about current-reading accuracy), DS18B20 temperature sensor (1-Wire, pin 2, monitors the onboard computer, not this board's own enclosure), cooling fan for that computer (MOSFET driver module, hardware PWM, pin 3, automatic/thermostatic). Telemetry-only - no command message, this board controls nothing an operator sends it | `rover_power` |
| Microscope USB camera | `/dev/rover/microscope_cam` | live view, snapshot, recording | `rover_microscope` |
| Main USB camera | `/dev/rover/main_cam` | forward perception (OpenCV) | `rover_sensors` |
| BNO086 IMU (SparkFun Qwiic breakout, UART-RVC jumper mode) | `/dev/rover/imu` (Waveshare USB-TTL (B), CH343G) | orientation + accel, **no Arduino** | `rover_sensors` |
| Waveshare L76X GPS | `/dev/rover/gps` (USB, NMEA0183) | position/velocity, **no Arduino** | `rover_sensors` |
| RPLIDAR C1 | `/dev/rover/lidar` | 360° scan for autonomous nav | external `rplidar_ros` (or `sllidar_ros2`) |
| Xbox 360 controller | USB/wireless receiver | mode-switched drive/arm/mast/microscope teleop | `rover_teleop` (on top of `joy`) |

The last five rows need no Arduino at all — the IMU and GPS modules
free-run straight to a USB-serial adapter, the cameras are plain
UVC/OpenCV devices, and the LIDAR ships its own ROS driver.

All nine serial/video devices are referenced by the stable
`/dev/rover/*` names above; see `rover_bringup/config/udev/99-rover-serial.rules`
for the udev rules that create them, and `tools/identify_rover_devices.py`
for generating them interactively from your own hardware rather than
hand-editing placeholders (including an important caveat about the
three Mega 2560 boards possibly sharing an identical or empty USB
serial number).


These show pin groupings and counts, not exact per-pin schematics —
cross-reference the exact pin constants in each board's `.ino` file
(linked from the hardware topology table above) before wiring, and see
that same file's header comment for power-supply notes (the DRI0002
logic-jumper caveat, servo power needing a separate 5-6V rail, the
lift motor's different dual-direction-pin H-bridge convention) that
don't fit in a pin-level diagram.

## Package map

```
src/
  rover_protocol/     shared checksummed serial framing + SerialLink, used by every bridge
  rover_msgs/         custom interfaces (Base/Arm/Mast/Microscope/Antenna Command+State, BoardStatus, DriveMode) + HomeJoint.srv
  rover_description/  URDF/xacro model (chassis, arm, mast, sensors, solar panels, antenna) + RViz display launch
  rover_base/         Mega #1 bridge: cmd_vel -> 4-corner-steering kinematics -> serial, + wheel odometry
  rover_arm/          Mega #2 bridge: joint-space ArmCommand -> serial, calibration homing, MoveIt trajectory bridge
  rover_arm_moveit_config/ MoveIt2 planning config for the 5-axis arm (SRDF, kinematics, OMPL, controllers)
  rover_mast/         Uno #3 bridge: pan/tilt head (calibration homing) + erect/stow lift -> serial
  rover_microscope/   Uno #4 bridge + USB camera publisher w/ snapshot & recording services
  rover_antenna/      Uno #5 bridge: 2-axis (azimuth/elevation) HGA gimbal -> serial
  rover_sensors/      BNO086 (UART-RVC) IMU driver, L76X GPS driver, main camera publisher
  rover_teleop/       Xbox 360 -> drive/arm/mast/microscope command mapping (built on `joy`)
  rover_web_gui/      FastAPI + WebSocket ground control console (see below)
  rover_navigation/   SLAM (slam_toolbox), sensor fusion + GPS (robot_localization), Nav2 config and launch files
  rover_bringup/      top-level launch file, udev rules, serial-port config
firmware/
  common/RoverProtocol/  Arduino-side counterpart to rover_protocol (install as a library)
  base_mega1/            base_mega1.ino
  arm_mega2/              arm_mega2.ino
  mast_uno3/             mast_uno3.ino
  microscope_uno4/       microscope_uno4.ino
tools/
  validate_workspace.py     syntax/parse sweep (Python/XML/YAML/JS/brace-balance/msg) over the whole tree
  identify_rover_devices.py interactive udev VID:PID/serial capture -> generates 99-rover-serial.rules
  raw_serial_probe.py       ROS-free probe: does a board reply to a raw frame at all? (bypasses rclpy/SerialLink)
```

## Base drive modes

The base supports three steering-geometry modes, selected via
`rover_msgs/DriveMode` on the `rover_base/drive_mode` topic (default
ACKERMANN if nothing has been received yet). `geometry_msgs/Twist` on
`cmd_vel` is reinterpreted differently depending on which mode is
active — see `rover_base/rover_base/kinematics.py` for the exact math.

| Mode | Twist fields used | What it does |
|---|---|---|
| `ACKERMANN` (0) | `linear.x`, `angular.z` | Normal driving — exact ICR-tangent corner steering, front/rear pairs mirror for coordinated arcs |
| `POINT_TURN` (1) | `angular.z` only | Rotate about the rover's own center — forward component forced to zero |
| `STOP` (2) | none (Twist ignored) | Unconditional zero throttle, centered steering |

Both from the Xbox controller (in DRIVE subsystem mode: **X** toggles
ACKERMANN↔POINT_TURN, **Y** forces STOP immediately — see
`rover_teleop/config/xbox_teleop.yaml`) and from the web GUI (mode
buttons at the top of the DRIVE panel, plus a draggable on-screen
joystick for proportional control — see `static/app.js`) publish to
the same topic, so either can drive and either can see what mode is
currently active (`rover_base/command_echo`'s `drive_mode` field is
the ground truth, which both the web GUI's telemetry and — if you
build one — any autonomy stack should read rather than assume).
Both input devices scale stick/joystick deflection by the same
`max_linear_mps`/`max_angular_radps`/`deadzone` values, loaded from
one shared file (`rover_teleop/config/drive_sensitivity.yaml`) rather
than two separately hardcoded copies — see `docs/INSTALL.md` Section
9.4 for how to tune them.

**Hardware caveat, not a bug** (documented in detail as a code comment
on `point_turn_wheel_commands()`): POINT_TURN is bounded by the same
±60° servo limit as ACKERMANN corner steering, so it can't reach a
perfect zero-radius pivot for wheels close to the vehicle's center
(the real angle needed there exceeds 90°) — the achieved motion is the
closest approximation the hardware allows, same limitation already
noted for ACKERMANN's own point-turn-via-zero-forward-velocity case.

## Navigation (SLAM + Nav2 + sensor fusion)

Wheel odometry, IMU/encoder fusion (`robot_localization`), GPS
conversion services, SLAM (`slam_toolbox`), and autonomous navigation
(Nav2) are all wired together. Sensor fusion runs unconditionally as
part of normal bringup (it strictly improves what SLAM/Nav2 already
consume); SLAM and Nav2 navigation are both off by default and
mutually exclusive with each other — build a map first, then navigate
against it, not both at once.

### Architecture

```
encoders (ML, MR) ──┐
                     ├─→ rover_base/odometry_node ─→ wheel_odom ──┐
IMU (BNO086) ────────┼──────────────────────────────────────────┤
                     │                                            ├─→ local EKF ─→ odom (TF: odom → base_link)
                     └────────────────────────────────────────────┘
GPS (L76X) ──→ navsat_transform_node ──→ /odometry/gps (telemetry) + /fromLL, /toLL (services)

slam_toolbox (mapping) or AMCL (navigating) ─→ TF: map → odom
```

- **Wheel odometry** (`rover_base/odometry_node`) reads only the two
  fixed middle wheels (ML/MR) — the only pair whose rolling axis never
  changes with corner steering. See `rover_base/rover_base/odometry.py`'s
  module docstring for the full reasoning. It no longer broadcasts any
  TF itself; it feeds the EKF instead (see below).
- **Local EKF** (`robot_localization`'s `ekf_node`, `rover_navigation/config/ekf_local_params.yaml`)
  fuses that wheel odometry with the BNO086 IMU and becomes the sole
  publisher of the `odom → base_link` TF — smoother and more accurate
  than either sensor alone, which directly improves slam_toolbox's
  scan matching and Nav2's local costmap. Fusion fields are matched
  to what each sensor actually measures rather than a generic
  "fuse-everything" template: wheel odometry contributes velocity
  only (not its own heading estimate, so the IMU's independently
  measured heading corrects drift instead of reinforcing the same
  error twice); the IMU contributes yaw and linear acceleration but
  **not** roll/pitch (redundant under `two_d_mode`, which already
  constrains the filter's own state to zero regardless) or angular
  velocity (the BNO086 in UART-RVC mode doesn't provide it — see
  `bno086_rvc_node.py`).
- **GPS** (`navsat_transform_node`, `rover_navigation/config/navsat_transform_params.yaml`)
  deliberately does **not** feed a second, GPS-fusing "global" EKF —
  that's the common `robot_localization` pattern for outdoor robots,
  but it would fight slam_toolbox/AMCL for ownership of the
  `map → odom` TF (both would try to publish the same edge). Instead
  GPS is used two conflict-free ways: telemetry (`/odometry/gps`, a
  Cartesian rendering of "where GPS thinks the rover is," useful to
  log/compare against the SLAM-based estimate) and **GPS waypoint
  goals** — `rover_navigation/scripts/gps_goal.py` converts a lat/lon
  into the current map-frame position via the `/fromLL` service, then
  sends that straight to Nav2's `navigate_to_pose` action. Nav2 and
  AMCL never need to know GPS was involved at all.
  ```bash
  ros2 run rover_navigation gps_goal.py 42.4373 -86.9436
  ```
  (Needs Nav2 already running against a saved, geo-referenced map —
  the conversion is relative to wherever the datum got set, i.e.
  wherever the rover had a GPS fix when `navsat_transform_node` first
  started.)
- `robot_state_publisher` (already running as part of normal bringup)
  fills in `base_link → ...sensor frames` from the URDF, including
  `base_link → lidar_link`. `slam_toolbox` (while mapping) or Nav2's
  AMCL (while navigating a saved map) supply the remaining
  `map → odom` link — the full chain
  (`map → odom → base_link → lidar_link`) is what lets the LIDAR's
  `/scan` data actually place obstacles on a consistent map.

### Build a map

```bash
ros2 launch rover_bringup bringup.launch.py use_slam:=true
```
Drive the rover around (Xbox teleop or the web GUI) until RViz shows a
complete map, then save it:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/mars_rover_ws/src/rover_navigation/maps/my_map
```

### Navigate autonomously against that map

```bash
ros2 launch rover_bringup bringup.launch.py use_navigation:=true \
    nav_map:=$HOME/mars_rover_ws/src/rover_navigation/maps/my_map.yaml
```
Send goals from RViz (`rover_navigation/rviz/navigation.rviz` has the
"2D Pose Estimate" and "Nav2 Goal" tools already added), via
`ros2 action send_goal /navigate_to_pose ...`, or by GPS coordinate
with `gps_goal.py` above.

### Dependencies

Not vendored in this workspace — see `docs/INSTALL.md`:
`slam_toolbox`, `navigation2`, `nav2_bringup`, `robot_localization`,
`geographic_msgs`. `slam_toolbox`/`navigation2`/`nav2_bringup` are
only actually required at runtime if you pass `use_slam:=true` or
`use_navigation:=true`; `robot_localization` and `geographic_msgs` are
needed unconditionally, since sensor fusion always runs.
`rover_navigation` itself (a lightweight config/launch/scripts-only
package) is always present.

### Not tuned for your environment

`rover_navigation/config/nav2_params.yaml` ships with standard,
reasonable starting values (footprint sized to the rover's actual
wheel extents, velocity limits matching `rover_base`), not something
pre-tuned against your specific space. `ekf_local_params.yaml`'s
process noise covariance is `robot_localization`'s own published
default, not independently re-derived; the measurement covariance
`rover_base/odometry_node.py` publishes on its wheel odometry
(`_TWIST_VARIANCE_*`) is a reasonable placeholder in the same spirit,
not characterized against this rover's actual encoders. Expect to
adjust all three after watching the rover drive and navigate a few
times.

## Arm motion planning (MoveIt2)

Task-space planning for the 5-axis arm (drag an interactive marker to
a target pose, plan a collision-aware path, execute it) via MoveIt2,
off by default (`use_moveit:=false`).

### Architecture

```
RViz "MotionPlanning" panel / any MoveGroupInterface client
        │  plans against the SRDF's "arm" group (shoulder_yaw..wrist_roll)
        ▼
move_group (rover_arm_moveit_config)
        │  planned trajectory (radians), via the FollowJointTrajectory action
        ▼
arm_controller/follow_joint_trajectory ─ rover_arm's trajectory_action_server
        │  converts each waypoint: radians → steps (joint_conversion.py),
        │  reorders to this rover's own joint order (MoveIt doesn't
        │  guarantee its joint_names arrives in that order)
        ▼
ArmCommand (joint_target_steps) ─ exactly what teleop/manual control already sends
        ▼
rover_arm_bridge → 'A' frame → arm Mega #2 → AccelStepper per joint
```

`rover_arm_bridge` and the firmware never change: MoveIt's trajectories
arrive as ordinary `ArmCommand` messages indistinguishable from
teleop's. The bridge between the two — `rover_arm`'s
`trajectory_action_server` — exists specifically because the firmware
already does its own velocity/acceleration profiling on-device
(`AccelStepper`); writing a full `ros2_control` hardware interface
just to satisfy MoveIt's usual execution path would mean reimplementing
that. `moveit_simple_controller_manager`'s `FollowJointTrajectory`
support is built for exactly this case — a robot controller that
already speaks that action interface directly.

Each waypoint is sent as an absolute joint-space target and the bridge
waits until the arm's within tolerance (`goal_tolerance_rad`, default
0.02) or the waypoint's planned time has elapsed, whichever comes
first, before advancing — not a precise replication of MoveIt's planned
velocity profile. Fine for a slow, deliberate sample-inspection arm;
worth knowing if trajectory timing fidelity ever matters more than it
does here.

The `steps_per_joint_rev` calibration (`rover_arm/config/arm_topology.yaml`)
existed as declared-but-unused placeholder data (200 full steps × 1/16
microstepping × 5:1 gear = 16000) since the arm's very first build —
this is what finally puts it to use.

### Using it

```bash
ros2 launch rover_bringup bringup.launch.py use_moveit:=true
ros2 launch rover_arm_moveit_config moveit_rviz.launch.py
```
In RViz's MotionPlanning panel: drag the interactive marker at the
arm's tip to a target pose, **Plan**, check the preview, **Execute**.
Or send a named target — `home` (all-zero, matching the arm's own
homed position) is defined in the SRDF.

The arm must have finished its own startup homing sequence
(`rover_arm_bridge`'s `home_on_startup`, see "How does the calibration
homing sequence work" — `ArmState.homed`) before `trajectory_action_server`
will accept a goal at all; it rejects goals otherwise rather than
commanding an un-homed arm to a fabricated "zero" that isn't real.

### Dependencies

Not vendored — see `docs/INSTALL.md`: `ros-humble-moveit`,
`ros-humble-moveit-configs-utils`. Only required at runtime if you
pass `use_moveit:=true`; `rover_arm_moveit_config` itself (config/launch
only) is always present, matching the same pattern SLAM/Nav2 already use.

## Arm calibration

Each joint's calibration-switch homing can now be triggered on demand
— either individually or all five at once — not just automatically at
bridge startup.

### Architecture

```
Web GUI "CALIBRATE J3" button
        │  POST /api/arm/home/2
        ▼
rover_arm_bridge's rover_arm/home_joint service (rover_msgs/srv/HomeJoint)
        │  validates joint_index locally, writes a 'Z' frame with that index
        ▼
arm Mega #2 → startHoming(2) → seeks J3's limit switch, zeroes it,
              leaves J1/J2/J4/J5's homed status and position untouched
```

`HomeJoint.srv` (`int8 joint_index` in; `bool accepted` + `string
message` out) is this project's first custom service — everything
before this was messages/topics only. `joint_index=-1` homes all 5
joints in one sequential pass (the same behavior `home_on_startup`
has always triggered once at boot); `0`-`4` homes just that one joint,
leaving every other joint's calibration and position alone —
`ArmState.joint_homed` (`bool[5]`) reports this per-joint, not as one
shared flag, precisely so partial calibration is visible rather than
collapsed into a single ambiguous "homed: false".

The service reports whether the request was successfully **written**
to the serial link, not whether the firmware will act on it — that's
a real boundary, not a shortcut: this layer can't see whether the
Arduino, say, silently ignores the request because a homing run is
already in progress. `ArmState.joint_homed` (via `ros2 topic echo
/rover_arm/state` or the web GUI's per-joint ✓/✗ indicators) is the
actual source of truth for whether/when homing completed.

### Deliberate simplification: no partial-homed motion

Every joint must be individually homed — and no homing run in
progress — before **any** joint-move command is accepted, even though
homing itself is now per-joint. A command that moved only the
already-homed joints while silently ignoring the rest would let the
arm move through configurations nothing has actually verified are
safe for the not-yet-homed joint's real position; waiting for a fully
calibrated arm before accepting any motion at all is simpler and
safer. `ArmState.homed` (`bool`) is `all(joint_homed)`, computed by
`rover_arm_bridge` — not sent over the wire, and kept specifically so
MoveIt's `trajectory_action_server` (which already gated goals on this
exact field) needed no changes for this to work.

### Configurable per-joint homing direction and order

Two things about the homing process that used to be a single, uniform
assumption applied to all five joints are now independently
configurable per joint, as constants in `arm_mega2.ino` — both
currently PLACEHOLDER values matching the previous, only behavior,
pending real mechanical verification on the bench:

- **`kHomingDirection[5]`** — which way each joint's stepper seeks
  toward its own limit switch (`+1` or `-1`). Previously a single
  hardcoded `-1` applied to every joint regardless of its own actual
  mechanical layout.
- **`kHomingOrder[5]`** — the sequence joints are homed in during an
  all-5 run (`joint_index=-1`). Previously a fixed J1 → J2 → J3 → J4 →
  J5 loop with no way to change it; a permutation of 0-4, useful for a
  mechanical/safety sequence where one joint genuinely needs to home
  before another (e.g. shoulder before wrist, to avoid a collision
  partway through the sweep). Only affects the all-5 case — a
  single-joint request (0-4) always just homes that one joint,
  unaffected by this order.

Neither is sent over the wire or configurable from the web GUI —
they're physical-calibration facts about the hardware itself, set once
in firmware to match the real joints, not something an operator
chooses per homing run.

### Steps-per-degree, the real lower limit, and the operational range

What this file used to call an "offset" (`kHomingOffsetSteps` — a
small, per-joint, separately-tuned distance from an assumed-zero
switch position to a joint's true reference point) has been replaced
entirely by a physically-grounded model, not just renamed. Three new
constants, none of them invented fresh — all three sourced directly
from numbers this project had already established elsewhere:

- **`kStepsPerDegree[5]`** — derived from, and must be kept in sync
  with, `rover_arm/config/arm_topology.yaml`'s own
  `steps_per_joint_rev` (currently 384000 for all five: 200 full steps
  × 1/16 microstepping × 120:1 EBA-17-M planetary gearbox — see that
  file's own comment). 384000 / 360 ≈ 1066.667 steps/degree.
- **`kMinDeg[5]` / `kMaxDeg[5]`** — each joint's own operational range
  in degrees, relative to true center. Sourced directly from
  `rover_description/urdf/arm.xacro`'s own joint `<limit>` tags
  (shoulder_yaw/elbow_pitch: ±150°, shoulder_pitch/wrist_pitch: ±100°,
  wrist_roll: ±170°) — the same numbers MoveIt itself already plans
  against, not a separately-guessed pair.
- **`kLowerLimitSteps[5]`** — the value assigned via
  `setCurrentPosition()` the instant a joint's limit switch trips. The
  limit switch's own physical trigger point is now understood to sit
  at each joint's own operational lower bound, not at an arbitrary
  "zero" the way `kHomingOffsetSteps` treated it. PLACEHOLDER values
  currently assume the switch sits exactly at `kMinDeg[j]` (converted
  to steps, rounded) — a reasonable starting assumption, not a
  bench-verified one. Kept independently adjustable from `kMinDeg` in
  code (not derived from it), so a real mechanical safety margin
  between "where the switch actually trips" and "the joint's own
  declared operational minimum" can be introduced later without
  touching either constant's own meaning.

Once the switch trips and `kLowerLimitSteps[j]` is assigned,
`serviceHoming()`'s own two-phase state machine (`SEEKING_LIMIT` →
`MOVING_TO_CENTER`, renamed from the old model's `MOVING_TO_OFFSET`
now that the destination is always exactly 0, not a per-joint tuned
value) drives the remaining distance to absolute step 0 using the
normal accel/speed profile — 0 now means each joint's own true center,
not an arbitrary reference point. This is still the same "seek limit,
then move to a real reference position" pattern `mast_uno3.ino`'s and
`antenna_uno5.ino`'s own post-calibration sequences use, just now
landing on a physically meaningful center rather than an arbitrary
offset destination.

**Every regular joint command, and every preset move, is now clamped**
to `[kMinDeg, kMaxDeg]` (converted to steps via `kStepsPerDegree`)
before being accepted — `clampToOperationalRange()`, applied at both
call sites. This is a real, firmware-side backstop, not merely a
courtesy: since MoveIt already plans within these same limits (sourced
from the same `arm.xacro`), this should rarely if ever actually clamp
anything reaching the firmware via a MoveIt-planned trajectory — it
exists for whatever else might reach it: a raw command from the web
GUI's own sliders, a hand-typed test frame, a future client with no
knowledge of these limits at all. The web GUI's own 5 joint sliders
were updated to match these same real bounds (`±160000` / `±106667` /
`±181333` steps) as a UI-side backstop alongside the firmware's own,
replacing an old, disconnected `±16000` placeholder that turned out to
trace back to the same stale 5:1 gear-ratio assumption `arm_topology.yaml`
had already moved past.

**Keeping these numbers in sync matters for a real reason, not just
tidiness**: a mismatch between `arm.xacro`'s own limits and this
firmware's `kMinDeg`/`kMaxDeg` would mean MoveIt could plan a
trajectory this firmware's own clamp then silently truncates — the arm
would quietly stop short of where MoveIt believes it commanded it to
go. There's no automatic single source of truth across the
URDF/firmware/ROS-config/browser boundary here — `arm.xacro`'s own
`<limit>` tags, `arm_topology.yaml`'s own `steps_per_joint_rev`,
`arm_mega2.ino`'s own `kStepsPerDegree`/`kMinDeg`/`kMaxDeg`/
`kLowerLimitSteps`, and the web GUI's own slider bounds are four
separate places carrying related numbers that all need updating
together by hand if any of them ever change.


### Predefined poses

Three fixed five-joint poses — `kInitialPoseSteps`, `kTransportPoseSteps`,
`kServicePoseSteps` in `arm_mega2.ino` — each reachable with a single
`'P'` frame (`rover_arm/arm_preset` service, `rover_msgs/srv/ArmPreset`)
rather than requiring the operator to know and send all 5 target steps
by hand. All three are currently PLACEHOLDER (all-zero, every joint at
its own homed zero) pending real-world calibration — not yet claimed
to be safe, useful, or even reachable poses. Deliberately implemented
as firmware-side constants, addressable by index, rather than computed
or stored at the ROS/web-GUI layer: this way the poses stay reachable
even if the ROS stack is down but the serial link to the Arduino
itself still works, and there's exactly one place these values live
rather than a web-GUI-side copy that could drift from a firmware-side
one. Subject to the same "fully homed, not mid-homing, not e-stopped"
gate as a regular joint command — a preset move is still new movement,
no different from an operator manually driving the sliders as far as
safety gating is concerned.

The web GUI's arm panel exposes these as three buttons (`INITIAL
POSITION`, `TRANSPORT POSITION`, `SERVICE POSITION`) — this **replaced**
an existing button (`RETURN HOME`) that used to send a hardcoded
`[0,0,0,0,0]` directly as a regular joint command, rather than leave
two parallel, potentially-diverging ways to reach a default pose.

## Arm emergency stop

A latching emergency stop, triggered/cleared via a `'X'` frame
(`rover_arm/emergency_stop` service, `rover_msgs/srv/EmergencyStop`) -
`ArmState.estop_active` reports the firmware's own current state.

**Deliberately does NOT de-energize the drivers - a real safety
trade-off, not an oversight.** Conventional e-stop behavior cuts power
to the actuators; this one instead calls `AccelStepper::stop()` on
every joint (a fast, controlled deceleration using each joint's own
configured acceleration - not an instantaneous halt, since forcing an
abrupt step-rate change instead risks the motor losing synchronization
with its driver) and leaves every driver energized throughout. This is
a gravity-loaded arm: de-energizing mid-air risks an uncontrolled
drop, judged worse than holding position under load, the same
philosophy this board's own watchdog-timeout behavior already applies
(and mast/antenna's own watchdogs share).

Once triggered, the e-stop blocks every source of new movement
(regular joint commands, preset requests) until an explicit clear is
received - the firmware itself is the authoritative source of truth
for this, not the ROS bridge node upstream of it, specifically so a
bridge-node restart or hiccup after an e-stop can't silently resume
motion. The web GUI's arm panel exposes this as two buttons (`E-STOP`,
styled distinctly - a solid, high-contrast button rather than this
project's usual outline style, matching real e-stop button
conventions - and `CLEAR E-STOP`), plus a live status readout in the
telemetry panel driven by `estop_active` itself, not by which button
was last clicked - so a page reload or another operator's own action
stays in sync automatically.

### Using it

Web GUI (arm panel): `CALIBRATE J1`-`CALIBRATE J5` for a single joint,
`CALIBRATE ALL 5` for the full sequence, `INITIAL POSITION` to move to
the arm's predefined starting pose (a preset move, not a calibration
action — only meaningful once actually homed, and safely a no-op
otherwise since the firmware ignores it like any other joint command
would be). Driver enable/disable is a single toggle (label and color
both follow the firmware's own reported `drivers_enabled` state, not
a locally-tracked guess) — this brings the arm in line with the same
toggle convention the mast, antenna, and microscope panels already use
for their own driver control; the arm was the one panel still using
two separate buttons for it before this.

Directly:
```bash
ros2 service call /rover_arm/home_joint rover_msgs/srv/HomeJoint "{joint_index: 2}"   # home J3 only
ros2 service call /rover_arm/home_joint rover_msgs/srv/HomeJoint "{joint_index: -1}"  # home all 5
```

## Antenna gimbal

A 2-axis high-gain-antenna pointing mechanism, top rear left of the
rover, modeled on the real Mars Exploration Rover (Spirit/Opportunity)
HGA gimbal — G1 (azimuth, primary axis, normal to the deck) and G2
(elevation, secondary axis, parallel to the deck), with the antenna
disk on a short arm at the end and the beam radiating perpendicular to
the disk face.

### Real-hardware research behind this

The spec for this subsystem gave two different ranges per axis: G1
`[-90°, 90°]` / G2 `[90°, 270°]`, and — specifically for the
*deployed* configuration — g1 `[15°, 285°]` / g2 `[0°, 180°]`
("software-limited"). Those two pairs don't reconcile numerically as a
simple offset (180° span vs. 270° span for azimuth), so before
building anything, this was checked against the real MER HGAG
engineering paper (Sokul et al., ESMATS 2004) rather than guessed at.
That paper confirms the real mechanism: a pyrotechnic-released,
one-way spring-gate deployment (the drive "can no longer return to the
original stowed position" once through the gate), with **280° azimuth
and 234° elevation** as the real, published post-deployment rotation
spans — closely corroborating g1's 270° span and confirming g2's 180°
is a deliberate *software* restriction of what's mechanically possible
(real elevation capability is 234°), matching this project's own
"software-limited" framing exactly.

**g1 `[15°, 285°]` and g2 `[0°, 180°]` are what's implemented and
enforced** (`antenna_uno5.ino`'s `constrain()` calls,
`antenna_topology.yaml`'s `min/max_azimuth_deg`/`min/max_elevation_deg`).
G1/G2 (uppercase) reads as a different reference frame — not
reconciled into a second active constraint here, since doing so would
require guessing at a transformation the source material doesn't
specify. Flagged here rather than silently picked one interpretation.

### What's modeled and what isn't

Only the two gimbal axes' *operational* range is implemented — there's
no launch-lock, pyrotechnic pin puller, or deploy-gate mechanism
modeled (out of scope for what was asked: 2 stepper axes + calibration
switches, not a deployment sequencer). The antenna is treated as
already deployed for all purposes here.

Each calibration switch is assumed mounted at that axis's own
operational **minimum** (15° azimuth, 0° elevation) — a real design
choice, not verified against an actual mechanical drawing, flagged
here and in `antenna_uno5.ino`'s header comment so it's easy to
correct against real hardware. This makes homing simpler than the
mast's: the switch position *is* each axis's minimum, not offset from
a separately-centered zero, so `setCurrentPosition()` at trigger time
directly establishes "home" — no follow-on move-to-a-different-
reference step the way the mast's own (corrected) calibration sequence
needs one.

### Hardware

Same actuator/driver combination as the arm's joints — 2x NEMA17 +
EBA-17-M planetary gearbox (120:1) + TB6600, on an Arduino Uno with 2
calibration switches and an FZ0430 voltage sensor. `steps_per_deg`
and the speed/acceleration/homing-speed constants in `antenna_uno5.ino`
are copied directly from `arm_mega2.ino` rather than re-derived — same
actuator, already-vetted numbers. The two TB6600 drivers share one
enable pin (`kGimbalEnablePin`), safe for the same reason the mast's
own two TB6600s share one: two opto-isolated ENA inputs draw
comfortably under a typical GPIO's safe sink rating, unlike the arm's
three TB6600s, which get independent pins instead.

### Using it

Web GUI (antenna panel): azimuth/elevation sliders (range fetched from
`antenna_topology.yaml` via `GET /api/config`, same mechanism the
mast's transport-position preset uses) and a `CLOSE DRIVER (ENABLE)` /
`OPEN DRIVER (DISABLE)` toggle — starts disabled after homing, same
convention as the mast, until explicitly enabled.

Xbox controller (LB until in ANTENNA mode): left stick jogs azimuth
(X) and elevation (Y) from the current position, rather than an
absolute-position mapping the way the mast's right stick works — the
antenna's operational range isn't centered around 0° the way the
mast's is, so mapping stick deflection straight to an absolute angle
would be a much less natural control feel here.

## Explicit assumptions and simplifications

These are the calls made where the spec was silent or where
"simplify" pointed toward a lighter-weight choice. Flag any of these
back if they don't match the real hardware:

- **Arm: driver-enable state is now real telemetry, not just a
  command the host remembers sending - and the web GUI's own button
  for it changed from two to one to match.** `arm_mega2.ino`'s own
  `driversEnabled` was already tracked internally; it just never made
  it into the outgoing `'S'` frame, so the only place that state
  existed was whatever the host last commanded via the `'A'` frame's
  own `enable` field. That gap was real, not hypothetical:
  `startHoming()` enables drivers automatically before seeking, with
  no operator action involved, so a value the web GUI only remembered
  sending would have silently drifted out of sync with reality the
  moment homing started on its own. Added `drivers_enabled` as a new,
  20th field on the `'S'` frame (threaded through the protocol layer,
  `ArmState.msg`, the bridge node, and the web GUI's own state dict) so
  this is now the firmware's own actual, current state, always - not a
  command echo.

  The web GUI's own two separate buttons (`ENABLE DRIVERS` /
  `DISABLE (FREE-SPIN)`) became one toggle, at the user's own explicit
  request - and this turned out to be less a departure from this
  project's own convention than a correction of one: the mast, antenna,
  and microscope panels already use a single `CLOSE DRIVER (ENABLE)` /
  `OPEN DRIVER (DISABLE)` toggle for the exact same kind of state; the
  arm was the one panel still using two separate buttons for it. Kept
  the arm's own, more explicit wording (`ENABLE DRIVERS` /
  `DISABLE DRIVERS (FREE-SPIN)`, the button's own label switching with
  the state rather than the other three panels' `CLOSE`/`OPEN`
  phrasing) since the free-spin consequence is a genuinely useful thing
  to say out loud for a 5-joint arm specifically, not just tidied away
  for consistency's sake - the toggle *mechanism* is what needed to
  match, not necessarily the exact words. The button's label and color
  both follow `data.arm.drivers_enabled` from live telemetry on every
  render, the same "driven by the firmware's own reported state, not a
  locally-tracked guess" approach already used for this panel's own
  E-STOP status readout.

  Verified this compiles cleanly against the real toolchain before
  calling it done, not just reviewed by eye: installed the actual AVR
  cross-compiler (`gcc-avr`, matching the Arduino IDE's own
  Atmel-flavored 7.3.0) plus `ArduinoCore-avr`, `AccelStepper`,
  `OneWire`, and `DallasTemperature` from their real source
  repositories, and compiled `arm_mega2.ino` with `-Wall` - zero
  errors, zero warnings, both before and after this session's own
  change to `sendStateFrame()`.

- **Arm: a physically-grounded homing/coordinate model replacing an
  arbitrary one, and a real operational-range clamp - sourced from
  numbers this project had already established, not invented fresh.**
  Full technical detail lives in "Arm calibration" above (see
  "Steps-per-degree, the real lower limit, and the operational range");
  this entry is about the sourcing decision itself, which is the part
  most worth flagging back if it's wrong.

  Rather than invent placeholder degree/range numbers the way most of
  this project's other uncalibrated constants default (all-zero, or
  "matches the previous behavior exactly"), this instead reused two
  sets of numbers this project already had on record before this
  session touched anything: `rover_arm/config/arm_topology.yaml`'s own
  `steps_per_joint_rev` (384000, already correctly reflecting the
  120:1 gearbox) for `kStepsPerDegree`, and
  `rover_description/urdf/arm.xacro`'s own joint `<limit>` tags (±150°/
  ±100°/±150°/±100°/±170°) for `kMinDeg`/`kMaxDeg` - both were real,
  specific, already-intentional values, not placeholders themselves,
  so treating them as placeholders here and inventing different
  numbers would have created a genuine inconsistency (MoveIt planning
  against one set of limits while the firmware clamped against
  another) rather than resolved one. `kLowerLimitSteps` (what the
  limit switch's own trigger point gets labeled as) uses the same
  numbers as `kMinDeg`, converted to steps, as its own placeholder -
  physically reasonable (switch mounted right at the operational
  minimum) but not itself bench-verified, and kept as an independently
  adjustable constant rather than computed from `kMinDeg` in code, so
  a real mechanical margin between the two can be introduced later
  without changing what either constant means.

  Found and fixed two genuine, unrelated staleness issues while
  sourcing these numbers, both traced to the same root: a stale 5:1
  gear-ratio assumption (200 full steps × 1/16 microstepping × 5:1 =
  16000) that predated this project's move to the real 120:1 EBA-17-M
  gearbox (384000) but was never fully swept away. `joint_conversion.py`'s
  own module docstring still cited the old 16000 figure; the web GUI's
  5 joint sliders still used a `±16000` bound that turned out to be
  exactly one motor revolution's worth of steps at that same old ratio
  - both fixed, the sliders updated to the real, `arm.xacro`-derived
  bounds (`±160000`/`±106667`/`±181333` steps) as a UI-side backstop
  alongside the firmware's own new clamp, not a replacement for it.

  A historical "Explicit assumptions" bullet from an earlier session
  (this file's own "Calibration direction, order, and post-limit-
  switch offset..." entry, further below) referenced
  `kHomingOffsetSteps` by name - since that constant no longer exists,
  updated that entry in place to redirect here rather than leave it
  factually describing code that isn't there anymore.

- **The actual root cause of base/arm never showing telemetry in the
  web GUI - found only after ruling out the firmware, the physical
  serial link, and the ROS topic layer entirely, each independently
  confirmed healthy first.** `rover_web_gui/ros_bridge.py`'s own
  `_on_base_state()`/`_on_arm_state()` callbacks converted fixed-size
  ROS array fields (`BaseState.encoder_ticks`/`encoder_delta_ticks`,
  `ArmState.joint_position_steps`/`limit_switch_triggered`/
  `joint_homed`) with a bare `list(msg.field)`. rclpy backs every
  fixed-size array message field with `numpy.ndarray` internally, so
  `list()` on one produces `numpy.int32`/`numpy.bool_` elements, not
  plain Python `int`/`bool` - and `json.dumps()` cannot serialize
  either numpy scalar type at all; it raises `TypeError` unconditionally.
  Reproduced directly, not just reasoned about: `json.dumps([numpy.int32(1)])`
  raises `TypeError: Object of type int32 is not JSON serializable`,
  confirming the mechanism exactly.

  **Why this took multiple sessions and several ruled-out hypotheses
  to actually find, rather than being obvious from the symptom
  alone:** `server.py`'s own `_telemetry_sender()` builds one combined
  snapshot for every board and serializes it in a single `json.dumps()`
  call inside one `asyncio` task, previously with no exception handling
  of its own around that call. An uncaught exception in an `asyncio`
  task doesn't crash the process - it just silently stops that task,
  logged only as "Task exception was never retrieved" server-side, with
  nothing the browser's own console would ever show (which is exactly
  why a browser dev-tools check came back completely clean - the
  failure never reached the browser to produce an error there in the
  first place). Every board's telemetry rode on that same one task and
  one combined snapshot, so base or arm's own bad array field silently
  killed telemetry for every board at once, not just its own - mast,
  microscope, antenna, and power *looked* like they still worked only
  because the browser kept displaying whatever their own last
  successfully-sent values had been, frozen, from before the task died;
  they were never actually still updating. Base and arm are also the
  *only* two boards whose own state messages contain any array field at
  all - checked directly against every board's own `.msg` file, not
  assumed - which is why this affected exactly and only those two,
  matching the actual reported symptom precisely.

  Fixed with an explicit per-element cast at both call sites
  (`[int(t) for t in ...]`, `[bool(t) for t in ...]`) rather than a
  generic serialization workaround, matching the same fix already
  applied once before to this exact bug class in a different file
  (`arm_bridge_node.py`'s own `_on_timer()`, from an earlier session) -
  a second, independent occurrence of the same underlying rclpy
  behavior, not a regression of the first fix. Also hardened
  `_telemetry_sender()` itself with its own per-tick exception handling
  around the send, specifically so a future, different field with the
  same problem can't silently take down every board's telemetry again
  the same way - it now logs clearly and skips that one tick instead of
  dying for the rest of the connection's lifetime.

- **A real, project-wide bug found via a live log capture, not code
  review: `SerialLink` could hang a bridge node's entire executor
  thread indefinitely on a single write, with no error and no further
  log output at all.** `rover_protocol/serial_link.py`'s underlying
  `serial.Serial` connection was constructed with a read `timeout`
  but no `write_timeout` - pyserial's own default when that's left
  unset is `None`, meaning a write blocks forever if the OS-level
  output buffer fills and the board on the other end isn't draining
  its own serial RX fast enough to keep up. This is shared
  infrastructure every bridge node (base, arm, mast, microscope,
  antenna, power) is built on, so the exposure was project-wide, not
  specific to any one board - but the arm was the one that actually
  hit it during testing, and for a specific, identifiable reason: its
  homing sequence legitimately runs far longer than this project's
  usual command-response turnaround (a 120:1 gearbox slows real joint
  motion, and `serviceHoming()`'s own seek phase doesn't yield back to
  frame handling as promptly as a quick command-and-respond cycle
  does), during which the bridge node's own continuous, periodic
  writes kept arriving while the firmware wasn't getting back around
  to draining its input quickly enough - eventually filling the
  board's tiny 64-byte hardware RX buffer and blocking the write call
  forever.

  **Diagnosed from an actual log capture, after code review alone had
  been exhausted without finding it** - a user-provided log showed the
  arm bridge node logging "sent homing request to arm Mega" and then
  going completely silent forever, no error, no crash, no further
  output of any kind - the exact signature of a thread stuck on a
  blocking syscall, not a program error, which produces neither. Since
  `_publish_status()` sits after the write call in the same function,
  a hung write there meant `BoardStatus` was never published even
  once - explaining a status lamp stuck at its default "never
  received" state (which reads as red) rather than a lamp that turns
  green and then goes stale, and why the arm specifically was the
  board most likely to actually trigger this given its own homing
  duration, even though the underlying vulnerability was never
  arm-specific at all.

  Fixed with a bounded `write_timeout` (defaulting to 0.2s, matching
  the existing read timeout's own general magnitude) threaded through
  `SerialLink`'s constructor to the underlying connection - a write
  that can't complete now raises `serial.SerialTimeoutException`,
  caught by the same exception handler every other write failure
  already goes through, rather than hanging the calling thread
  indefinitely. Verified every one of the six bridge nodes constructs
  `SerialLink` using keyword arguments exclusively before considering
  this fix backward-compatible - none needed any changes themselves,
  since a purely-keyword call site can't be affected by a new
  parameter's position in the signature. Two dedicated regression
  tests confirm `write_timeout` is actually threaded through to the
  underlying connection, not just accepted and silently dropped, and
  that it defaults to a real, bounded value rather than the `None`
  that was the root cause here.

- **Arm: a latching emergency stop, and calibration made independently
  configurable per joint where it used to be a single hardcoded
  assumption applied to all five.** Also found and fixed a real
  documentation gap while updating this table: the arm's own cooling
  fan (present in firmware for several sessions) had never made it
  into this hardware topology row at all.

  **Emergency stop deliberately does NOT de-energize the drivers - a
  real, non-obvious safety trade-off, not an oversight.** Conventional
  e-stop behavior cuts power to the actuators; this one instead calls
  AccelStepper's own `stop()` on every joint (a fast, controlled
  deceleration using that joint's own configured acceleration, not an
  instantaneous halt - forcing an abrupt step-rate change instead
  risks the motor losing synchronization with its driver, arguably a
  worse outcome than a fast but controlled stop) and leaves every
  driver energized throughout. This is a gravity-loaded arm: de-
  energizing mid-air risks an uncontrolled drop, which this project has
  already judged worse than holding position under load in the
  near-identical watchdog-timeout situation this same board's firmware
  already had (and mast/antenna's own watchdogs share the same
  philosophy). Once triggered, the e-stop is latching and blocks every
  source of new movement (regular joint commands, preset requests)
  until explicitly cleared - the firmware itself is the authoritative
  source of truth for this state, not the ROS bridge node upstream of
  it, specifically so a bridge-node restart or hiccup after an e-stop
  can't silently resume motion.

  **Calibration direction and order are now independently configurable
  per joint, as PLACEHOLDER firmware constants** (`kHomingDirection`,
  `kHomingOrder` in `arm_mega2.ino`) pending real mechanical
  verification on the bench - each currently set to match the
  previous, only behavior (uniform direction, sequential J1-J5 order)
  rather than a guessed-at "improved" default.

  **UPDATE (later session): the post-limit-switch "offset" this bullet
  originally described here has since been replaced entirely, not
  merely adjusted** - `kHomingOffsetSteps` no longer exists.
  `kLowerLimitSteps`, `kMinDeg`/`kMaxDeg`, and `kStepsPerDegree` took
  its place, along with a firmware-side clamp on every joint command
  and preset move - see "Steps-per-degree, the real lower limit, and
  the operational range" above for the complete, current model; not
  duplicated here to avoid two descriptions drifting apart from each
  other over time.

  **Three predefined poses (initial/transport/service), reachable via
  a new firmware-level command, not just a web-GUI convenience.** All
  three are ALL-PLACEHOLDER (all-zero, every joint at its own homed
  zero) pending real-world calibration - not claimed to be safe,
  useful, or even reachable poses yet. Deliberately implemented as
  fixed constants in `arm_mega2.ino` itself, addressable by index over
  the wire, rather than joint angles computed or stored at the ROS/
  web-GUI layer - this way the poses stay reachable even if the ROS
  stack is down but the serial link to the Arduino itself still works,
  and there's exactly one place these values live rather than a
  web-GUI-side copy that could drift from a firmware-side one. This
  **replaced, rather than duplicated, an existing endpoint**: the web
  GUI already had a "RETURN HOME" button sending a hardcoded
  `[0,0,0,0,0]` directly - upgraded to call the new firmware-owned
  INITIAL preset instead of leaving two parallel, potentially-
  diverging ways to reach a default pose. The "SERVICE" pose's actual
  intended purpose (what it needs to clear or reach) wasn't specified
  further when this was added - kept as a generically-named preset
  pending that detail, rather than guess at a real value without
  knowing what it's actually for.

- **Microscope: the LED/cover "protect the optics" fail-safe is
  removed entirely, at the user's own explicit request - a genuine
  safety trade-off, not a cosmetic change, worth stating plainly
  rather than folding into a routine changelog line.** Two
  independent implementations of the same behavior existed - the
  firmware's own watchdog (`microscope_uno4.ino`'s `loop()`, forcing
  the LED off and the cover closed whenever no command had arrived
  for over a second) and the bridge node's own parallel version at
  the ROS layer (`microscope_bridge_node.py`'s `_on_timer()`, forcing
  the same values into every command frame it sent whenever the ROS
  command stream itself went stale) - both found and removed
  together; removing only one would have left the other silently
  reintroducing the exact behavior being removed; the firmware has no
  way to distinguish a genuine operator command from the bridge
  node's own override.

  **What this actually means**: the LED and cover now stay in
  whatever state they were last explicitly commanded to,
  indefinitely, even if the serial link between the bridge node and
  the Uno drops, or the ROS command stream from the web GUI stops
  entirely. Previously, either kind of link loss would have
  automatically turned the LED off and closed the cover within about
  a second. **If the LED is left on or the cover left open when a
  link drops now, nothing in this software will turn it off or close
  it on its own** - that's on the operator (or whatever's upstream)
  to notice and handle. The focus stepper was never part of this
  fail-safe either way - it already just holds its last commanded
  position on a dropped link, with no separate timeout logic of its
  own, unaffected by this change.

  Removed cleanly, not just disabled: `kWatchdogTimeoutMs`/
  `lastCommandMillis` in the firmware, `command_timeout_sec`/
  `_last_command_time` (and the now-unused `time` import) in the
  bridge node, and the `command_timeout_sec` config parameter in
  `microscope_topology.yaml` are all gone, not left declared-but-
  unused - a note in that yaml file explains the parameter's absence
  explicitly, since every other board's own config still has one and
  its absence here could otherwise look like an oversight rather than
  a deliberate choice.

- **Microscope: most of a multi-part request turned out to already be
  done, verified rather than assumed.** Asked for: remove ServoEasing
  from the cover servo, add explicit OPEN/CLOSE cover buttons, add
  explicit LED ON/OFF buttons, and express LED dimming in 0-5V rather
  than raw PWM. Checked the actual current state of each part before
  doing any work rather than trust memory of what had been built: the
  ServoEasing removal (firmware header explicitly records it as
  already done "at the user's own request"), the cover buttons, and
  the LED on/off buttons were all already fully implemented and wired
  up, in both `app.js` and `index.html`. Only the 0-5V display change
  was genuinely outstanding.

  **The LED change is display-only, not a wire-protocol or hardware
  change** - the microscope's LED is still driven by plain
  `analogWrite()` PWM (0-255 duty cycle), not a true analog output or
  DAC; nothing in the request mentioned new hardware to enable one.
  0-255 duty cycle on a 5V logic pin genuinely *is* 0-5V of effective
  average output, so relabeling the existing range this way is
  accurate, not approximate. Changing the wire protocol itself to
  carry millivolts would have meant a firmware-side conversion back to
  PWM for no real gain, and could misleadingly imply a precision the
  hardware doesn't actually have - so `led_pwm` stays exactly what it
  was (0-255) all the way from the slider's own raw value through the
  wire protocol to `analogWrite()`; only the label the operator sees
  changed, via a small shared `formatLedVoltage()` helper used
  consistently by the slider, both buttons, and the telemetry panel's
  own LED readout, rather than four separate copies of the same
  conversion.

- **Base: a second FZ0430, splitting one shared voltage reading into
  two independent rail readings.** Requested as "add a second FZ0430
  with the same configuration... designate the first for driving
  current and the second for steering current" - the FZ0430 measures
  voltage, not current, so this was confirmed rather than guessed at
  before any code changed: the user's own follow-up, "I confirm my
  mistake; I meant the voltage capture," settled it as two voltage
  rails, not current sensing (which would have meant a different part
  entirely, like the ACS712 or INA226 already used elsewhere in this
  project).

  The existing FZ0430 (A0) is now explicitly the **drive** motors' own
  supply rail - previously just a generic "main supply" reading, since
  there was only ever one rail to report before this. A second,
  identically-configured FZ0430 (A1, same 5:1 divider, same conversion
  math) reports the **steering** servos' own supply rail. This is a
  genuine rename, not just an addition: `BaseState.msg`'s
  `supply_voltage_mv` became `drive_voltage_mv`, with `steering_voltage_mv`
  added alongside it - every layer (firmware, protocol, message,
  bridge node, web GUI) updated together for the rename, not just the
  new field, including proactively checking `ros_bridge.py`'s own
  state-capture callback before it could become another instance of
  the same bug class prior sessions have repeatedly found there.

  Two other pre-existing, unrelated staleness issues were caught and
  fixed while working in this same firmware section: `base_protocol.py`'s
  own 'D' frame docstring still said steering was sent to "direct Mega
  pins A4-A7, no PCA9685," despite that migration happening several
  sessions ago; and the voltage section's own top-of-block comment
  still said "voltage sensor" (singular) after this session's own
  second sensor was added, missed in an earlier pass through the same
  edit. Fixed both rather than let stale text stand next to newly
  correct code.

- **Board-status lamps could never actually show a "connected but
  receiving garbage" state, on any board, until now** — found while
  investigating a report of missing power-panel telemetry. The green
  lamp's own meaning had been misleading the whole time: `connected`
  becomes `true` the moment a serial port opens successfully,
  completely independent of whether any valid frame has ever been
  received - that's a separate counter (`rx_frame_count`) entirely.
  A three-state helper function (`boardStatusPillClass()`, fault/
  warn/ok) already existed in `app.js`, correctly written, but was
  never actually called anywhere - the real lamp-rendering loop used
  a simpler connected-only check instead, so a board silently
  producing nothing but checksum errors (most plausibly a `/dev/rover/X`
  symlink resolving to the *wrong* physical device - a real risk
  anywhere two boards share a VID:PID; `power_uno6` now shares this
  risk with mast, microscope, and antenna, all four genuine Unos
  reporting the identical 2341:0043, see that board's own bullet
  above for the history) would show exactly the same
  plain green as a board that's genuinely working, with nothing to
  visually distinguish the two.

  Wired the existing function into the actual rendering loop, added
  the missing CSS for the state that never had it (`--amber`, already
  an established color in this project's palette, reused rather than
  introducing a new one), and surfaced `checksum_error_count`
  directly in the lamp's own meta text (e.g. "12 rx, 340 bad") so the
  distinction is visible without needing to check a ROS topic
  directly. This applies to every board in the project, not just the
  one whose report surfaced it - any board that's ever been "green
  but not actually working" was experiencing exactly this same gap.

- **`power_nano6.ino` failed to compile in the Arduino IDE: "error:
  'BatteryReading' does not name a type" — a real, documented
  Arduino toolchain bug, not a mistake in the code as written.** The
  struct itself was correctly defined, with its semicolon, before its
  first use - reading the file top to bottom, a human would see
  nothing wrong. The actual cause: the Arduino IDE/arduino-cli
  automatically generates forward declarations (prototypes) for every
  function in a sketch and inserts them immediately after the file's
  `#include` lines, *before* any type declared later in the file -
  including a custom struct that's fully defined earlier than the
  function using it, from the actual source file's own perspective.
  This is a known, long-open toolchain issue (see
  `arduino/arduino-cli#2696` and `arduino/Arduino#8014`/`#8050`, the
  oldest reports going back over a decade), not specific to this
  project or this session's code.

  Fixed by removing the custom `BatteryReading` struct entirely and
  rewriting `readBattery()` to use output parameters
  (`int32_t &voltageMv, int32_t &currentMa`) instead of returning a
  struct by value - every type in the function's signature is a
  built-in type already known before any auto-generated prototype
  could need it, so there's nothing left for the toolchain's ordering
  bug to trip over. The officially documented workaround (move the
  struct to a separate header file, since the auto-prototype step
  only scans the `.ino` file itself) was available too, but would
  have introduced a multi-file sketch structure no other board in
  this project uses - the output-parameter rewrite fixes the same
  problem while keeping every board's firmware a single, self-
  contained file.

  Swept every other firmware file afterward for the same pattern (any
  custom `struct`/`enum` used as a function parameter or return type,
  not just declared for local variables) rather than assume this was
  an isolated case - found none; every `enum` elsewhere in this
  project is only ever used for `int8_t`-typed variables and their
  own enumerator constants, never in a function signature, so none of
  them were ever at risk of this same failure.

- **New subsystem: power/environmental monitoring, board #6** — two
  24V batteries' own voltage and current, onboard computer
  temperature, and that computer's automatic cooling fan. The sixth
  board in this project, and structurally different from the other
  five: it has no command message at all. Every other board's
  bridge node sends commands and receives state in response; this
  one only ever receives, since there's nothing for an operator to
  command here - both sensors are read-only and the fan is fully
  automatic, matching every other fan in this project. The firmware
  proactively sends its own state frame every ~200ms rather than
  reactively, in response to an incoming command, the way every
  other board's `sendStateFrame()` is triggered - see
  `power_uno6.ino`'s own header comment for the reasoning.

  **UPDATED: the original FZ0430 + ACS712 sensor suite was replaced
  with two INA226 voltage+current monitors behind a TCA9548A I2C
  multiplexer**, after the FZ0430's real safety concern (a hard 25V
  ceiling that a nominal 24V pack could plausibly exceed in normal
  operation - see the earlier version of this bullet's own history
  in `docs/journal.md` for the full original reasoning) led to a
  direct recommendation for the INA226 instead: its 0-36V bus range
  gives genuine headroom above the ~28-29V worst case a 24V pack can
  realistically reach, not a replaced-but-still-tight margin. This
  also gives each battery its own current reading for the first
  time, rather than the single ACS712's combined draw across both -
  a real, if secondary, improvement that fell out of the switch
  rather than the primary reason for it.

  **Why a TCA9548A multiplexer, given the INA226 already supports 16
  addresses via its own A0/A1 pins**: both INA226 units in this
  design sit at their shared, unmodified default address (0x40, both
  address pins grounded) - the mux is specifically what makes two
  identical, off-the-shelf breakouts workable together on one bus,
  without needing to reconfigure either one's address pins. The mux
  itself defaults to 0x70, unjumpered - see the next bullet for why
  that number specifically needed its own explicit callout.

  **0x70 now appears in this project for two unrelated reasons, on
  two physically separate boards** — the PCA9685's own built-in "All
  Call" address on the base board (see that bullet above), and now
  the TCA9548A's own default address on the power board. These don't
  conflict with each other in any way - separate chips, separate
  Arduinos, separate I2C buses - but 0x70 has already caused one
  real, documented confusion in this project (an earlier session
  mistakenly jumpered a PCA9685 *toward* 0x70, precisely the address
  that should never be a device's primary one), so a second,
  unrelated appearance of the same number is named explicitly here
  and in `power_uno6.ino`'s own header, rather than left as a
  coincidence someone has to notice on their own later.

  **A real, must-verify hardware concern, not routine calibration**:
  the INA226 measures current via the voltage drop across an
  external shunt resistor, and its own shunt-voltage measurement
  range is hard-capped at ±81.9mV. Many generic INA226 breakouts ship
  with a fixed, onboard 0.1Ω shunt - with that value, the *maximum*
  current such a board could ever report before its shunt voltage
  saturates is 81.9mV / 0.1Ω ≈ 0.82A, far below what this rover's
  batteries actually need to measure (the ACS712 this replaced was
  rated for 30A). `kInaShuntOhms` in `power_uno6.ino` is set to
  0.002Ω - a smaller, higher-current-range value some INA226
  breakouts use specifically for this reason - as a placeholder, not
  a verified fact about the actual boards in hand. **Check what
  shunt resistor your specific breakout boards actually have before
  trusting any current reading beyond a rough one** - if readings
  clip at a suspiciously round, low number regardless of real load,
  this is almost certainly why, not a firmware bug.

  Library: **INA226** (by Rob Tillaart, MIT licensed) - confirmed
  directly from the library's own repository page, not assumed. No
  dedicated library for the TCA9548A itself: channel selection is a
  single I2C write of one byte (`Wire.beginTransmission(0x70);
  Wire.write(1 << channel); Wire.endTransmission();`), simple enough
  that a library would add a dependency for less code than it saves,
  the same reasoning this project has applied to other single-purpose
  I2C writes elsewhere.

  **UPDATED: this board is now an Arduino Uno, not a Nano.** Swapped
  after repeated, unresolved hardware trouble with the Nano units in
  hand - the user's own words, "with which we have often had
  difficulties," without a single, isolated root cause identified
  (the missing-telemetry investigation two sessions ago found the
  code correct at every layer; the compile error before that was a
  genuine Arduino toolchain bug, not the board's fault - neither
  incident actually indicts the Nano hardware itself, but repeated,
  hard-to-pin-down trouble with a specific batch of units is a
  legitimate reason to just swap them, whether or not any single
  session's root cause was ever conclusively the board). No pin
  reassignment was needed for the swap: the Nano and Uno share the
  same ATmega328P, the same D2-D13 digital range, the same
  D3/D5/D6/D9/D10/D11 PWM pins, and the same A4/A5 I2C pins - every
  pin this board's own firmware actually uses. The Nano's two bonus
  analog-only pins (A6/A7), which the Uno lacks, were never used by
  this board's sensor suite either way, so nothing was lost.

  **The CH340-clone-vs-GPS-adapter USB collision risk from the
  original Nano build is retired by this swap, not carried forward**
  - that risk was specific to the Nano's own USB-serial chip
  situation (a genuine FTDI-based Nano vs. a common CH340-based clone
  reporting completely different VID:PIDs). **A different, but
  already-documented and already-understood, collision risk takes
  its place**: this board now reports the same VID:PID (2341:0043)
  as every other genuine Arduino Uno in this project. Found while
  updating the udev rules for this swap that this was actually a
  *fourth* instance of that exact situation, not a third - mast,
  microscope, and antenna were already sharing this VID:PID, but an
  earlier version of the mast's own udev entry had a real, separate,
  previously-uncaught bug: it matched `idProduct` `0042` (the Mega's
  ID) with a placeholder literally named `MAST_MEGA_SERIAL`, left
  over from before the mast itself became an Uno many sessions ago.
  As written, that rule could only ever have matched base's or arm's
  own Mega - never the mast's actual hardware. Fixed alongside this
  session's own change, not left standing next to it.

- **Arm: a real driver-enable race condition, found while answering a
  question about the state machine's behavior, not while looking for
  bugs specifically.** `arm_mega2.ino`'s `handleJointCommand()` used
  to call `setDriversEnabled(enable)` unconditionally, before its own
  `homed`/`homingInProgress` gate - `mast_uno3.ino`'s and
  `antenna_uno5.ino`'s equivalent handlers already gate this
  correctly (`setGimbalEnabled`/`setHeadEnabled` only inside `if
  (homed)`), a fix built specifically to prevent this exact problem
  several sessions ago on those two boards. The arm's own handler
  predates that fix and never received the equivalent treatment.

  `arm_bridge_node.py` sends 'A' frames continuously at its own
  control rate, including while homing is in progress - by design,
  and normally harmless, since the frame's `enable` field is meant to
  be ignored during homing. But with the gate in the wrong place, it
  wasn't ignored: the bridge's own default (`enable=False`, until an
  operator has actually touched the arm panel) meant every regular
  frame arriving mid-homing would immediately undo the `enable=true`
  `startHoming()` had just set. On a real, previously-untouched boot,
  this meant the drivers could be toggled enabled (once, by
  `startHoming()`) and disabled (repeatedly, by every regular frame
  arriving during the entire seek) throughout homing - not just an
  inert flag flipping back and forth, but the physical drivers
  actually losing power while `AccelStepper` kept counting step
  pulses as if they'd been executed, risking the exact kind of
  position-tracking corruption per-joint homing exists to establish
  in the first place.

  Fixed by moving `setDriversEnabled()` inside the same gate the
  joint-target application already used, matching mast/antenna's
  pattern exactly rather than inventing a new one. Also corrected the
  bridge node's own comment, which had asserted the firmware already
  handled this correctly - it didn't, and a comment describing
  intended behavior as already-true is worse than no comment once
  it's wrong.
- **Microscope's driver enable/disable button never actually worked,
  since the day it was built — the other three (mast, antenna, arm)
  did, and were verified to, not assumed to.** Reported as "enable/
  disable doesn't physically work" without specifying which
  subsystem, so every driver-enable path got checked end to end
  (web GUI → `server.py` → `ros_bridge.py` → bridge node → wire
  protocol → firmware) rather than guessed at: arm's, mast's, and
  antenna's chains were all confirmed correct at every layer,
  including firmware-side gating logic and each bridge node's
  command-resend behavior. Microscope's wasn't.

  `app.js` had been correctly sending `driver_enable` in every
  microscope command since the button was built — the bug was
  entirely on the backend: `server.py`'s WebSocket dispatch for
  `"microscope"` never read it from the payload, and
  `ros_bridge.py`'s `send_microscope()` didn't even accept it as a
  parameter. The value was silently dropped before ever reaching the
  ROS message, which meant every microscope command carried
  `driver_enable=False` (a ROS bool field's own default) regardless
  of which button was clicked or what the button's label said.
  Clicking "enable" updated the button's own text and toggle state —
  which is why it could look like it was working — without ever
  actually being able to energize the driver, since the one value
  that mattered never left the browser. Confirmed the rest of the
  chain (bridge node, firmware) was already correct and just waiting
  for a real value to arrive - fixed at the two points responsible,
  not worked around further downstream.
- **Mast: automatic, thermostatically-controlled cooling fan** — a
  generic N-channel MOSFET driver module (IRF520-style: SIG/VCC/GND
  control header, V+/V- screw terminals for the load), entirely
  closed-loop against the mast's own DS18B20 reading, not
  operator-commanded — no field for it on `MastCommand`, only
  telemetry on `MastState` (`fan_duty_percent`, 0-100).

  **Wiring is a low-side switch, worth being explicit about since
  it's a common mix-up**: the fan's positive lead goes directly to
  the external supply (the module's V+/Vin, permanently on), and the
  MOSFET switches the fan's *negative* lead (through V-/OUT) to
  ground — not the fan's positive side the way a more intuitive
  high-side design might suggest. The module's VCC pin only powers
  its own onboard status LED and can be left disconnected; only GND
  and SIG are actually required for switching.

  **No hardware PWM pin was available for this** — the mast Uno's
  entire PWM budget (D3/D5/D6/D9/D10/D11) was already committed to
  the yaw/pitch/lift functions before this was added. `kFanPwmPin`
  (A2) runs a simple `millis()`-based software PWM instead
  (`updateFanPwm()`, 50Hz) rather than force a conflict — a MOSFET
  module switching a slowly-responding thermal load doesn't need
  hardware-timer precision the way a stepper pulse train does.

  **Thermostat thresholds are placeholders** (`kFanOnTempDeciC`=35°C,
  `kFanOffTempDeciC`=30°C, `kFanMaxTempDeciC`=50°C,
  `kFanMinDutyPercent`=30%) — bench-tune once the real enclosure's
  actual thermal behavior is known, the same as every other
  uncalibrated constant in this project. `kFanOffTempDeciC` sits
  below `kFanOnTempDeciC` deliberately (hysteresis): once running,
  the fan stays on until temperature drops to the *lower* threshold,
  not the one it turned on at, to avoid rapid on/off cycling right at
  a single boundary. Below `kFanOnTempDeciC` the fan is off; between
  the ON and MAX thresholds its duty cycle ramps linearly, clamped to
  at least `kFanMinDutyPercent` once running — many small DC fans
  won't reliably start or stay spinning much below that, so this
  avoids a smooth ramp landing right at a near-zero, potentially-
  stalling duty cycle.

  **A deliberate fail-safe choice, not an oversight**: if the DS18B20
  itself isn't responding (`cachedTemperatureDeciC` at its own
  `-9999` sentinel), the fan defaults to running at
  `kFanMinDutyPercent` rather than staying off. Unnecessary fan noise
  is a minor cost; an overheating board with no way to know it's
  overheating is a real one — the two failure modes aren't
  symmetric, so the default shouldn't be either.

  **Found and fixed while documenting this, not introduced by it**: all
  four protocol files, all four `.msg` files, and one `ros_bridge.py`
  state callback still described the temperature sensor as a BMP280
  over I2C — leftover language from before that sensor was replaced
  with the DS18B20 two sessions ago, missed at the time because the
  swap's own review focused on the firmware and the primary docs, not
  every inline field comment referencing it. Fixed all of it, not
  just the mast's own copy.

- **Temperature sensor on base, arm, mast, and antenna (not
  microscope) is a DS18B20, not a BMP280** — the BMP280/I2C version
  documented in an earlier session was fully replaced, not layered
  alongside; nothing about the temperature feature itself changed
  (same field, same wire format, same sentinel convention, same web
  GUI display), only the sensor hardware and the firmware talking to
  it.

  DS18B20, TO-92 package, communicates over 1-Wire on a single digital
  data pin rather than I2C's two-pin shared bus — no dedicated
  hardware pin requirement the way I2C's TWI peripheral has, so pin
  choice was open on all four boards. Reused each board's now-freed
  former I2C SDA pin rather than pick a new one (Mega: pin 20, Uno:
  A4) — avoids leaving a pin idle for no reason. **Requires an
  external 4.7kΩ pull-up resistor between DQ and VDD** — a real,
  physical component to add, not optional and not achievable via the
  Arduino's internal `INPUT_PULLUP` (documented as too weak to hold
  reliable 1-Wire bus timing).

  Libraries: **OneWire** (Paul Stoffregen, MIT) and **DallasTemperature**
  (Miles Burton, **LGPL-2.1**) — a third license in these firmware
  files alongside this project's own Apache-2.0. LGPL is generally more
  permissive than GPL-3.0 about linking against differently-licensed
  code, but that distinction matters less for a statically-linked,
  monolithic firmware binary than for a desktop application
  dynamically linking a shared library — worth its own explicit
  licensing review regardless, not waved off as equivalent to
  OneWire's own MIT license just because they're installed together.
  (This bullet used to also compare against ServoEasing's GPL-3.0 on
  the base board specifically - stale even before this session, since
  base's steering moved off ServoEasing several sessions ago, and the
  library is gone from this project entirely as of this session; see
  the dedicated servo-easing bullet elsewhere in this section.)

  **Read via a non-blocking two-phase state machine, not the
  library's own default blocking pattern.** The DS18B20's conversion
  takes up to ~750ms at full (12-bit) resolution — the library's own
  documented example simply calls `delay(750)` between requesting a
  conversion and reading it, which would stall `loop()` for 750ms on
  every read on boards whose actual job is real-time motor control.
  Instead: `requestTemperatures()` (non-blocking, via
  `setWaitForConversion(false)`) starts a conversion and returns
  immediately; a separate check on a later `loop()` iteration reads
  the result once the worst-case conversion time has elapsed,
  without ever blocking. Still on the same outer 1-second cadence as
  before (`kTemperatureReadIntervalMs`) for starting a new cycle — the
  underlying reason is unchanged: board temperature moves over
  seconds-to-minutes, not tens-of-milliseconds, so there's nothing to
  gain from polling faster than that regardless of how fast any given
  read technically could be.

  **A genuine improvement over the BMP280 version, not just a
  like-for-like swap**: DallasTemperature reports a specific
  `DEVICE_DISCONNECTED_C` sentinel on any read that fails, checked
  fresh on *every* cycle — unlike the BMP280 version's one-time
  `bmp.begin()` check at boot, a DS18B20 that's disconnected and later
  reconnected recovers on its own next read, with no reset needed.
  The wire-level sentinel this maps to (`-9999`, i.e. -999.9°C) is
  unchanged from before, still comfortably outside the DS18B20's real
  range (-55 to +125°C, wider than the BMP280's own -40 to +85°C, for
  what it's worth) — the web GUI's `formatTemperature()` needed no
  changes at all to keep rendering it as a plain "N/A".

- **Two real, pre-existing bugs found and fixed while building the
  antenna's web GUI/Xbox plumbing, not introduced by it** — both meant
  the mast's `driver_enable` field, added a couple of sessions back,
  never actually worked outside the one path that happened to get
  tested by hand:
  1. The web GUI's mast panel sent `driver_enable` in its WebSocket
     payload, but `server.py`'s dispatch never read it and
     `ros_bridge.py`'s `send_mast()` didn't even accept it as a
     parameter — the value was silently dropped before ever reaching
     the ROS message. `_on_mast_state()`'s telemetry capture had the
     same gap in reverse (never read `driver_enabled` back out of the
     state message), so the web GUI's `DRIVER: ENABLED/DISABLED`
     readout was always showing `DISABLED` regardless of the real
     state. Fixed in both directions.
  2. Separately, `xbox_teleop_node.py`'s `_handle_mast()` never set
     `cmd.driver_enable` at all, leaving it at its unset default
     (`False`, a ROS bool field's default). Since the firmware gates
     movement and enable/disable on the same command, this meant
     **Xbox controller mast control was completely non-functional**
     since `driver_enable` was introduced — every joystick command
     disabled the drivers at the exact moment it tried to move them.
     Fixed by setting `driver_enable = True` when actively jogging,
     matching how `_handle_arm` already sets `cmd.enable = True` for
     the same reason. Applied the same pattern to the antenna's own
     `_handle_antenna` from the start, rather than propagate the gap
     into new code.
- **A third instance of the same underlying bug class, this one only
  found by actually running the system, not caught in review** —
  `ros_bridge.py`'s `get_snapshot()` is a hand-constructed dict
  listing each subsystem's key explicitly, and "antenna" was simply
  missing from it, even though `self._state["antenna"]` was being
  correctly populated by `_on_antenna_state()` the whole time. The
  data existed server-side and never reached the frontend — every
  other piece (the telemetry panel's HTML, the JS rendering code, the
  ROS subscription) was correct in isolation, which is exactly why
  static review across several passes never caught it: nothing was
  individually wrong, one dict just never got the new key added to it.
  Fixed, and added a comment linking `get_snapshot()` to `self._state`'s
  own declaration explicitly, so the next subsystem addition has an
  actual pointer to keep these two in sync rather than relying on
  remembering to.
- **A fourth instance, same bug class again, this time affecting
  every subsystem's new `board_temperature_decic` field at once** —
  when that field was added (base, arm, mast, antenna), the ROS
  message, bridge nodes, and frontend rendering all got updated
  correctly, but `ros_bridge.py`'s four `_on_X_state()` callbacks —
  the code that actually unpacks an incoming ROS message into the
  dict `get_snapshot()` later serves — were never touched. The field
  existed on the wire and in the ROS message; it just never got
  copied out of it. Reported by the user as "antenna telemetry
  appears, but temp is missing" — checking confirmed it was missing
  from all four panels, not just the one that happened to get
  noticed first. Fixed all four, and extended the existing sync
  comment (see the bullet above) to name this specific class of
  callback as a third place that has to stay in sync, not just the
  two `get_snapshot()`/`self._state` already called out — this bug
  was in exactly the kind of spot that comment didn't yet cover.
- **A fifth instance, this one the actual explanation for what turned
  out to be two sessions of "the Antenna label is missing" reports**
  — `app.js`'s `BOARD_NAMES` array (four hardcoded `[board_name,
  display_label]` pairs, driving the left rail's board-status lamps)
  was never given a fifth entry for the antenna when that subsystem
  was built. Not a rendering bug, a CSS issue, or a stale cache, all
  of which got ruled out first, thoroughly, across the two prior
  reports — antenna's board-status lamp didn't render in the wrong
  state, it simply never existed, since the loop that builds each
  lamp row iterates this exact array and had no fifth board to
  iterate over. The backend side was already correct the whole time
  (`rover_antenna/board_status` was properly subscribed to and
  published), which is exactly why nothing on that side ever looked
  wrong under investigation — the gap was purely in this one frontend
  array. Confirmed the fix's key (`antenna_uno5`) matches exactly what
  `antenna_bridge_node.py` actually publishes before wiring it in,
  the same care this project has taken with board-name string matches
  since the mast's own Mega-to-Uno migration first surfaced this exact
  failure mode. Swept the rest of `app.js`/`ros_bridge.py`/`server.py`
  for any other hardcoded four-board list afterward rather than assume
  this was the only one — found none.
- **A sixth instance, in `index.html` itself, missed by that same
  sweep** — asked directly ("did you also forget index.html on the
  left rail?") after the fifth instance above, because that sweep
  only covered `app.js`/`ros_bridge.py`/`server.py` and, in hindsight,
  had no real reason to have covered the HTML too when the bug it was
  chasing was in a JS array. It should have swept wider. `index.html`
  has its own four static, hardcoded lamp rows for exactly the same
  left-rail list — a design detail worth understanding correctly:
  `app.js` clears and fully rebuilds this container
  (`lampList.innerHTML = ""`) on every telemetry update, so in normal
  operation with JS running these static rows are invisible, replaced
  before a user ever sees them. That does not make leaving them wrong
  acceptable — they're still incorrect markup, they're what actually
  renders if JS fails to load or errors out before this code runs, and
  the file should describe the real page regardless of whether
  something else immediately overwrites it. Fixed by adding the
  missing fifth row. Also found and fixed a second, unrelated,
  pre-existing staleness in the same four rows while there: the mast's
  said `MAST / MEGA #3`, unchanged since long before the antenna ever
  existed — the mast became an Uno many sessions ago, and `app.js`'s
  own `BOARD_NAMES` array had already been corrected to `MAST / UNO
  #3` at the time, just never mirrored back into this static
  duplicate. A concrete example of why keeping duplicated content in
  sync needs an explicit reason to trust it has been, not an
  assumption that it has.
- **Mast: "RETURN HOME" and "TRANSPORT POSITION" web GUI buttons** —
  both are ordinary `MastCommand`s (yaw/pitch only, `lift_mode`
  untouched), not new firmware or protocol capability: Return Home
  sends 0°/0°, Transport Position sends whatever's configured in
  `rover_mast/config/mast_topology.yaml`'s new
  `transport_head_yaw_deg`/`transport_head_pitch_deg` (placeholder
  0.0/0.0 — dead ahead, level — until a real safe transport
  orientation is known; tune those two values once it is, no code
  change needed either way). Deliberately doesn't also command the
  lift — sequencing "re-center the head, then stow" vs. "stow, then
  re-center" is left to the operator via the existing Stow button,
  rather than baked into one fixed order.
- **`max_head_yaw_deg`/`max_head_pitch_deg` deduplicated** — found
  while adding the above: this pair was declared independently in
  both `rover_mast/config/mast_topology.yaml` (unused there —
  `rover_mast_bridge` never actually read it) and
  `rover_teleop/config/xbox_teleop.yaml` (the one actually used, for
  MAST-mode stick-to-angle scaling), with no shared source of truth
  between the two copies. Consolidated into `mast_topology.yaml`'s own
  `/**:` shared block — the same fix `drive_sensitivity.yaml` already
  applied to `max_linear_mps`/`max_angular_radps`/`deadzone` — and
  `xbox_teleop.launch.py` updated to load it from there instead.
  `rover_web_gui` now loads this same file too, for the transport
  position values above.
- **Mast pitch range corrected: ±60° was wrong, real range is ±180°** —
  confirmed against real hardware while adding the post-calibration
  sequence below (needed a genuine target position, 170°/180°, that
  turned out to exceed the old placeholder). Updated everywhere that
  figure was used: `mast.xacro`'s joint limit, `mast_topology.yaml`'s
  `max_head_pitch_deg`, `joy_mapping.py`'s fallback default, the web
  GUI's pitch slider range, and — because pitch is now the *larger* of
  the two ranges (360° total vs. yaw's 340°) — the reasoning behind
  `mast_uno3.ino`'s `kHomingMaxTravelSteps` safety margin, which used
  to size itself specifically against yaw being the bigger one. The
  numeric margin (500°) still comfortably covers the new range without
  needing to change, but the comment explaining *why* did.
- **Mast: calibration now offsets to each axis's minimum, then drives
  to true zero from there — not "zero at the switch, then a separate
  verification move"** — corrected from an earlier version of this
  same feature (see `docs/journal.md` for that history) that had it
  backwards: each limit switch is physically at that axis's minimum
  bound (-170° yaw, -180° pitch — real, bench-confirmed values, not
  placeholders), not its center, so `serviceHoming()` now recognizes
  triggering it as reaching that minimum (`setCurrentPosition()` to
  the minimum in steps, not to 0), then drives from there to true
  zero. Only once both axes actually arrive does `homed` become true —
  this is genuinely what "home" means here, the centered position
  reached *from* the minimum, not the minimum itself — and the TB6600
  drivers disable themselves (`servicePostCalibration()`, now a single
  move-to-zero phase instead of the previous two-phase verify-then-zero).
  `MastCommand`'s `driver_enable` field and the web GUI's matching
  `CLOSE DRIVER (ENABLE)`/`OPEN DRIVER (DISABLE)` toggle are unchanged
  by this correction — both are still needed so the mast stays usable
  after the sequence auto-disables it.

  The gating logic this required is unchanged too, and the reasoning
  behind it matters enough to spell out regardless of which version of
  the sequence is running: `rover_mast_bridge` sends `MastCommand`
  frames continuously at its own control rate once past its one-shot
  homing request — using whatever's in its last-known command, which
  starts as all-zero/disabled before the operator has ever touched the
  mast panel. Applying `driver_enable` unconditionally on every frame
  (the way the arm's own enable field works) would disable the drivers
  mid-seek or mid-move-to-zero from nothing more than the bridge's own
  routine resend — indistinguishable at the protocol level from a
  genuine command — stranding a stepper that can't move de-energized.
  Both `driver_enable` and yaw/pitch targets are gated on a single
  check now (`if (homed)`), which is enough on its own: `homed` is
  already false for the entire seek-then-move-to-zero sequence, so
  there's no separate "is a sequence currently active" flag needed
  the way an earlier version of this required.
- **Base steering servos: per-corner calibration, not one shared set** —
  `base_mega1.ino`'s `kServoMinUs`/`kServoMaxUs`/`kServoNeutralUs`
  became 4-entry arrays (FL, FR, RL, RR), replacing one set of
  constants shared across all four. The file's own comment had always
  said to "bench-test each servo," but the code only ever supported
  tuning them all identically — real 40kg servos, even same model and
  batch, vary enough in actual center/travel that a shared calibration
  was only ever a starting point. Fixed a real accuracy bug in the
  same pass: the angle-to-pulse-width conversion always scaled off the
  max-side span (`kServoMaxUs - neutral`), even for negative angles,
  which is only correct if neutral happens to sit exactly centered
  between min and max. A real bench-calibrated servo often won't -
  using the wrong span for negative angles doesn't just misreport the
  angle, it silently shrinks the achievable range on whichever side is
  shorter. Now uses the min-side span (`neutral - kServoMinUs`) for
  negative angles and the max-side span for positive ones. All four
  corners still start at identical placeholder values (600/2400/1500us) -
  intentional, not an oversight, until each is actually bench-tested.
- **Servo movement: eased, not instant, on both axes that move a
  physical servo — and, as of this session, neither one via
  ServoEasing anymore.** The microscope's lens cover moved off
  ServoEasing too, at the user's own explicit request - unlike the
  base's steering servos below, this wasn't a technical necessity
  (ServoEasing genuinely can't drive a PCA9685; the cover servo is
  still direct-pin, so ServoEasing itself would have kept working
  fine here) but a deliberate choice. Smoothed movement was preserved
  anyway, reimplemented as the same kind of small, non-blocking
  custom ramp (`updateCoverEasing()` in `microscope_uno4.ino`,
  mirroring `base_mega1.ino`'s own `updateSteerEasing()` almost
  exactly) rather than dropped to an instant snap - this project has
  consistently valued eased servo motion, and removing one specific
  library was never a reason to lose it. `kCoverEaseSpeedDegPerSec` =
  60°/s (slower on purpose, since the cover is a rarely-triggered
  binary toggle rather than a continuously-recommanded value under
  responsiveness pressure) is unchanged in value and meaning, just
  read by a different mechanism now - still a placeholder like every
  other uncalibrated constant in this project, pending bench-tuning.

  **The base's 4 steering servos moved off ServoEasing entirely** when
  they moved to a PCA9685 (see the dedicated bullet on that below) -
  ServoEasing doesn't work with an I2C PWM driver, only direct-pin,
  standard-Servo-library-style control. Smoothed movement was
  reimplemented as a small, non-blocking custom ramp
  (`updateSteerEasing()` in `base_mega1.ino`) rather than dropped -
  every corner's commanded position moves toward its latest target at
  a fixed rate (`kSteerEaseSpeedUsPerSec`, 3000us/sec - a direct
  translation of the previous 300°/s ServoEasing rate into this
  board's own pulse-width terms, not a re-guess) each `loop()`
  iteration, rather than jumping there in one write. Both axes still
  share the same underlying fix from when steering first got its own
  per-corner calibration (previous bullet): a servo's actual physical
  position at power-on is unknown to either mechanism, so both seed
  their own tracking at each servo's calibrated neutral/closed
  position and write it immediately at boot, rather than let the
  first real command jump from an assumed, possibly-wrong starting
  point.

  **Licensing note, stated plainly rather than as a legal opinion**:
  ServoEasing is GPL-3.0, while the rest of this project's ROS
  packages are Apache-2.0 - this used to be worth flagging per-board
  (base's steering lost it several sessions ago; the microscope's
  cover kept it until this one). **As of this session, ServoEasing no
  longer appears anywhere in this project at all** - confirmed
  directly (`grep -rn "ServoEasing" firmware/`) rather than assumed
  from memory. A firmware binary
  that compiles in GPL-3.0 code is generally itself subject to
  GPL-3.0's copyleft terms if that binary (or the sketch producing it)
  is ever distributed to others - this concern is now retired for
  this project's firmware entirely, not just reduced on one board.

- **Microscope: cover open/close became two explicit buttons, not one
  toggle; LED gained on/off buttons alongside its existing slider.**
  Both requested together with the ServoEasing removal above. The
  cover's web GUI control changed from a single button that flipped
  its own label and state each click to two separate, stateless
  buttons (`OPEN COVER`/`CLOSE COVER`) - matching this project's own
  established pattern for a binary physical state (see the mast's own
  `ERECT`/`STOW` buttons for the precedent this follows) rather than
  inventing a new one. The actual current state is read from the
  telemetry panel's own `COVER` field, not tracked by which button was
  clicked last.

  **LED PWM interpretation, stated explicitly so it can be corrected
  if wrong**: requested as "change the PWM function to 0~5V for
  dimming" - standard Arduino PWM on a 5V board already swings between
  0V and 5V (duty-cycle-modulated, not a filtered analog level), which
  is exactly what this board's existing `analogWrite()`-based LED
  dimming already does. Read as describing that existing mechanism
  rather than requesting new filtering hardware (an RC low-pass
  filter or similar, to produce a true analog DC level rather than a
  switched signal) - no firmware or wiring change was made to the LED
  output itself, only the web GUI gained `LED ON`/`LED OFF` buttons
  alongside the existing brightness slider, setting it to 255 or 0
  respectively and keeping the slider's own displayed value in sync
  either way it's driven. Flag this back if an actual analog dimming
  circuit was intended instead - that would be a real hardware
  addition, not a firmware-only change.
- **Base steering: moved from direct Mega pins to a PCA9685, reversing
  an earlier explicit decision** — an earlier session's own header
  comment said plainly "no PCA9685 (middle wheels are fixed)"; this
  session added one back, deliberately. `base_mega1.ino`'s 4 steering
  servos now connect to a PCA9685 16-channel I2C PWM driver (channels
  0-3 -> FL/FR/RL/RR) instead of Mega pins A4-A7 directly.

  **CORRECTED: address is 0x40 (factory default, unjumpered), not
  0x70 as an earlier version of this same entry said** — a real
  mistake in that earlier session, caught by the user's own I2C scan
  turning up 0x70 on a board that hadn't been jumpered yet. 0x70 isn't
  a normal, jumper-configurable slave address on this chip at all -
  it's the PCA9685's built-in "LED All Call" address, enabled on
  every unit at power-up regardless of A0-A5 jumper state (stated
  directly in the chip's own datasheet: "the default LED All Call
  I2C-bus address... must not be used as a regular I2C-bus slave
  address since this address is enabled at power-up"; Adafruit's own
  FAQ says it more plainly still - "set it to 0x71 or anything other
  than the default 0x70"). This project only ever has one PCA9685, so
  there's no reason to move it off 0x40 at all - no jumpers to solder,
  `kPca9685Address` in firmware matches the factory-default address
  directly.

  **A real pin conflict had to be resolved, not just a new pin
  chosen**: I2C on a Mega is hardwired to pins 20 (SDA) and 21 (SCL) -
  not configurable the way 1-Wire is - and pin 20 was already
  committed to the DS18B20's own data line (moved there in an earlier
  session specifically because it was the Mega's *former*, then-unused
  SDA pin). Moved the DS18B20 to A4 instead - one of the pins the
  steering servos vacated moving to the PCA9685 - rather than search
  for a third free pin or leave a genuine conflict unresolved.

  **The PCA9685's V+ terminal restores a dedicated servo power rail**
  that the direct-pin version of this same steering setup never had
  and had to call out separately as a gap to fill: wire V+ to its own
  adequately-rated 5-6V supply (four 40kg-class digital servos can
  draw well beyond what the Mega's onboard 5V regulator safely
  delivers), sharing ground with the Arduino, never from the Mega's
  own 5V pin.

  Library: **Adafruit PWM Servo Driver Library** (BSD licensed,
  confirmed against the library's own source headers directly rather
  than trusted to a third-party index that shows "NOASSERTION" for it
  - a known quirk of automated license detection against a
  non-standard header format, not an actual absence of a license).
  `writeMicroseconds()` needed no rework of the existing per-corner
  `kServoMinUs`/`kServoMaxUs`/`kServoNeutralUs` calibration arrays at
  all - the library's own example code happens to use the identical
  600-2400us convention this project's placeholders already used, and
  the angle-to-microseconds math itself is entirely independent of
  which mechanism ultimately receives the resulting value.
- **Arm actuators: EBA-17-M planetary gearbox, 120:1, per joint** —
  updates the arm's originally-placeholder 5:1 gear assumption
  throughout: `steps_per_joint_rev` (200 full steps × 1/16
  microstepping × 120:1 = 384000, `rover_arm/config/arm_topology.yaml`
  and `trajectory_action_server.py`'s fallback default), the firmware's
  homing safety-cutoff (`arm_mega2.ino`'s `kHomingMaxTravelSteps`,
  scaled to preserve the same ~675° angular safety margin as before),
  and `arm.xacro`/`joint_limits.yaml`'s velocity/acceleration limits
  (recomputed from the firmware's own unchanged motor-shaft speed cap
  through the new, much higher reduction — not independently chosen).
  Worth being explicit about the real consequence, not just the
  numbers: achievable joint speed drops from what the old 5:1
  placeholder implied (~45°/s) to about **1.9°/s** at the same motor
  step rate — the genuine tradeoff of a 24× higher reduction ratio
  (more torque and precision, less speed). All five joints now show
  the *same* velocity/acceleration limit rather than the previous
  per-joint spread, correctly — that spread was never grounded in
  anything joint-specific, and all five share the same motor + gearbox
  + firmware speed cap. One more thing worth double-checking on the
  bench: the publicly documented ratio for the EBA-17/EBA-17-S product
  line (ToolBoxRobotics) is 38.4:1, not 120:1 — 120:1 is what was
  specified for the "M" variant actually in use here, but worth
  confirming against the datasheet/nameplate if these numbers ever
  look off in practice.

- **User-supplied URDF template**: a user-uploaded xacro file
  (differential/bogie suspension, "head" instead of the pan/tilt mast,
  `arm_j1`-`arm_j5` joint naming, solar panels, high-gain antenna,
  mesh-based visuals) was corrected — it had several real bugs
  independent of anything project-specific: empty wheel-generating
  macros producing no geometry at all, arm transmissions referencing
  joint names that didn't exist, a duplicate transmission name, an
  entire right-side suspension typed `fixed` with zero-width limits
  (permanently rigid) while the left side was fully functional,
  duplicate chassis links, missing mass/inertia, and 15 `<material>`
  references with no matching declaration anywhere. The corrected
  version is real and validated (`xacro` expansion plus a full link/
  joint/transmission consistency check both pass cleanly) but lives at
  `rover_description/reference/opportunity_style_template.urdf.xacro`,
  **not** wired into the active model — its wheel/arm architecture and
  naming conflict directly with `rover_base`'s kinematics and the
  MoveIt config built in an earlier session, both keyed to this
  project's actual, hardware-tested joint names and topology. Swapping
  it in live would have silently broken both. The two genuinely new,
  non-conflicting pieces from that template — solar panels and a
  high-gain antenna — **were** added to the active model
  (`rover_description/urdf/accessories.xacro`), as primitive geometry
  rather than the template's mesh references (no `.stl` files were
  supplied, and this project's whole model has always used primitives
  — see `rover_description/meshes/README.md`). Both are `fixed`
  joints, not `revolute`: neither has real actuation hardware or
  firmware anywhere in this project, so making them look plannable
  would overstate what's actually there. A full migration to the
  corrected template's own architecture — if that turns out to be
  what's actually wanted — is real, scoped, deferred work; see "Known
  gaps."

- **FZ0430 voltage sensors**: one per Arduino Mega (base, arm, mast -
  not the microscope Uno, which wasn't asked for one), on analog pin
  A0 on each board, reporting the main battery supply in millivolts.
  "FZ043" in the original request is treated as shorthand for the
  FZ0430, a common 5:1 resistive-divider breakout (0-25V range on a
  5V-logic board) - flag this if a different sensor was actually
  meant. Conversion math
  (`(raw / 1023.0) * 25000.0` → millivolts) assumes the standard
  5V AREF and the module's documented 5:1 divider ratio; if your
  specific board's divider resistors measure differently, that's a
  one-line change to each firmware's `readSupplyVoltageMv()`.
  Voltage-only, no current sensing - see "Known gaps" for what that
  would take.
- **IMU board**: SparkFun VR IMU Breakout - BNO086 (Qwiic). Its factory
  I2C address is 0x4B, but that's inert here — its PS0/PS1 jumpers
  must be strapped for **UART-RVC** mode (not I2C), since it's wired
  via its UART edge pins to a Waveshare "USB TO TTL (B)" (CH343G),
  which is a plain USB-to-UART bridge and can't carry I2C at all. If
  you ever rewire it for genuine I2C, this UART-RVC driver no longer
  applies and you'd need an address-aware I2C driver instead — and
  note SparkFun's own docs advise against 8-bit AVR (Uno/Mega) hosts
  for that chip over I2C; they recommend an ESP32 or SAMD51.
- **Base wheel/steer kinematics**: exact instant-center-of-rotation
  geometric steering, standard differential/skid throttle mixing
  rather than an exact zero-slip velocity model. See "Base drive
  modes" above and `rover_base/rover_base/kinematics.py`.
- **Mast erect/stow**: modeled as a single revolute "pivot" joint
  driven by the lift DC motor (0° = horizontal/transport, 90° =
  vertical/service), since the spec describes it as "laterally
  pivoting," not as a linear lift.
- **Mast: Arduino Uno (was a Mega), HW-039 lift driver, yaw/pitch
  calibration switches** — the mast's controller was originally a
  Mega with no head calibration switches (head zeroed to whatever
  orientation it powered up in) and an assumed-generic 2-direction-pin
  H-bridge for the lift motor (no specific driver part number had been
  given). All three changed together: it's an Uno now, which meant a
  full pin remap under a genuinely tight budget (11 of the Uno's 12
  usable digital pins committed, one spare) since none of the old
  Mega-only pins (22, 23, 30, 31) exist on an Uno at all; yaw and
  pitch each got their own calibration switch and now home on startup
  through a sequential state machine mirroring `rover_arm`'s exact
  pattern (`mast_uno3.ino`'s `startHoming()`/`serviceHoming()`), not a
  new design invented from scratch; and the lift now drives through an
  HW-039 (dual BTS7960B half-bridge) module, whose native RPWM/LPWM
  interface is genuinely different from both the base's DRI0002
  boards (single direction pin + PWM) and the *previous* generic
  H-bridge assumption (PWM + 2 direction pins) — simplified to a
  single shared enable pin (R_EN/L_EN tied together) specifically to
  fit the Uno's pin budget, with R_IS/L_IS (current sense) left
  unconnected since this application doesn't need them. The lift
  itself still doesn't home — it has no step-relative position to
  zero, just directly-read limit switches — so lift commands work
  immediately regardless of yaw/pitch homing state, same as before.
- **Mast yaw/pitch: TB6600 driver (was A4988)** — a follow-up to the
  Uno rebuild above, prompted by a real debugging session: the
  original A4988 wiring never had an explicit enable pin driven by
  firmware at all, relying on whatever that specific breakout's
  floating-EN default happened to be. TB6600's opto-isolated ENA input
  is less forgiving of being left floating, so `mast_uno3.ino` now
  drives it explicitly (`kHeadEnablePin`) — shared between both axes,
  since this commits the very last available Uno pin (13), leaving the
  board's digital pin budget fully spent (12 of 12). Wiring is assumed
  common-anode (PUL+/DIR+/ENA+ to +5V, the three "-" lines to the
  Arduino pins, ENA active-low) — the standard way to drive one of
  these from 5V logic, but flip the enable polarity in firmware if
  wired common-cathode instead. TB6600 sets microstepping via DIP
  switches on the driver rather than firmware, same idea as an A4988's
  MS1/MS2/MS3 pins but physical rather than software-controlled —
  `kStepsPerDegYaw`/`kStepsPerDegPitch` (already placeholder values,
  not yet bench-calibrated) need to match whatever those switches are
  actually set to. STEP/DIR pin assignments themselves didn't change —
  TB6600's PUL/DIR inputs are functionally identical to STEP/DIR from
  AccelStepper's perspective, so this was a wiring-and-firmware-enable
  change, not a pin-remap.
- **Arm: mixed drivers — J1-J3 TB6600, J4/J5 still A4988** — unlike
  the mast's two TB6600s, which share one enable pin, each of the
  arm's three TB6600 drivers gets its own independent enable pin
  (`arm_mega2.ino`'s `kTb6600EnablePin[3]`, pins 13/14/15). This is a
  real electrical distinction, not inconsistency for its own sake:
  TB6600's opto-isolated ENA input sinks real current per board
  (roughly 10-15mA, driver-dependent) when driven active, and ganging
  three of them onto one Arduino pin risks exceeding a single GPIO's
  safe sink rating (~20-40mA depending on the chip) — a risk the
  mast's two-board case didn't carry as heavily, and one the Mega's
  abundant spare pins (unlike the mast's tightly-budgeted Uno) meant
  there was no real reason to accept anyway. J4 and J5's A4988s keep
  sharing one enable pin between the two of them (`kA4988EnablePin`,
  still pin 12) — A4988's EN is a simple direct-logic input, not
  opto-isolated, and safely shareable the same way all five joints
  used to share this exact pin before the split. One real consequence
  worth flagging: `arm_topology.yaml`'s `steps_per_joint_rev` is still
  five identical values, but that's no longer guaranteed correct —
  TB6600 sets microstepping via DIP switches, and there's no guarantee
  those end up matched to the A4988s' own microstepping setting. If
  they don't, this needs five genuinely different values, not five
  copies of one; `joint_conversion.py` already handles that correctly
  per-joint (see its own tests), so it's a "measure and update the
  yaml" task, not a code change, whenever the real hardware is bench-verified.
- **Microscope focus/zoom**: modeled as a single combined stepper axis
  (as literally specified: "a stepper motor ... for focusing and
  zooming") rather than two independent axes. No calibration switch
  either — same power-on-zero convention the mast head used to use,
  before this session's calibration switches gave it a real one.
- **Microscope: DRV8825 replaces ULN2003, 24BYJ-48 replaces 28BYJ-48** —
  a real compatibility question, not just a part swap: DRV8825 is a
  bipolar-only driver, and both the 28BYJ-48 and 24BYJ-48 ship as
  5-wire unipolar motors. They're only usable together with a specific
  wiring technique — connect just the four coil-end wires to the
  driver's two coil outputs and leave the center-tap (common) wire
  completely disconnected, effectively treating the motor as a 4-wire
  bipolar one. This is a documented, real technique, not something
  improvised for this project, but it's an easy wire to connect by
  mistake, and doing so can short part of a coil — flagged prominently
  in `microscope_uno4.ino`'s header, worth reading before wiring this
  up. Also requires the 12V-rated 24BYJ-48 variant (DRV8825 needs
  8.2-45V on its motor supply; the 5V variant won't work). Upside: this
  unifies the microscope's stepper control with the arm and mast's own
  STEP/DIR/ENABLE pattern (`AccelStepper::DRIVER`), replacing ULN2003's
  4-pin phase sequencing — one less pin needed, and a driver with a
  real enable pin where the old one had none.
- **"Open/close driver" read as enable/disable** — DRV8825's new
  enable pin (`MicroscopeCommand.driver_enable`) is exposed as a web
  GUI toggle, labeled `CLOSE DRIVER (ENABLE)` / `OPEN DRIVER (DISABLE)`
  — mapped to the electrical open-circuit (no current, disabled) /
  closed-circuit (current flows, enabled) convention. Flagging this
  explicitly since the mapping isn't the first thing everyone reaches
  for; the parenthetical on each button label exists specifically so
  the actual effect is never ambiguous regardless of which
  interpretation someone starts from. Starts disabled at boot, same
  convention as the arm/mast boards.
- **Lens cover open/close** — already existed (`scope-cover` button,
  full firmware/protocol/GUI wiring) before this session; confirmed
  rather than rebuilt.
- **Focus/zoom presets are client-side only** — the 3 "record"/"go to"
  button pairs remember slider positions in the browser's own JS
  state, with no firmware, protocol, or backend involvement at all.
  Deliberately lightweight for what's fundamentally a convenience
  feature: presets don't survive a page reload, and there's no
  persistence across sessions. If that turns out to matter, the
  natural upgrade is moving preset storage server-side (`ros_bridge.py`)
  so it survives a browser refresh — not a firmware change either way,
  since presets are just remembered `focus_target_steps` values, not a
  new hardware capability.
- **Arm inverse kinematics**: now available via MoveIt2
  (`rover_arm_moveit_config` + `rover_arm`'s `trajectory_action_server`)
  — see "Arm motion planning" below. `ArmCommand` itself is still
  joint-space (motor steps) only; MoveIt plans in task space and the
  trajectory action server converts each planned waypoint before it
  ever reaches `rover_arm_bridge`, which has no idea MoveIt is
  involved at all.
- **LIDAR/GPS/joystick drivers**: reused from the ROS ecosystem
  (`rplidar_ros`/`sllidar_ros2`, `joy`) rather than reimplemented —
  install these separately, see below.

## Building

For a full walkthrough (fresh-machine ROS 2 install, rosdep gaps,
Arduino firmware flashing, udev rules, per-subsystem verification,
troubleshooting table) see **`docs/INSTALL.md`**. Condensed version:

```bash
cd ~/mars_rover_ws
rosdep install --from-paths src --ignore-src -r -y
```

A few rosdep keys are worth a heads-up (mirroring issues hit on this
project before):
- `python3-pynmea2` may not resolve via rosdep on all Ubuntu 22.04
  setups — if it fails, `pip3 install pynmea2` directly.
- `python3-fastapi` / `python3-uvicorn` likewise — if rosdep can't
  find them, `pip3 install fastapi uvicorn`.
- `rplidar_ros` is **not** in this workspace; clone it (or
  `sllidar_ros2`) into `src/` yourself before building, or pass
  `use_lidar:=false` to bringup if you don't have the hardware yet.
- Arduino firmware needs the **AccelStepper** library
  from the Arduino Library Manager (no longer **ServoEasing** as of
  this session - it was previously needed for base_mega1's steering
  and microscope_uno4's lens cover servo, both of which now use a
  small custom easing ramp instead; see README's "Explicit
  assumptions" for the history. The standard **Servo** library, which
  ships bundled with the AVR core, is still used directly by both -
  nothing separate to install for that part), plus `firmware/common/RoverProtocol` installed as a
  library (copy that folder into your
  `Arduino/libraries/` directory, or Sketch → Include Library → Add
  .ZIP Library on it).

```bash
colcon build --symlink-install
source install/setup.bash
```

## Running

```bash
# Everything: base/arm/mast bridges, microscope, sensors, LIDAR, Xbox teleop, web GUI
ros2 launch rover_bringup bringup.launch.py

# Bench-testing without a controller or the LIDAR installed yet:
ros2 launch rover_bringup bringup.launch.py use_teleop:=false use_lidar:=false

# Visualize the URDF alone (no hardware needed):
ros2 launch rover_description display.launch.py
```

Ground control console: open `http://<rover-host>:8080/` in a browser.
The microscope panel has an "open in new tab" link
(`http://<rover-host>:8080/microscope`) for a dedicated, full-page
microscope view with its own snapshot/recording controls, per the
project brief.

Xbox 360 controller: hold **RB** (deadman) for any command to leave
neutral, **LB** cycles DRIVE → ARM → MAST → MICROSCOPE. See
`rover_teleop/config/xbox_teleop.yaml` for the full mapping and a
reminder to verify axis/button indices against your actual
driver (`ros2 topic echo /joy`) before relying on it.

## Testing

Every serial protocol, the base kinematics, the BNO086 UART-RVC frame
parser (checksum verified against the datasheet's own worked example),
the GPS NMEA parser, the Xbox teleop mapping, and the udev device
identification/rules-generation logic all have unit tests that run
with plain `pytest` — no ROS 2 install, hardware, or colcon build
required:

```bash
cd ~/mars_rover_ws
PYTHONPATH=src/rover_protocol python3 -m pytest src/*/test tools/test -q
```

207 tests as of the last update. Before touching firmware or protocol
code again, also run the syntax/parse sweep:

```bash
python3 tools/validate_workspace.py
```

## Known gaps / natural next steps

- A corrected, validated, but **not active** alternative rover model
  (differential/bogie suspension, `arm_j1`-`arm_j5` naming) sits at
  `rover_description/reference/opportunity_style_template.urdf.xacro`.
  If that architecture is actually wanted over the current one, the
  real work is reconciling it with `rover_base`'s kinematics
  (currently built for 4-corner-steering + 2 fixed middle wheels, not
  a differential/bogie linkage), `rover_arm`'s joint names/limits
  (`arm_topology.yaml`, `joint_conversion.py`), the MoveIt SRDF, and
  every wiring diagram — a deliberate migration, not a file swap. See
  "Explicit assumptions" for what was and wasn't done with it.
- Voltage monitoring (base/arm/mast, FZ0430 sensors) is voltage-only —
  no current sensing, and no low-voltage warning in the web GUI. The
  latter was deliberately left out rather than guessed at: a sensible
  threshold depends on the actual battery chemistry/cell count, which
  isn't specified anywhere in this project yet. Cheap to add
  (`formatVoltage()` in `app.js`) once that's known.
- `udev` rules ship with placeholder VID:PID/serial values captured
  from no particular hardware — run `python3 tools/identify_rover_devices.py`
  (see docs/INSTALL.md Section 8) to generate a real, filled-in rules
  file interactively instead of hand-editing the placeholders.
- SLAM/Nav2 are wired up but ship with standard, untuned starting
  parameters (see "Navigation" above) — expect to adjust costmap
  inflation and controller behavior against your actual environment.
- MoveIt2's arm planning config (`rover_arm_moveit_config`) is
  hand-authored, not generated by MoveIt Setup Assistant, and several
  values are explicitly-flagged placeholders rather than measured from
  the real hardware — acceleration limits in `joint_limits.yaml`, and
  the collision matrix in the SRDF, which covers the arm's own chain
  but isn't an exhaustive whole-rover pass. See "Arm motion planning"
  below and the comments in those files for specifics.
- `rover_web_gui`'s ROS-facing code has no automated tests (it needs a
  live rclpy environment to exercise meaningfully); the pure protocol
  and kinematics logic it depends on is fully covered instead.
