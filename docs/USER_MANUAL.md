# Mars Rover User Manual

This is the day-to-day operating guide: starting the rover up, driving
it, using the arm/mast/microscope, and reading what it's telling you.

It assumes the rover is **already built and configured** — firmware
flashed, udev rules installed, the ROS 2 workspace built. If any of
that isn't done yet, start with `docs/INSTALL.md` instead; this manual
picks up where that one leaves off. For the underlying engineering
detail behind anything mentioned here (exact topic names, kinematics,
design decisions), see `README.md`.

---

## Before you operate it: safety

This rover has real motors, servos, and steppers that move with real
force. A few things worth internalizing before the first power-up:

- **The deadman switch matters.** Driving from the Xbox controller
  requires holding **RB** the whole time — in *every* control mode,
  not just DRIVE. Release it and commands stop being sent. This is
  deliberate, not a bug: it's the difference between "the controller
  is in your hands" and "the controller fell on the floor still
  pointed at a stick."
- **Every board has its own watchdog.** If commands stop arriving
  (dropped connection, closed terminal, crashed node) for about half a
  second to a second depending on the board, that board stops or holds
  position on its own — you don't have to trigger a stop manually for
  a dropped link to be safe, but don't rely on it as your *primary*
  stopping method either.
- **STOP mode is the fastest way to stop driving.** Web GUI: the red
  `STOP` button. Xbox controller: **Y**, from any drive mode, at any
  time — it overrides everything else immediately. Press **X** to
  resume (STOP doesn't clear itself).
- **The arm won't move until it's calibrated**, and that's
  intentional — see [Arm](#arm) below. Don't fight this by trying to
  force movement before `homed: true`; it's a safety gate, not a bug.
- **The mast moves on its own right after calibration finishes** —
  each axis drives from its limit switch (at that axis's extreme)
  back to center, then its drivers disable themselves. Expected, not a
  malfunction — see [Mast](#mast) below — but worth knowing before it
  happens the first time so it isn't mistaken for something going wrong.
- **Stay clear of moving parts** during arm motion, mast erect/stow,
  and while driving — obvious, but worth saying once, plainly, before
  the rest of this document gets into the details.

---

## 1. Overview

| Subsystem | What it does | Controlled via |
|---|---|---|
| **Base** | 6-wheel drive, 4-corner steering | Xbox controller, web GUI joystick |
| **Arm** | 5-axis manipulator | Web GUI (sliders + buttons), Xbox controller, optionally MoveIt2 |
| **Mast** | Pan/tilt camera head + erect/stow lift | Web GUI, Xbox controller |
| **Microscope** | Focus/zoom, LED, lens cover, snapshot/recording | Web GUI, Xbox controller |
| **Antenna** | High-gain-antenna gimbal (azimuth/elevation pointing) | Web GUI, Xbox controller |
| **Sensors** | IMU, GPS, LIDAR, main + microscope cameras | Automatic once launched |
| **Navigation** | SLAM mapping, autonomous driving, GPS waypoints | `ros2` commands (advanced) |

Two ways to control the rover, and you can use either or both at once:

- **Xbox controller** — full manual control, all subsystems, one
  device. **LB** cycles which subsystem the controller is currently
  driving: `DRIVE → ARM → MAST → MICROSCOPE → ANTENNA → DRIVE`.
- **Web GUI** — open `http://<rover-host>:8080/` in a browser. Buttons
  and sliders for every subsystem, plus live telemetry. No controller
  needed.

Both talk to the same underlying rover at the same time — driving from
the controller while watching telemetry in the browser is the normal
way to work.

---

## 2. Starting up

### 2.1 Power on

Power on the base, arm, mast, and microscope boards per your own
wiring, then power on (or SSH into) the host computer. Give the
Arduino boards a moment — they auto-reset when the host's serial
connections first open, which the software already accounts for
(`boot_grace_sec` in each board's config), but it's still worth a
few seconds before expecting a response.

### 2.2 Launch

The everyday case — driving, arm, mast, microscope, and sensors, no
mapping or autonomy:

```bash
ros2 launch rover_bringup bringup.launch.py
```

Common variations:

```bash
# Bench-testing without the physical LIDAR or Xbox controller plugged in
ros2 launch rover_bringup bringup.launch.py use_lidar:=false use_teleop:=false

# Building a map of a new area (see Navigation, below)
ros2 launch rover_bringup bringup.launch.py use_slam:=true

# Driving autonomously against a map saved from a previous SLAM session
ros2 launch rover_bringup bringup.launch.py use_navigation:=true nav_map:=/path/to/map.yaml

# Arm motion planning via MoveIt2, on top of manual arm control
ros2 launch rover_bringup bringup.launch.py use_moveit:=true
```

These combine freely except `use_slam` and `use_navigation` — don't
set both at once (build a map, save it, *then* navigate against it).

### 2.3 What to expect while it comes up

- **Arm and mast home themselves automatically.** Watch for "sent
  homing request" in the terminal log from each. This can take a
  while — the arm's actuators are heavily geared, so a joint starting
  far from its limit switch can take tens of seconds; that's normal,
  not stuck. Don't send arm or mast movement commands until homing
  finishes (they'd be silently ignored anyway — see
  [Arm](#arm)/[Mast](#mast)). The mast specifically drives from each
  axis's limit switch back to center as part of this same process —
  `homed` doesn't actually go true until it arrives there and disables
  its own drivers, not the instant a switch triggers — so expect a few
  seconds of movement before it reports homed, then press
  `CLOSE DRIVER (ENABLE)` in the web GUI before expecting the mast
  sliders to do anything.
- **The antenna gimbal also homes itself automatically**, simpler than
  the mast's own sequence: each axis's calibration switch sits at that
  axis's real operational minimum, so `homed` goes true as soon as
  both switches are found, with no follow-on movement. Its drivers
  start enabled (homing needs them energized) but won't accept manual
  commands until `homed` is true, and — like the microscope — need
  `CLOSE DRIVER (ENABLE)` pressed before the azimuth/elevation sliders
  do anything.
- **The microscope's focus driver starts disabled** and the lens cover
  starts closed — both by design, not homing. Enable/open them
  explicitly once you're ready to use the microscope.
- **Board status should go green.** Web GUI: status lamps at the top
  light up as each board's serial link comes online. If one stays red
  for more than a few seconds after power-on, see
  [Troubleshooting](#troubleshooting).

### 2.4 Open the web GUI

```
http://<rover-host>:8080/
```

(Replace `<rover-host>` with the rover's hostname or IP — `localhost`
if you're on the rover's own machine.) The microscope also has its own
dedicated page at `/microscope`, with the same controls as the main
dashboard's microscope panel, useful for a full-screen camera view.

---

## 3. Shutting down

1. Bring the arm and mast back to a safe, stowed position first (see
   [Arm](#arm) → Return Home, [Mast](#mast) → Stow) — don't just kill
   power with the arm extended or the mast erect.
2. `Ctrl-C` the `bringup.launch.py` terminal, or stop it however you
   normally manage long-running processes. Every bridge node sends an
   explicit stop command to its board as it shuts down — you don't
   need to manually zero anything first.
3. Power off the boards.

---

## 4. Operating the rover

### Driving

Three drive modes, switched independently of whatever's currently
commanding the rover:

| Mode | What it does |
|---|---|
| **ACKERMANN** | Normal driving — forward/back plus a turn, corner wheels steer through an arc like a car |
| **POINT-TURN** | Rotates about the rover's own center, no forward motion |
| **STOP** | Unconditional stop — ignores any drive input until switched out of |

**Web GUI**: `ACKERMANN` / `POINT-TURN` / `STOP` buttons at the top of
the drive panel, plus a draggable on-screen joystick — drag to set
speed and turn rate proportionally, release to snap back to zero. The
joystick is visually and functionally disabled while in STOP mode.

**Xbox controller** (in DRIVE mode — press LB until the controller is
in DRIVE): left stick Y = throttle, left stick X = turn, **X** toggles
ACKERMANN ↔ POINT-TURN, **Y** forces STOP (press **X** to resume).
Remember: **RB must be held** for any of this to reach the rover.

Both the controller and the web GUI joystick share one speed/turn-rate
ceiling and deadzone (`rover_teleop/config/drive_sensitivity.yaml`) —
tune that one file if driving feels too twitchy or too sluggish;
changing it there affects both control surfaces at once.

### Arm

**The arm will not move until every joint is individually
calibrated.** This isn't a fault condition — it's how the arm
establishes where its joints actually are, since the steppers have no
absolute position sensor of their own. It calibrates all 5 joints
automatically on startup; the web GUI also lets you re-calibrate
individually.

**Web GUI, arm panel:**

| Control | Effect |
|---|---|
| `E-STOP` | Immediately stops all 5 joints (a fast, controlled deceleration, not an instant freeze) and blocks any further movement until cleared. **Does not de-energize the drivers** — see the callout below before assuming that's a bug |
| `CLEAR E-STOP` | Releases the e-stop, allowing movement again |
| 5 sliders | Direct per-joint position (motor steps from the calibrated center — J1/J3 ±150°, J2/J4 ±100°, J5 ±170°; each slider's own range matches its joint's real operational limit, and is enforced again on the firmware side regardless of what the slider allows) |
| `ENABLE DRIVERS` / `DISABLE DRIVERS (FREE-SPIN)` | Single toggle — energize/de-energize all 5 joint motors. Label and color both follow the arm's own actual current state, not which one you last clicked, same as the `DRIVERS` line in the telemetry panel below |
| `CALIBRATE J1`–`CALIBRATE J5` | Re-calibrate one joint on its own, leaving the other four alone |
| `CALIBRATE ALL 5` | Full re-calibration sequence, same as what runs automatically at startup |
| `INITIAL POSITION` | Move to the arm's predefined starting pose |
| `TRANSPORT POSITION` | Move to a compact pose safe for driving/moving the rover |
| `SERVICE POSITION` | Move to a predefined pose for servicing/accessing the arm |

A status line under those buttons shows the result of whichever one
you last pressed. Each joint's position in the telemetry panel is
marked ✓ or ✗ for calibrated/not, and the panel's own `DRIVERS` and
`E-STOP` rows show the arm's actual current state — driven by the arm
itself, not by which button you last clicked, so both stay correct
even after a page reload or if someone else triggered a change.

**All three preset buttons currently move to the same pose** (every
joint at its own true center) — distinct, genuinely useful poses for
each are still pending real-world calibration. They'll diverge once
that's done; there's nothing to configure on your end for this.

**The emergency stop does not cut power to the joint motors.** This is
deliberate, not an oversight: the arm is gravity-loaded, and cutting
power mid-motion risks it dropping under its own weight rather than
holding position — judged the worse outcome. `E-STOP` halts movement
quickly but leaves the drivers holding the arm exactly where it
stopped; `DISABLE DRIVERS (FREE-SPIN)` above is the separate control
for actually de-energizing the joints, and doing that with the arm
unsupported will let it fall.

**Xbox controller** (press LB until the controller is in ARM mode):
left stick → joints 1/2, right stick → joints 3/4, either trigger →
joint 5. This jogs continuously from the current position while the
stick is deflected, rather than setting an absolute target. The e-stop
and preset-position buttons above are web-GUI-only for now — there's
no controller shortcut for either yet.

**MoveIt2** (only if launched with `use_moveit:=true`): plan and
execute collision-aware motion by dragging an interactive marker in
RViz rather than commanding individual joints. See README's "Arm
motion planning" section for the full walkthrough — this is the one
piece of arm control this manual doesn't cover in detail, since it's
closer to a development tool than a routine operating control.

### Mast

**Web GUI, mast panel:** two sliders/readouts for head yaw (±170°) and
pitch (±180°) — absolute position, not a jog — `ERECT (SERVICE)` and
`STOW (TRANSPORT)` for the lift, `CLOSE DRIVER (ENABLE)` /
`OPEN DRIVER (DISABLE)` for the yaw/pitch drivers, plus `RETURN HOME`
(send the head to 0°/0°) and `TRANSPORT POSITION` (send it to
whatever's configured in `mast_topology.yaml`'s
`transport_head_yaw_deg`/`transport_head_pitch_deg` — not necessarily
the same as 0°/0°, and a real, bench-tunable placeholder until an
actual safe transport orientation is known). Return Home and Transport
Position deliberately don't touch the lift by themselves — sequencing
them with Stow is up to you (e.g. re-center the head first, so it
isn't wherever it last happened to be pointed while the lift lowers).

**Calibration itself now includes a drive back to center, not just
finding the limit switch.** Each axis seeks its switch (mounted at
that axis's extreme — -170° yaw, -180° pitch) the same way it always
has, but rather than calling that position "home," the mast now drives
from there back to 0°/0° first, *then* disables its own drivers.
That's what actually establishes home — the centered position, not the
switch itself. Once it's done, the yaw/pitch sliders won't do anything
until you press `CLOSE DRIVER (ENABLE)` — the telemetry panel's
`DRIVER` line shows
`ENABLED`/`DISABLED` so you can tell at a glance whether that's why
nothing's moving.

**The mast also runs a cooling fan automatically**, entirely against
its own `TEMP` reading — there's nothing to control here, just the
`FAN` telemetry line showing its current speed (or `OFF`). See
[Reading the telemetry](#5-reading-the-telemetry) below for how the
on/off thresholds work.

**Xbox controller** (LB until in MAST mode): right stick sets head
yaw/pitch directly (proportional to how far you push it, not a jog —
center the stick to return to center), **A** erects the mast to its
service position, **B** stows it back down. No button held = the lift
just holds wherever it currently is. There's no controller shortcut
for Return Home, Transport Position, or the driver enable toggle — use
the web GUI for those.

The mast's yaw/pitch axes calibrate automatically on startup, the same
way the arm's do — but unlike the arm, there's currently no per-axis,
on-demand re-calibration button for the mast specifically; calibration
only runs as a single all-axes sequence, automatically, at boot.

### Microscope

**Web GUI, microscope panel** (also at the standalone `/microscope`
page):

| Control | Effect |
|---|---|
| Focus/zoom slider | Direct position (motor steps) |
| LED brightness slider | Displayed as 0.00V–5.00V (still driven by plain PWM under the hood, 0-255 duty cycle — the display shows its volt-equivalent rather than the raw count, since there's no actual DAC or true analog output) |
| `LED ON` / `LED OFF` | Set the slider straight to full brightness (5.00V) or off (0.00V) — two separate buttons, not a toggle; the slider itself stays in sync either way it's driven |
| `CLOSE DRIVER (ENABLE)` / `OPEN DRIVER (DISABLE)` | Energize/de-energize the focus stepper — starts disabled at boot |
| `OPEN COVER` / `CLOSE COVER` | Two separate buttons (not one toggle that relabels itself) — click either any time; the cover's actual current state is shown in the telemetry panel's own `COVER` field |
| `TAKE SNAPSHOT` | Save the current camera frame to disk |
| `START RECORDING` / `STOP RECORDING` | Toggle video recording |
| `RECORD 1`/`2`/`3` + `GO TO 1`/`2`/`3` | Save the current focus position as a preset, or jump back to a saved one |

**The LED and cover no longer turn themselves off/closed if the
connection drops.** This used to happen automatically within about a
second of losing the link — it doesn't anymore, at the user's own
request. If you leave the LED on or the cover open and then lose
connection to the rover, they'll stay exactly as you left them,
indefinitely, until you reconnect and change them yourself. Worth
remembering before walking away mid-session.

The focus driver has to be explicitly enabled before the slider does
anything — it starts disabled every boot, on purpose, so nothing
drives against a mechanical stop before you're actually looking at the
live image. Presets are a browser-only convenience — they reset if you
reload the page, they're not saved anywhere permanent.

**Xbox controller** (LB until in MICROSCOPE mode): left stick Y jogs
focus/zoom, right trigger sets LED brightness, **A** toggles the lens
cover. There's no controller shortcut for enabling the focus driver,
snapshot, recording, or presets — use the web GUI for those.

### Antenna

The high-gain-antenna gimbal — azimuth (G1) and elevation (G2) — for
pointing the antenna disk. Modeled on the real Mars Exploration Rover
HGA gimbal; see README's "Antenna gimbal" section if you want the full
story behind the range numbers.

**Web GUI, antenna panel:** two sliders — azimuth (15°–285°) and
elevation (0°–180°), real operational limits, not arbitrary round
numbers — plus `CLOSE DRIVER (ENABLE)` / `OPEN DRIVER (DISABLE)`.
Unlike the microscope's driver toggle, the antenna's drivers actually
start *enabled* (homing needs them energized) — what you're really
waiting on after homing is `homed` going true, not the enable state
itself; the sliders simply won't do anything until both are ready.

**Xbox controller** (LB until in ANTENNA mode): left stick jogs
azimuth (X) and elevation (Y) continuously from the current position,
the same jog-style control the arm and microscope use — not an
absolute-position mapping the way the mast's right stick works. That's
deliberate: the antenna's operational range isn't centered around 0°
the way the mast's is, so an absolute mapping would feel much less
natural here.

### Navigation

Building a map and driving autonomously against it are both real
capabilities here, but they're closer to development/field-setup
tasks than routine daily operation, so this manual keeps it brief —
see README's "Navigation" section for the complete picture.

**Building a map:**
```bash
ros2 launch rover_bringup bringup.launch.py use_slam:=true
# drive the rover around manually (Xbox or web GUI) until the map looks complete
ros2 run nav2_map_server map_saver_cli -f ~/mars_rover_ws/src/rover_navigation/maps/my_map
```

**Driving autonomously against a saved map:**
```bash
ros2 launch rover_bringup bringup.launch.py use_navigation:=true nav_map:=/path/to/my_map.yaml
```
From there, autonomous goals are normally sent through RViz's "2D Goal
Pose" tool, or for a GPS coordinate specifically:
```bash
ros2 run rover_navigation gps_goal.py <latitude> <longitude>
```

---

## 5. Reading the telemetry

The web GUI's right-hand panel shows live status for every subsystem.
A few things worth knowing about what you're looking at:

- **Board status lamps, three states**: green means connected and
  receiving valid frames; **amber means the serial link is open but
  producing bad/unrecognized data** (the lamp's own meta text shows a
  "bad" count alongside the receive count when this is happening) —
  worth treating as a real problem, not a lesser version of green,
  since it usually means the port opened successfully but is talking
  to the wrong thing entirely (see `docs/INSTALL.md`'s troubleshooting
  table for the most common cause); red means the serial link is down
  or erroring outright. Check `docs/INSTALL.md`'s troubleshooting
  table if one won't go plain green.
- **`SUPPLY`** on arm/mast/antenna panels is the main battery
  voltage, read from each board's own voltage sensor — a real,
  independent reading per board, not one shared value. Worth a glance
  before a long session. **The base panel shows two separate
  readings instead of one**: **`DRIVE SUPPLY`** and **`STEER
  SUPPLY`**, from two independent voltage sensors on that board -
  each covers only its own rail (drive motors, steering servos), so
  the two can genuinely disagree if one rail sees more voltage drop
  under load than the other; reading the same value on both is
  expected, not a sign one sensor is redundant, if both rails happen
  to share a single battery in your build. The power panel's own
  **`BATTERY 1`**/
  **`BATTERY 2`** readings are related but distinct — see that
  panel's own bullet below.
- **`TEMP`** on base/arm/mast/antenna/microscope panels is that
  board's own enclosure temperature from a DS18B20, not ambient air —
  it's whatever's near wherever the sensor is actually mounted.
  Reads **`N/A`** if that board's sensor didn't respond on its most
  recent read — checked fresh continuously, not just once at boot, so
  a temporarily disconnected sensor recovers on its own once
  reconnected. Not a fault in normal operation, since none of these
  boards' actual jobs depend on it, but worth checking `docs/INSTALL.md`'s
  troubleshooting table if you're specifically trying to get it
  working. The power panel's own **`COMPUTER TEMP`** is a separate
  reading of a different thing entirely — see below.
- **`FAN`** on base/arm/mast/antenna/microscope panels shows that
  board's own cooling fan's current speed as a percentage, or
  **`OFF`** at 0% — entirely automatic, driven by that same panel's
  own `TEMP` reading, with no manual control anywhere in this manual
  because there isn't one to give, on any of these five panels or the
  power panel's own fan. Don't expect it to track ambient conditions
  moment-to-moment: it only turns on above a threshold, ramps with
  temperature above that, and turns back off at a *lower* threshold
  than it turned on at (intentional hysteresis, avoiding rapid on/off
  cycling right at one boundary) — a fan reading `0%` on a board that
  doesn't feel especially warm is expected, not evidence of a fault.
- **The power panel** is read-only, like every field on it — there's
  nothing to click or command here, only telemetry. **`BATTERY 1`**/
  **`BATTERY 2`** each show that battery's own voltage *and* current
  together (e.g. "24.20V / 4.50A") — independent readings from that
  battery's own sensor, not a shared value between the two.
  **`COMPUTER TEMP`** and this panel's own **`FAN`** work exactly
  like every other panel's `TEMP`/`FAN` pair above, just monitoring
  and cooling the onboard computer
  specifically rather than a board enclosure.
- **`HOMED`** (arm) and the per-joint ✓/✗ marks: whether calibration
  has completed. Movement commands are silently ignored while this
  reads false — that's expected, see [Arm](#arm).
- Position readouts (arm joint positions, mast yaw/pitch, microscope
  focus) are in raw motor steps or decidegrees, not real-world units —
  useful for confirming something is actually moving in response to a
  command, less useful as an absolute physical measurement without
  the calibration constants in `README.md`.

---

## 6. Troubleshooting

This is the short, operator-facing version — for real debugging depth
(wiring, firmware internals, protocol-level issues), `docs/INSTALL.md`
has a much longer troubleshooting table.

| Symptom | Likely cause | What to do |
|---|---|---|
| Nothing responds to the controller | RB not held, or controller in the wrong subsystem mode | Confirm RB is held continuously; press LB to check/cycle mode |
| Arm/mast won't move | Not calibrated yet | Check `HOMED`/per-joint ✓ in telemetry; wait for or re-trigger calibration |
| A board status lamp is red | Serial link down | Check the physical USB connection; see INSTALL.md's troubleshooting table |
| Rover drives but steering seems off | Servo calibration not bench-tuned for your actual hardware | See README's "Explicit assumptions" — steering calibration is a real, expected tuning step, not necessarily a bug |
| Microscope slider does nothing | Focus driver not enabled | Web GUI: `CLOSE DRIVER (ENABLE)` |
| Antenna sliders do nothing | Not homed yet, or drivers explicitly disabled | Check `HOMED`/`DRIVER` in antenna telemetry; wait for homing or press `CLOSE DRIVER (ENABLE)` |
| Mast yaw/pitch sliders do nothing, but lift buttons work fine | Head drivers not enabled - likely just finished its calibration sequence, which disables them on purpose | Check the `DRIVER` line in mast telemetry; Web GUI: `CLOSE DRIVER (ENABLE)` |
| Everything stopped and won't restart | STOP mode is latched | Press **X** on the controller, or the `ACKERMANN`/`POINT-TURN` button in the web GUI |

If none of this covers it, the underlying system is almost certainly
still running fine — `docs/INSTALL.md`'s troubleshooting table and
`README.md`'s "Known gaps" section are the next places to look before
assuming something's broken.

---

## 7. Quick reference

**Launch:**
```bash
ros2 launch rover_bringup bringup.launch.py                          # everyday driving
ros2 launch rover_bringup bringup.launch.py use_slam:=true            # build a map
ros2 launch rover_bringup bringup.launch.py use_navigation:=true nav_map:=<path>   # autonomous
ros2 launch rover_bringup bringup.launch.py use_moveit:=true          # arm motion planning
```

**Web GUI:** `http://<rover-host>:8080/` (dashboard), `/microscope`
(camera-focused view).

**Xbox controller, every mode:** hold **RB** (deadman switch), **LB**
cycles DRIVE → ARM → MAST → MICROSCOPE → ANTENNA → DRIVE.

| Mode | Left stick | Right stick | Triggers | A | B | X | Y |
|---|---|---|---|---|---|---|---|
| DRIVE | throttle / turn | — | — | — | — | toggle ACKERMANN/POINT-TURN | force STOP |
| ARM | joints 1/2 | joints 3/4 | joint 5 | — | — | — | — |
| MAST | — | yaw / pitch | — | erect | stow | — | — |
| MICROSCOPE | focus/zoom | — | LED (right only) | toggle cover | — | — | — |
| ANTENNA | azimuth/elevation | — | — | — | — | — | — |
