# Installation & Configuration Guide

Step-by-step setup for the rover workspace, from a bare Ubuntu 22.04.5
install through a full `ros2 launch rover_bringup bringup.launch.py`.
Written as a checklist — work through it in order the first time;
after that, jump to whichever section you need.

Companion docs: `README.md` (architecture/reference), `docs/journal.md`
(build history and decisions).

---

## Quick reference

For anyone who's done this before and just needs the commands:

```bash
# ROS 2 + build tooling
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc

# Workspace
cd ~ && unzip mars_rover_ws.zip && cd mars_rover_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install --break-system-packages pynmea2 fastapi uvicorn   # if rosdep can't resolve these
sudo usermod -aG dialout,video $USER   # then log out/in
colcon build --symlink-install
source install/setup.bash

# Test suite (no hardware/ROS needed)
PYTHONPATH=src/rover_protocol python3 -m pytest src/*/test tools/test -q

# Sensor fusion (always needed - runs as part of every normal bringup)
sudo apt install ros-humble-robot-localization ros-humble-geographic-msgs

# Optional: SLAM/Nav2 (only if you'll use use_slam:=true / use_navigation:=true)
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup

# Optional: MoveIt2 arm planning (only if you'll use use_moveit:=true)
sudo apt install ros-humble-moveit ros-humble-moveit-configs-utils

# Bringup
ros2 launch rover_bringup bringup.launch.py
```

Everything below explains each of these, plus firmware flashing, udev
rules, and per-subsystem verification, which don't compress into a
snippet.

---

## 1. Host prerequisites

**1.1 Confirm the OS**

```bash
lsb_release -a
```
Expect `Ubuntu 22.04.5 LTS` (or close — any 22.04 point release is fine).

**1.2 Install ROS 2 Humble**, if it isn't already:

```bash
locale  # confirm UTF-8; if not, follow ROS 2's locale setup first
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep
```

**1.3 Initialize rosdep and source ROS** (every new shell needs this —
the last line makes it automatic):

```bash
sudo rosdep init   # "already exists" is fine if you've done this before
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify:**
```bash
printenv ROS_DISTRO   # expect: humble
ros2 pkg list | head  # should print a long list without errors
```

If you skip 1.3, every `colcon build` fails with `Could not find a
package configuration file provided by "ament_cmake"` — that's the
single most common first-time mistake with this workspace.

---

## 2. Get the workspace

```bash
cd ~
unzip mars_rover_ws.zip     # produces ~/mars_rover_ws
cd ~/mars_rover_ws
ls src                       # should list all 15 rover_* packages
```

### Replacing an existing workspace with a newly-delivered zip

Not a fresh install — this is for when you already have a
`~/mars_rover_ws` from an earlier session and a new zip supersedes it
entirely. **Fully delete the old directory before extracting the new
one — don't extract on top of it.** Package contents get renamed,
merged, and removed between sessions (an old board's own directory
getting renamed is a real, recent example — `power_nano6/` became
`power_uno6/` in one past session), and extracting a new zip over an
old, un-deleted directory leaves both the new files *and* every
now-orphaned old one sitting side by side. That's not a cosmetic
problem — a stale, no-longer-referenced package or interface
definition left alongside its replacement is exactly the kind of thing
that produces confusing, hard-to-attribute build or runtime behavior
later, with no obvious cause in the current source tree to point at.

```bash
# 1. Stop everything currently running first (Ctrl+C any active
#    ros2 launch/ros2 run processes) - not strictly required to delete
#    files out from under a running process on Linux, but avoids any
#    confusion about which version of anything is actually active
#    while you're doing this.

# 2. Fully remove the old workspace - not an overlay, a replacement.
#    Back it up first only if you have local, uncommitted changes of
#    your own inside it worth keeping (this project's own iteration
#    doesn't preserve anything you edited by hand outside of what
#    was actually asked for).
rm -rf ~/mars_rover_ws

# 3. Extract the new zip fresh, same as a first-time install.
cd ~
unzip mars_rover_ws.zip
cd ~/mars_rover_ws

# 4. Full clean rebuild - not an incremental one. build/, install/,
#    and log/ are generated FROM src/, not part of the zip itself; a
#    partial or incremental build against a fully-replaced src/ tree
#    risks mixing artifacts compiled from the old source with the new
#    one. This is the same command this doc's own troubleshooting
#    table already gives for stale build dirs (below) - applied here
#    proactively, not just as a reaction to a specific error.
rm -rf build install log
colcon build --symlink-install

# 5. Re-source in every terminal that needs this workspace - a
#    terminal that sourced the OLD install/setup.bash before step 2
#    still has stale environment variables pointing at paths that no
#    longer exist; re-sourcing (or just opening a fresh terminal)
#    after the rebuild is what actually picks up the new one.
source install/setup.bash
```

If `rosdep`/`pip3` dependencies changed between the old and new zip
(a new package added, e.g.), re-run 3.1/3.2 below too before the
build step - a clean `src/` tree doesn't retroactively install
anything this workspace's own packages now depend on that wasn't
needed before.

---

## 3. Install dependencies

**3.1 rosdep** (pulls in rclpy, sensor_msgs, joy, pyserial, OpenCV
bindings, etc. from the package.xml files):

```bash
cd ~/mars_rover_ws
rosdep install --from-paths src --ignore-src -r -y
```

**3.2 Known rosdep gaps** — a few Python deps aren't reliably in every
Ubuntu 22.04 rosdep database. If `rosdep install` reports it can't
resolve any of these, install them directly:

```bash
pip3 install --break-system-packages pynmea2 fastapi uvicorn
```

(`--break-system-packages` is needed on Ubuntu 22.04's system Python;
omit it if you're inside a virtualenv instead.)

**3.3 External ROS packages not vendored in this workspace** — clone
these into `src/` before building:

```bash
cd ~/mars_rover_ws/src

# RPLIDAR C1 driver (pick ONE; rover_bringup's launch file expects
# rplidar_ros by default — see step 3.4 if you use sllidar_ros2 instead)
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# Xbox 360 controller raw input
sudo apt install ros-humble-joy

cd ~/mars_rover_ws
rosdep install --from-paths src --ignore-src -r -y   # re-run for the new package
```

**3.3b Sensor fusion packages** — needed unconditionally; sensor
fusion (wheel odometry + IMU, plus GPS conversion services) runs as
part of every normal bringup, not just when using SLAM/Nav2:
```bash
sudo apt install ros-humble-robot-localization ros-humble-geographic-msgs
```

**3.3c SLAM/Nav2 packages** — only needed if you actually plan to use
`use_slam:=true` or `use_navigation:=true` (see README's "Navigation"
section); skip this if you just want teleop/manual driving for now.
Unlike RPLIDAR/joy above, these install cleanly via `apt` — no need to
clone anything:
```bash
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
```

**3.3d MoveIt2 packages** — only needed if you plan to use
`use_moveit:=true` (see README's "Arm motion planning" section):
```bash
sudo apt install ros-humble-moveit ros-humble-moveit-configs-utils
```

**3.4 If you use `sllidar_ros2` instead of `rplidar_ros`:** open
`src/rover_bringup/launch/bringup.launch.py` and swap the package name
`rplidar_ros` → `sllidar_ros2` and the launch filename
`rplidar_c1_launch.py` → `sllidar_c1_launch.py` in the LIDAR
`IncludeLaunchDescription` block near the bottom of the file, and the
matching `<exec_depend>` in `src/rover_bringup/package.xml`.

**3.5 User permissions** — your user needs to be in `dialout` (serial
port access for the six Arduino boards) and `video` (USB cameras)
groups, or every serial/camera open will fail with a permissions error:

```bash
sudo usermod -aG dialout,video $USER
```
**Log out and back in** (group membership doesn't apply to your
current session otherwise) before continuing.

---

## 4. Build

```bash
cd ~/mars_rover_ws
colcon build --symlink-install
source install/setup.bash
echo "source ~/mars_rover_ws/install/setup.bash" >> ~/.bashrc
```

**Verify:**
```bash
ros2 pkg list | grep rover   # should list all 11 rover_* packages
```

### Troubleshooting the build

| Symptom | Cause | Fix |
|---|---|---|
| `Could not find a package configuration file provided by "ament_cmake"` | ROS 2 not sourced in this shell | `source /opt/ros/humble/setup.bash`, then rebuild |
| `package 'rover_X' found ... but libexec directory ... does not exist` | Stale `build`/`install` dirs from an earlier failed build | `rm -rf build install log && colcon build --symlink-install` |
| `rosidl_generate_interfaces` errors in `rover_msgs` | `std_msgs`/`builtin_interfaces` not resolved | `rosdep install --from-paths src --ignore-src -r -y`, rebuild |
| `ModuleNotFoundError: pynmea2` / `fastapi` / `uvicorn` at runtime | Step 3.2 skipped | `pip3 install --break-system-packages pynmea2 fastapi uvicorn` |
| `PermissionError` opening `/dev/ttyACM*` or `/dev/video*` | Not in `dialout`/`video` group, or didn't re-login | Step 3.5, then log out/in |
| `Package 'rover_X' not found: searching: ['/opt/ros/humble']` | Workspace `install/` never sourced in this shell (search list only has one entry), or that package never actually built | `ls install/ \| grep rover_X` to check it built; `source install/setup.bash` and confirm with `echo $AMENT_PREFIX_PATH` (expect two colon-separated paths, not one) |

---

## 5. Run the test suite (no hardware needed)

Do this now, before touching any hardware — it validates the serial
protocol, kinematics, and sensor parsers in isolation:

```bash
cd ~/mars_rover_ws
PYTHONPATH=src/rover_protocol python3 -m pytest src/*/test tools/test -q
```
Expect `207 passed`. If anything fails here, hardware bring-up isn't
going to work either — fix this first.

---

## 6. Arduino firmware

**6.1 Install the Arduino IDE** (2.x recommended):
```bash
sudo snap install arduino
```
or download from arduino.cc. `arduino-cli` works too if you prefer
the command line; the steps below assume the IDE's GUI.

**6.2 Add board support**: Tools → Board → Boards Manager → search
"Arduino AVR Boards" → Install (covers both the Mega 2560 and Uno).

**6.3 Install required libraries**: Tools → Manage Libraries, install:
  - **AccelStepper** (by Mike McCauley) — used by `arm_mega2`,
    `mast_uno3`, `microscope_uno4`, `antenna_uno5`
  - **Adafruit PWM Servo Driver Library** (by Adafruit) — used by
    `base_mega1` for the PCA9685 driving its 4 steering servos over
    I2C. BSD licensed (confirmed against the library's own source
    headers — some third-party library indexes show "NOASSERTION" for
    it, a known quirk of automated license detection against a
    non-standard header format, not an actual absence of a license).
  - **Servo** (bundled with AVR board support, nothing separate to
    install) — used directly by `microscope_uno4` (lens cover) for
    smoothed movement instead of an instant jump to each new
    commanded position, via a small non-blocking custom ramp
    (`updateCoverEasing()`) rather than the ServoEasing library this
    used before. **ServoEasing is no longer required by any board in
    this project** — `base_mega1`'s steering moved off it several
    sessions ago (its PCA9685 doesn't support it — it's built for
    direct-pin, standard-Servo-library-style control, not an I2C PWM
    chip), and `microscope_uno4`'s cover moved off it too, at the
    user's own explicit request (not a technical necessity this time
    — ServoEasing would have kept working fine on a direct-pin servo
    like this one). Both boards reimplement the same smoothed-
    movement idea themselves instead (`base_mega1.ino`'s
    `updateSteerEasing()`, `microscope_uno4.ino`'s
    `updateCoverEasing()`) — a genuine, project-wide side effect
    worth noting: this project no longer has a GPL-3.0 dependency
    anywhere (ServoEasing was the only one; see README's "Explicit
    assumptions" for the fuller history).
  - **OneWire** (by Paul Stoffregen) and **DallasTemperature** (by
    Miles Burton) — used by `base_mega1`, `arm_mega2`, `mast_uno3`,
    `antenna_uno5`, `microscope_uno4`, and `power_uno6` (all six
    boards) for each board's own temperature sensor (a DS18B20).
    Install both from the
    Library Manager. **Different licenses**: OneWire is MIT,
    DallasTemperature is **LGPL-2.1** — see `base_mega1.ino`'s own
    header comment for the licensing consideration, not a detail to
    wave off just because the two temperature libraries are installed
    together.
    **Requires an external 4.7kΩ
    pull-up resistor** between the DS18B20's DQ (data) pin and VDD —
    a real component to physically add, not optional, and not
    achievable via the Arduino's internal pull-ups (too weak for
    reliable 1-Wire timing). See 6.8 below for wiring.
  - **INA226** (by Rob Tillaart) — used by `power_uno6` for its two
    battery voltage+current monitors. MIT licensed. **No separate
    library needed for the TCA9548A multiplexer these sit behind** —
    channel selection is a single I2C write of one byte, not worth a
    dependency for. See 6.8d below for the mux/address reasoning and
    a real, must-verify shunt-resistor caveat before trusting current
    readings.

**6.4 Install the shared RoverProtocol library** (all six sketches
depend on this):
```
Sketch → Include Library → Add .ZIP Library...
```
Zip up `firmware/common/RoverProtocol/` first, or simpler: just copy
the folder directly —
```bash
cp -r ~/mars_rover_ws/firmware/common/RoverProtocol ~/Arduino/libraries/
```
Restart the IDE afterward so it picks up the new library.

**6.5 Flash each board — one at a time, with only that board plugged
in** (avoids ambiguity about which `/dev/ttyACM*` is which until udev
rules are in place in step 8):

| # | Sketch | Board type | Notes |
|---|---|---|---|
| 1 | `firmware/base_mega1/base_mega1.ino` | Arduino Mega 2560 | |
| 2 | `firmware/arm_mega2/arm_mega2.ino` | Arduino Mega 2560 | Mixed drivers - J1-J3 TB6600 (independent enable pins each), J4/J5 A4988 (shared enable pin) - see the file's own header comment |
| 3 | `firmware/mast_uno3/mast_uno3.ino` | Arduino Uno | Tight pin budget - all 12 usable digital pins committed (D2-D13, none spare), see the file's own header comment |
| 4 | `firmware/microscope_uno4/microscope_uno4.ino` | Arduino Uno | |
| 5 | `firmware/antenna_uno5/antenna_uno5.ino` | Arduino Uno | Same EBA-17-M + TB6600 combination as the arm's joints - see the file's own header comment |

For each: open the `.ino`, Tools → Board → select the correct type,
Tools → Port → select the port that appeared when you plugged it in,
click Upload. Watch for "Done uploading" with no errors.

**6.6 Bench-test each board before wiring it into the full rover** —
open the IDE's Serial Monitor at **115200 baud**, line ending "Newline",
and send raw protocol frames by hand to confirm the board responds.
For example, to the base Mega (all-zero drive command, checksum
precomputed):
```
D,0,0,0,0,0,0,0,0,0,0*44
```
You should see an `E,...` encoder frame come back. If nothing comes
back, double check baud rate, wiring, and that the correct sketch was
flashed to that board.

**Wheels off the ground (or at least the rover on blocks) before any
of this** — a wheel spinning at speed on a bench can walk the whole
rover off the edge. For the base Mega specifically, this is also the
right way to isolate *which* motor or servo has a wiring problem
before anything is wired to the whole vehicle: every checksum below
was computed by the real `rover_protocol.framing.encode_frame`, not by
hand, so a failure here means the hardware, not a typo'd frame.

```
All stop / centered                     D,0,0,0,0,0,0,0,0,0,0*44
FL wheel forward 30%                    D,300,0,0,0,0,0,0,0,0,0*47
FL wheel reverse 30%                    D,-300,0,0,0,0,0,0,0,0,0*6A
FR wheel forward 30%                    D,0,300,0,0,0,0,0,0,0,0*47
ML wheel forward 30%                    D,0,0,300,0,0,0,0,0,0,0*47
MR wheel forward 30%                    D,0,0,0,300,0,0,0,0,0,0*47
RL wheel forward 30%                    D,0,0,0,0,300,0,0,0,0,0*47
RR wheel forward 30%                    D,0,0,0,0,0,300,0,0,0,0*47
All six wheels forward 30%              D,300,300,300,300,300,300,0,0,0,0*44
FL servo to +30 deg                     D,0,0,0,0,0,0,300,0,0,0*47
FR servo to +30 deg                     D,0,0,0,0,0,0,0,300,0,0*47
RL servo to +30 deg                     D,0,0,0,0,0,0,0,0,300,0*47
RR servo to +30 deg                     D,0,0,0,0,0,0,0,0,0,300*47
All 4 servos to +60 deg (max)           D,0,0,0,0,0,0,600,600,600,600*44
All 4 servos to -60 deg (max)           D,0,0,0,0,0,0,-600,-600,-600,-600*44
```

Send **all-stop** between every test — don't rely on the 500ms
watchdog timeout during active testing. Field order in every frame is
`w_fl,w_fr,w_ml,w_mr,w_rl,w_rr,s_fl,s_fr,s_rl,s_rr` (see
`rover_base/rover_base/base_protocol.py`).

If a wheel spins the wrong way, fix `setWheelThrottle()` in
`base_mega1.ino` (flip the `HIGH`/`LOW`) - **don't** rewire the screw
terminal, that just moves the ambiguity around rather than resolving
it. If a servo moves the wrong direction, or hits a mechanical stop
before reaching the commanded angle, adjust that corner's own entry in
`kServoMinUs[]` / `kServoMaxUs[]` / `kServoNeutralUs[]` in the firmware
(indexed FL, FR, RL, RR, same order as `kSteerServoPin`) and/or
`max_steer_deg` in `base_topology.yaml` rather than force it. All four
corners start at identical placeholder values - that's intentional,
not a bug, since real servos vary enough that a shared calibration is
only ever a starting point; bench-test and tune each corner's own
three values independently as you go, without needing to touch the
other three corners' entries.

**6.7 FZ0430 voltage sensors** — base, arm, mast, and antenna (not
the microscope Uno) all have at least one battery-voltage reading;
**base specifically has two**, not one - see the note at the end of
this section for why. Each FZ0430 is a passive 5:1 resistive divider
on a small breakout: two screw
terminals (`V+`/`V-`) for the battery voltage being measured, and
three pins (`+`/`-`/`S`) that go to the Arduino. On arm, mast, and
antenna, wire `S` to **A0** (see
`docs/diagrams/03_base_mega1_wiring.svg` /
`04_arm_mega2_wiring.svg` / `05_mast_uno3_wiring.svg` — the antenna's
own wiring diagram was never created, see the note in `README.md`'s
"Explicit assumptions" about diagrams), `-` to Arduino `GND`, and `+`
to Arduino `5V` if the module needs its own logic power (some don't —
check your specific board). Double-check `V+`/`V-` polarity before
connecting to a real battery — reversed polarity on a resistive
divider won't necessarily fail safely. Verified in
10.2/10.3/10.4/10.5b below, once each board is flashed and running.

**Base is the exception: two FZ0430s, not one.** The original unit
(`S` to **A0**, same wiring as above) is now explicitly the **drive**
motors' own supply rail. A second, identically-wired FZ0430 (`S` to
**A1**, otherwise the same three connections as the first) reports
the **steering** servos' own supply rail independently - each unit
wired to whichever battery/supply lead actually feeds that half of
the base board, not both to the same source. If both rails are
genuinely powered from the same single battery in your build, both
sensors will simply read the same voltage, which is expected, not a
wiring error - the point of two independent sensors is to catch the
case where they *don't* agree (a voltage drop specific to one rail
under load), not to assume they must differ.

**6.8 DS18B20 temperature sensors** — base, arm, mast, antenna, and
microscope (5 of the first 5 boards; the 6th, power_uno6, gets its
own dedicated sensor list in 6.8d below since it measures something
different - the onboard computer, not its own board enclosure), one
DS18B20 each,
TO-92 package, over 1-Wire rather than I2C or a single analog pin.
**Pinout, looking at the flat face with the three legs pointing down,
left to right: GND, DQ (data), VDD** — double-check this against your
specific sensor's datasheet before wiring; some manufacturers vary.
**VDD** to the Arduino's 5V pin (the DS18B20 tolerates 3.0-5.5V, so
3.3V also works if that's more convenient), **GND** to Arduino GND,
**DQ** to that board's own 1-Wire data pin, which differs per board
rather than following one shared convention: **arm** uses digital pin
**20** (the Mega's dedicated I2C SDA pin, repurposed since nothing
else on that board needs I2C); **base** uses **A4** instead of that
same pin 20, because base's own I2C bus is genuinely in use now (the
PCA9685 steering driver - see 6.8c below), so its DS18B20 moved to A4
(one of the pins steering itself vacated moving to the PCA9685) to
free pin 20 back up; **mast** and **antenna** both use **A4** (the
Uno's own default I2C SDA pin, similarly repurposed, unrelated to
base's own A4 choice above - just the same pin number reused for the
same underlying reason on a different board type); **microscope**
uses digital pin **11** (this board's own digital budget was
different enough - DRV8825 instead of ULN2003 - that 11 was
genuinely spare rather than needing to reclaim an I2C pin at all).
Unlike I2C, 1-Wire has no dedicated hardware pin requirement, so each
of these is a deliberate reuse of whatever pin was actually free on
that specific board, not one rule applied uniformly.

**A real, physical 4.7kΩ resistor is required between DQ and VDD** —
this is not optional, and the Arduino's internal `INPUT_PULLUP` will
not substitute for it (documented as too weak to hold reliable 1-Wire
bus timing). If you're testing on a breadboard, this is one more
component to have on hand beyond the sensor itself; breakout boards
that include the DS18B20 already mounted often have this resistor
built onto the PCB — check yours before adding a second one in series.

Unlike the earlier BMP280 version's one-time "found at boot" check,
this firmware checks for a responding sensor on every single read
cycle (DallasTemperature's own `DEVICE_DISCONNECTED_C`) — a DS18B20
that's unplugged and later reconnected recovers on its own next
cycle, no reset needed. A missing/failed sensor is non-fatal by
design either way — none of these boards' real jobs depend on
temperature telemetry — so a disconnected sensor just means the
temperature field stays at its `-9999` ("sensor not found") sentinel,
not a board that fails to boot.

**6.8b Mast cooling fan** — mast only, not the other three
temperature-sensor boards. A generic N-channel MOSFET driver module
(IRF520-style breakout, or equivalent) switches a 12V-class DC fan
based on the mast's own DS18B20 reading — entirely automatic, no
operator control, no field for it on `MastCommand`.

**This is a low-side switch, and the wiring is easy to get backwards**:
connect the fan's **positive** lead directly to the module's **V+**
(a.k.a. Vin) screw terminal, wired to your external fan supply (not
the Arduino) — this is permanently on, not switched. Connect the
fan's **negative** lead to the module's **V-**/**OUT** terminal — this
is what the MOSFET actually switches to ground. Wiring the fan the
other way round (positive to the switched terminal) won't damage
anything on most of these modules, but the fan will run permanently
regardless of what the firmware commands, defeating the entire point.

On the control side: **GND** to the Arduino's GND, **SIG** to
**A2** (`kFanPwmPin`) — this drives the MOSFET gate directly and is
the only signal connection actually required. **VCC** only powers the
module's own onboard status LED and can be left disconnected if you
don't care about that indicator.

**No hardware PWM pin was available for this** — the mast's entire
PWM budget (D3/D5/D6/D9/D10/D11) was already spent by the yaw/pitch/
lift functions, so A2 runs a software PWM instead
(`updateFanPwm()` in `mast_uno3.ino`, ~50Hz via `millis()`). Nothing
to configure or verify about this beyond the wiring above — it's
firmware-internal, the MOSFET module doesn't know or care whether the
signal it's receiving came from a hardware timer or a software loop.

**The thermostat thresholds are placeholders**
(`kFanOnTempDeciC`/`kFanOffTempDeciC`/`kFanMaxTempDeciC` in
`mast_uno3.ino`) until the real enclosure's thermal behavior is known
on the bench — see `README.md`'s "Explicit assumptions" section for
the full reasoning behind the current values and the hysteresis
design. If the fan never seems to turn on, that may simply mean the
enclosure hasn't reached the placeholder's 35°C threshold yet, not a
wiring fault — check `fan_duty_percent` and `board_temperature_decic`
together in `/rover_mast/state` before assuming something's broken.

**6.8c Base steering: PCA9685 servo driver** — base only. Reverses an
earlier explicit decision documented in `base_mega1.ino`'s own header
comment ("no PCA9685"); this is the one board where that's no longer
true.

**I2C wiring** (Mega, not configurable): **SDA** to pin **20**, **SCL**
to pin **21**. This is why the DS18B20 on this board specifically
lives on **A4** rather than pin 20 the way it briefly did in an
earlier session — pin 20 was reclaimed for genuine I2C once the
PCA9685 needed it, and A4 (one of the pins steering vacated moving to
the PCA9685) took its place.

**CORRECTED: the address is 0x40, the factory default, deliberately
left unjumpered** — an earlier version of this section said 0x70 and
told you to jumper for it; that was a real mistake, caught by a user's
own I2C scan turning up 0x70 on a board that hadn't been jumpered at
all. 0x70 isn't a normal, jumper-configurable slave address on this
chip - it's the PCA9685's built-in "LED All Call" address, which
every unit responds to at power-up regardless of A0-A5 jumper state
(the chip's own datasheet: "the default LED All Call I2C-bus
address... must not be used as a regular I2C-bus slave address since
this address is enabled at power-up"). This project only ever has one
PCA9685, so there's no reason to move it off 0x40 - **leave the
address jumpers unbridged entirely**, `kPca9685Address` in
`base_mega1.ino` matches the factory default directly, nothing to
solder for this specific step.

**Servo wiring**: connect each of the 4 steering servos to PCA9685
channels **0-3** (FL, FR, RL, RR, in that order — `kSteerChannel` in
firmware). Each channel has the same three pins as a direct-pin servo
connection (signal, V+, GND) — the module's own **V+** screw terminal
is a dedicated servo power rail, separate from its **VCC** logic pin.
Wire V+ to its own adequately-rated 5-6V supply (four 40kg-class
digital servos can draw well beyond what the Mega's onboard 5V
regulator safely delivers), sharing ground with the Arduino, never
from the Mega's own 5V pin — this dedicated rail is something the
earlier direct-pin version of this same steering setup never had and
had to call out as a gap to fill separately; the PCA9685 fills it
natively.

Bench-test once wired:
```bash
ros2 run rover_base base_bridge_node --ros-args --params-file src/rover_base/config/base_topology.yaml
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}" --once
```
Steering should glide to its new angle over about 0.4s
(`updateSteerEasing()`'s own ramp, not ServoEasing — that library
doesn't work with a PCA9685; see README's "Explicit assumptions" for
why), the same visible easing behavior as before, just produced a
different way. If nothing moves at all, use an I2C scanner sketch
first to confirm the PCA9685 actually responds at 0x40 before assuming
a firmware or servo-calibration problem — if a scan instead turns up
0x70, that's the chip's own built-in All Call address responding
(present regardless of jumpers, see above), not evidence the device
is at 0x70 - don't jumper toward it. A genuinely missing/unpowered
PCA9685 (VCC not connected, or a bad I2C connection) looks identical
to "servos not connected" from the ROS side either way.

**6.8d Power/environmental monitoring Uno #6** — this board's own
sensor suite (2x INA226 behind a TCA9548A mux, 1x DS18B20, 1x cooling
fan), covered here separately from 6.7/6.8 above rather than folded
into those sections, since this board's own use of each sensor
genuinely differs from every other board's: I2C voltage+current
sensing no other board has at all, and a DS18B20 measuring the
onboard computer rather than this board's own enclosure. **UPDATED**
from this board's original FZ0430 + ACS712 design - see README's
"Explicit assumptions" for the full reasoning behind the switch.
**UPDATED AGAIN**: this board itself was originally an Arduino Nano,
swapped for an Uno after repeated hardware trouble with the Nano
units in hand - see README's "Explicit assumptions" for that history
too. No pin numbers below changed as a result of that swap - the Nano
and Uno share every pin this board's own firmware actually uses.

**TCA9548A wiring — SDA/SCL to this board's own A4/A5 (its I2C pins),
VIN to 5V, GND to GND.** Address pins (A0/A1/A2 on the mux itself)
left unconnected for the default 0x70 address - nothing else on this
board's I2C bus needs a different one. **0x70 here is unrelated to
the PCA9685's own 0x70 on the base board** (see README for why that
coincidence is worth knowing about, not worth worrying about) - two
separate chips on two separate Arduinos' own independent I2C buses.

**INA226 x2 wiring** — each unit's VCC to 5V, GND to GND, SDA/SCL to
the TCA9548A's own **SD0/SC0** (battery 1) and **SD1/SC1** (battery
2) channel pins, not directly to this board. Each INA226's own
address pins (A0/A1) left unconnected for the shared default address
(**0x40**) - this is the entire point of routing both through the
mux rather than reconfiguring one unit's address pins: two identical,
unmodified breakouts, isolated onto separate channels instead. Each
INA226's IN+/IN- (or VBUS+/VBUS- depending on the specific breakout's
labeling) goes in series with that battery's own positive lead, not
across it - this is a series current-sensing connection, wired
incorrectly it either reads nothing or reads the wrong thing
entirely, not just inaccurately.

**READ THIS BEFORE TRUSTING ANY CURRENT READING**: the INA226 senses
current via the voltage drop across an external shunt resistor, and
its own shunt-voltage range tops out at ±81.9mV. Many generic INA226
breakouts ship with a fixed 0.1Ω onboard shunt - with that value, the
*maximum* current such a board could ever report before saturating is
81.9mV / 0.1Ω ≈ 0.82A, far below what this rover's batteries need to
measure. `kInaShuntOhms` in `power_uno6.ino` (0.002Ω) is a
placeholder for a higher-current-range shunt value, not a verified
fact about your specific boards. **Check what shunt resistor value
your actual breakout boards have — printed on the PCB near the shunt
component itself on most boards, or in the listing/datasheet if
not — before assuming current readings are meaningful at anything
beyond a rough level.** Voltage readings are unaffected by this
concern; only current is.

**DS18B20 (computer temperature) — same wiring as 6.8 above, on
digital pin 2.** The one genuinely different thing about this
board's copy of the sensor: physically mount it against or near the
onboard computer itself, not this board's own enclosure - the whole
point of this particular sensor on this particular board is what the
computer is doing thermally, not what this small monitoring board
itself is doing.

**Cooling fan (for the computer) — same MOSFET-module wiring as
6.8b/6.8c's own fans, on pin 3 (hardware PWM, this board has no pin
budget pressure at all so no software-PWM workaround is needed).**
Same low-side-switch wiring caveat as every other fan in this
project (fan's positive lead to the module's always-on V+, negative
lead to the switched V-/OUT terminal) - see 6.8b for the full
reasoning if this is the first fan section you're reading.

Bench-test once wired:
```bash
ros2 run rover_power power_bridge_node --ros-args --params-file src/rover_power/config/power_topology.yaml
ros2 topic echo /rover_power/state
```
Unlike every other board, there's no command to publish here first -
this board starts sending `/rover_power/state` on its own, roughly
every 200ms, as soon as it's powered and the bridge node is running.
`battery1_voltage_mv`/`battery2_voltage_mv` should each read close to
that battery's actual voltage on a multimeter; if either reads 0 or
implausible regardless of the real battery, use an I2C scanner first
to confirm both the TCA9548A (0x70) and, with each mux channel
selected in turn, that channel's INA226 (0x40) actually respond,
before assuming a firmware bug - a wiring mistake on the mux side
looks identical to a dead sensor from the ROS side.
`battery1_current_ma`/`battery2_current_ma` should read close to 0
with nothing drawing current from that battery - if a reading is
implausibly small regardless of real load, see the shunt-resistor
warning above before assuming a fault.
`computer_temperature_decic` reading exactly `-9999` means the
DS18B20 didn't respond on its most recent read - same troubleshooting
as 6.8 above. `fan_duty_percent` is entirely automatic against that
same temperature reading, so at room temperature it should read `0`
unless you're actively warming the DS18B20 by hand to test the
thermostat logic.

---

## 7. Sensor and camera setup

**7.1 BNO086 IMU** — confirm the PS0/PS1 protocol-select jumpers are
strapped for **UART-RVC** mode (not I2C/UART/SPI) — see the board's
silkscreen jumper table on the underside. Wire its UART pins to the
Waveshare "USB TO TTL (B)" converter, plug that into USB. No firmware
flashing needed for this device — it free-runs the instant it's powered.

**7.2 L76X GPS** — plug in via USB. No configuration needed at default
settings (9600 baud NMEA output).

**7.3 RPLIDAR C1** — plug in via USB.

**7.4 Cameras** — plug in the main perception camera and the
microscope's USB camera. Confirm both enumerate:
```bash
v4l2-ctl --list-devices    # sudo apt install v4l-utils if missing
```

---

## 8. udev rules — stable `/dev/rover/*` device names

Without this step, devices enumerate as `/dev/ttyACM0`,
`/dev/ttyACM1`, ... in whatever order they happened to power up, which
changes across reboots. Do this once, carefully — it's the step most
worth taking slowly.

**8.1 Identify every device and generate the rules file (recommended).**
```bash
cd ~/mars_rover_ws
python3 tools/identify_rover_devices.py
```
Prompts you to plug in each of the 9 devices (3 Megas, 1 Uno, IMU
adapter, GPS adapter, LIDAR, 2 cameras) one at a time, auto-detects
which new `/dev` entry just appeared, pulls its VID:PID/serial via
`udevadm`, and — after showing you the result and asking for
confirmation — writes a filled-in
`src/rover_bringup/config/udev/99-rover-serial.rules`. Devices can
stay plugged in cumulatively; you only need to unplug everything
before starting. It also automatically detects and handles the "three
identical Mega 2560 boards report the same or an empty serial number"
case described below, falling back to matching on physical USB port
path for just those boards rather than needing you to notice and fix
it by hand.

Needs `udevadm` (part of systemd, present on any Ubuntu install) and
must run **on the rover's own computer** with the real hardware
attached — it exits immediately with a clear message if `udevadm`
isn't found, e.g. if run somewhere else by mistake.

**Manual alternative**, if you'd rather do it by hand or the script
doesn't work for some reason — with **only one device plugged in at a
time**, run:
```bash
udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial|KERNELS' | head -8
```
(substitute `/dev/ttyACM0` for whatever it actually enumerated as, and
for cameras use `/dev/video0` and `SUBSYSTEM=="video4linux"` instead).
Repeat for all 9 devices, noting the values each time, then edit
`src/rover_bringup/config/udev/99-rover-serial.rules` yourself,
replacing every `REPLACE_WITH_...` placeholder with the real values.

**Important caveat for the three Mega 2560 boards** (handled
automatically by the script above, but worth understanding if you're
doing this by hand): many Mega units report an **empty or identical**
`ATTRS{serial}` across boards, since the ATmega16u2 USB-serial bridge
only has a real per-unit serial number if it was explicitly DFU-flashed
with one. If `grep serial` comes back empty or the same for all three,
use the `KERNELS=="..."` physical-USB-port-path value instead (the
shortest one shown) — see the comments in the rules file itself for
exactly how.

**8.2 Install and reload:**
```bash
sudo cp ~/mars_rover_ws/src/rover_bringup/config/udev/99-rover-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**8.3 Verify** — unplug/replug everything, then:
```bash
ls -l /dev/rover/
```
Expect to see `base`, `arm`, `mast`, `microscope`, `imu`, `gps`,
`lidar`, `main_cam`, `microscope_cam` all symlinked to the correct
underlying device. If one's missing, re-check that device's rule line
against `udevadm info` output — a single mismatched character in the
VID:PID or serial won't match.

---

## 9. Xbox 360 controller

**9.1 Connect** (USB cable or a wireless receiver dongle).

**9.2 Verify it's recognized:**
```bash
ls /dev/input/js*     # expect /dev/input/js0
sudo apt install joystick   # if jstest isn't already installed
jstest /dev/input/js0       # move sticks/press buttons, confirm values change
```

**9.3 Verify axis/button indices match the config.** Gamepad mappings
vary by driver — don't assume the defaults in
`src/rover_teleop/config/xbox_teleop.yaml` are correct for your exact
controller/OS combo without checking:
```bash
ros2 run joy joy_node &
ros2 topic echo /joy
```
Move each stick and press each button one at a time, note which array
index changes. If any differ from the yaml (`axis_left_x: 0`,
`button_lb: 4`, etc.), edit that file to match before relying on
teleop.

**9.4 Tuning steering sensitivity.** Steering angle isn't commanded
directly from the stick — it's a consequence of the commanded turn
rate (`angular_z`) and forward speed via the base's kinematics, so
"steering sensitivity" and "turn-rate sensitivity" are the same knob:

| Parameter | File | Default | Effect |
|---|---|---|---|
| `max_angular_radps` | `rover_teleop/config/drive_sensitivity.yaml` | 1.5 rad/s | Full stick/joystick deflection always commands this value; partial deflection scales linearly. Raise for snappier turns, lower for gentler. Applies to both ACKERMANN and POINT_TURN. |
| `deadzone` | same file | 0.12 | How much stick/joystick movement near center is ignored before anything is commanded. |
| `max_steer_deg` | `rover_base/config/base_topology.yaml` | 60.0° | Hard mechanical clamp matching the servos' real travel - not a sensitivity knob, just the ceiling the two above can't exceed. |

`drive_sensitivity.yaml` is a **single shared file**, loaded by both
the physical Xbox controller and the web GUI's on-screen joystick (via
ROS 2's `/**` wildcard node match — see the file itself) — editing it
once changes the feel of both input devices together, restart both to
pick it up. `xbox_teleop.yaml` still holds everything specific to the
Xbox controller itself (button/axis indices, arm/mast/microscope jog
speeds).

No true PID access on the steering servos themselves - they're
standard PWM hobby servos (`base_mega1.ino`'s `setSteerAngle()` computes
a target pulse width and hands it to `updateSteerEasing()`'s own
non-blocking ramp, which smooths the approach to that target but still
just ends up sending pulse widths to the PCA9685, same as writing one
directly would have); whatever position-control loop runs inside the
servo is factory-fixed and isn't exposed over that signal. The drive
motors are open-loop PWM too, for the same reason - the encoders feed
odometry, not a closed-loop speed controller.

---

## 10. First bring-up — one subsystem at a time

Do this before the full `bringup.launch.py` the first time, so a
problem in one subsystem doesn't get lost in a wall of log output from
all of them starting together.

**10.1 URDF sanity check (no hardware required):**
```bash
ros2 launch rover_description display.launch.py
```
RViz should open with the robot model visible; the joint_state_publisher_gui
sliders should move the arm/mast/wheels. You should also see 5 solar
panels and a small antenna on the chassis — both static (no sliders),
since there's no actual antenna/panel-actuation hardware in this
project yet. See README's "Explicit assumptions" for why.

**10.2 Base bridge:**
```bash
ros2 run rover_base base_bridge_node --ros-args --params-file src/rover_base/config/base_topology.yaml
```
This level tests whole-vehicle *kinematics* (`cmd_vel` commands the
vehicle, not one wheel) - use 6.6's raw frames instead if you need to
isolate a single wonky actuator.
```bash
ros2 topic echo /rover_base/board_status   # connected: true, rx_frame_count climbing

# ACKERMANN (default): forward, then a turn
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}" --once

# POINT_TURN: switch mode, then rotate in place
ros2 topic pub /rover_base/drive_mode rover_msgs/msg/DriveMode "{mode: 1}" --once
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --once

# STOP: switch mode - motion stops regardless of any Twist still arriving
ros2 topic pub /rover_base/drive_mode rover_msgs/msg/DriveMode "{mode: 2}" --once

ros2 topic echo /rover_base/state          # encoder ticks changing, drive_voltage_mv and steering_voltage_mv both non-zero and plausible, board_temperature_decic a plausible room-temperature-ish value
ros2 topic echo /rover_base/command_echo   # drive_mode + the actual wheel/steer numbers sent
```
`drive_voltage_mv` and `steering_voltage_mv` should each read close to
their own rail's actual battery voltage (×1000 for millivolts) — if
either reads near 0 or near 25000 regardless of the real battery,
check that specific FZ0430's own wiring (drive on A0, steering on A1;
see the base wiring diagram, `docs/diagrams/03_base_mega1_wiring.svg`,
though it predates the second sensor and only shows the original A0
unit) before assuming a firmware bug. Reading the same plausible
voltage on both is expected if both rails share one battery in your
build - the two sensors existing independently doesn't mean they must
disagree. `board_temperature_decic` reading exactly
`-9999` means the DS18B20 didn't respond on its most recent read —
see 6.8 above for the wiring/pull-up requirements before assuming a
firmware bug. Steering servos now glide to each new commanded angle
over about 0.4s (a custom non-blocking ramp in `updateSteerEasing()`,
not ServoEasing - see `README.md`'s "Explicit assumptions" for why
that library doesn't work with the PCA9685 these servos now connect
through) rather than snapping instantly — expected, not lag;
`kSteerEaseSpeedUsPerSec` in `base_mega1.ino` is the tuning knob if
that feels too slow or too fast once real servos are on the bench.

**10.3 Arm bridge** (homes automatically on startup — watch the log
for "sent homing request" then wait for `homed: true`):
```bash
ros2 run rover_arm arm_bridge_node --ros-args --params-file src/rover_arm/config/arm_topology.yaml
ros2 topic echo /rover_arm/state
```
Expect homing to take noticeably longer than it would on a low-ratio
gearbox — the EBA-17-M's 120:1 reduction means real joint speed is
only about 1.9°/s at the firmware's current motor-shaft speed cap (see
README's "Explicit assumptions"), so a joint starting far from its
limit switch can take tens of seconds to reach it. That's expected,
not a sign something's stuck — `homingInProgress` staying `true` for a
while is normal here; only worry if it's still not `homed: true` after
several minutes.

Same kind of FZ0430 voltage check as 10.2 applies here too, though
this board still has just one sensor, not base's own two
(`supply_voltage_mv`
in the state message) — same sensor, same wiring pattern, on this
board's own A0. Same DS18B20 temperature check too
(`board_temperature_decic`), on this board's own 1-Wire data pin (20,
same choice as the base board — this is a Mega).

Startup homing covers all 5 joints in one pass, but each can also be
re-homed individually afterward without disturbing the others:
```bash
ros2 service call /rover_arm/home_joint rover_msgs/srv/HomeJoint "{joint_index: 2}"
ros2 topic echo /rover_arm/state   # joint_homed[2] should go false, then true again once J3 re-triggers its switch
```
Each joint's own homing direction (`kHomingDirection`) and the order
joints home in during an all-5 run (`kHomingOrder`) are independently
configurable in `arm_mega2.ino` — currently PLACEHOLDER values
matching the previous behavior (uniform direction, sequential J1-J5)
pending real mechanical verification; see README's "Arm calibration"
section for the full reasoning behind each. If a joint seeks in the
wrong direction or never triggers its switch, check `kHomingDirection`
for that joint before assuming a wiring fault.

Once a joint's switch trips, it's assigned `kLowerLimitSteps[j]` (that
joint's own real, physical lower bound - PLACEHOLDER, currently
mirroring `kMinDeg[j]` converted to steps) and then drives to absolute
step 0, which this project now defines as that joint's own true
center - not an arbitrary reference point. Once real, distinct values
replace the current placeholders, watch `ros2 topic echo
/rover_arm/state`'s own `joint_position_steps` settle at 0 after this
second move completes, not at the switch's own trigger point - that's
the design working as intended, not a bug. Every joint command (and
preset move) is also clamped to `[kMinDeg, kMaxDeg]` in degrees
(`kStepsPerDegree` converts) before being accepted - bench-verify this
directly by commanding a target well outside a joint's own declared
range and confirming `joint_position_steps` settles at the clamped
bound, not the requested one:
```bash
ros2 topic pub --once /rover_arm/command rover_msgs/msg/ArmCommand "{joint_target_steps: [999999, 0, 0, 0, 0], enable: true}"
ros2 topic echo /rover_arm/state   # joint_position_steps[0] should settle at 160000 (J1's own kMaxDeg=150deg in steps), not 999999
```
`kStepsPerDegree`, `kMinDeg`/`kMaxDeg`, and `kLowerLimitSteps` are all
sourced from numbers this project had already established elsewhere
(`rover_arm/config/arm_topology.yaml`'s own `steps_per_joint_rev`, and
`rover_description/urdf/arm.xacro`'s own joint `<limit>` tags) rather
than freshly guessed here — see README's "Explicit assumptions" for
the full sourcing rationale and the manual-sync obligation this
creates across those files plus the web GUI's own slider bounds.

`ArmState`'s own `drivers_enabled` field is the firmware's actual,
current state, not an echo of what was last commanded — bench-verify
this distinction directly by triggering homing and watching it flip on
its own, with no `enable: true` sent from anywhere:
```bash
ros2 topic echo /rover_arm/state   # watch drivers_enabled
ros2 service call /rover_arm/home_joint rover_msgs/srv/HomeJoint "{joint_index: 0}"
```
`drivers_enabled` should go `true` the moment homing starts, before
any joint command with `enable: true` has been sent. The web GUI's own
arm panel has a single toggle button for this now (`ENABLE DRIVERS` /
`DISABLE DRIVERS (FREE-SPIN)`, replacing what used to be two separate
buttons) — its label and color both follow this same field on every
telemetry update, not a value the browser remembers clicking.

Once calibrated, three predefined poses are reachable directly:
```bash
ros2 service call /rover_arm/arm_preset rover_msgs/srv/ArmPreset "{preset: 1}"   # 0=initial, 1=transport, 2=service
ros2 topic echo /rover_arm/state   # joint_position_steps should move toward that preset's own kXPoseSteps in arm_mega2.ino
```
All three presets currently move to the same all-zero pose — real,
distinct poses for each are pending bench calibration in
`arm_mega2.ino`'s own firmware constants; don't expect the three
buttons to visibly differ yet. Rejected (`accepted: false`) if the arm
isn't fully homed, a homing run is in progress, or the emergency stop
below is currently latched.

The emergency stop can be tested independently of homing status —
it's accepted regardless of whether the arm is homed, mid-homing, or
already e-stopped (engaging it again while already latched is a
harmless no-op):
```bash
ros2 service call /rover_arm/emergency_stop rover_msgs/srv/EmergencyStop "{engage: true}"
ros2 topic echo /rover_arm/state   # estop_active should read true; joint_position_steps should stop changing shortly after (a controlled deceleration, not instant)
ros2 service call /rover_arm/emergency_stop rover_msgs/srv/EmergencyStop "{engage: false}"
```
While latched, regular joint commands and preset requests are both
silently ignored (not queued for later) — send a small joint move or a
preset request while `estop_active: true` and confirm
`joint_position_steps` genuinely doesn't change, then clear it and
confirm the same command now works, to bench-verify the gate is
actually wired correctly before relying on it for real. **The drivers
stay energized throughout an e-stop, on purpose** — see README's "Arm
emergency stop" section for the full safety reasoning before assuming
this is a bug; a real power-cutting e-stop would need different
firmware, not just a different comment.

The web GUI's arm panel has all of the above as buttons (`CALIBRATE
J1`-`J5`, `CALIBRATE ALL 5`, `INITIAL POSITION`, `TRANSPORT POSITION`,
`SERVICE POSITION`, `E-STOP`, `CLEAR E-STOP`) — see README's "Arm
calibration" and "Arm emergency stop" sections for the full picture.

**10.4 Mast bridge** (yaw/pitch now home automatically on startup —
same pattern as the arm, watch the log for "sent homing request to
mast Uno" then wait for `homed: true`; the lift is unaffected and
works immediately regardless of homing state):
```bash
ros2 run rover_mast mast_bridge_node --ros-args --params-file src/rover_mast/config/mast_topology.yaml
ros2 topic echo /rover_mast/state
```
Same FZ0430 voltage check applies here too, and the same DS18B20
temperature check (`board_temperature_decic`), on this board's own
1-Wire data pin (A4 - this is an Uno). `fan_duty_percent` in the same
state message is worth a glance too - entirely automatic against that
same temperature reading, so on a bench-top test at room temperature
it should read `0` (fan off) unless you're actively warming the
DS18B20 by hand to test the thermostat logic; see 6.8b for wiring and
the placeholder threshold values. Watch `yaw_limit_triggered`/
`pitch_limit_triggered` in the state message while homing runs — each
should flip briefly to `true` right as that axis reaches its switch,
then homing moves on to the next axis (or finishes, for pitch). Pitch's
real range is ±180° (confirmed against hardware - not the ±60° an
earlier placeholder assumed), so don't be surprised if that axis takes
noticeably longer to reach its switch than yaw does from a similar
starting position.

Once a given axis's limit switch triggers, its position is set to that
axis's minimum (-170° yaw, -180° pitch), not zero - the switch is
physically at that extreme, not center. Watch both axes then drive
from there back toward 0°/0° on their own, without any command from
you - this is still part of calibration, not a separate step, and
`homed: true` only appears once both axes actually arrive there and
the drivers disable themselves (watch `driver_enabled` stay `true`
through this move, then go `false` at the end). Yaw/pitch commands are
silently ignored the whole time, and won't do anything afterward
either until re-enabled:
```bash
ros2 topic pub -r 10 /rover_mast/command rover_msgs/msg/MastCommand "{head_yaw_decideg: 0, head_pitch_decideg: 0, lift_mode: 0, driver_enable: true}"
```

**10.5 Microscope (bridge + camera):**
```bash
ros2 launch rover_microscope microscope.launch.py
ros2 topic echo /rover_microscope/state
ros2 topic hz /rover_microscope/image/compressed   # confirm frames are flowing
```
The DRV8825 focus/zoom driver starts disabled (`driver_enabled: false`
in the state message) — nothing will move until a command explicitly
enables it (the web GUI's `CLOSE DRIVER (ENABLE)` button, or
`driver_enable: true` in a manual `MicroscopeCommand`). Before
powering this up for the first time: confirm the 24BYJ-48's
center-tap wire is genuinely disconnected, not just unused — see
`firmware/microscope_uno4/microscope_uno4.ino`'s header comment for
why this matters (a real short risk, not just a "won't work" mistake).
The lens cover now glides open/closed over about 1 second
(a custom non-blocking ramp, `updateCoverEasing()` - no longer
ServoEasing, see README's "Explicit assumptions" for why) rather than
snapping instantly — `kCoverEaseSpeedDegPerSec`
in that same file is the tuning knob. Open and close are now two
separate buttons in the web GUI (`OPEN COVER`/`CLOSE COVER`), not one
toggle — click each to bench-test both directions independently.

**10.5b Antenna gimbal** (azimuth/elevation home automatically on
startup — same pattern as the mast, watch the log for "sent homing
request to antenna gimbal Uno" then wait for `homed: true`):
```bash
ros2 run rover_antenna antenna_bridge_node --ros-args --params-file src/rover_antenna/config/antenna_topology.yaml
ros2 topic echo /rover_antenna/state
```
Same FZ0430 voltage check as 10.2 applies here too, and the same
DS18B20 temperature check (`board_temperature_decic`), on this board's
own 1-Wire data pin (A4 - this is an Uno). Unlike the mast,
homing here is a single pass with no follow-on move-to-center step —
each calibration switch is assumed mounted at that axis's own
operational minimum (15° azimuth, 0° elevation), so `homed: true`
appears as soon as both switches are found, not after a further
move. `driver_enabled` starts `true` (homing needs the drivers
energized) but azimuth/elevation commands are ignored until `homed`
is `true` — same reasoning as the mast's own gating (see
`rover_antenna/rover_antenna/antenna_protocol.py`): the bridge sends
command frames continuously at its own control rate once past its
one-shot homing request, and applying a stale/default command
unconditionally during homing would strand a stepper mid-seek.

**10.6 Sensors (IMU, GPS, main camera):**
```bash
ros2 launch rover_sensors sensors.launch.py
ros2 topic echo /rover_sensors/imu/data
ros2 topic echo /rover_sensors/gps/fix
ros2 topic hz /rover_sensors/main_camera/image/compressed
```
GPS needs a sky view to get a fix — `fix.status.status` will read
`-1` (no fix) indoors, which is expected, not a bug.

**10.7 LIDAR:**
```bash
ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/rover/lidar
ros2 topic hz /scan
```

**10.8 Teleop:**
```bash
# Terminal 1 (if not already running from 10.2)
ros2 run rover_base base_bridge_node --ros-args --params-file src/rover_base/config/base_topology.yaml

# Terminal 2
ros2 launch rover_teleop xbox_teleop.launch.py
ros2 topic echo /rover_teleop/mode   # should print DRIVE
```

Everything below needs the deadman held - nothing moves without it:

| Do this | Expect |
|---|---|
| Hold **RB**, push left stick forward/back | Drives forward/back (ACKERMANN) |
| Hold **RB**, push left stick left/right | Turns |
| Release **RB** | Immediate stop, in any mode |
| Tap **X** | Toggles ACKERMANN ↔ POINT-TURN |
| In POINT-TURN, hold **RB** + push stick left/right | Rotates in place (forward/back on the stick does nothing here - expected, see README) |
| Tap **Y** | Forces STOP immediately |
| Tap **X** after STOP | Resumes at ACKERMANN (Y is deliberately one-way, not a toggle) |

Watch a second/third terminal to confirm the controller and the
bridge agree with each other:
```bash
ros2 topic echo /rover_teleop/mode                  # DRIVE/ARM/MAST/MICROSCOPE/ANTENNA (subsystem)
ros2 topic echo /rover_teleop/drive_geometry_mode   # ACKERMANN/POINT_TURN/STOP, as teleop sees it
ros2 topic echo /rover_base/command_echo            # ground truth: what's actually being sent to the Mega
```
Or skip the terminals entirely and use the web GUI instead (`ros2
launch rover_web_gui web_gui.launch.py`, then browse to
`http://<rover-host>:8080/`) - the DRIVE panel's mode buttons and
board-status lamps show all of this live.

**10.9 Web GUI:**
```bash
ros2 launch rover_web_gui web_gui.launch.py
```
Open `http://<rover-host>:8080/` in a browser. Board status lamps
should light up as the other subsystems come online; the microscope
tab link opens `/microscope` in a new tab. The arm panel's
`CALIBRATE J1`-`J5` / `CALIBRATE ALL 5` / `INITIAL POSITION` /
`TRANSPORT POSITION` / `SERVICE POSITION` / `E-STOP` / `CLEAR E-STOP`
buttons need
`rover_arm_bridge` running (10.3) — the status line under them shows
each request's result message. The mast panel's `RETURN HOME` /
`TRANSPORT POSITION` buttons work the same way against
`rover_mast_bridge` (10.4) - if `TRANSPORT POSITION` sends the head to
somewhere unexpected, check `rover_mast/config/mast_topology.yaml`'s
`transport_head_yaw_deg`/`transport_head_pitch_deg` values rather than
assume it's a bug; they're placeholders (0.0/0.0) until bench-tuned.
The antenna panel's azimuth/elevation sliders need `rover_antenna_bridge`
running (10.5b) - like the microscope, its `CLOSE DRIVER (ENABLE)`
button needs pressing before the sliders do anything, though for a
different reason (the antenna's own homing sequence disables nothing
automatically the way the mast's used to - it simply never enables
manual commands until `homed` is true).

**10.10 Wheel odometry + sensor fusion** (runs automatically as part
of the full bringup, but worth checking in isolation first). Standing
alone, `odometry_node` still publishes on `/odom` directly — the
`wheel_odom` remap and EKF fusion only happen when launched through
`bringup.launch.py` or `localization.launch.py`:
```bash
ros2 run rover_base odometry_node --ros-args --params-file src/rover_base/config/base_topology.yaml
ros2 topic echo /odom
```
Drive forward (Level 2/3 of the motor-testing walkthrough above) and
confirm `pose.pose.position.x` increases; turn in place (POINT_TURN)
and confirm `pose.pose.orientation` changes while position stays put.

**10.10b Full fusion pipeline** (wheel odometry + IMU via the local
EKF, plus GPS conversion services):
```bash
ros2 launch rover_navigation localization.launch.py
ros2 topic echo /odom          # now the EKF's fused output, not raw wheel_odom
ros2 topic echo /odometry/gps  # only publishes once the GPS has a fix - try this outdoors
```
Compare `/odom`'s motion against the raw `/wheel_odom` topic while
turning — the fused estimate should track it closely but noticeably
smoother, especially right after a turn. If `/fromLL` and `/toLL`
don't show up in `ros2 service list`, `navsat_transform_node` isn't
running — check its terminal output for GPS fix errors.

**10.11 SLAM and Navigation** — needs the LIDAR (10.7) and fusion
(10.10b) both working first:
```bash
# Mapping: drive the rover around, then save the map
ros2 launch rover_bringup bringup.launch.py use_slam:=true
ros2 run nav2_map_server map_saver_cli -f ~/mars_rover_ws/src/rover_navigation/maps/my_map

# Navigation: autonomous driving against that saved map
ros2 launch rover_bringup bringup.launch.py use_navigation:=true \
    nav_map:=$HOME/mars_rover_ws/src/rover_navigation/maps/my_map.yaml
```
Open `rover_navigation/rviz/navigation.rviz` in RViz to watch the map/
costmaps build or to send navigation goals ("2D Pose Estimate" to
localize, "Nav2 Goal" to send a destination). Outdoors, with a GPS fix,
you can also send a goal by coordinate instead:
```bash
ros2 run rover_navigation gps_goal.py 42.4373 -86.9436
```
Full walkthrough and architecture are in the README's
"Navigation (SLAM + Nav2 + sensor fusion)" section.

**10.12 MoveIt2 arm planning** — needs the arm bridge (10.3) already
homed (`ArmState.homed: true`) first; goals are rejected otherwise:
```bash
ros2 launch rover_bringup bringup.launch.py use_moveit:=true
ros2 launch rover_arm_moveit_config moveit_rviz.launch.py
```
In RViz's MotionPlanning panel, drag the interactive marker at the
arm's tip, **Plan**, then **Execute**. Watch `ros2 topic echo
/rover_arm/command` in a separate terminal while it executes — you
should see `joint_target_steps` update through a sequence of
intermediate values as the trajectory plays out, not jump straight
from start to goal. If `Execute` does nothing, check
`ros2 action list` for `/arm_controller/follow_joint_trajectory` — if
it's missing, `trajectory_action_server` isn't running (needs
`use_moveit:=true`, not just `move_group` on its own).

---

## 11. Full bring-up

Once every subsystem above checks out individually:
```bash
ros2 launch rover_bringup bringup.launch.py
```
Useful flags for bench-testing without full hardware:
```bash
ros2 launch rover_bringup bringup.launch.py use_teleop:=false use_lidar:=false
```
(`use_web_gui:=false` is also available. `use_slam:=true` or
`use_navigation:=true nav_map:=...` add SLAM/Nav2 on top - see 10.11
above; don't set both at once. `use_moveit:=true` adds arm motion
planning - see 10.12 - independent of SLAM/navigation, combine freely.)

---

## 12. Troubleshooting quick-reference

| Symptom | Likely cause | Where to look |
|---|---|---|
| `board_status.connected: false` for one board | Wrong `/dev/rover/*` symlink, or that board's firmware wasn't flashed | Step 6 (firmware), Step 8 (udev) |
| `board_status.connected: true` but `rx_frame_count` stuck at 0 (or a board-status lamp shows amber/"warn" instead of plain green) | Either (a) `/dev/rover/X` resolved to the *wrong* physically-identical board (esp. the two Megas, or any of the now-four Unos sharing 2341:0043 - mast, microscope, antenna, `power_uno6`; see the udev serial-collision caveats), which opens fine but never recognizes that board's frame types (surfacing as climbing `checksum_error_count`, the lamp's own "bad" counter), or (b) the board was still finishing its own auto-reset (opening a serial connection resets Arduino-family boards) when the bridge sent its first frame | `tools/raw_serial_probe.py <port>` to test the board directly, independent of ROS — this still works for `power_uno6` even though it sends a 'D' frame that board ignores, since it also proactively sends its own state frames on a timer regardless of what arrives; confirm the symlink with `ls -l /dev/rover/*` against `ls /dev/ttyACM* /dev/ttyUSB*` with only that board plugged in. (b) should self-resolve automatically now — `SerialLink` has a `boot_grace_sec` (default 2.0s) that no-ops I/O until the board's had time to reboot; raise it in that board's config yaml if 2s isn't enough for your bootloader |
| Arm never reports `homed: true` | A calibration switch not wired/triggering, or `home_on_startup` never sent | Check `arm_mega2` wiring against `firmware/arm_mega2/arm_mega2.ino`'s pin comments |
| Arm: `homed: true`, some joints move and others don't | Since J1-J3 moved to TB6600 (each with its own enable pin) and J4/J5 stayed on A4988 (one shared enable pin), a wiring mistake on any *one* of those four enable connections now only affects the joint(s) behind it, not the whole arm | Check `kTb6600EnablePin[3]` (pins 13/14/15, one per J1/J2/J3) and `kA4988EnablePin` (pin 12, shared by J4/J5) against actual wiring; measure each driver's own ENA/EN pin directly rather than assuming they're all still tied together the way they used to be |
| Mast never reports `homed: true` | Same as the arm above, but for the yaw/pitch calibration switches specifically (added when the mast moved to an Uno) | Check `mast_uno3` wiring against `firmware/mast_uno3/mast_uno3.ino`'s pin comments; `ros2 topic echo /rover_mast/state` for `yaw_limit_triggered`/`pitch_limit_triggered` — neither ever flipping `true` while the head physically reaches its travel limits means that switch isn't wired/triggering |
| `board_temperature_decic` reads `-9999` on base/arm/mast/antenna | DS18B20 didn't respond on its most recent read - most likely the external 4.7kΩ pull-up resistor is missing (this is not optional and the Arduino's internal pull-ups cannot substitute), or DQ/GND/VDD are swapped | Check the 4.7kΩ pull-up is actually present between DQ and VDD; verify pinout against the sensor's own datasheet (GND/DQ/VDD left-to-right facing the flat TO-92 face is common but not universal); see 6.8 |
| Mast fan runs constantly regardless of temperature | Fan wired to the module's always-on V+ terminal instead of the switched V-/OUT terminal (an easy mix-up on a low-side switch design), or `board_temperature_decic` is stuck at `-9999` (the deliberate fail-toward-running behavior on sensor failure, not a fan bug - see 6.8b/README) | Check the fan's positive lead is on V+ and negative on V-/OUT, not reversed; check `board_temperature_decic` isn't `-9999` before assuming the fan control logic itself is wrong |
| Mast fan never turns on even when the board feels warm | `fan_duty_percent` reading `0` while `board_temperature_decic` is below the placeholder 35°C threshold is expected, not a bug - "feels warm to the touch" and "35.0°C" aren't the same thing | Check the actual `board_temperature_decic` value against `kFanOnTempDeciC` in `mast_uno3.ino` before assuming a wiring fault; bench-tune the threshold once real enclosure behavior is known |
| I2C scan on the base's bus shows `0x70` (and possibly a reserved-range address like `0x03`) | `0x70` is the PCA9685's own built-in "All Call" address, present on every unit at power-up regardless of jumper state - seeing it does NOT mean the board is jumpered to 0x70, and does not mean anything is wrong. A reserved-range hit (`0x00`-`0x07`) is very likely a scanner artifact, not a second real device | No action needed - the PCA9685 should be left at its factory-default 0x40 (unjumpered) either way; see 6.8c |
| `battery1_voltage_mv`/`battery2_voltage_mv` reads 0 or implausible regardless of the real battery | Either that battery's INA226 isn't responding (wiring, or the wrong mux channel), or the TCA9548A itself isn't responding | Use an I2C scanner to confirm the TCA9548A answers at 0x70, then - with each mux channel selected in turn - that channel's INA226 answers at 0x40; a wiring mistake anywhere in that chain looks identical to a dead sensor from the ROS side |
| `battery1_current_ma`/`battery2_current_ma` never reads more than a small value regardless of real load | Very likely the shunt-resistor concern in 6.8d, not a fault - a common 0.1Ω onboard shunt caps the INA226's measurable current at ~0.82A, far below real battery current | Check what shunt resistor value the actual breakout board has (usually printed on the PCB) before assuming a firmware bug; see 6.8d and README's "Explicit assumptions" |
| Mast `homed: true`, yaw/pitch commands silently do nothing | Just finished calibration - it disables the drivers automatically once both axes reach true zero | Check `driver_enabled` in the state message; if `false`, send a command with `driver_enable: true` (or the web GUI's `CLOSE DRIVER (ENABLE)`) |
| Mast lift doesn't move | Limit switches wired backwards (already at a triggered state), or the HW-039's enable pin isn't wired | `mast_uno3.ino` — check `stowedLimitTriggered()`/`erectLimitTriggered()` against actual switch wiring; confirm `kLiftEnPin` reads HIGH (R_EN/L_EN tied together on the HW-039 module) |
| Mast yaw/pitch: `homed: true`, drivers otherwise seem fine, but nothing moves | TB6600's ENA input wired wrong polarity, or ENA+ not actually tied to +5V | `kHeadEnablePin` (pin 13) is driven LOW = enabled, assuming standard common-anode wiring (ENA+ to +5V, ENA- to the Arduino pin) — if wired common-cathode instead this is backwards; measure the actual voltage across ENA+/ENA- on each TB6600 while the board is running to confirm which convention it's actually wired for |
| Microscope focus/zoom motor runs hot, stalls, or the DRV8825 gets hot fast | 24BYJ-48's center-tap wire is still connected somewhere (grounded, or tied to the driver) instead of left disconnected | Physically verify only the 4 coil-end wires reach the driver's A1/A2/B1/B2 - see `microscope_uno4.ino`'s header comment; disconnect power immediately if this is suspected, this is a real short risk, not just a performance issue |
| Microscope focus/zoom doesn't move, `driver_enabled: false` in the state message | Nothing has enabled the driver yet - it starts disabled at boot | Web GUI: click `CLOSE DRIVER (ENABLE)`; manually: send a `MicroscopeCommand` with `driver_enable: true` |
| Antenna azimuth/elevation don't move, `homed: true` in the state message | Nothing has enabled the drivers via a command yet - they start enabled internally for homing's own sake, but `antenna_uno5.ino` ignores every command's `driver_enable` value until `homed` is true, so the very first command after that needs to explicitly ask for it | Web GUI: click `CLOSE DRIVER (ENABLE)`; manually: send an `AntennaCommand` with `driver_enable: true` |
| Antenna never reports `homed: true` | Same as the arm/mast above, but for the azimuth/elevation calibration switches specifically | Check `antenna_uno5` wiring against `firmware/antenna_uno5/antenna_uno5.ino`'s pin comments; `ros2 topic echo /rover_antenna/state` for `azimuth_limit_triggered`/`elevation_limit_triggered` - neither ever flipping `true` while the gimbal physically reaches its travel limits means that switch isn't wired/triggering |
| Camera topic publishes nothing | Wrong `camera_device` path, or camera needs a moment after plug-in | `v4l2-ctl --list-devices`, confirm the udev symlink from step 8 |
| Camera log shows GStreamer warnings ("unable to start pipeline") followed by a `CV_IMAGES`/`icvExtractPattern` "can't find starting number" error | Not actually a missing-camera problem - OpenCV's backend auto-detection tries FFMPEG then GStreamer before ever reaching V4L2, and a udev symlink can fail against those two even though it's a perfectly ordinary V4L2 device, cascading all the way to `CV_IMAGES` (meant for numbered image-file sequences, not devices) | Fixed in `main_camera_node.py`/`camera_publisher_node.py` by requesting `cv2.CAP_V4L2` explicitly rather than relying on auto-detection - if you see this exact cascade elsewhere (e.g. a new camera node), the fix is the same |
| MoveIt goal rejected immediately, no execution attempted | Arm hasn't finished homing yet, or `trajectory_action_server` isn't running | `ros2 topic echo /rover_arm/state` for `homed: true`; `ros2 action list` for `/arm_controller/follow_joint_trajectory` |
| "Unable to identify any set of controllers" from `move_group` | `moveit_controllers.yaml`'s controller name/`action_ns` doesn't match what `trajectory_action_server` actually serves | Confirm the action name printed by `ros2 action list` is exactly `arm_controller/follow_joint_trajectory` |
| `arm_bridge_node` crashes with `RoverFrameError: field 0 is not an int` (or similar) | Fixed — was a real bug, not a config issue. rclpy backs fixed-size numeric array message fields (`ArmCommand`'s `int32[5] joint_target_steps`) with `numpy.ndarray`; its elements are `numpy.int32`, not Python `int`, and used to reach the protocol layer's strict int-only check unconverted. If you see this again on a *different* field, the fix is the same: cast to `int(...)` where the array field's value is first read out of the message, not just where it's encoded. |
| A board's own status lamp never goes green *or* amber at all — not "connected with bad data", just no data, ever, right after adding or pulling in a new message/service type (e.g. `rover_msgs/srv/EmergencyStop`, `rover_msgs/srv/ArmPreset`) | The bridge node that `import`s the new type crashed on startup, before publishing anything — a brand-new message/service type needs a rebuild to actually generate its Python bindings; the node's own `from rover_msgs.srv import ...` (or `.msg import`) fails with an `ImportError` until then, and the whole process exits immediately | Check that specific node's own console/log output first — an import traceback there confirms this directly. `colcon build --symlink-install`, re-source, relaunch. This isn't arm-specific — any board's bridge node hits the same failure mode after any new message/service type is added, if launched before the next rebuild |
| A bridge node logs normally at startup, then goes completely silent forever — no error, no crash, no further output at all, `board_status` never published (lamp stuck red) — especially right around a slow firmware operation (a real example: the arm mid-homing, against its own 120:1 gearbox) | FIXED — was a real, project-wide bug, not this board's own fault: `SerialLink`'s underlying connection had a read timeout but no write one before this fix, so a write could block the calling thread indefinitely if the board's own tiny hardware serial buffer filled faster than its firmware was draining it. Silence with no error is the actual signature of this - a thread stuck on a blocking syscall produces neither | Confirm you're on a build that includes `write_timeout` in `rover_protocol/serial_link.py`'s own `SerialLink.__init__()` (see README's "Explicit assumptions" for the full incident) - `colcon build --symlink-install` after pulling in a fix, then relaunch. If it recurs even on a fixed build, that's a different, new issue - open with the actual log output, not a description of the symptom |
| A board's own lamp never turns green *and* its own telemetry panel never shows anything, even though `ros2 topic echo` on that board's own `state`/`board_status` topics shows real, valid messages actively streaming, and the browser's own dev-tools console shows no errors at all — specifically the base and arm panels, with every other board's own panel looking fine | FIXED — was a real, project-wide bug in `rover_web_gui`, not that board's own bridge node or firmware at all (both fully exonerated by the exact `ros2 topic echo` + browser-console checks described in the symptom column - see README's "Explicit assumptions" for the full incident). `ros_bridge.py`'s own `_on_base_state()`/`_on_arm_state()` read fixed-size ROS array fields (base's `encoder_ticks`, arm's `joint_position_steps` and others) via a bare `list(msg.field)`, which rclpy backs with `numpy.ndarray` - producing `numpy.int32`/`numpy.bool_` elements that `json.dumps()` cannot serialize at all. That exception was previously uncaught inside `server.py`'s own combined, single-task telemetry sender, silently killing telemetry for **every** board at once, not just the one with the bad field - the other boards only looked fine because the browser was still displaying their own last successfully-sent values, frozen, never actually still updating. This is exactly why the browser console stayed clean: the failure happened server-side, in Python, inside an `asyncio` task whose own uncaught exceptions never reach the browser at all | Confirm you're on a build with the per-element casts in `ros_bridge.py`'s own `_on_base_state()`/`_on_arm_state()` (`[int(t) for t in ...]`, not `list(...)` directly) and the defensive per-tick exception handling in `server.py`'s own `_telemetry_sender()` - `colcon build --symlink-install`, relaunch. If a *different* board's own telemetry silently stops working after this fix, check that board's own state message for any array field first (`grep -E '^\w+\[' src/rover_msgs/msg/<Board>State.msg`) - the same bug class applies to any array field on any board, not just these original two, though `_telemetry_sender()`'s own hardening should now at least keep it from taking every other board down with it |
| GPS `fix.status.status` always `-1` | No sky view (expected indoors), or wrong baud | Move outdoors; confirm `serial_baud: 9600` in `sensors.yaml` matches the module |
| Xbox teleop does nothing | Deadman (RB) not held, or axis/button indices wrong for your controller | Step 9.3 |
| Web GUI loads but shows no telemetry | ROS bridge not connected — check the `web_gui_node` terminal for errors | Confirm other subsystems are actually running/publishing first |
| `colcon build` errors | See the table in Step 4 | |

---

## 13. Next steps

Once bring-up is solid, see the "Known gaps / natural next steps"
section of `README.md` — arm IK, wheel odometry, and Nav2 integration
are the natural follow-ons, none of which block using the rover
manually via the web GUI or Xbox controller today.
