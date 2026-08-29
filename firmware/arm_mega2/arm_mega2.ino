// arm_mega2.ino
//
// Arduino Mega #2 - 5-axis robotic arm controller.
//   * Mixed stepper drivers per joint: J1 (shoulder_yaw), J2
//     (shoulder_pitch), and J3 (elbow_pitch) use TB6600; J4
//     (wrist_pitch) and J5 (wrist_roll) still use A4988. Each TB6600
//     gets its own independent enable pin (see kTb6600EnablePin's own
//     comment for why - not shared the way the mast's two TB6600s
//     are); the two remaining A4988s still share one enable pin
//     between themselves, same as all five used to share before this
//     change. PUL/DIR (TB6600) and STEP/DIR (A4988) are functionally
//     identical from AccelStepper's perspective, so kStepPin/kDirPin
//     below are unchanged and apply uniformly across all 5 joints
//     regardless of which driver type is actually behind them.
//   * 5x calibration (limit) switches, one per joint, used to home the
//     arm to a known zero position - homing can run against all 5
//     joints in one sequential pass (mirroring the original startup
//     behavior) or against a single joint on its own, selected via a
//     parameter on the 'Z' frame (see handleHomeRequest()) - re-homing
//     just one joint doesn't disturb the others' already-established
//     zero or homed status. Every joint must be individually homed
//     before ANY joint-move command is accepted, though - see
//     handleJointCommand()'s own comment for why partial-homed motion
//     isn't supported.
//
//     UPDATED: per-joint homing direction (kHomingDirection) and the
//     order joints are homed in during an all-5 run (kHomingOrder)
//     are independently configurable constants, not hardcoded
//     assumptions - see each constant's own comment below for the
//     full reasoning. Both are PLACEHOLDER values pending real
//     mechanical verification on the bench, same status as every
//     other uncalibrated constant in this project -
//     kHomingDirection defaults to the previous, only behavior (all
//     5 joints seek in the same, negative direction); kHomingOrder
//     defaults to the previous, only order (J1 through J5,
//     sequentially).
//
//     UPDATED AGAIN: the post-limit-switch behavior this file used to
//     call an "offset" (kHomingOffsetSteps) has been replaced entirely
//     by a physically-grounded model, not just renamed. Each joint now
//     has a real steps-per-degree conversion (kStepsPerDegree) and a
//     real operational range in degrees (kMinDeg/kMaxDeg) - sourced
//     directly from this project's own existing, already-established
//     numbers (rover_arm/config/arm_topology.yaml's own
//     steps_per_joint_rev, and rover_description/urdf/arm.xacro's own
//     joint <limit> tags), not invented fresh here. The limit switch's
//     own physical trigger point is now understood to sit at each
//     joint's own operational LOWER bound (kLowerLimitSteps) rather
//     than at an arbitrary "zero" - so serviceHoming() now assigns
//     that real, negative value via setCurrentPosition() the moment
//     the switch trips, then drives the remaining distance to absolute
//     0 (which this model defines as each joint's own true center, not
//     an arbitrary reference point) using the normal accel profile.
//     Every regular joint command (and preset move) is now also
//     clamped to [kMinDeg, kMaxDeg] (converted to steps via
//     kStepsPerDegree) before being accepted - see
//     clampToOperationalRange(). kLowerLimitSteps is independently
//     settable from kMinDeg (not derived from it in code) so a real
//     mechanical safety margin between "where the switch actually
//     trips" and "the joint's own declared operational minimum" can be
//     introduced later without changing how either constant works -
//     PLACEHOLDER values for both currently assume they coincide
//     exactly (the switch sits right at the operational minimum, no
//     margin), which is a reasonable starting assumption, not a
//     verified one.
//
//     Also added: three predefined poses (kInitialPoseSteps,
//     kTransportPoseSteps, kServicePoseSteps), each an ALL-PLACEHOLDER
//     (all-zero) five-joint target set pending real-world calibration,
//     reachable via a new 'P' frame (see handlePresetRequest()) -
//     gated by the same "fully homed, not mid-homing" requirement as
//     a regular 'A' command, and additionally blocked while an
//     emergency stop is latched (see below).
//
//     Also added: a latching emergency stop, triggered/cleared via a
//     new 'X' frame (see handleEmergencyStop()). Deliberately does
//     NOT de-energize the drivers - see that function's own comment
//     for the full reasoning (a gravity-loaded arm dropping
//     uncontrolled if de-energized mid-air is a worse outcome than
//     holding position under load, the same philosophy this file's
//     own watchdog-timeout behavior already applies at the bottom of
//     loop()). Once triggered, every source of new movement (regular
//     joint commands, preset requests) is blocked until an explicit
//     clear is received - the firmware itself is the source of truth
//     for this, not the ROS bridge node upstream of it, specifically
//     so a bridge-node restart or hiccup after an e-stop can't
//     silently resume motion.
//   * DS18B20 temperature sensor, TO-92, 1-Wire on a single digital
//     pin (kDs18b20DataPin, reusing the Mega's former SDA pin 20) -
//     see base_mega1.ino's own copy of this sensor for the full
//     reasoning (external pull-up requirement, non-blocking read
//     state machine, sentinel convention); identical here.
//   * Cooling fan via a generic N-channel MOSFET driver module - same
//     automatic, thermostatic design as base_mega1.ino's own copy of
//     this feature (see that file's header for the full reasoning:
//     low-side-switch wiring, hysteresis thresholds, fail-toward-
//     running on sensor failure). Genuine hardware PWM via
//     analogWrite() on kFanPwmPin (44) - this Mega's 2-13 PWM range is
//     entirely committed already (every joint's STEP/DIR plus all
//     three TB6600 enables), so this uses one of the Mega's extra
//     44-46 PWM pins instead, the same choice base_mega1.ino makes for
//     the same reason.
//
// Talks to the ROS 2 `rover_arm` bridge node using the shared
// RoverProtocol framing. Requires the AccelStepper library (Library
// Manager: "AccelStepper" by Mike McCauley), "OneWire" (by Paul
// Stoffregen), and "DallasTemperature" (by Miles Burton) in addition
// to RoverProtocol. See base_mega1.ino's own header comment for the
// DallasTemperature LGPL-2.1 licensing note - a third license
// alongside this project's own Apache-2.0 and, on the base board,
// ServoEasing's GPL-3.0.
//
// Joint order everywhere below: J1 shoulder_yaw, J2 shoulder_pitch,
// J3 elbow_pitch, J4 wrist_pitch, J5 wrist_roll (see rover_arm's
// arm_topology.yaml for the matching ROS-side joint_names list).

#include <AccelStepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RoverProtocol.h>

constexpr uint8_t kNumJoints = 5;

constexpr uint8_t kStepPin[kNumJoints] = {2, 4, 6, 8, 10};
constexpr uint8_t kDirPin[kNumJoints] = {3, 5, 7, 9, 11};
constexpr uint8_t kLimitPin[kNumJoints] = {22, 23, 24, 25, 26};

// J1/J2/J3: TB6600, each with its OWN independent enable pin -
// deliberately not ganged onto one shared pin the way mast_uno3.ino
// ties its two TB6600 enables together. That was fine there: only two
// boards, under a genuinely tight Uno pin budget. Here there are
// three, and TB6600's ENA input is opto-isolated - driving it LOW
// (enabled, common-anode wiring) sinks real current through that
// opto-coupler's internal LED, typically on the order of 10-15mA
// depending on the specific board's internal resistor. Three of those
// tied to one Arduino pin risks exceeding a single GPIO's safe sink
// rating (~20-40mA depending on the chip, and that number assumes no
// other load sharing the same pin) - independent pins sidestep the
// question entirely, and the Mega has no shortage of spare pins the
// way the mast's Uno did, so there's no real cost to being careful here.
// Wiring assumed common-anode (ENA+ to +5V, ENA- to these pins, active
// LOW), same convention as mast_uno3.ino's TB6600 - flip the polarity
// below if wired common-cathode instead.
constexpr uint8_t kTb6600EnablePin[3] = {13, 14, 15};  // J1, J2, J3 - pins 14/15 double as Serial3 TX/RX, unused in this sketch

// J4/J5: still A4988, sharing one enable pin between the two of them -
// A4988's EN is a simple direct-logic input, not opto-isolated, and
// safely shareable the same way all five joints used to share this
// exact pin before J1-J3 moved to TB6600.
constexpr uint8_t kA4988EnablePin = 12;

// FZ0430 voltage sensor (main battery supply) - see base_mega1.ino's
// own copy of this same sensor for the full conversion-math comment;
// kept identical across all three boards, including the pin choice.
constexpr uint8_t kVoltageSensorPin = A0;

// DS18B20 temperature sensor - see base_mega1.ino's own copy for the
// full reasoning on pin choice, the pull-up requirement, the
// non-blocking read timing, and the sentinel convention; identical
// constants here (including reusing this Mega's own former SDA pin).
constexpr uint8_t kDs18b20DataPin = 20;
constexpr unsigned long kTemperatureReadIntervalMs = 1000;
constexpr unsigned long kTemperatureConversionMs = 750;
constexpr int32_t kTemperatureInvalidDeciC = -9999;

// Cooling fan - see base_mega1.ino for the full reasoning on every
// constant below; values kept identical across every board with a
// fan. kFanPwmPin=44 for the same reason as base_mega1.ino: this
// Mega's 2-13 PWM range is fully committed (unlike base, where it
// isn't, but 44 was chosen there too for consistency), so this uses
// one of the extra 44-46 PWM pins instead.
constexpr uint8_t kFanPwmPin = 44;
constexpr int32_t kFanOnTempDeciC = 350;
constexpr int32_t kFanOffTempDeciC = 300;
constexpr int32_t kFanMaxTempDeciC = 500;
constexpr uint8_t kFanMinDutyPercent = 30;

AccelStepper joints[kNumJoints] = {
    AccelStepper(AccelStepper::DRIVER, kStepPin[0], kDirPin[0]),
    AccelStepper(AccelStepper::DRIVER, kStepPin[1], kDirPin[1]),
    AccelStepper(AccelStepper::DRIVER, kStepPin[2], kDirPin[2]),
    AccelStepper(AccelStepper::DRIVER, kStepPin[3], kDirPin[3]),
    AccelStepper(AccelStepper::DRIVER, kStepPin[4], kDirPin[4]),
};

// Motor-shaft speed/accel limits for the NEMA17 + A4988 combo itself -
// independent of whatever gearbox is attached, so these don't change
// with gear ratio. What DOES change is the resulting real-world output
// (joint) speed: with the EBA-17-M's 120:1 reduction (200 full steps *
// 1/16 microstepping * 120:1 = 384000 steps/rev - see
// rover_arm/config/arm_topology.yaml), 2000 steps/sec at the motor
// works out to about 1.87 deg/sec at the joint - correspondingly slower
// than the 45 deg/sec the original 5:1 placeholder gear ratio implied.
// Raise kMaxSpeedStepsPerSec (within whatever this motor/driver can
// actually sustain) if that's too slow for the application; the
// tradeoff is losing some of the torque/precision headroom a 120:1
// reduction buys over a lower ratio.
constexpr float kMaxSpeedStepsPerSec = 2000.0f;
constexpr float kAccelStepsPerSec2 = 4000.0f;
constexpr float kHomingSpeedStepsPerSec = 400.0f;
// Safety cutoff: abort homing a joint that travels this many steps
// without ever triggering its limit switch (mechanical fault, unplugged
// switch, etc.) rather than driving it into a hard stop indefinitely.
// Scaled to preserve the same ~675 degree angular safety margin as the
// original 5:1-gearing placeholder value (comfortably over the largest
// real joint range - wrist_roll's +-170 deg - specifically so this
// isn't tightened right down to that range and risk false-triggering
// on a joint that's merely a bit off-center at power-up). At 120:1
// gearing this margin now costs real time to traverse in a genuine
// fault case: worst-case time-to-abort is 720000 / 400 =~ 1800
// seconds (30 minutes) at kHomingSpeedStepsPerSec's default. Raise
// kHomingSpeedStepsPerSec, or deliberately tighten this margin nearer
// 340 degrees if that worst case matters more than the extra safety
// margin, if 30 minutes is impractical for how homing faults get
// noticed and handled in practice.
constexpr long kHomingMaxTravelSteps = 720000;

// Per-joint homing direction - CONFIGURABLE now, was previously a
// single hardcoded -1 applied uniformly to all 5 joints regardless of
// each one's own actual mechanical layout. +1 or -1 only (which way
// this joint's stepper seeks toward its limit switch); any other
// value is nonsensical and not validated against, same as this
// project's other placeholder physical-calibration constants.
// PLACEHOLDER - matches the previous, only behavior (all negative)
// until verified per joint on the bench; flip individual entries to
// +1 for any joint whose limit switch actually sits on the
// increasing-step side instead.
constexpr int8_t kHomingDirection[kNumJoints] = {-1, -1, -1, -1, -1};

// Order joints are homed in during an all-5 run (requestedJoint < 0
// to startHoming()) - CONFIGURABLE now, was previously a fixed J1,
// J2, J3, J4, J5 sequential loop with no way to change it. A
// permutation of 0..4, each index appearing exactly once - not
// validated against malformed entries (e.g. a duplicate or an
// out-of-range value), same placeholder-trust-the-constant status as
// this project's other physical-layout arrays. Only affects the
// all-5 case: a single-joint homing request (0-4 on the 'Z' frame)
// always just homes that one joint directly, unaffected by this
// order. PLACEHOLDER - matches the previous, only order (sequential
// J1 through J5) until a different mechanical/safety sequence is
// actually needed (e.g. homing the shoulder before the wrist to
// avoid a collision partway through the sweep).
constexpr uint8_t kHomingOrder[kNumJoints] = {0, 1, 2, 3, 4};

// Steps per degree of real joint rotation - derived from, and must be
// kept in sync with, rover_arm/config/arm_topology.yaml's own
// steps_per_joint_rev (currently 384000 for all 5: 200 full steps *
// 1/16 microstepping * 120:1 EBA-17-M planetary gearbox). 384000/360 =
// 1066.667 (repeating), the same value for all 5 joints below - that
// yaml file's own comment already flags this uniformity as provisional
// (J1-J3 use TB6600, J4/J5 use A4988, and there's no guarantee both
// driver types end up configured for the same microstep factor unless
// separately verified) - the same caveat applies here, for the same
// reason, not repeated in full at every use. If that yaml's own
// numbers change, these five must change with it - there's no
// automatic single source of truth across the ROS/firmware language
// boundary, so this is a manual-sync obligation, not a one-time copy.
constexpr float kStepsPerDegree[kNumJoints] = {
    384000.0f / 360.0f, 384000.0f / 360.0f, 384000.0f / 360.0f, 384000.0f / 360.0f, 384000.0f / 360.0f};

// Each joint's own operational range in degrees, relative to the true
// center (absolute step position 0 - see kLowerLimitSteps's own
// comment below for how that center is actually established during
// homing). NOT a placeholder guessed here - sourced directly from
// rover_description/urdf/arm.xacro's own joint <limit> tags (shoulder_
// yaw/elbow_pitch: +-150deg, shoulder_pitch/wrist_pitch: +-100deg,
// wrist_roll: +-170deg), which is also what MoveIt itself already
// plans against - keeping these two numbers in sync matters for a real
// reason, not just tidiness: a mismatch would mean MoveIt could plan a
// trajectory this firmware's own clampToOperationalRange() then
// silently truncates, so the arm quietly stops short of where MoveIt
// believes it commanded it to go. If arm.xacro's own limits are ever
// revised, these ten numbers need revising with them, the same
// manual-sync obligation as kStepsPerDegree above and for the same
// underlying reason (no automatic single source of truth across the
// URDF/firmware boundary).
constexpr float kMinDeg[kNumJoints] = {-150.0f, -100.0f, -150.0f, -100.0f, -170.0f};
constexpr float kMaxDeg[kNumJoints] = {150.0f, 100.0f, 150.0f, 100.0f, 170.0f};

// The value assigned via setCurrentPosition() the instant a joint's
// limit switch trips - REPLACES this file's own former
// kHomingOffsetSteps entirely, not just a rename: previously an
// arbitrary small offset from an assumed-zero switch position:
// this project's OLD homing model treated the switch's own trigger
// point as a temporary "zero", then moved a short, separately-tuned
// distance to whatever the joint's real reference point was. The new
// model treats the switch's own trigger point as the joint's own real,
// physical LOWER bound instead - PLACEHOLDER values below assume the
// switch sits exactly at each joint's own kMinDeg (i.e.
// kMinDeg[j]*kStepsPerDegree[j], rounded to the nearest whole step) as
// a reasonable starting assumption, not a bench-verified one - a real
// mechanical safety margin between the two (switch trips slightly
// before the declared operational minimum, for example) can be
// introduced later by changing these five numbers alone, without
// touching kMinDeg or any other constant, since this array is kept
// independently adjustable from kMinDeg on purpose, not derived from
// it in code.
//   J1 -150deg: -150 * 3200/3 = -160000 (exact)
//   J2 -100deg: -100 * 3200/3 = -106666.67, rounded -106667
//   J3 -150deg: -160000 (exact, same math as J1)
//   J4 -100deg: -106667 (same math as J2)
//   J5 -170deg: -170 * 3200/3 = -181333.33, rounded -181333
// Once the limit switch trips and this value is assigned, homing
// drives the joint the remaining distance to absolute step 0 - this
// project's own definition of each joint's true center - using the
// normal accel/speed profile; see serviceHoming()'s own comment for
// the full two-phase sequence this drives.
constexpr long kLowerLimitSteps[kNumJoints] = {-160000, -106667, -160000, -106667, -181333};

// Three predefined five-joint poses, in steps, each reachable with a
// single 'P' frame (see handlePresetRequest()) rather than requiring
// the operator (or whatever's upstream) to know and send all 5 target
// steps by hand. ALL THREE ARE PLACEHOLDERS - all-zero (every joint at
// its own homed zero), the same "flag clearly, pending real
// calibration" status as every other uncalibrated constant in this
// project, not a claim these are actually safe, useful, or reachable
// poses yet. Bench-verify and replace all three before relying on any
// of them for real:
//   INITIAL   - the pose the arm should present in for normal
//               operation once calibration completes.
//   TRANSPORT - a compact pose safe for driving/moving the rover with
//               the arm folded out of the way, not actively in use.
//   SERVICE   - a pose for accessing/servicing the arm or something
//               near it - the specific purpose wasn't detailed
//               further when this was added; revisit this comment
//               once that's known, rather than guess at a real value
//               without knowing what it needs to clear or reach.
constexpr long kInitialPoseSteps[kNumJoints] = {0, 0, 0, 0, 0};
constexpr long kTransportPoseSteps[kNumJoints] = {0, 0, 0, 0, 0};
constexpr long kServicePoseSteps[kNumJoints] = {0, 0, 0, 0, 0};

bool driversEnabled = false;
bool jointHomed[kNumJoints] = {false, false, false, false, false};
bool homingInProgress = false;
int8_t homingJointIndex = -1;  // index INTO homingSequence[] below, not a joint index directly
int8_t homingRangeEnd = -1;  // one past the last position in homingSequence[] this homing run covers
long homingStartPosition = 0;
// The actual joint indices this homing run covers, in the order
// they're actually homed - for an all-5 run this is kHomingOrder[]
// copied in at start; for a single-joint run it's just that one
// joint. homingJointIndex above indexes into THIS array, not
// directly into joints[]/jointHomed[] - homingSequence[homingJointIndex]
// is the actual joint currently being homed.
uint8_t homingSequence[kNumJoints];
// Non-blocking two-phase per-joint homing state, mirroring
// mast_uno3.ino's own postCalState pattern: SEEKING_LIMIT drives the
// joint at constant speed toward its limit switch; once triggered,
// MOVING_TO_CENTER takes over and runs the joint the rest of the way
// to absolute step 0 - this project's own definition of each joint's
// true center, see kLowerLimitSteps's own comment - using the normal
// accel/speed profile, checked non-blockingly every loop() iteration
// rather than with a blocking wait - see serviceHoming() for the full
// state machine. Renamed from this file's own former
// MOVING_TO_OFFSET: the destination used to be a per-joint, separately
// -tuned kHomingOffsetSteps value; it's now always exactly 0, so
// "offset" stopped describing what this phase actually does.
enum HomingPhase : int8_t { SEEKING_LIMIT = 0, MOVING_TO_CENTER = 1 };
int8_t homingPhase = SEEKING_LIMIT;

// Latching emergency stop - see handleEmergencyStop()'s own comment
// for the full reasoning. Once true, blocks every source of new
// movement (handleJointCommand(), handlePresetRequest()) until an
// explicit clear arrives; does NOT stop joints[].run() from still
// being called each loop() iteration, since that's what actually
// executes the controlled deceleration stop() started - see loop()'s
// own comment on this.
bool estopActive = false;

OneWire oneWire(kDs18b20DataPin);
DallasTemperature ds18b20(&oneWire);
enum TempReadState : int8_t { TEMP_IDLE = 0, TEMP_CONVERTING = 1 };
int8_t tempReadState = TEMP_IDLE;
unsigned long tempConversionStartMillis = 0;
int32_t cachedTemperatureDeciC = kTemperatureInvalidDeciC;
unsigned long lastTemperatureReadMillis = 0;

bool fanRunning = false;
uint8_t fanDutyPercent = 0;

RoverProtocol::LineReader lineReader;
char lineBuf[RoverProtocol::kMaxLineLen];

unsigned long lastCommandMillis = 0;
constexpr unsigned long kWatchdogTimeoutMs = 1000;

// ------------------------------------------------------------- helpers ---
void setDriversEnabled(bool enabled) {
  driversEnabled = enabled;
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(kTb6600EnablePin[i], enabled ? LOW : HIGH);
  }
  digitalWrite(kA4988EnablePin, enabled ? LOW : HIGH);
}

bool limitTriggered(uint8_t joint) {
  // Wired NC-to-ground with INPUT_PULLUP: triggered == LOW.
  return digitalRead(kLimitPin[joint]) == LOW;
}

bool allJointsHomed() {
  for (uint8_t i = 0; i < kNumJoints; i++) {
    if (!jointHomed[i]) return false;
  }
  return true;
}

// requestedJoint: -1 homes all 5 joints in the order kHomingOrder[]
// specifies (previously always J1..J5; now configurable, see that
// constant's own comment); 0..4 homes just that one joint, leaving
// every other joint's homed state and position untouched -
// recalibrating one joint doesn't invalidate the others' already-
// established zero.
void startHoming(int8_t requestedJoint) {
  setDriversEnabled(true);
  homingInProgress = true;
  homingPhase = SEEKING_LIMIT;
  if (requestedJoint < 0) {
    for (uint8_t i = 0; i < kNumJoints; i++) homingSequence[i] = kHomingOrder[i];
    homingJointIndex = 0;
    homingRangeEnd = kNumJoints;
  } else {
    homingSequence[0] = (uint8_t)requestedJoint;
    homingJointIndex = 0;
    homingRangeEnd = 1;
  }
  uint8_t j = homingSequence[homingJointIndex];
  jointHomed[j] = false;
  joints[j].setMaxSpeed(kHomingSpeedStepsPerSec);
  joints[j].setSpeed(kHomingDirection[j] * kHomingSpeedStepsPerSec);  // per-joint configurable direction, see kHomingDirection's own comment
  homingStartPosition = joints[j].currentPosition();
}

// Two-phase per joint, non-blocking throughout - see homingPhase's
// own comment for why this exists (the move from kLowerLimitSteps[j]
// to absolute center needing a real, accel-profiled move after the
// limit switch trips, not just an instant re-label at the switch's
// own trigger point).
void serviceHoming() {
  if (!homingInProgress) return;
  uint8_t j = homingSequence[homingJointIndex];

  if (homingPhase == SEEKING_LIMIT) {
    bool traveledTooFar =
        labs(joints[j].currentPosition() - homingStartPosition) > kHomingMaxTravelSteps;

    if (limitTriggered(j) || traveledTooFar) {
      joints[j].stop();
      // The switch's own trigger point IS this joint's own real,
      // physical lower bound now, not an arbitrary temporary zero -
      // see kLowerLimitSteps's own comment for the full reasoning and
      // where this specific number comes from.
      joints[j].setCurrentPosition(kLowerLimitSteps[j]);
      joints[j].setMaxSpeed(kMaxSpeedStepsPerSec);
      joints[j].setAcceleration(kAccelStepsPerSec2);
      joints[j].moveTo(0);  // absolute center - see kLowerLimitSteps's own comment for why 0 means this now
      homingPhase = MOVING_TO_CENTER;
      return;
    }

    joints[j].runSpeed();  // constant-speed seek, no accel profile while seeking the switch itself
    return;
  }

  // MOVING_TO_CENTER
  joints[j].run();
  if (joints[j].distanceToGo() != 0) return;  // still moving toward center - check again next loop() iteration

  // Arrived at this joint's own true center. setCurrentPosition(0)
  // here is provably redundant given exact step-counted motion from a
  // correctly-labeled starting point (moveTo(0) from kLowerLimitSteps[j]
  // lands exactly on 0 by construction, not approximately) - kept
  // anyway as the same cheap, defensive re-assertion this file already
  // used in the old offset-based model, guarding against any future
  // change to this sequence that might not preserve that guarantee.
  joints[j].setCurrentPosition(0);
  jointHomed[j] = true;

  homingJointIndex++;
  if (homingJointIndex >= homingRangeEnd) {
    homingInProgress = false;
    return;
  }
  uint8_t next = homingSequence[homingJointIndex];
  jointHomed[next] = false;
  joints[next].setMaxSpeed(kHomingSpeedStepsPerSec);
  joints[next].setSpeed(kHomingDirection[next] * kHomingSpeedStepsPerSec);
  homingStartPosition = joints[next].currentPosition();
  homingPhase = SEEKING_LIMIT;
}

void handleHomeRequest(const RoverProtocol::ParsedFrame& frame) {
  if (frame.fieldCount != 1) return;  // malformed, ignore
  int32_t requested = frame.fields[0];
  if (requested < -1 || requested >= kNumJoints) return;  // out of range, ignore
  if (homingInProgress) return;  // one homing run at a time
  startHoming((int8_t)requested);
}

// Clamps a requested step target into [kMinDeg, kMaxDeg] for the given
// joint, converted to steps via kStepsPerDegree - applied to every
// regular joint command and every preset move (see both call sites)
// as a real, firmware-side backstop, not merely a courtesy: MoveIt
// itself already plans within these same limits (they're sourced
// directly from arm.xacro - see kMinDeg's own comment), so this should
// rarely if ever actually clamp anything in normal operation. It
// exists for whatever DOES reach the firmware outside of a MoveIt-
// planned trajectory - a raw joint command from the web GUI's own
// sliders, a hand-typed test frame, a future client that doesn't know
// about these limits at all - none of which have any obligation to
// have already respected arm.xacro's own bounds before this point.
long clampToOperationalRange(uint8_t joint, long targetSteps) {
  long lowerSteps = (long)lroundf(kMinDeg[joint] * kStepsPerDegree[joint]);
  long upperSteps = (long)lroundf(kMaxDeg[joint] * kStepsPerDegree[joint]);
  if (targetSteps < lowerSteps) return lowerSteps;
  if (targetSteps > upperSteps) return upperSteps;
  return targetSteps;
}

void handleJointCommand(const RoverProtocol::ParsedFrame& frame) {
  if (frame.fieldCount != (kNumJoints + 1)) return;  // malformed, ignore

  bool enable = frame.fields[kNumJoints] != 0;

  // All-or-nothing gate: every joint must be individually homed (and
  // no homing run currently in progress) before ANY joint target is
  // accepted, even though homing itself can now run per-joint. A
  // command that moved only the already-homed joints while silently
  // ignoring the rest would let the arm move through configurations
  // nothing has actually verified are safe for the not-yet-homed
  // joint's real position - simpler and safer to just wait for a
  // fully calibrated arm before accepting any motion at all.
  //
  // estopActive added to this same gate, not a separate check
  // elsewhere - once latched, this function does nothing at all
  // (not even the enable/disable toggle below), matching this gate's
  // own existing "simpler and safer to just wait" philosophy rather
  // than carve out a special case for which parts of a blocked
  // command are still allowed through.
  //
  // setDriversEnabled(enable) is INSIDE this gate now, not before it -
  // a real bug fixed here, not a stylistic choice. The bridge sends
  // 'A' frames continuously at its own control rate, including while
  // homingInProgress is true (by design - see arm_bridge_node.py's
  // own comment on why that's normally harmless), carrying whatever
  // enable value it last had - which defaults to false until an
  // operator has actually touched the arm panel. Applying that
  // unconditionally, before this gate, meant the very first homing
  // sequence at startup could have its drivers repeatedly toggled
  // enabled (by startHoming(), once) and disabled (by every regular
  // 'A' frame arriving mid-seek, continuously) - the same class of
  // problem mast_uno3.ino and antenna_uno5.ino's own handleXCommand()
  // were already built to avoid, that this one wasn't.
  if (!allJointsHomed() || homingInProgress || estopActive) return;

  setDriversEnabled(enable);
  for (uint8_t i = 0; i < kNumJoints; i++) {
    joints[i].moveTo(clampToOperationalRange(i, frame.fields[i]));
  }
  lastCommandMillis = millis();
}

// preset: 0=INITIAL, 1=TRANSPORT, 2=SERVICE - anything else is
// ignored as malformed. Same "fully homed, not mid-homing" gate as a
// regular joint command, plus the same estopActive block - a preset
// move is still new movement, no different from an operator manually
// driving the sliders as far as safety gating is concerned.
void handlePresetRequest(const RoverProtocol::ParsedFrame& frame) {
  if (frame.fieldCount != 1) return;  // malformed, ignore
  if (!allJointsHomed() || homingInProgress || estopActive) return;

  int32_t preset = frame.fields[0];
  const long* target;
  if (preset == 0) {
    target = kInitialPoseSteps;
  } else if (preset == 1) {
    target = kTransportPoseSteps;
  } else if (preset == 2) {
    target = kServicePoseSteps;
  } else {
    return;  // unknown preset index, ignore
  }

  for (uint8_t i = 0; i < kNumJoints; i++) {
    // Clamped for the same reason as handleJointCommand() above -
    // defense in depth, given these particular targets are firmware-
    // defined constants that should already be within range by
    // construction. Costs nothing when they already are; catches a
    // future bench-calibration typo cleanly rather than letting it
    // drive a joint straight past its own declared operational limit.
    joints[i].moveTo(clampToOperationalRange(i, target[i]));
  }
  lastCommandMillis = millis();  // a preset move is a real command too - keeps the watchdog from treating it as a stale link right after issuing it
}

// engage=true latches the e-stop and immediately begins a controlled
// stop on every joint; engage=false clears it, allowing new movement
// again. See this file's own header comment for the full reasoning
// behind not de-energizing the drivers here.
void handleEmergencyStop(const RoverProtocol::ParsedFrame& frame) {
  if (frame.fieldCount != 1) return;  // malformed, ignore
  bool engage = frame.fields[0] != 0;
  estopActive = engage;
  if (engage) {
    // AccelStepper::stop() is the library's own recommended way to
    // halt "as quickly as possible" - it sets a new, nearby target
    // computed from the current speed and this joint's own configured
    // acceleration, then relies on run() (still called every loop()
    // iteration regardless of estopActive - see loop()'s own comment)
    // to actually decelerate into it. Forcing an instantaneous
    // step-rate change instead, rather than a profiled deceleration,
    // risks the motor losing synchronization with its driver -
    // arguably a worse outcome than a fast but controlled stop.
    // Drivers are left enabled/energized throughout - not touched
    // here at all - deliberately: this is a gravity-loaded arm, and
    // de-energizing mid-air risks an uncontrolled drop, which this
    // project has already judged worse than holding position under
    // load in the near-identical watchdog-timeout situation below in
    // loop() (and on mast_uno3.ino/antenna_uno5.ino's own watchdogs).
    // A true, power-cutting emergency stop is the more conventional
    // industrial default; this arm intentionally departs from that
    // convention for this specific reason. Flagged here and in this
    // file's own header comment, not a silent design choice.
    for (uint8_t i = 0; i < kNumJoints; i++) {
      joints[i].stop();
    }
  }
  lastCommandMillis = millis();
}

void updateCachedTemperature() {
  unsigned long now = millis();
  if (tempReadState == TEMP_IDLE) {
    if ((now - lastTemperatureReadMillis) < kTemperatureReadIntervalMs) return;
    ds18b20.requestTemperatures();
    tempConversionStartMillis = now;
    tempReadState = TEMP_CONVERTING;
  } else {
    if ((now - tempConversionStartMillis) < kTemperatureConversionMs) return;
    float c = ds18b20.getTempCByIndex(0);
    cachedTemperatureDeciC =
        (c == DEVICE_DISCONNECTED_C) ? kTemperatureInvalidDeciC : (int32_t)lround(c * 10.0f);
    lastTemperatureReadMillis = now;
    tempReadState = TEMP_IDLE;
  }
}

// Thermostat + hardware PWM output - see base_mega1.ino's own
// updateFanControl() for the full reasoning; logic is identical.
void updateFanControl() {
  if (cachedTemperatureDeciC == kTemperatureInvalidDeciC) {
    fanRunning = true;
    fanDutyPercent = kFanMinDutyPercent;
  } else {
    if (!fanRunning) {
      if (cachedTemperatureDeciC >= kFanOnTempDeciC) fanRunning = true;
    } else {
      if (cachedTemperatureDeciC <= kFanOffTempDeciC) fanRunning = false;
    }

    if (!fanRunning) {
      fanDutyPercent = 0;
    } else if (cachedTemperatureDeciC >= kFanMaxTempDeciC) {
      fanDutyPercent = 100;
    } else {
      int32_t pos = cachedTemperatureDeciC - kFanOnTempDeciC;
      if (pos < 0) pos = 0;
      int32_t range = kFanMaxTempDeciC - kFanOnTempDeciC;
      fanDutyPercent = kFanMinDutyPercent + (uint8_t)(((100 - kFanMinDutyPercent) * pos) / range);
    }
  }
  analogWrite(kFanPwmPin, (uint8_t)(((uint16_t)fanDutyPercent * 255) / 100));
}

int32_t readSupplyVoltageMv() {
  int raw = analogRead(kVoltageSensorPin);
  return (int32_t)lround((raw / 1023.0f) * 25000.0f);
}

void sendStateFrame() {
  int32_t out[kNumJoints * 3 + 5];
  for (uint8_t i = 0; i < kNumJoints; i++) {
    out[i] = joints[i].currentPosition();
    out[kNumJoints + i] = limitTriggered(i) ? 1 : 0;
    out[kNumJoints * 2 + i] = jointHomed[i] ? 1 : 0;
  }
  out[kNumJoints * 3] = readSupplyVoltageMv();
  out[kNumJoints * 3 + 1] = cachedTemperatureDeciC;
  out[kNumJoints * 3 + 2] = fanDutyPercent;
  out[kNumJoints * 3 + 3] = estopActive ? 1 : 0;
  // Added at the user's own request, alongside the web GUI's own
  // single-toggle driver button - previously driversEnabled was only
  // ever a host-COMMANDED value (the 'A' frame's own enable field),
  // never reported back, which meant the web GUI had no honest way to
  // know the actual, current state - only what it last told the
  // firmware to do. That gap was real, not hypothetical:
  // startHoming() enables drivers automatically before seeking, with
  // no operator action involved, so a value the web GUI merely
  // remembered sending would have silently drifted out of sync with
  // reality the moment homing started on its own.
  out[kNumJoints * 3 + 4] = driversEnabled ? 1 : 0;
  RoverProtocol::sendFrame(Serial, 'S', out, kNumJoints * 3 + 5);
}

// -------------------------------------------------------------- setup ---
void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 3; i++) pinMode(kTb6600EnablePin[i], OUTPUT);
  pinMode(kA4988EnablePin, OUTPUT);
  setDriversEnabled(false);

  for (uint8_t i = 0; i < kNumJoints; i++) {
    pinMode(kLimitPin[i], INPUT_PULLUP);
    joints[i].setMaxSpeed(kMaxSpeedStepsPerSec);
    joints[i].setAcceleration(kAccelStepsPerSec2);
  }

  ds18b20.begin();
  ds18b20.setWaitForConversion(false);  // non-blocking - see base_mega1.ino's own copy of this same setup() line

  pinMode(kFanPwmPin, OUTPUT);
  analogWrite(kFanPwmPin, 0);  // starts off - updateFanControl() takes over once a real temperature reading exists

  lastCommandMillis = millis();
}

// --------------------------------------------------------------- loop ---
void loop() {
  updateCachedTemperature();
  updateFanControl();

  if (lineReader.poll(Serial, lineBuf, sizeof(lineBuf))) {
    RoverProtocol::ParsedFrame frame = RoverProtocol::parseFrame(lineBuf);
    if (frame.valid) {
      if (frame.type == 'A') {
        handleJointCommand(frame);
        sendStateFrame();
      } else if (frame.type == 'Z') {
        handleHomeRequest(frame);
        sendStateFrame();
      } else if (frame.type == 'P') {
        handlePresetRequest(frame);
        sendStateFrame();
      } else if (frame.type == 'X') {
        handleEmergencyStop(frame);
        sendStateFrame();
      }
    }
  }

  if (homingInProgress) {
    serviceHoming();
  } else {
    // Still called every iteration regardless of estopActive - this
    // is what actually executes handleEmergencyStop()'s own stop()
    // calls as a real, accel-profiled deceleration rather than a
    // frozen mid-step halt. estopActive only blocks NEW targets from
    // being set (handleJointCommand(), handlePresetRequest()), never
    // this call itself.
    for (uint8_t i = 0; i < kNumJoints; i++) joints[i].run();

    // estopActive deliberately excluded from this condition, not an
    // oversight - during a deliberate e-stop, lastCommandMillis stops
    // advancing (handleJointCommand() returns before reaching it, see
    // that function's own gate), which would otherwise make this
    // fire repeatedly while a joint is still mid-deceleration from
    // its own stop() call above. Re-issuing moveTo(currentPosition())
    // against a position that's still actively changing would fight
    // that deceleration instead of complementing it - the watchdog's
    // own "hold position" fallback exists for a lost comms link
    // specifically, and a live, deliberate e-stop already has its own
    // active handling; this fallback isn't needed on top of it and
    // would only add jitter.
    if (allJointsHomed() && !estopActive && (millis() - lastCommandMillis) > kWatchdogTimeoutMs) {
      // Comms lost: hold the current position rather than continuing
      // toward a stale target. Re-issuing moveTo(currentPosition())
      // cancels any in-flight AccelStepper motion cleanly.
      for (uint8_t i = 0; i < kNumJoints; i++) {
        joints[i].moveTo(joints[i].currentPosition());
      }
    }
  }
}
