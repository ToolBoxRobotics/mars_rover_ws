// antenna_uno5.ino
//
// Arduino Uno #5 - high-gain-antenna gimbal controller, mounted top
// rear left of the rover. Modeled on the real Mars Exploration Rover
// (Spirit/Opportunity) HGA gimbal: a 2-axis pointing mechanism -
// primary axis G1 (azimuth, normal to the deck) and secondary axis G2
// (elevation, parallel to the deck) - with the antenna disk on a
// short arm at the end, beam radiating perpendicular to the disk face.
// The real HGAG is launch-locked (a pyrotechnic pin puller) and
// deploys through a one-way spring-loaded gate that widens azimuth's
// usable range and prevents returning to the stowed position - this
// board doesn't model the launch/deploy mechanism itself (no
// pyrotechnics, no gate actuator - out of scope for what's being
// built here), only the two gimbal axes' *operational*, post-deployment
// range: azimuth 15-285 deg, elevation 0-180 deg. Those two numbers
// are what get enforced below (kAzimuthMinDeg/MaxDeg,
// kElevationMinDeg/MaxDeg) - the spec's other pair of numbers (G1
// -90/90, G2 90/270) reads as a different reference frame, not a
// second constraint to reconcile with these; flagged in README rather
// than guessed into a specific transform.
//
//   * 2x NEMA17 + EBA-17-M planetary gearbox (120:1) + TB6600, same
//     actuator/driver combination as the arm's joints - steps_per_deg
//     below uses the identical 200 full steps * 1/16 microstepping *
//     120:1 math as arm_mega2.ino's steps_per_joint_rev, and
//     kMaxSpeedStepsPerSec/kAccelStepsPerSec2/kHomingSpeedStepsPerSec
//     are copied directly from there too, not re-guessed - same
//     actuator, already-vetted numbers, no reason to diverge.
//     TB6600's PUL/DIR inputs are functionally equivalent to STEP/DIR
//     for AccelStepper::DRIVER, and its enable pin is shared between
//     both axes (kGimbalEnablePin) rather than wired independently -
//     safe here for the same reason the mast's own two TB6600s share
//     one enable pin (mast_uno3.ino): the current draw of two
//     opto-isolated ENA inputs on one Arduino GPIO stays comfortably
//     under a typical pin's safe sink rating, unlike the arm's three
//     TB6600s, which is why those get independent pins instead.
//   * 2x calibration switches, one per axis. Each is assumed mounted
//     at that axis's own operational minimum (15 deg azimuth, 0 deg
//     elevation) - a real design choice, not verified against any
//     actual mechanical drawing, flagged here and in README so it's
//     easy to correct against real hardware. Homing seeks each switch
//     the same direction arm_mega2.ino/mast_uno3.ino already do, and
//     because the switch position *is* each axis's minimum rather
//     than an offset from it, setCurrentPosition() at trigger time
//     directly establishes "home" - no separate move-to-a-different-
//     reference step needed the way the mast's corrected calibration
//     sequence needs one (see mast_uno3.ino's own header comment for
//     why that axis is different: its switches sit at each axis's
//     extreme, but "home" for the mast is the *centered* zero, not
//     the extreme itself - not true here, where the switch position
//     and the operational minimum are the same point).
//   * DS18B20 temperature sensor, TO-92, 1-Wire on a single digital
//     pin (kDs18b20DataPin = A4, reusing the Uno's former I2C SDA
//     pin) - free, same as every other pin from D9-D13 up (see the
//     pin section below for why this board's budget was never tight
//     the way the mast's Uno is). See base_mega1.ino's own copy of
//     this sensor for the full reasoning (external pull-up
//     requirement, non-blocking read state machine, sentinel
//     convention); identical here.
//   * Cooling fan via a generic N-channel MOSFET driver module - same
//     automatic, thermostatic design as base_mega1.ino's own copy of
//     this feature (see that file's header for the full reasoning:
//     low-side-switch wiring, hysteresis thresholds, fail-toward-
//     running on sensor failure). Genuine hardware PWM via
//     analogWrite() on kFanPwmPin (9) - unlike the mast's Uno, this
//     board's PWM pins (3/5/6/9/10/11) aren't all committed; 3/5/6 are
//     spoken for (azimuth/elevation DIR, shared gimbal enable), but
//     9/10/11 are free, so this needs no software-PWM workaround.
//
// Talks to the ROS 2 `rover_antenna` bridge node using the shared
// RoverProtocol framing. Requires the AccelStepper library, "OneWire"
// (by Paul Stoffregen), and "DallasTemperature" (by Miles Burton) -
// see base_mega1.ino's own header comment for the DallasTemperature
// LGPL-2.1 licensing note.

#include <AccelStepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RoverProtocol.h>

constexpr uint8_t kAzimuthStepPin = 2;
constexpr uint8_t kAzimuthDirPin = 3;
constexpr uint8_t kElevationStepPin = 4;
constexpr uint8_t kElevationDirPin = 5;
// Shared between both TB6600 drivers - active LOW, same convention as
// every other TB6600/A4988 enable in this project. Common-anode
// wiring assumed (ENA+ to +5V, ENA- to this pin) - flip the polarity
// below if wired common-cathode instead.
constexpr uint8_t kGimbalEnablePin = 6;
constexpr uint8_t kAzimuthLimitPin = 7;
constexpr uint8_t kElevationLimitPin = 8;
// D9-D13 intentionally spare - this board's pin budget is nowhere
// near as tight as the mast's Uno (no third/lift axis here), but
// there's no reason to spread pins out further than needed either.

constexpr uint8_t kVoltageSensorPin = A0;

// DS18B20 temperature sensor - see base_mega1.ino's own copy for the
// full reasoning on the pull-up requirement, the non-blocking read
// timing, and the sentinel convention. Pin differs from the Megas'
// (this Uno's own former SDA pin, A4, not digital pin 20).
constexpr uint8_t kDs18b20DataPin = A4;
constexpr unsigned long kTemperatureReadIntervalMs = 1000;
constexpr unsigned long kTemperatureConversionMs = 750;
constexpr int32_t kTemperatureInvalidDeciC = -9999;

// Cooling fan - see base_mega1.ino for the full reasoning on every
// constant below; values kept identical across every board with a
// fan. kFanPwmPin=9 is a genuinely free hardware PWM pin here, unlike
// the mast's own copy of this feature, which needed software PWM.
constexpr uint8_t kFanPwmPin = 9;
constexpr int32_t kFanOnTempDeciC = 350;
constexpr int32_t kFanOffTempDeciC = 300;
constexpr int32_t kFanMaxTempDeciC = 500;
constexpr uint8_t kFanMinDutyPercent = 30;

AccelStepper azimuthAxis(AccelStepper::DRIVER, kAzimuthStepPin, kAzimuthDirPin);
AccelStepper elevationAxis(AccelStepper::DRIVER, kElevationStepPin, kElevationDirPin);

// EBA-17-M planetary gearbox (120:1), same math as arm_mega2.ino's
// steps_per_joint_rev: 200 full steps * 1/16 microstepping * 120:1 =
// 384000 steps/rev = 1066.667 steps/deg.
constexpr float kStepsPerDeg = 384000.0f / 360.0f;

// Operational (post-deployment) range - see this file's header
// comment for why these are the numbers enforced here, not the
// spec's other pair. Each axis's calibration switch is assumed
// mounted at its own minimum - see kAzimuthMinDeg/kElevationMinDeg's
// use in serviceHoming() below.
constexpr float kAzimuthMinDeg = 15.0f;
constexpr float kAzimuthMaxDeg = 285.0f;
constexpr float kElevationMinDeg = 0.0f;
constexpr float kElevationMaxDeg = 180.0f;

// Copied directly from arm_mega2.ino - identical actuator (EBA-17-M,
// 120:1), already-vetted values, no reason to pick different ones.
constexpr float kMaxSpeedStepsPerSec = 2000.0f;
constexpr float kAccelStepsPerSec2 = 4000.0f;
constexpr float kHomingSpeedStepsPerSec = 400.0f;
// Safety cutoff: abort homing an axis that travels this many steps
// without ever triggering its limit switch, rather than driving it
// indefinitely - same reasoning as arm_mega2.ino/mast_uno3.ino's own
// copies of this constant. Sized against azimuth's 270 deg span (the
// larger of the two operational ranges here) with a comfortable
// margin: 350 deg here comfortably covers it, at
// kStepsPerDeg=1066.667 that's 373333 steps, ~933 sec (~15.6 min)
// worst-case at kHomingSpeedStepsPerSec - long, but this is the same
// heavily-geared actuator the arm's own homing margin already
// documents a ~30 minute worst case for; not a new problem introduced
// here.
constexpr long kHomingMaxTravelSteps = 373333;

bool driversEnabled = false;
bool homed = false;
bool homingInProgress = false;
int8_t homingAxisIndex = -1;  // 0 = azimuth, 1 = elevation
long homingStartPosition = 0;

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
void setGimbalEnabled(bool enabled) {
  driversEnabled = enabled;
  digitalWrite(kGimbalEnablePin, enabled ? LOW : HIGH);
}

bool azimuthLimitTriggered() { return digitalRead(kAzimuthLimitPin) == LOW; }
bool elevationLimitTriggered() { return digitalRead(kElevationLimitPin) == LOW; }

AccelStepper& axisByIndex(uint8_t i) { return (i == 0) ? azimuthAxis : elevationAxis; }
bool limitTriggeredByIndex(uint8_t i) { return (i == 0) ? azimuthLimitTriggered() : elevationLimitTriggered(); }
float minDegByIndex(uint8_t i) { return (i == 0) ? kAzimuthMinDeg : kElevationMinDeg; }

void startHoming() {
  setGimbalEnabled(true);
  homed = false;
  homingInProgress = true;
  homingAxisIndex = 0;
  AccelStepper& axis = axisByIndex(0);
  axis.setMaxSpeed(kHomingSpeedStepsPerSec);
  axis.setSpeed(-kHomingSpeedStepsPerSec);  // homing direction: toward the limit switch
  homingStartPosition = axis.currentPosition();
}

void serviceHoming() {
  if (!homingInProgress) return;
  uint8_t a = (uint8_t)homingAxisIndex;
  AccelStepper& axis = axisByIndex(a);

  bool traveledTooFar = labs(axis.currentPosition() - homingStartPosition) > kHomingMaxTravelSteps;

  if (limitTriggeredByIndex(a) || traveledTooFar) {
    axis.stop();
    // The switch is this axis's own operational minimum, not an
    // offset from it - setCurrentPosition() here directly establishes
    // the real angle, no further move needed to reach "home".
    axis.setCurrentPosition((long)(minDegByIndex(a) * kStepsPerDeg));
    axis.setMaxSpeed(kMaxSpeedStepsPerSec);
    axis.setAcceleration(kAccelStepsPerSec2);

    homingAxisIndex++;
    if (homingAxisIndex >= 2) {
      homingInProgress = false;
      homed = true;
      // Hold position exactly where homing left each axis (its own
      // minimum) rather than jump anywhere else - unlike the mast,
      // there's no separate centered "zero" to drive to here.
      azimuthAxis.moveTo(azimuthAxis.currentPosition());
      elevationAxis.moveTo(elevationAxis.currentPosition());
    } else {
      AccelStepper& next = axisByIndex((uint8_t)homingAxisIndex);
      next.setMaxSpeed(kHomingSpeedStepsPerSec);
      next.setSpeed(-kHomingSpeedStepsPerSec);
      homingStartPosition = next.currentPosition();
    }
    return;
  }

  axis.runSpeed();  // constant-speed seek, no accel profile while homing
}

void handleGimbalCommand(const RoverProtocol::ParsedFrame& frame) {
  if (frame.fieldCount != 3) return;  // malformed, ignore

  float azimuthDeg = frame.fields[0] / 10.0f;
  float elevationDeg = frame.fields[1] / 10.0f;
  bool driverEnable = frame.fields[2] != 0;

  // Both driver_enable and target application are gated on homed
  // alone - same reasoning as mast_uno3.ino's handleMastCommand():
  // the bridge sends command frames continuously at its own control
  // rate once past its one-shot homing request, using whatever's in
  // its last-known command (defaulting to driver_enable=false before
  // the operator has ever touched the antenna panel). Applying that
  // unconditionally would disable the drivers mid-seek, stranding a
  // stepper that can't move de-energized.
  if (homed) {
    setGimbalEnabled(driverEnable);
    azimuthDeg = constrain(azimuthDeg, kAzimuthMinDeg, kAzimuthMaxDeg);
    elevationDeg = constrain(elevationDeg, kElevationMinDeg, kElevationMaxDeg);
    azimuthAxis.moveTo((long)(azimuthDeg * kStepsPerDeg));
    elevationAxis.moveTo((long)(elevationDeg * kStepsPerDeg));
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
  int32_t azimuthDecideg = (int32_t)lround((azimuthAxis.currentPosition() / kStepsPerDeg) * 10.0f);
  int32_t elevationDecideg = (int32_t)lround((elevationAxis.currentPosition() / kStepsPerDeg) * 10.0f);
  int32_t fields[9] = {
      azimuthDecideg,
      elevationDecideg,
      azimuthLimitTriggered() ? 1 : 0,
      elevationLimitTriggered() ? 1 : 0,
      homed ? 1 : 0,
      readSupplyVoltageMv(),
      driversEnabled ? 1 : 0,
      cachedTemperatureDeciC,
      fanDutyPercent,
  };
  RoverProtocol::sendFrame(Serial, 'S', fields, 9);
}

// -------------------------------------------------------------- setup ---
void setup() {
  Serial.begin(115200);

  pinMode(kGimbalEnablePin, OUTPUT);
  setGimbalEnabled(true);  // starts enabled - homing needs the drivers energized to seek the limit switches
  pinMode(kAzimuthLimitPin, INPUT_PULLUP);
  pinMode(kElevationLimitPin, INPUT_PULLUP);

  azimuthAxis.setMaxSpeed(kMaxSpeedStepsPerSec);
  azimuthAxis.setAcceleration(kAccelStepsPerSec2);
  elevationAxis.setMaxSpeed(kMaxSpeedStepsPerSec);
  elevationAxis.setAcceleration(kAccelStepsPerSec2);

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
      if (frame.type == 'G') {
        handleGimbalCommand(frame);
        sendStateFrame();
      } else if (frame.type == 'Z') {
        startHoming();
        sendStateFrame();
      }
    }
  }

  if (homingInProgress) {
    serviceHoming();
  } else {
    azimuthAxis.run();
    elevationAxis.run();
  }

  if ((millis() - lastCommandMillis) > kWatchdogTimeoutMs) {
    if (homed) {
      // Comms lost: hold the current position rather than continuing
      // toward a stale target - same pattern as arm_mega2.ino/
      // mast_uno3.ino's own watchdog handling. homed is already false
      // throughout the entire homing-seek sequence, so this alone
      // correctly avoids freezing an axis mid-seek.
      azimuthAxis.moveTo(azimuthAxis.currentPosition());
      elevationAxis.moveTo(elevationAxis.currentPosition());
    }
  }
}
