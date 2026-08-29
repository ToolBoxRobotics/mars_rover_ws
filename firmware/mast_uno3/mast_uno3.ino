// mast_uno3.ino

#include <AccelStepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RoverProtocol.h>

constexpr uint8_t kNumHeadAxes = 2;  // yaw, pitch

constexpr uint8_t kStepPin[kNumHeadAxes] = {2, 4};   // yaw, pitch - PUL on the TB6600
constexpr uint8_t kDirPin[kNumHeadAxes] = {3, 5};    // yaw, pitch - DIR on the TB6600
constexpr uint8_t kHeadLimitPin[kNumHeadAxes] = {8, 10};  // yaw, pitch calibration switches

// Shared ENA for both TB6600 drivers (common-anode wiring: ENA+ to
// +5V, ENA- to this pin - LOW = enabled, matching the same
// active-low convention arm_mega2.ino's kEnablePin uses for its own
// shared A4988 enable). This is the last available Uno pin - see the
// file header for the full reasoning on why it's worth the pin.
constexpr uint8_t kHeadEnablePin = 13;

// HW-039 lift driver - RPWM/LPWM both need real PWM pins (D3/D5/D6/D9/
// D10/D11 on the Uno); EN doesn't, since it's just held HIGH once
// setup() enables it, not toggled per-command.
constexpr uint8_t kLiftRpwmPin = 6;
constexpr uint8_t kLiftLpwmPin = 9;
constexpr uint8_t kLiftEnPin = 7;

constexpr uint8_t kLimitStowedPin = 11;  // horizontal / transport position
constexpr uint8_t kLimitErectPin = 12;   // vertical / service position
constexpr uint8_t kLiftPwmDuty = 200;    // 0..255, tune to a safe erect/stow speed
// Every usable Uno pin (D2-D13) is now committed - see kHeadEnablePin
// above for why D13 specifically went to the head-axis enable rather
// than staying spare.

// FZ0430 voltage sensor (main battery supply) - see base_mega1.ino's
// own copy of this same sensor for the full conversion-math comment;
// kept identical across all three boards, including the pin choice.
constexpr uint8_t kVoltageSensorPin = A0;

// DS18B20 temperature sensor - see base_mega1.ino's own copy for the
// full reasoning on the pull-up requirement, the non-blocking read
// timing, and the sentinel convention. Pin differs from the Megas'
// (this Uno's own former SDA pin, A4, not digital pin 20).
constexpr uint8_t kDs18b20DataPin = A4;
constexpr unsigned long kTemperatureReadIntervalMs = 1000;
constexpr unsigned long kTemperatureConversionMs = 750;
constexpr int32_t kTemperatureInvalidDeciC = -9999;

// Cooling fan, via a generic MOSFET driver module - see the file
// header for the full wiring/design reasoning (low-side switch,
// software PWM, fail-toward-running on sensor failure).
constexpr uint8_t kFanPwmPin = A2;
// Software PWM period - 20ms (50Hz) is well within what a simple
// millis()-based toggle can hit accurately, and comfortably fast
// enough that a MOSFET-switched DC fan won't audibly click or buzz at
// the switching frequency itself, unlike very low frequencies (a few
// Hz) which can. Nowhere near needing the precision hardware PWM
// would give - this is a slowly-responding thermal load, not a
// stepper pulse train.
constexpr unsigned long kFanPwmPeriodMs = 20;
// Thermostat thresholds, all placeholders - bench-tune once the real
// enclosure's thermal behavior is known. kFanOffTempDeciC sits below
// kFanOnTempDeciC deliberately (hysteresis): once running, the fan
// stays on until temperature drops to the lower threshold, not the
// same one it turned on at, to avoid rapid on/off cycling right at a
// single boundary.
constexpr int32_t kFanOnTempDeciC = 320;    // 32.0 deg C - start spinning up
constexpr int32_t kFanOffTempDeciC = 300;   // 30.0 deg C - stop (lower than ON, not the same value)
constexpr int32_t kFanMaxTempDeciC = 350;   // 35.0 deg C - full speed at/above this
// Many small DC fans won't reliably start or stay spinning much below
// this - once the fan is running at all, its duty cycle is clamped to
// at least this, rather than ramping smoothly down to a near-zero
// duty right at kFanOnTempDeciC where it might stall or just buzz.
constexpr uint8_t kFanMinDutyPercent = 30;

AccelStepper headAxes[kNumHeadAxes] = {
    AccelStepper(AccelStepper::DRIVER, kStepPin[0], kDirPin[0]),  // yaw
    AccelStepper(AccelStepper::DRIVER, kStepPin[1], kDirPin[1]),  // pitch
};

// Steps-per-degree at each head axis (full steps * microstep factor *
// gear reduction / 360) - placeholder, tune once gearing is finalized.
// The microstep factor here MUST match whatever the TB6600's own DIP
// switches are physically set to (TB6600 sets microstepping and
// current limit via switches on the driver, not firmware, unlike some
// A4988 breakouts that expose MS1/MS2/MS3 pins) - update this if the
// DIP switches are ever changed, the same way arm_topology.yaml's
// steps_per_joint_rev has to track the arm's actual gearing.
constexpr float kStepsPerDegYaw = 341.3f;   // 200 * 16 / 360 * 5:1 gear, e.g.
constexpr float kStepsPerDegPitch = 341.3f;

constexpr float kMaxSpeedStepsPerSec = 2000.0f;
constexpr float kAccelStepsPerSec2 = 1600.0f;
constexpr float kHomingSpeedStepsPerSec = 2000.0f;
// Safety cutoff, same reasoning as arm_mega2.ino's kHomingMaxTravelSteps:
// abort homing an axis that travels this many steps without ever
// triggering its limit switch, rather than driving it indefinitely.
// Sized against the actual established ranges - pitch is now +-180
// deg (360 total, confirmed against real hardware and now the larger
// of the two - yaw's +-170/340 total was originally the bigger range
// before pitch's own real limit turned out to be wider than the
// initial 60 deg placeholder) - at 500 degrees of margin, comfortably
// over pitch's full range with room to spare, and the same single
// constant covers yaw's smaller range with even more margin. Worst-
// case time to abort a genuine fault: ~22200 / 200 =~ 111 seconds.
constexpr long kHomingMaxTravelSteps = 222000;

// Each axis's minimum (most-negative) bound - where its limit switch
// physically is. Homing seeks the switch, then recognizes that
// position as this minimum rather than zero (setCurrentPosition() in
// serviceHoming() below), so the subsequent move to true zero is a
// real, correctly-sized move toward center - not the no-op it would
// be if the switch were incorrectly treated as zero already. See
// servicePostCalibration() for that move.
constexpr float kYawMinDeg = -170.0f;
constexpr float kPitchMinDeg = -90.0f;

enum LiftState : int8_t { STATE_UNKNOWN = 0, STATE_TRANSPORT = 1, STATE_SERVICE = 2, STATE_MOVING = 3 };
enum LiftMode : int8_t { LIFT_STOW = -1, LIFT_HOLD = 0, LIFT_ERECT = 1 };

int8_t currentLiftState = STATE_UNKNOWN;
int8_t currentLiftMode = LIFT_HOLD;  // last commanded mode; re-applied every loop so limits are re-checked continuously

bool homed = false;
bool homingInProgress = false;
int8_t homingAxisIndex = -1;
long homingStartPosition = 0;

bool headDriversEnabled = true;

OneWire oneWire(kDs18b20DataPin);
DallasTemperature ds18b20(&oneWire);
enum TempReadState : int8_t { TEMP_IDLE = 0, TEMP_CONVERTING = 1 };
int8_t tempReadState = TEMP_IDLE;
unsigned long tempConversionStartMillis = 0;
int32_t cachedTemperatureDeciC = kTemperatureInvalidDeciC;
unsigned long lastTemperatureReadMillis = 0;

// Fan: fanRunning is the hysteresis latch (see updateFanControl()),
// fanDutyPercent (0-100) is what updateFanPwm() actually outputs.
// Decoupled from the thermostat decision on purpose - updateFanPwm()
// just outputs whatever duty it's told, regardless of why.
bool fanRunning = false;
uint8_t fanDutyPercent = 0;
unsigned long fanPwmCycleStartMillis = 0;

// Post-calibration sequence state - see kYawMinDeg/kPitchMinDeg
// and servicePostCalibration() for the full picture.
enum PostCalState : int8_t { POST_CAL_IDLE = 0, POST_CAL_TO_ZERO = 1, POST_CAL_DONE = 2 };
int8_t postCalState = POST_CAL_IDLE;

RoverProtocol::LineReader lineReader;
char lineBuf[RoverProtocol::kMaxLineLen];

unsigned long lastCommandMillis = 0;
constexpr unsigned long kWatchdogTimeoutMs = 1000;

// ------------------------------------------------------------- helpers ---
void setHeadEnabled(bool enabled) {
  headDriversEnabled = enabled;
  digitalWrite(kHeadEnablePin, enabled ? LOW : HIGH);  // active LOW - see kHeadEnablePin's own comment
}

bool stowedLimitTriggered() { return digitalRead(kLimitStowedPin) == LOW; }
bool erectLimitTriggered() { return digitalRead(kLimitErectPin) == LOW; }

// Wired NC-to-ground with INPUT_PULLUP, same convention as the arm's
// calibration switches: triggered == LOW.
bool headLimitTriggered(uint8_t axis) { return digitalRead(kHeadLimitPin[axis]) == LOW; }

void startHoming() {
  homed = false;
  homingInProgress = true;
  homingAxisIndex = 0;
  headAxes[0].setMaxSpeed(kHomingSpeedStepsPerSec);
  headAxes[0].setSpeed(-kHomingSpeedStepsPerSec);  // homing direction: toward the limit switch
  homingStartPosition = headAxes[0].currentPosition();
}

void serviceHoming() {
  if (!homingInProgress) return;
  uint8_t a = (uint8_t)homingAxisIndex;

  bool traveledTooFar =
      labs(headAxes[a].currentPosition() - homingStartPosition) > kHomingMaxTravelSteps;

  if (headLimitTriggered(a) || traveledTooFar) {
    headAxes[a].stop();
    // The switch is physically at this axis's minimum bound, not its
    // center - recognize that here rather than calling this position
    // zero. yaw is axis 0, pitch is axis 1 throughout this file.
    float minDeg = (a == 0) ? kYawMinDeg : kPitchMinDeg;
    float stepsPerDeg = (a == 0) ? kStepsPerDegYaw : kStepsPerDegPitch;
    headAxes[a].setCurrentPosition((long)(minDeg * stepsPerDeg));
    headAxes[a].setMaxSpeed(kMaxSpeedStepsPerSec);
    headAxes[a].setAcceleration(kAccelStepsPerSec2);

    homingAxisIndex++;
    if (homingAxisIndex >= kNumHeadAxes) {
      homingInProgress = false;
      // Not homed yet - homed becomes true once both axes actually
      // arrive at true zero below, not the instant their switches
      // trigger. That's what "home" means here: the centered
      // position reached *from* the minimum, not the minimum itself.
      postCalState = POST_CAL_TO_ZERO;
      headAxes[0].moveTo(0);
      headAxes[1].moveTo(0);
    } else {
      uint8_t next = (uint8_t)homingAxisIndex;
      headAxes[next].setMaxSpeed(kHomingSpeedStepsPerSec);
      headAxes[next].setSpeed(-kHomingSpeedStepsPerSec);
      homingStartPosition = headAxes[next].currentPosition();
    }
    return;
  }

  headAxes[a].runSpeed();  // constant-speed seek, no accel profile while homing
}

// Completes the calibration sequence once both axes actually arrive
// at true zero (not just get commanded toward it) - AccelStepper's
// distanceToGo()==0 is exactly that "arrived" signal. This is where
// "home" actually gets established: homed only becomes true here,
// once the axes are centered, not back in serviceHoming() when their
// switches first trigger (that position is each axis's minimum, not
// home - see kYawMinDeg/kPitchMinDeg). Called every loop() iteration;
// a no-op whenever nothing's in flight (IDLE/DONE) or the axes are
// still moving toward zero.
void servicePostCalibration() {
  if (postCalState != POST_CAL_TO_ZERO) return;
  bool bothArrived = (headAxes[0].distanceToGo() == 0) && (headAxes[1].distanceToGo() == 0);
  if (!bothArrived) return;

  homed = true;
  setHeadEnabled(false);
  postCalState = POST_CAL_DONE;
}

void driveLift(int8_t mode) {
  if (mode == LIFT_ERECT && !erectLimitTriggered()) {
    analogWrite(kLiftRpwmPin, kLiftPwmDuty);
    analogWrite(kLiftLpwmPin, 0);
    currentLiftState = STATE_MOVING;
  } else if (mode == LIFT_STOW && !stowedLimitTriggered()) {
    analogWrite(kLiftRpwmPin, 0);
    analogWrite(kLiftLpwmPin, kLiftPwmDuty);
    currentLiftState = STATE_MOVING;
  } else {
    analogWrite(kLiftRpwmPin, 0);
    analogWrite(kLiftLpwmPin, 0);
    if (erectLimitTriggered()) {
      currentLiftState = STATE_SERVICE;
    } else if (stowedLimitTriggered()) {
      currentLiftState = STATE_TRANSPORT;
    } else if (mode == LIFT_HOLD) {
      // Stopped mid-travel by request; state stays MOVING-last-known
      // rather than claiming a limit we haven't reached.
      if (currentLiftState != STATE_TRANSPORT && currentLiftState != STATE_SERVICE) {
        currentLiftState = STATE_UNKNOWN;
      }
    }
  }
}

void handleMastCommand(const RoverProtocol::ParsedFrame& frame) {
  if (frame.fieldCount != 4) return;  // malformed, ignore

  float yawDeg = frame.fields[0] / 10.0f;
  float pitchDeg = frame.fields[1] / 10.0f;
  int8_t liftMode = (int8_t)constrain(frame.fields[2], -1, 1);
  bool driverEnable = frame.fields[3] != 0;

  // Lift is independent of head-axis homing - it has no step-relative
  // position to zero, just directly-read limit switches - so it's
  // always allowed to move, even before yaw/pitch have homed.
  currentLiftMode = liftMode;
  driveLift(currentLiftMode);

  // Both driver_enable and yaw/pitch targets are ignored until homed
  // is true - which, since serviceHoming()/servicePostCalibration()
  // now only sets it once an axis has actually arrived at true zero
  // (not the instant its switch triggers), is already false for the
  // entire homing-seek-then-move-to-zero sequence. One flag is enough
  // to gate both here: the bridge sends 'M' frames continuously at
  // its own control rate once past its one-shot homing request, using
  // whatever's in its last-known command - which defaults to
  // driver_enable=false (a ROS bool field's default) before the
  // operator has ever touched the mast panel. Applying that
  // unconditionally, the way arm_mega2.ino applies its own enable
  // field, would disable the drivers mid-seek or mid-move-to-zero
  // from nothing more than the bridge's own routine resend, stranding
  // a stepper that can't move de-energized.
  if (homed) {
    setHeadEnabled(driverEnable);
    headAxes[0].moveTo((long)(yawDeg * kStepsPerDegYaw));
    headAxes[1].moveTo((long)(pitchDeg * kStepsPerDegPitch));
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

// Thermostat: decides fanRunning (the hysteresis latch) and
// fanDutyPercent from the current cached temperature. Cheap enough to
// call every loop() iteration without throttling, even though
// cachedTemperatureDeciC itself only actually changes roughly every
// 1.75s (see updateCachedTemperature()) - recomputing a few
// comparisons that often against an unchanged input costs nothing
// worth guarding against.
void updateFanControl() {
  if (cachedTemperatureDeciC == kTemperatureInvalidDeciC) {
    // Sensor not responding - fail toward running rather than off;
    // see this file's header comment for the reasoning.
    fanRunning = true;
    fanDutyPercent = kFanMinDutyPercent;
    return;
  }

  if (!fanRunning) {
    if (cachedTemperatureDeciC >= kFanOnTempDeciC) fanRunning = true;
  } else {
    if (cachedTemperatureDeciC <= kFanOffTempDeciC) fanRunning = false;
  }

  if (!fanRunning) {
    fanDutyPercent = 0;
    return;
  }

  if (cachedTemperatureDeciC >= kFanMaxTempDeciC) {
    fanDutyPercent = 100;
    return;
  }

  // Linear ramp from kFanMinDutyPercent (at kFanOnTempDeciC) to 100
  // (at kFanMaxTempDeciC). Clamped at zero because this branch is
  // also reached purely from hysteresis, with temperature anywhere
  // down to kFanOffTempDeciC (below kFanOnTempDeciC) - the raw ramp
  // position would otherwise go negative there.
  int32_t pos = cachedTemperatureDeciC - kFanOnTempDeciC;
  if (pos < 0) pos = 0;
  int32_t range = kFanMaxTempDeciC - kFanOnTempDeciC;
  fanDutyPercent = kFanMinDutyPercent + (uint8_t)(((100 - kFanMinDutyPercent) * pos) / range);
}

// Software PWM: no hardware timer/PWM pin available (see file header
// for why), so this bit-bangs it against millis() instead - called
// every loop() iteration, unlike updateFanControl() above, since
// accurate timing within a 20ms period actually depends on being
// checked frequently.
void updateFanPwm() {
  unsigned long now = millis();
  unsigned long elapsed = now - fanPwmCycleStartMillis;
  if (elapsed >= kFanPwmPeriodMs) {
    fanPwmCycleStartMillis = now;
    elapsed = 0;
  }
  unsigned long onTimeMs = ((unsigned long)fanDutyPercent * kFanPwmPeriodMs) / 100;
  digitalWrite(kFanPwmPin, (elapsed < onTimeMs) ? HIGH : LOW);
}

int32_t readSupplyVoltageMv() {
  int raw = analogRead(kVoltageSensorPin);
  return (int32_t)lround((raw / 1023.0f) * 25000.0f);
}

void sendStateFrame() {
  int32_t yawDecideg = (int32_t)lround((headAxes[0].currentPosition() / kStepsPerDegYaw) * 10.0f);
  int32_t pitchDecideg = (int32_t)lround((headAxes[1].currentPosition() / kStepsPerDegPitch) * 10.0f);
  int32_t fields[10] = {
      yawDecideg,
      pitchDecideg,
      currentLiftState,
      headLimitTriggered(0) ? 1 : 0,
      headLimitTriggered(1) ? 1 : 0,
      homed ? 1 : 0,
      readSupplyVoltageMv(),
      headDriversEnabled ? 1 : 0,
      cachedTemperatureDeciC,
      fanDutyPercent,
  };
  RoverProtocol::sendFrame(Serial, 'S', fields, 10);
}

// -------------------------------------------------------------- setup ---
void setup() {
  Serial.begin(115200);

  pinMode(kLiftRpwmPin, OUTPUT);
  pinMode(kLiftLpwmPin, OUTPUT);
  pinMode(kLiftEnPin, OUTPUT);
  digitalWrite(kLiftEnPin, HIGH);  // both half-bridges enabled; RPWM/LPWM do the actual work
  pinMode(kLimitStowedPin, INPUT_PULLUP);
  pinMode(kLimitErectPin, INPUT_PULLUP);
  driveLift(LIFT_HOLD);

  for (uint8_t i = 0; i < kNumHeadAxes; i++) {
    pinMode(kHeadLimitPin[i], INPUT_PULLUP);
    headAxes[i].setMaxSpeed(kMaxSpeedStepsPerSec);
    headAxes[i].setAcceleration(kAccelStepsPerSec2);
  }
  pinMode(kHeadEnablePin, OUTPUT);
  setHeadEnabled(true);  // starts enabled - homing needs the drivers energized to seek the limit switches

  ds18b20.begin();
  ds18b20.setWaitForConversion(false);  // non-blocking - see base_mega1.ino's own copy of this same setup() line

  pinMode(kFanPwmPin, OUTPUT);
  digitalWrite(kFanPwmPin, LOW);  // starts off - updateFanControl() takes over once a real temperature reading exists

  // Establish the lift's known state at boot without moving it, in
  // case it already sits on one of the two limit switches.
  if (erectLimitTriggered()) {
    currentLiftState = STATE_SERVICE;
  } else if (stowedLimitTriggered()) {
    currentLiftState = STATE_TRANSPORT;
  }

  lastCommandMillis = millis();
}

// --------------------------------------------------------------- loop ---
void loop() {
  updateCachedTemperature();
  updateFanControl();
  updateFanPwm();

  if (lineReader.poll(Serial, lineBuf, sizeof(lineBuf))) {
    RoverProtocol::ParsedFrame frame = RoverProtocol::parseFrame(lineBuf);
    if (frame.valid) {
      if (frame.type == 'M') {
        handleMastCommand(frame);
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
    for (uint8_t i = 0; i < kNumHeadAxes; i++) headAxes[i].run();
    servicePostCalibration();
  }

  // Re-apply the last commanded lift mode every loop (not just when a
  // new command arrives) so driveLift() keeps re-checking the limit
  // switches and cuts power the instant stow/erect travel completes.
  // Independent of homing state, same as handleMastCommand() above.
  driveLift(currentLiftMode);

  if ((millis() - lastCommandMillis) > kWatchdogTimeoutMs) {
    currentLiftMode = LIFT_HOLD;
    if (homed) {
      // Comms lost: hold the current head position rather than
      // continuing toward a stale target - same pattern as
      // arm_mega2.ino's watchdog handling. homed is already false
      // throughout the entire homing-seek-then-move-to-zero sequence
      // (see servicePostCalibration()), so this alone correctly
      // avoids freezing the head mid-sequence - entirely normal
      // during an automatic move nobody's actively driving - which
      // would otherwise strand it partway there, never reaching true
      // zero or disabling the drivers.
      headAxes[0].moveTo(headAxes[0].currentPosition());
      headAxes[1].moveTo(headAxes[1].currentPosition());
    }
  }
}
