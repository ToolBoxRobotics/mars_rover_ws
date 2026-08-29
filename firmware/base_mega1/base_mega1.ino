// base_mega1.ino

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RoverProtocol.h>

// ---------------------------------------------------------------- pins ---
// Wheel order used for driving/steering below: FL, FR, ML, MR, RL, RR
constexpr uint8_t kNumWheels = 6;
constexpr uint8_t kNumSteer = 4;  // FL, FR, RL, RR

// Only ML and MR are physically encoded - see rover_base/odometry.py's
// module docstring for the full reasoning: they're the only pair whose
// rolling axis never changes with corner steering, so they're the only
// pair standard differential-drive wheel odometry actually needs.
// Wiring individual encoders on the 4 steerable corners would add
// hardware complexity (and firmware complexity - see the PCINT note
// this used to require for RL/RR before they were removed) for data
// nothing consumes.
constexpr uint8_t kNumEncoders = 2;  // ML, MR
constexpr uint8_t kEncA[kNumEncoders] = {18, 19};   // ML, MR channel A
constexpr uint8_t kEncB[kNumEncoders] = {24, 25};   // ML, MR channel B
// Pins 18/19 are INT5/INT4 on the Mega - true external-interrupt-capable
// pins, so plain attachInterrupt() is enough; no PCINT/port-register
// wrangling needed (that was only required for the since-removed RL/RR
// encoders, which lived on A8/A9).

// FZ0430 voltage sensors (drive rail + steering rail, one each): a
// passive 5:1 resistive divider bringing a 0-25V input down into the
// Mega's 0-5V ADC range, signal output on one analog pin. Every
// analog pin was free before this, so A0 is an arbitrary but
// permanent choice, not one forced by pin scarcity - keep it
// consistent with arm_mega2.ino/mast_uno3.ino, which use the same
// pin for their own supply sensors.
//
// UPDATED: this board now has two FZ0430s, not one - a second unit
// added on A1, same configuration as this one (same 5:1 divider, same
// conversion math), specifically to give the drive motors' own supply
// rail and the steering servos' own supply rail independent voltage
// readings rather than one shared value covering both. This first
// sensor is now explicitly the DRIVE rail's own reading, not a
// generic "main supply" the way it was before the second unit existed
// - see kSteeringVoltageSensorPin below for the second.
constexpr uint8_t kDriveVoltageSensorPin = A0;
// Second FZ0430, steering servos' own supply rail - see the comment
// on kDriveVoltageSensorPin above for the full reasoning. Same
// configuration as the first unit, deliberately: same 5:1 divider,
// same conversion math, same wiring pattern - the only real
// difference between the two is which physical rail each is actually
// connected to.
constexpr uint8_t kSteeringVoltageSensorPin = A1;

// DS18B20 temperature sensor, 1-Wire on a single digital pin. Was the
// Mega's former I2C SDA pin (20) until the PCA9685 below needed that
// pin back for genuine I2C - moved to A4, one of the pins the
// steering servos vacated moving to that same PCA9685, rather than
// search out a third free pin.
constexpr uint8_t kDs18b20DataPin = A4;
// How often to START a new conversion cycle, milliseconds - not on
// every state frame (~20Hz) the way voltage is, since board
// temperature changes over seconds-to-minutes, not tens-of-milliseconds;
// polling that often would just be wasted bus traffic for a value
// that's the same reading, over and over.
constexpr unsigned long kTemperatureReadIntervalMs = 1000;
// Worst-case 12-bit-resolution conversion time per the DS18B20
// datasheet - how long to wait after requestTemperatures() before the
// result is actually ready. Read via a non-blocking two-phase state
// machine (see updateCachedTemperature()) rather than the library's
// own default blocking `delay(750)` pattern - see this file's header
// comment for why blocking loop() for 750ms on every read isn't
// acceptable here.
constexpr unsigned long kTemperatureConversionMs = 750;
// Sentinel sent over the wire when the sensor didn't respond on its
// most recent read - -999.9 deg C in decidegrees, comfortably outside
// the DS18B20's real -55 to +125 deg C range, so it can't be confused
// with an actual (if extreme) reading. Chosen over a separate boolean
// "temperature valid" field to avoid growing every consumer of this
// value (protocol, message, bridge node, web GUI) by one more field
// each - one clearly-impossible number carries the same information.
// Checked fresh on every read cycle (DallasTemperature's own
// DEVICE_DISCONNECTED_C, not a one-time boot check the way the BMP280
// this replaced only checked once at setup()) - a sensor that's
// disconnected and later reconnected recovers on its own next cycle,
// it doesn't need a reset.
constexpr int32_t kTemperatureInvalidDeciC = -9999;

// Cooling fan - see mast_uno3.ino for the full reasoning on every
// constant below (thermostat design, hysteresis, sensor-failure
// fail-safe); values kept identical across every board with a fan,
// same as the temperature-sensor constants above. kFanPwmPin differs
// per board only because pin availability differs per board - see
// this file's own header comment for why 44 specifically here.
constexpr uint8_t kFanPwmPin = 44;
constexpr int32_t kFanOnTempDeciC = 350;
constexpr int32_t kFanOffTempDeciC = 300;
constexpr int32_t kFanMaxTempDeciC = 500;
constexpr uint8_t kFanMinDutyPercent = 30;

// Motor driver pins: one direction pin (Mx) + one PWM/speed pin (Ex)
// per DRI0002 channel - see the file header for the M-pin convention
// and the logic-supply jumper note. Three boards x two channels each:
// board 1 -> FL,FR; board 2 -> ML,MR; board 3 -> RL,RR. All 6 wheels
// are driven regardless of which are encoded.
constexpr uint8_t kMotorPwm[kNumWheels] = {4, 5, 6, 7, 8, 9};        // Ex
constexpr uint8_t kMotorDir[kNumWheels] = {28, 29, 30, 31, 32, 33};  // Mx

// PCA9685 16-channel PWM driver, I2C - drives the 4 steering servos.
// CORRECTED from an earlier session's own mistake: 0x70 is NOT a
// normal, jumper-configurable address on this chip - it's the
// PCA9685's built-in "LED All Call" address, enabled by every unit at
// power-up regardless of A0-A5 jumper state (confirmed in the chip's
// own datasheet, and Adafruit's own FAQ explicitly warns against
// configuring a device to that same address for exactly this reason).
// This project only ever has one PCA9685, so there's no reason to
// move it off its factory default at all - 0x40, unjumpered, no
// solder pads to bridge.
constexpr uint8_t kPca9685Address = 0x40;
constexpr uint8_t kSteerChannel[kNumSteer] = {0, 1, 2, 3};  // FL,FR,RL,RR -> PCA9685 channels
// Index into the 6-wide wheel arrays that each steer channel corresponds to,
// used only for readability/debug prints.
constexpr uint8_t kSteerWheelIndex[kNumSteer] = {0, 1, 4, 5};  // FL,FR,RL,RR

Adafruit_PWMServoDriver pwm(kPca9685Address);

// Easing rate for steering, microseconds/second - a direct translation
// of the previous ServoEasing-based rate (300 deg/sec) into this
// board's own pulse-width terms, not a re-guess: at this project's
// placeholder calibration (900us neutral-to-max span for a 90 deg
// swing, i.e. ~10us/deg), 300 deg/sec times 10us/deg is ~3000us/sec,
// preserving the same "a full +-60 deg swing takes ~0.4s" feel the
// previous constant's own comment described. Same placeholder-
// pending-bench-tuning status as every other uncalibrated constant in
// this project - revisit once kServoMinUs/kServoMaxUs/kServoNeutralUs
// below are themselves bench-calibrated, since this rate was derived
// from their current placeholder values specifically.
constexpr float kSteerEaseSpeedUsPerSec = 3000.0f;

// ------------------------------------------------------------- state ---
volatile int32_t encoderTicks[kNumEncoders] = {0, 0};  // ML, MR

OneWire oneWire(kDs18b20DataPin);
DallasTemperature ds18b20(&oneWire);
enum TempReadState : int8_t { TEMP_IDLE = 0, TEMP_CONVERTING = 1 };
int8_t tempReadState = TEMP_IDLE;
unsigned long tempConversionStartMillis = 0;
int32_t cachedTemperatureDeciC = kTemperatureInvalidDeciC;
unsigned long lastTemperatureReadMillis = 0;

bool fanRunning = false;
uint8_t fanDutyPercent = 0;

// Servo neutral/travel calibration, per corner (FL, FR, RL, RR - same
// order as kSteerChannel). 40kg metal-gear servos commonly want a
// wider-than-hobby-standard pulse range, and real units - even same
// model, same batch - can vary enough in their actual center/travel
// that a single shared calibration was only ever a starting point,
// not a real bench calibration. All four start at the same
// placeholder values below on purpose - that's the safe default until
// each corner is actually bench-tested, not a mistake to fix here -
// then tune each servo's own entry independently as its real travel
// is measured, without touching the other three.
constexpr float kServoMinUs[kNumSteer] = {600.0f, 600.0f, 600.0f, 600.0f};
constexpr float kServoMaxUs[kNumSteer] = {2400.0f, 2400.0f, 2400.0f, 2400.0f};
constexpr float kServoNeutralUs[kNumSteer] = {1500.0f, 1500.0f, 1500.0f, 1500.0f};

// Custom easing state - see updateSteerEasing() below. steerTargetUs
// is where setSteerAngle() wants each corner to end up; steerCurrentUs
// is where it actually, physically is right now (in this firmware's
// own tracking - the PCA9685 has no position feedback of its own),
// ramping toward the target a little each loop() iteration rather
// than jumping there in one write().
float steerTargetUs[kNumSteer];
float steerCurrentUs[kNumSteer];
unsigned long lastSteerEaseMillis = 0;

constexpr unsigned long kWatchdogTimeoutMs = 500;
unsigned long lastCommandMillis = 0;

RoverProtocol::LineReader lineReader;
char lineBuf[RoverProtocol::kMaxLineLen];

// ---------------------------------------------------- encoder helpers ---
inline void handleEncoderEdge(uint8_t encoderIndex, bool aState, bool bState) {
  // Standard quadrature decode: on A's rising edge, B high means one
  // direction, B low means the other. (Swap the sign below if a wheel
  // reports the wrong direction on the bench - do not rewire.)
  if (aState) {
    encoderTicks[encoderIndex] += bState ? -1 : 1;
  } else {
    encoderTicks[encoderIndex] += bState ? 1 : -1;
  }
}

void isrEncML() { handleEncoderEdge(0, digitalRead(kEncA[0]), digitalRead(kEncB[0])); }
void isrEncMR() { handleEncoderEdge(1, digitalRead(kEncA[1]), digitalRead(kEncB[1])); }

// ------------------------------------------------------- motor output ---
void setWheelThrottle(uint8_t wheel, int32_t throttle) {
  throttle = constrain(throttle, -1000, 1000);
  uint8_t pwmVal = (uint8_t)(abs(throttle) * 255L / 1000L);

  // DRI0002's documented convention is Mx=LOW -> "forward"; flip this
  // comparison (not the wiring) if a wheel bench-tests backward. At
  // pwmVal == 0 no current flows regardless of the direction pin, so
  // throttle == 0 already coasts without any special-casing here.
  digitalWrite(kMotorDir[wheel], throttle < 0 ? HIGH : LOW);
  analogWrite(kMotorPwm[wheel], pwmVal);
}

void setSteerAngle(uint8_t steerIndex, float angleDeg) {
  angleDeg = constrain(angleDeg, -90.0f, 90.0f);
  float neutral = kServoNeutralUs[steerIndex];
  // Use the min-side span for negative angles and the max-side span
  // for positive ones, rather than always scaling off (max - neutral)
  // - those two spans are only equal if neutral happens to sit exactly
  // centered between min and max. A real bench-calibrated servo often
  // won't: e.g. min=600/max=2400/neutral=1450 has a 850us min-side span
  // and a 950us max-side span, not one shared 900us span either
  // direction. Using the wrong span doesn't just misreport the angle -
  // it silently shrinks the achievable range on whichever side is
  // shorter, since the final constrain() below then clips to a pulse
  // width that no longer corresponds to the actual commanded angle.
  float span = (angleDeg >= 0.0f)
      ? (kServoMaxUs[steerIndex] - neutral)
      : (neutral - kServoMinUs[steerIndex]);
  float us = neutral + (angleDeg / 90.0f) * span;
  us = constrain(us, kServoMinUs[steerIndex], kServoMaxUs[steerIndex]);
  // Just records where this corner should end up - the actual
  // movement happens incrementally in updateSteerEasing() below, not
  // here, so this function itself is effectively instantaneous no
  // matter how far the target moved.
  steerTargetUs[steerIndex] = us;
}

// Non-blocking custom easing, replacing ServoEasing's own
// startEaseTo()/timer-interrupt approach - see this file's header
// comment for why a PCA9685 needs this reimplemented rather than
// reused. Moves each corner's steerCurrentUs a little closer to its
// steerTargetUs every call, at kSteerEaseSpeedUsPerSec, and writes the
// result to the PCA9685 - called every loop() iteration for smooth,
// frequent updates, the same way ServoEasing's own interrupt-driven
// updates were effectively continuous.
void updateSteerEasing() {
  unsigned long now = millis();
  float elapsedSec = (now - lastSteerEaseMillis) / 1000.0f;
  lastSteerEaseMillis = now;
  float maxStepUs = kSteerEaseSpeedUsPerSec * elapsedSec;

  for (uint8_t i = 0; i < kNumSteer; i++) {
    float delta = steerTargetUs[i] - steerCurrentUs[i];
    if (delta > maxStepUs) {
      steerCurrentUs[i] += maxStepUs;
    } else if (delta < -maxStepUs) {
      steerCurrentUs[i] -= maxStepUs;
    } else {
      steerCurrentUs[i] = steerTargetUs[i];  // within one step - snap the remainder rather than asymptotically crawl the last fraction of a microsecond forever
    }
    pwm.writeMicroseconds(kSteerChannel[i], (uint16_t)steerCurrentUs[i]);
  }
}

void stopAllMotors() {
  for (uint8_t i = 0; i < kNumWheels; i++) setWheelThrottle(i, 0);
}

// ------------------------------------------------------------ frames ---
void updateCachedTemperature() {
  unsigned long now = millis();
  if (tempReadState == TEMP_IDLE) {
    if ((now - lastTemperatureReadMillis) < kTemperatureReadIntervalMs) return;
    ds18b20.requestTemperatures();  // non-blocking - setWaitForConversion(false) set in setup()
    tempConversionStartMillis = now;
    tempReadState = TEMP_CONVERTING;
  } else {  // TEMP_CONVERTING
    if ((now - tempConversionStartMillis) < kTemperatureConversionMs) return;  // not ready yet
    float c = ds18b20.getTempCByIndex(0);
    cachedTemperatureDeciC =
        (c == DEVICE_DISCONNECTED_C) ? kTemperatureInvalidDeciC : (int32_t)lround(c * 10.0f);
    lastTemperatureReadMillis = now;
    tempReadState = TEMP_IDLE;
  }
}

// Thermostat + hardware PWM output in one - see mast_uno3.ino's own
// updateFanControl() for the full reasoning behind every constant and
// branch here; logic is identical, only the output mechanism differs
// (analogWrite() directly, since this board has a genuine hardware
// PWM pin to spare, rather than the mast's software-PWM workaround).
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

int32_t readVoltageMv(uint8_t pin) {
  // (raw / 1023.0) * 5.0V is the conventional Arduino ADC-to-volts
  // conversion (1023 as the full-scale calibration point, not the
  // ADC's literal 5V/1024 quantization width - this is the
  // near-universal community convention, and matches the FZ0430's own
  // commonly quoted 0.00489V/step resolution figure: 5/1023 = 0.004888V,
  // vs 5/1024 = 0.004883V). *5.0 undoes the divider to recover the
  // real input voltage; result in millivolts to keep the wire protocol
  // integer-only, matching every other field. Takes a pin argument,
  // not hardcoded to one sensor, since this board now has two
  // identically-configured FZ0430s (drive rail, steering rail) - one
  // function, called twice, rather than duplicating this same math a
  // second time for no reason.
  int raw = analogRead(pin);
  return (int32_t)lround((raw / 1023.0f) * 25000.0f);
}

void sendEncoderFrame() {
  int32_t snapshot[kNumEncoders + 4];
  noInterrupts();
  memcpy(snapshot, (const void*)encoderTicks, sizeof(int32_t) * kNumEncoders);
  interrupts();
  snapshot[kNumEncoders] = readVoltageMv(kDriveVoltageSensorPin);
  snapshot[kNumEncoders + 1] = readVoltageMv(kSteeringVoltageSensorPin);
  snapshot[kNumEncoders + 2] = cachedTemperatureDeciC;
  snapshot[kNumEncoders + 3] = fanDutyPercent;
  RoverProtocol::sendFrame(Serial, 'E', snapshot, kNumEncoders + 4);
}

void handleDriveFrame(const RoverProtocol::ParsedFrame& frame) {
  if (frame.fieldCount != (kNumWheels + kNumSteer)) return;  // malformed, ignore
  for (uint8_t i = 0; i < kNumWheels; i++) {
    setWheelThrottle(i, frame.fields[i]);
  }
  for (uint8_t i = 0; i < kNumSteer; i++) {
    float angleDeg = frame.fields[kNumWheels + i] / 10.0f;  // decidegrees -> degrees
    setSteerAngle(i, angleDeg);
  }
  lastCommandMillis = millis();
}

// -------------------------------------------------------------- setup ---
void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < kNumWheels; i++) {
    pinMode(kMotorPwm[i], OUTPUT);
    pinMode(kMotorDir[i], OUTPUT);
  }
  for (uint8_t i = 0; i < kNumEncoders; i++) {
    pinMode(kEncA[i], INPUT_PULLUP);
    pinMode(kEncB[i], INPUT_PULLUP);
  }
  stopAllMotors();

  attachInterrupt(digitalPinToInterrupt(kEncA[0]), isrEncML, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kEncA[1]), isrEncMR, CHANGE);

  // Same intent as the old ServoEasing setup this replaced: a servo's
  // real physical position at power-on is unknown to this firmware
  // either way, so seed both easing state variables at this corner's
  // own calibrated neutral (not e.g. 0us, which isn't even a valid
  // pulse width) and write it immediately, rather than let the first
  // real command jump from an undefined starting point.
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);  // standard analog-servo refresh rate
  for (uint8_t i = 0; i < kNumSteer; i++) {
    steerCurrentUs[i] = kServoNeutralUs[i];
    steerTargetUs[i] = kServoNeutralUs[i];
    pwm.writeMicroseconds(kSteerChannel[i], (uint16_t)kServoNeutralUs[i]);
  }
  lastSteerEaseMillis = millis();

  ds18b20.begin();
  ds18b20.setWaitForConversion(false);
  // No boot-time "found" check the way the BMP280 this replaced had -
  // DallasTemperature reports DEVICE_DISCONNECTED_C on every read that
  // fails, not just the first one, so updateCachedTemperature() checks
  // fresh each cycle instead (see its own comment). This board's real
  // job (driving) doesn't depend on temperature telemetry either way -
  // a missing/failed sensor stays a visible but non-fatal condition.

  pinMode(kFanPwmPin, OUTPUT);
  analogWrite(kFanPwmPin, 0);  // starts off - updateFanControl() takes over once a real temperature reading exists

  lastCommandMillis = millis();
}

// --------------------------------------------------------------- loop ---
void loop() {
  updateCachedTemperature();
  updateFanControl();
  updateSteerEasing();

  if (lineReader.poll(Serial, lineBuf, sizeof(lineBuf))) {
    RoverProtocol::ParsedFrame frame = RoverProtocol::parseFrame(lineBuf);
    if (frame.valid) {
      if (frame.type == 'D') {
        handleDriveFrame(frame);
        sendEncoderFrame();
      } else if (frame.type == 'H') {
        lastCommandMillis = millis();
        sendEncoderFrame();
      }
    }
    // Invalid frames (bad checksum/garbage) are silently dropped; the
    // watchdog below will stop the rover if valid commands stop arriving.
  }

  if ((millis() - lastCommandMillis) > kWatchdogTimeoutMs) {
    stopAllMotors();
  }
}
