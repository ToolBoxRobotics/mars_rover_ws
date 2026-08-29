// microscope_uno4.ino

#include <AccelStepper.h>
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RoverProtocol.h>

constexpr uint8_t kStepperStepPin = 8;
constexpr uint8_t kStepperDirPin = 9;
constexpr uint8_t kStepperEnablePin = 10;  // active LOW, same convention as this project's A4988/TB6600 enables
AccelStepper focusStepper(AccelStepper::DRIVER, kStepperStepPin, kStepperDirPin);

constexpr uint8_t kLedPin = 6;      // PWM-capable
constexpr uint8_t kCoverServoPin = 5;

// DS18B20 temperature sensor - see base_mega1.ino for the full
// reasoning on the pull-up requirement, the non-blocking read timing,
// and the sentinel convention; identical constants here. Pin 11 was
// genuinely spare (DRV8825 needs 3 control pins where ULN2003 needed
// 4) before this - see this file's header comment.
constexpr uint8_t kDs18b20DataPin = 11;
constexpr unsigned long kTemperatureReadIntervalMs = 1000;
constexpr unsigned long kTemperatureConversionMs = 750;
constexpr int32_t kTemperatureInvalidDeciC = -9999;

// Cooling fan - see base_mega1.ino for the full reasoning on every
// constant below; values kept identical across every board with a
// fan. Pin 3 is the other pin that was genuinely spare here, and
// happens to be PWM-capable - no software-PWM workaround needed,
// same as base/arm/antenna.
constexpr uint8_t kFanPwmPin = 3;
constexpr int32_t kFanOnTempDeciC = 320;
constexpr int32_t kFanOffTempDeciC = 300;
constexpr int32_t kFanMaxTempDeciC = 350;
constexpr uint8_t kFanMinDutyPercent = 30;

constexpr float kMaxSpeedStepsPerSec = 500.0f;  // 24BYJ-48 is geared; keep this modest until bench-verified against the actual DIP/jumper microstepping setting
constexpr float kAccelStepsPerSec2 = 800.0f;

constexpr int kCoverClosedDeg = 41;
constexpr int kCoverOpenDeg = 146;
// Slower than the base's steering easing (300 deg/sec) on purpose -
// this is a binary open/closed toggle triggered rarely, not a
// continuously-recommanded value, so there's no responsiveness
// pressure the way there is for steering. A gentler, more visibly
// eased motion actually reads better for a physical cover sliding
// open. Same placeholder-pending-bench-tuning status as every other
// uncalibrated constant in this project. UPDATED: now consumed by
// this file's own custom ramp (updateCoverEasing() below) rather
// than ServoEasing's startEaseTo() - same unit (degrees/second), same
// meaning, just a different mechanism reading it now.
constexpr uint16_t kCoverEaseSpeedDegPerSec = 60;

Servo coverServo;
// Custom easing state - see base_mega1.ino's own
// steerTargetUs/steerCurrentUs for the identical pattern this mirrors,
// just for one servo instead of four, and degrees instead of
// microseconds since this is still a direct-pin Servo, not a PCA9685.
// coverTargetDeg is where the latest command wants the cover; 
// coverCurrentDeg is where this firmware's own tracking believes it
// actually is right now, ramping toward the target a little each
// loop() iteration rather than jumping there in one write().
float coverTargetDeg = kCoverClosedDeg;
float coverCurrentDeg = kCoverClosedDeg;
unsigned long lastCoverEaseMillis = 0;
bool coverOpenCommanded = false;
bool driverEnabled = true;
uint8_t lastLedPwm = 0;

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

// ------------------------------------------------------------- helpers ---
void setDriverEnabled(bool enabled) {
  driverEnabled = enabled;
  digitalWrite(kStepperEnablePin, enabled ? LOW : HIGH);
}

// Non-blocking custom easing, replacing ServoEasing's own
// startEaseTo()/timer-interrupt approach - see this file's header
// comment for the full reasoning, and base_mega1.ino's own
// updateSteerEasing() for the pattern this mirrors almost exactly.
// Moves coverCurrentDeg a little closer to coverTargetDeg every call,
// at kCoverEaseSpeedDegPerSec, and writes the result to the servo -
// called every loop() iteration for smooth, frequent updates, the
// same way ServoEasing's own interrupt-driven updates were
// effectively continuous.
void updateCoverEasing() {
  unsigned long now = millis();
  float elapsedSec = (now - lastCoverEaseMillis) / 1000.0f;
  lastCoverEaseMillis = now;
  float maxStepDeg = kCoverEaseSpeedDegPerSec * elapsedSec;

  float delta = coverTargetDeg - coverCurrentDeg;
  if (delta > maxStepDeg) {
    coverCurrentDeg += maxStepDeg;
  } else if (delta < -maxStepDeg) {
    coverCurrentDeg -= maxStepDeg;
  } else {
    coverCurrentDeg = coverTargetDeg;  // within one step - snap the remainder rather than asymptotically crawl the last fraction of a degree forever
  }
  coverServo.write((int)lround(coverCurrentDeg));
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

void handleCommand(const RoverProtocol::ParsedFrame& frame) {
  if (frame.fieldCount != 4) return;  // malformed, ignore

  int32_t focusTarget = frame.fields[0];
  uint8_t ledPwm = (uint8_t)constrain(frame.fields[1], 0, 255);
  bool coverOpen = frame.fields[2] != 0;
  bool driverEnable = frame.fields[3] != 0;

  setDriverEnabled(driverEnable);
  focusStepper.moveTo(focusTarget);
  analogWrite(kLedPin, ledPwm);

  if (coverOpen != coverOpenCommanded) {
    coverTargetDeg = coverOpen ? kCoverOpenDeg : kCoverClosedDeg;
    coverOpenCommanded = coverOpen;
  }
}

void sendStateFrame() {
  int32_t fields[7] = {
      focusStepper.currentPosition(),
      (int32_t)0,  // filled below with the last commanded LED PWM
      coverOpenCommanded ? 1 : 0,
      1,  // homed: always true, no calibration switch on this axis
      driverEnabled ? 1 : 0,
      cachedTemperatureDeciC,
      fanDutyPercent,
  };
  fields[1] = lastLedPwm;
  RoverProtocol::sendFrame(Serial, 'S', fields, 7);
}

// -------------------------------------------------------------- setup ---
void setup() {
  Serial.begin(115200);

  pinMode(kLedPin, OUTPUT);
  analogWrite(kLedPin, 0);

  pinMode(kStepperEnablePin, OUTPUT);
  setDriverEnabled(false);  // start disabled, same convention as the arm/mast boards - a command has to explicitly enable it

  // Same intent as the ServoEasing-based setup this replaced: a
  // servo's real physical position at power-on is unknown to this
  // firmware either way, so seed both easing state variables at the
  // closed position explicitly and write it immediately, rather than
  // let the first real command jump from an undefined starting point
  // - see base_mega1.ino's own copy of this same reasoning for its
  // steering servos.
  coverServo.attach(kCoverServoPin);
  coverServo.write(kCoverClosedDeg);
  coverCurrentDeg = kCoverClosedDeg;
  coverTargetDeg = kCoverClosedDeg;
  coverOpenCommanded = false;
  lastCoverEaseMillis = millis();

  focusStepper.setMaxSpeed(kMaxSpeedStepsPerSec);
  focusStepper.setAcceleration(kAccelStepsPerSec2);

  ds18b20.begin();
  ds18b20.setWaitForConversion(false);  // non-blocking - see base_mega1.ino's own copy of this same setup() line

  pinMode(kFanPwmPin, OUTPUT);
  analogWrite(kFanPwmPin, 0);  // starts off - updateFanControl() takes over once a real temperature reading exists
}

// --------------------------------------------------------------- loop ---
void loop() {
  updateCachedTemperature();
  updateFanControl();
  updateCoverEasing();

  if (lineReader.poll(Serial, lineBuf, sizeof(lineBuf))) {
    RoverProtocol::ParsedFrame frame = RoverProtocol::parseFrame(lineBuf);
    if (frame.valid && frame.type == 'C') {
      lastLedPwm = (uint8_t)constrain(frame.fields[1], 0, 255);
      handleCommand(frame);
      sendStateFrame();
    }
  }

  focusStepper.run();
}
