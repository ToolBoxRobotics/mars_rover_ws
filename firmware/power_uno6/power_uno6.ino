// power_uno6.ino
//
// Arduino Uno #6 - power/environmental monitoring board. Unlike
// every other board in this project, this one commands nothing: it's
// a pure telemetry source (2x battery voltage+current, onboard
// computer temperature) plus one automatic, thermostatic actuator
// (a cooling fan for that computer). No steppers, no servos, no
// operator-facing command frame at all - see the wire protocol note
// near the bottom of this file for what that changes structurally
// compared to every other board here.
//
// UPDATED from this board's original design: the two FZ0430 voltage
// sensors and the single shared ACS712 current sensor were replaced
// with two INA226 voltage+current sensors behind a TCA9548A I2C
// multiplexer - see the dedicated section below for the full
// reasoning. Removing the FZ0430s also removes the safety concern
// that came with them (a 25V hard ceiling on a nominal 24V pack that
// could plausibly exceed it) - the INA226's own 36V bus range gives
// genuine headroom instead, not a replaced-but-still-tight margin.
//
// UPDATED AGAIN: this board was originally an Arduino Nano
// (power_nano6.ino) - swapped for an Uno after repeated, unresolved
// hardware trouble with the Nano units in hand (see README's
// "Explicit assumptions" for what's known about the specific
// symptoms). The Nano and Uno share the same ATmega328P, the same
// D2-D13 digital range, the same D3/D5/D6/D9/D10/D11 PWM pins, and
// the same A4/A5 I2C pins - every pin this board's own firmware
// actually uses - so this swap needed no pin reassignment at all,
// only the board-type-specific concerns below (udev VID:PID, the
// physical form factor, and this file's own name). The Nano's two
// bonus analog-only pins (A6/A7), which the Uno lacks, were never
// used by this board's own sensor suite either way.
//
// Hardware:
//   * 2x INA226 voltage+current monitors, one per 24V battery, behind
//     a TCA9548A 8-channel I2C multiplexer (battery 1 on mux channel
//     0, battery 2 on channel 1) - both INA226 units at their shared
//     default I2C address (0x40, A0/A1 both grounded), which the mux
//     is specifically what makes possible: without it, two identical,
//     unmodified INA226 breakouts couldn't coexist on one bus at all.
//     TCA9548A itself at its own default address (0x70, unjumpered) -
//     see the dedicated I2C ADDRESSING section below for why 0x70
//     showing up here isn't the same situation as the PCA9685's own
//     built-in 0x70 on the base board, despite the coincidence.
//     Bus voltage range 0-36V - genuine headroom above the ~28-29V
//     worst case a nominal 24V pack can realistically reach (see
//     README's "Explicit assumptions" for the full reasoning this
//     board's earlier FZ0430s never had). SEE THE SHUNT RESISTOR
//     WARNING BELOW before trusting current readings from either
//     unit - this is a real, must-verify hardware detail, not routine
//     calibration.
//   * 1x DS18B20 temperature sensor, TO-92, 1-Wire on pin D2 -
//     monitoring the onboard COMPUTER's temperature specifically
//     (wherever this sensor is actually thermally coupled to it),
//     not this Uno's own enclosure the way every other board's
//     DS18B20 monitors its own - see base_mega1.ino's own copy of
//     this sensor for the full reasoning (external pull-up
//     requirement, non-blocking read state machine, sentinel
//     convention); identical here, just measuring a different thing.
//   * Cooling fan via a generic N-channel MOSFET driver module (D3,
//     PWM) - same automatic, thermostatic design as every other fan
//     in this project (see base_mega1.ino's header for the full
//     reasoning: low-side-switch wiring, hysteresis thresholds,
//     fail-toward-running on sensor failure), just cooling the
//     computer instead of a board enclosure. Genuine hardware PWM
//     (this board's entire pin budget is wide open - nothing here
//     competes for D3).
//
// ============================================================
// I2C ADDRESSING - 0x70 appears on two unrelated boards in this
// project, for two unrelated reasons
// ============================================================
// The TCA9548A's own default address (0x70, unjumpered) happens to
// match the PCA9685's built-in "LED All Call" address on the base
// board - see base_mega1.ino's own header comment for that situation.
// These are genuinely unrelated: two separate chips, on two
// physically separate Arduinos with their own independent I2C buses,
// that happen to share a number. Nothing here conflicts with
// anything on base_mega1 - flagged only because 0x70 has already
// caused real, documented confusion once in this project (see
// README's "Explicit assumptions" for that history), and a second
// unrelated appearance of the same number is worth naming explicitly
// rather than let it become a second point of confusion later.
//
// ============================================================
// SHUNT RESISTOR WARNING - verify before trusting current readings
// ============================================================
// The INA226 measures current by reading the voltage drop across an
// external shunt resistor, and its shunt-voltage measurement range is
// hard-capped at +-81.9mV. Many generic INA226 breakout boards ship
// with a fixed, onboard 0.1-ohm shunt - with that value, the MAXIMUM
// current this sensor could ever report before its shunt voltage
// saturates is 81.9mV / 0.1 ohm =~ 0.82A, far below what this rover's
// batteries actually need to measure (the ACS712 this replaced was
// rated for 30A). kInaShuntOhms below is set to 0.002 ohm (a smaller,
// higher-current-range value some INA226 breakouts use specifically
// for this reason, not the common 0.1-ohm default) as a placeholder -
// THIS MUST BE VERIFIED against whatever your actual breakout board
// physically has before trusting any current reading from it. If
// readings clip at a suspiciously round, low number regardless of
// real load, this - not a firmware bug - is almost certainly why.
//
// Talks to the ROS 2 `rover_power` bridge node using the shared
// RoverProtocol framing. Requires "OneWire" (by Paul Stoffregen) and
// "DallasTemperature" (by Miles Burton) for the temperature sensor -
// see base_mega1.ino's own header comment for the DallasTemperature
// LGPL-2.1 licensing note. Also requires "INA226" (by Rob Tillaart,
// MIT licensed) for the two voltage/current sensors - no dedicated
// TCA9548A library, since channel selection is a single I2C write of
// one byte (see tcaSelectChannel() below), not worth a library for.
//
// Wire protocol - structurally different from every other board:
// this one has NO INCOMING COMMAND FRAME. Every other board's
// sendStateFrame() is called reactively, from inside its own
// handleXCommand(), in response to an incoming command - there's
// nothing for this board to react to, so its 'S' frame is sent
// proactively instead, on its own fixed interval (kStateSendIntervalMs)
// from loop() directly. The bridge node on the ROS side is
// correspondingly simpler too - see rover_power/power_bridge_node.py's
// own header comment for what that changes there.

#include <Wire.h>
#include <INA226.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RoverProtocol.h>

// ----------------------------------------------------------- pins ---
// A4/A5 (SDA/SCL) go to the TCA9548A - not listed as discrete pin
// constants the way single-purpose pins are elsewhere in this file,
// since I2C uses whichever pins the hardware TWI peripheral is wired
// to, not an arbitrary choice (same reasoning given on every other
// board's own I2C section).
constexpr uint8_t kDs18b20DataPin = 2;
constexpr uint8_t kFanPwmPin = 3;

// --------------------------------------------- TCA9548A + INA226 x2 --
// See the file header's I2C ADDRESSING and SHUNT RESISTOR WARNING
// sections for the full reasoning behind every constant below.
constexpr uint8_t kTca9548aAddress = 0x70;      // default, unjumpered
constexpr uint8_t kBattery1MuxChannel = 0;
constexpr uint8_t kBattery2MuxChannel = 1;
constexpr uint8_t kIna226Address = 0x40;        // default, both A0/A1 grounded - safe to reuse on both units since the mux isolates them onto separate sub-buses
constexpr float kInaMaxCurrentA = 30.0f;        // placeholder, matches the ACS712 this replaced - not a measured limit of the actual battery/wiring
constexpr float kInaShuntOhms = 0.002f;         // PLACEHOLDER - verify against the actual breakout board, see the SHUNT RESISTOR WARNING above

void tcaSelectChannel(uint8_t channel) {
  Wire.beginTransmission(kTca9548aAddress);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

INA226 ina226Battery1(kIna226Address);
INA226 ina226Battery2(kIna226Address);

// Selects this battery's mux channel first - the TCA9548A only ever
// has one channel active at a time, so every I2C transaction with
// either INA226 (including this one) has to be preceded by selecting
// the right channel, not just once at startup.
//
// Out-parameters here, not a returned struct - a real Arduino IDE/
// arduino-cli bug (not this project's own mistake, and not fixed as
// of writing: see arduino/arduino-cli#2696 and arduino/Arduino#8014,
// #8050, both open issues going back years), where the IDE's
// automatic function-prototype generation inserts each function's
// auto-generated prototype immediately after the sketch's #include
// lines, before any type declared later in the file - including a
// custom struct defined and used correctly, in the right order,
// everywhere a human reads the file top to bottom. A struct return
// type here would compile fine with a standard C++ toolchain and
// fail specifically under the Arduino IDE's own preprocessing step
// with "'BatteryReading' does not name a type", pointing at this
// function's own definition rather than the real cause. Sidestepping
// entirely by keeping the signature to types already known before
// any prototype could reference them, rather than moving the struct
// to a separate header (the officially documented workaround) and
// introducing a multi-file sketch structure no other board here
// uses.
void readBattery(uint8_t muxChannel, INA226 &ina, int32_t &voltageMv, int32_t &currentMa) {
  tcaSelectChannel(muxChannel);
  voltageMv = (int32_t)lround(ina.getBusVoltage_mV());
  currentMa = (int32_t)lround(ina.getCurrent_mA());
}

// ------------------------------------------------- DS18B20 + fan ----
// Identical constants/design to every other board's own copy of this
// pair - see base_mega1.ino for the full reasoning behind each one
// (non-blocking read state machine, sentinel convention, hysteresis,
// fail-toward-running on sensor failure). Values kept the same across
// every board with these two components, this one included.
constexpr unsigned long kTemperatureReadIntervalMs = 1000;
constexpr unsigned long kTemperatureConversionMs = 750;
constexpr int32_t kTemperatureInvalidDeciC = -9999;

constexpr int32_t kFanOnTempDeciC = 350;
constexpr int32_t kFanOffTempDeciC = 300;
constexpr int32_t kFanMaxTempDeciC = 500;
constexpr uint8_t kFanMinDutyPercent = 30;

OneWire oneWire(kDs18b20DataPin);
DallasTemperature ds18b20(&oneWire);
enum TempReadState : int8_t { TEMP_IDLE = 0, TEMP_CONVERTING = 1 };
int8_t tempReadState = TEMP_IDLE;
unsigned long tempConversionStartMillis = 0;
int32_t cachedTemperatureDeciC = kTemperatureInvalidDeciC;
unsigned long lastTemperatureReadMillis = 0;

bool fanRunning = false;
uint8_t fanDutyPercent = 0;

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

// --------------------------------------------------------- frames ---
// No incoming command frame on this board - see the file header's
// wire-protocol note. This is the only frame this board ever sends,
// on its own fixed interval rather than in response to anything.
void sendStateFrame() {
  int32_t battery1Mv, battery1Ma, battery2Mv, battery2Ma;
  readBattery(kBattery1MuxChannel, ina226Battery1, battery1Mv, battery1Ma);
  readBattery(kBattery2MuxChannel, ina226Battery2, battery2Mv, battery2Ma);
  int32_t fields[6] = {
      battery1Mv,
      battery1Ma,
      battery2Mv,
      battery2Ma,
      cachedTemperatureDeciC,
      fanDutyPercent,
  };
  RoverProtocol::sendFrame(Serial, 'S', fields, 6);
}

// How often to proactively send a state frame - not tied to any
// control-loop rate the way other boards' reactive sends are, since
// there's no incoming command driving this one. 200ms (5Hz) is fast
// enough for a human watching a telemetry panel to perceive as live,
// slow enough that it isn't meaningfully more bus traffic than the
// ~10Hz command/response traffic every other board already produces.
constexpr unsigned long kStateSendIntervalMs = 200;
unsigned long lastStateSendMillis = 0;

// -------------------------------------------------------- setup -----
void setup() {
  Serial.begin(115200);

  Wire.begin();
  tcaSelectChannel(kBattery1MuxChannel);
  ina226Battery1.begin();
  ina226Battery1.setMaxCurrentShunt(kInaMaxCurrentA, kInaShuntOhms);
  tcaSelectChannel(kBattery2MuxChannel);
  ina226Battery2.begin();
  ina226Battery2.setMaxCurrentShunt(kInaMaxCurrentA, kInaShuntOhms);
  // No boot-time "found" check the way some sensors on other boards
  // have - begin()/isConnected() exist on this library but aren't
  // checked here, matching this board's own established pattern for
  // the DS18B20 below: a missing/failed sensor is non-fatal, this
  // board's real job doesn't depend on any single reading succeeding,
  // and a disconnected INA226 simply won't ACK its I2C transactions,
  // which the ROS-side protocol layer has no way to distinguish from
  // "reads as zero" either way without a dedicated sentinel this
  // library doesn't provide.

  ds18b20.begin();
  ds18b20.setWaitForConversion(false);  // non-blocking - see base_mega1.ino's own copy of this same setup() line

  pinMode(kFanPwmPin, OUTPUT);
  analogWrite(kFanPwmPin, 0);  // starts off - updateFanControl() takes over once a real temperature reading exists

  lastStateSendMillis = millis();
}

// --------------------------------------------------------- loop -----
void loop() {
  updateCachedTemperature();
  updateFanControl();

  unsigned long now = millis();
  if ((now - lastStateSendMillis) >= kStateSendIntervalMs) {
    lastStateSendMillis = now;
    sendStateFrame();
  }
}
