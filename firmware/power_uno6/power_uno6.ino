// power_uno6.ino

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

constexpr int32_t kFanOnTempDeciC = 340;
constexpr int32_t kFanOffTempDeciC = 300;
constexpr int32_t kFanMaxTempDeciC = 360;
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
