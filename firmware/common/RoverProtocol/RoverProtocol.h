// RoverProtocol.h
//
// Header-only Arduino counterpart to rover_protocol/framing.py on the
// ROS 2 side. Every board (base Mega #1, arm Mega #2, mast Uno #3,
// microscope Uno #4) includes this and speaks the identical wire
// format, so the protocol is only designed and tested once even
// though it is implemented twice (Python host side / C++ firmware
// side).
//
// Frame format:  <TYPE>,<f1>,<f2>,...,<fn>*<CC>\n
//   TYPE - single uppercase letter
//   f1..fn - signed decimal integers (no floats on the wire)
//   CC   - two upper-case hex digits, XOR checksum of everything
//          before the '*'
//
// Install: copy this RoverProtocol/ folder into your Arduino
// libraries directory (normally ~/Arduino/libraries/) before
// compiling any of the four board sketches, or open the Arduino IDE
// "Sketch > Include Library > Add .ZIP Library..." on this folder.

#pragma once

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

namespace RoverProtocol {

constexpr uint8_t kMaxFields = 12;
constexpr uint16_t kMaxLineLen = 128;

inline uint8_t checksum(const char* payload, uint8_t len) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < len; i++) {
    c ^= (uint8_t)payload[i];
  }
  return c;
}

struct ParsedFrame {
  char type = '\0';
  int32_t fields[kMaxFields];
  uint8_t fieldCount = 0;
  bool valid = false;
};

// Parses a NUL-terminated line (no trailing \r\n). Mutates `line` in
// place (inserts NULs at token boundaries) - pass a scratch buffer,
// not a literal.
inline ParsedFrame parseFrame(char* line) {
  ParsedFrame result;

  char* star = strrchr(line, '*');
  if (star == nullptr) return result;
  uint8_t payloadLen = (uint8_t)(star - line);
  if (payloadLen == 0) return result;

  char* ccStr = star + 1;
  if (strlen(ccStr) < 2) return result;
  char ccBuf[3] = {ccStr[0], ccStr[1], '\0'};
  char* endptr = nullptr;
  uint8_t ccReceived = (uint8_t)strtoul(ccBuf, &endptr, 16);
  if (endptr == ccBuf) return result;  // not valid hex
  uint8_t ccExpected = checksum(line, payloadLen);
  if (ccReceived != ccExpected) return result;

  *star = '\0';  // terminate payload at '*' so strtok_r stops there

  char* saveptr = nullptr;
  char* tok = strtok_r(line, ",", &saveptr);
  if (tok == nullptr || strlen(tok) != 1 || !isupper((unsigned char)tok[0])) {
    return result;
  }
  result.type = tok[0];

  while ((tok = strtok_r(nullptr, ",", &saveptr)) != nullptr) {
    if (result.fieldCount >= kMaxFields) {
      result.valid = false;
      return result;
    }
    result.fields[result.fieldCount++] = atol(tok);
  }

  result.valid = true;
  return result;
}

// Sends "type,f1,f2,...*CC\n" on `out` (typically Serial).
inline void sendFrame(Stream& out, char type, const int32_t* fields, uint8_t count) {
  char buf[kMaxLineLen];
  uint16_t pos = 0;
  buf[pos++] = type;
  for (uint8_t i = 0; i < count && pos < sizeof(buf) - 12; i++) {
    buf[pos++] = ',';
    pos += (uint16_t)snprintf(buf + pos, sizeof(buf) - pos, "%ld", (long)fields[i]);
  }
  uint8_t cc = checksum(buf, (uint8_t)pos);
  pos += (uint16_t)snprintf(buf + pos, sizeof(buf) - pos, "*%02X", cc);
  out.write((const uint8_t*)buf, pos);
  out.write('\n');
}

// Convenience overload for the very common zero/one/two-field cases.
inline void sendFrame0(Stream& out, char type) {
  sendFrame(out, type, nullptr, 0);
}

// Non-blocking newline-delimited line reader. Call `poll()` once per
// loop() iteration; it returns true exactly once per complete line,
// with the line (sans '\n'/'\r') copied into `lineOut`.
class LineReader {
 public:
  bool poll(Stream& in, char* lineOut, uint16_t maxLen) {
    while (in.available()) {
      char c = (char)in.read();
      if (c == '\n') {
        buf_[len_] = '\0';
        bool haveLine = (len_ > 0);
        len_ = 0;
        if (haveLine) {
          strncpy(lineOut, buf_, maxLen - 1);
          lineOut[maxLen - 1] = '\0';
          return true;
        }
        continue;
      }
      if (c == '\r') continue;
      if (len_ < sizeof(buf_) - 1) {
        buf_[len_++] = c;
      } else {
        // Overflow: drop this (garbage) line and resync on the next '\n'.
        len_ = 0;
      }
    }
    return false;
  }

 private:
  char buf_[kMaxLineLen];
  uint16_t len_ = 0;
};

}  // namespace RoverProtocol
