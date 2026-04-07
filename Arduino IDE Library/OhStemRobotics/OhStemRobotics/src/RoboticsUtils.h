#ifndef OHSTEM_ROBOTICS_UTILS_H
#define OHSTEM_ROBOTICS_UTILS_H

#include <Arduino.h>
#include <Wire.h>

namespace ohstem {
namespace robotics {

#ifndef OHSTEM_I2C_SDA
#define OHSTEM_I2C_SDA 11
#endif

#ifndef OHSTEM_I2C_SCL
#define OHSTEM_I2C_SCL 12
#endif

template <typename T>
inline T clampValue(T value, T lower, T upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

inline float translateValue(float value, float fromLow, float fromHigh, float toLow,
                            float toHigh) {
  const float fromSpan = fromHigh - fromLow;
  if (fromSpan == 0.0f) {
    return toLow;
  }
  const float scaled = (value - fromLow) / fromSpan;
  return toLow + (scaled * (toHigh - toLow));
}

inline int16_t readInt16BE(const uint8_t* data) {
  return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

inline int16_t readInt16LE(const uint8_t* data) {
  return static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
}

inline int32_t readInt32BE(const uint8_t* data) {
  return static_cast<int32_t>((static_cast<uint32_t>(data[0]) << 24) |
                              (static_cast<uint32_t>(data[1]) << 16) |
                              (static_cast<uint32_t>(data[2]) << 8) | data[3]);
}

inline int32_t readInt32LE(const uint8_t* data) {
  return static_cast<int32_t>((static_cast<uint32_t>(data[3]) << 24) |
                              (static_cast<uint32_t>(data[2]) << 16) |
                              (static_cast<uint32_t>(data[1]) << 8) | data[0]);
}

inline uint32_t elapsedMillis(uint32_t startMs) {
  return static_cast<uint32_t>(millis() - startMs);
}

inline uint32_t elapsedMicros(uint32_t startUs) {
  return static_cast<uint32_t>(micros() - startUs);
}

inline void configureDefaultWirePins(TwoWire& wire = Wire) {
#if defined(ARDUINO_ARCH_ESP32)
  // Configure the Wire object first, so sketches can keep the traditional
  // syntax:
  //   Wire.begin();
  // and still get the custom default I2C pins below.
  wire.setPins(OHSTEM_I2C_SDA, OHSTEM_I2C_SCL);
#endif
}

}  // namespace robotics
}  // namespace ohstem

#endif
