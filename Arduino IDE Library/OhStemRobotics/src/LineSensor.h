#ifndef OHSTEM_LINE_SENSOR_H
#define OHSTEM_LINE_SENSOR_H

#include <Arduino.h>
#include <Wire.h>

#include "PCF8574.h"
#include "RoboticsConstants.h"

namespace ohstem {
namespace robotics {

class LineSensor {
 public:
  virtual ~LineSensor() = default;
  virtual bool begin() { return true; }
  virtual int read(uint8_t index) = 0;
  virtual LineState check() = 0;
};

class LineSensor2P : public LineSensor {
 public:
  LineSensor2P(uint8_t s1Pin, uint8_t s2Pin) : s1Pin_(s1Pin), s2Pin_(s2Pin) {}

  bool begin() override;
  int read(uint8_t index) override;
  LineState check() override;

 private:
  uint8_t s1Pin_;
  uint8_t s2Pin_;
};

class LineSensor3P : public LineSensor {
 public:
  LineSensor3P(uint8_t s1Pin, uint8_t s2Pin, uint8_t s3Pin)
      : s1Pin_(s1Pin), s2Pin_(s2Pin), s3Pin_(s3Pin) {}

  bool begin() override;
  int read(uint8_t index) override;
  LineState check() override;

 private:
  uint8_t s1Pin_;
  uint8_t s2Pin_;
  uint8_t s3Pin_;
};

class LineSensorI2C : public LineSensor {
 public:
  explicit LineSensorI2C(TwoWire& wire = Wire, uint8_t address = 0x23)
      : pcf_(wire, address) {}

  bool begin() override;
  int read(uint8_t index) override;
  LineState check() override;

 private:
  PCF8574 pcf_;
};

}  // namespace robotics
}  // namespace ohstem

#endif
