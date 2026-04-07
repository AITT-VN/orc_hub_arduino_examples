#ifndef OHSTEM_PCF8574_H
#define OHSTEM_PCF8574_H

#include <Arduino.h>
#include <Wire.h>

namespace ohstem {
namespace robotics {

class PCF8574 {
 public:
  explicit PCF8574(TwoWire& wire, uint8_t address = 0x20)
      : wire_(&wire), address_(address) {}

  bool begin();
  uint8_t port();
  void writePort(uint8_t value);
  int pin(uint8_t index);
  void pin(uint8_t index, bool value);
  void toggle(uint8_t index);

 private:
  bool readInternal();

  TwoWire* wire_;
  uint8_t address_;
  uint8_t portState_ = 0xFF;
};

}  // namespace robotics
}  // namespace ohstem

#endif
