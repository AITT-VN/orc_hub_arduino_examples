#include "PCF8574.h"

namespace ohstem {
namespace robotics {

bool PCF8574::begin() {
  wire_->begin();
  wire_->beginTransmission(address_);
  return wire_->endTransmission() == 0;
}

bool PCF8574::readInternal() {
  const int count = wire_->requestFrom(static_cast<int>(address_), 1);
  if (count != 1 || !wire_->available()) {
    return false;
  }
  portState_ = wire_->read();
  return true;
}

uint8_t PCF8574::port() {
  readInternal();
  return portState_;
}

void PCF8574::writePort(uint8_t value) {
  portState_ = value;
  wire_->beginTransmission(address_);
  wire_->write(portState_);
  wire_->endTransmission();
}

int PCF8574::pin(uint8_t index) {
  if (index > 7) {
    return 0;
  }
  readInternal();
  return (portState_ >> index) & 0x01;
}

void PCF8574::pin(uint8_t index, bool value) {
  if (index > 7) {
    return;
  }
  if (value) {
    portState_ |= (1 << index);
  } else {
    portState_ &= ~(1 << index);
  }
  writePort(portState_);
}

void PCF8574::toggle(uint8_t index) {
  if (index > 7) {
    return;
  }
  portState_ ^= (1 << index);
  writePort(portState_);
}

}  // namespace robotics
}  // namespace ohstem
