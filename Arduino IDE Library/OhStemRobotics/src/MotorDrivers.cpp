#include "MotorDrivers.h"

#include <math.h>

namespace ohstem {
namespace robotics {

namespace {

constexpr uint8_t kMD4CRegCh1 = 0;
constexpr uint8_t kMD4CRegCh2 = 1;
constexpr uint8_t kMD4CRegCh3 = 2;
constexpr uint8_t kMD4CRegCh4 = 3;

constexpr uint8_t kDirForward = 0;
constexpr uint8_t kDirBackward = 1;

constexpr uint8_t kServoRegs[] = {24, 26, 28, 30};

}  // namespace

MotorDriverV1::MotorDriverV1(TwoWire& wire, uint8_t address)
    : wire_(&wire), address_(address) {}

bool MotorDriverV1::begin() {
  wire_->begin();
  stop(ALL);
  return true;
}

void MotorDriverV1::writeCommand(uint8_t reg, const uint8_t* data, size_t len) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(data, len);
  wire_->endTransmission();
}

void MotorDriverV1::setMotors(uint8_t motors, int speed) {
  speed = clampValue(speed, -100, 100);
  const uint8_t direction = speed < 0 ? kDirBackward : kDirForward;
  const uint16_t absSpeed = static_cast<uint16_t>(abs(speed));

  auto send = [&](uint8_t reg) {
    uint8_t payload[8] = {
        reg, direction, static_cast<uint8_t>(absSpeed >> 8),
        static_cast<uint8_t>(absSpeed & 0xFF), 0, 0, 0, 0};
    writeCommand(0, payload, sizeof(payload));
  };

  if (motors & M1) send(kMD4CRegCh1);
  if (motors & M2) send(kMD4CRegCh2);
  if (motors & M3) send(kMD4CRegCh3);
  if (motors & M4) send(kMD4CRegCh4);
}

void MotorDriverV1::stop(uint8_t motors) { setMotors(motors, 0); }

void MotorDriverV1::brake(uint8_t motors) { setMotors(motors, 0); }

void MotorDriverV1::stepperSpeed(uint8_t stepper, uint16_t stepsPerRev,
                                 uint16_t speed, uint8_t style) {
  if (stepper > 1) {
    return;
  }
  stepsPerRev_ = stepsPerRev;
  speed = static_cast<uint16_t>(clampValue<int>(speed, 0, 255));
  uint8_t payload[8] = {
      stepper,
      static_cast<uint8_t>(stepsPerRev_ >> 8),
      static_cast<uint8_t>(stepsPerRev_ & 0xFF),
      style,
      0,
      kDirForward,
      static_cast<uint8_t>(speed >> 8),
      static_cast<uint8_t>(speed & 0xFF)};
  writeCommand(1, payload, sizeof(payload));
}

void MotorDriverV1::stepperStep(uint8_t stepper, int steps, uint8_t style) {
  if (stepper > 1) {
    return;
  }
  const uint8_t direction = steps < 0 ? kDirBackward : kDirForward;
  const uint16_t absSteps = static_cast<uint16_t>(abs(steps));
  uint8_t payload[8] = {
      stepper,
      static_cast<uint8_t>(stepsPerRev_ >> 8),
      static_cast<uint8_t>(stepsPerRev_ & 0xFF),
      style,
      1,
      direction,
      static_cast<uint8_t>(absSteps >> 8),
      static_cast<uint8_t>(absSteps & 0xFF)};
  writeCommand(1, payload, sizeof(payload));
}

MotorDriverV2::MotorDriverV2(TwoWire& wire, uint8_t address, int m3In1Pin,
                             int m3In2Pin, int m4In1Pin, int m4In2Pin)
    : wire_(&wire),
      address_(address),
      m3In1Pin_(m3In1Pin),
      m3In2Pin_(m3In2Pin),
      m4In1Pin_(m4In1Pin),
      m4In2Pin_(m4In2Pin) {}

bool MotorDriverV2::begin() {
  wire_->begin();

  if (m3In1Pin_ >= 0 && m3In2Pin_ >= 0 && m4In1Pin_ >= 0 && m4In2Pin_ >= 0) {
    pinMode(m3In1Pin_, OUTPUT);
    pinMode(m3In2Pin_, OUTPUT);
    pinMode(m4In1Pin_, OUTPUT);
    pinMode(m4In2Pin_, OUTPUT);
    localMotorsReady_ = true;
  }

  const uint8_t whoAmI = read8(kRegWhoAmI);
  stop(ALL);
  return whoAmI == address_;
}

String MotorDriverV2::firmwareVersion() {
  const uint8_t minor = read8(kRegFwVersion);
  const uint8_t major = read8(kRegFwVersion + 1);
  return String(major) + "." + String(minor);
}

float MotorDriverV2::battery() {
  return read8(kRegBattery) / 10.0f;
}

void MotorDriverV2::write8(uint8_t reg, uint8_t value) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(value);
  wire_->endTransmission();
}

void MotorDriverV2::write16(uint8_t reg, int16_t value) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(static_cast<uint8_t>(value & 0xFF));
  wire_->write(static_cast<uint8_t>((value >> 8) & 0xFF));
  wire_->endTransmission();
}

void MotorDriverV2::write16Array(uint8_t reg, const int16_t* data, size_t len) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  for (size_t i = 0; i < len; ++i) {
    wire_->write(static_cast<uint8_t>(data[i] & 0xFF));
    wire_->write(static_cast<uint8_t>((data[i] >> 8) & 0xFF));
  }
  wire_->endTransmission();
}

uint8_t MotorDriverV2::read8(uint8_t reg) {
  uint8_t value = 0;
  if (readBytes(reg, &value, 1)) {
    return value;
  }
  return 0;
}

bool MotorDriverV2::readBytes(uint8_t reg, uint8_t* buffer, size_t len) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) {
    return false;
  }

  const size_t readLen = wire_->requestFrom(static_cast<int>(address_),
                                            static_cast<int>(len));
  if (readLen != len) {
    while (wire_->available()) {
      wire_->read();
    }
    return false;
  }

  for (size_t i = 0; i < len; ++i) {
    buffer[i] = wire_->read();
  }
  return true;
}

void MotorDriverV2::setLocalMotor(uint8_t index, int speed) {
  if (!localMotorsReady_) {
    return;
  }

  speed = clampValue(speed, -100, 100);
  const int pwm = static_cast<int>(translateValue(abs(speed), 0, 100, 0, 255));

  int in1 = -1;
  int in2 = -1;
  if (index == M3) {
    in1 = m3In1Pin_;
    in2 = m3In2Pin_;
  } else if (index == M4) {
    in1 = m4In1Pin_;
    in2 = m4In2Pin_;
  }

  if (in1 < 0 || in2 < 0) {
    return;
  }

  if (speed > 0) {
    analogWrite(in1, pwm);
    analogWrite(in2, 0);
  } else if (speed < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, pwm);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void MotorDriverV2::brakeLocalMotor(uint8_t index) {
  if (!localMotorsReady_) {
    return;
  }

  int in1 = -1;
  int in2 = -1;
  if (index == M3) {
    in1 = m3In1Pin_;
    in2 = m3In2Pin_;
  } else if (index == M4) {
    in1 = m4In1Pin_;
    in2 = m4In2Pin_;
  }

  if (in1 < 0 || in2 < 0) {
    return;
  }

  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
}

void MotorDriverV2::setMotors(uint8_t motors, int speed) {
  speed = clampValue(speed, -100, 100);
  const int16_t payload[2] = {static_cast<int16_t>(motors),
                              static_cast<int16_t>(speed * 10)};
  write16Array(kRegMotorIndex, payload, 2);

  if (motors & M3) {
    setLocalMotor(M3, speed);
  }
  if (motors & M4) {
    setLocalMotor(M4, speed);
  }
}

void MotorDriverV2::stop(uint8_t motors) {
  setMotors(motors, 0);
}

void MotorDriverV2::brake(uint8_t motors) {
  write8(kRegMotorBrake, motors);
  if (motors & M3) {
    brakeLocalMotor(M3);
  }
  if (motors & M4) {
    brakeLocalMotor(M4);
  }
}

void MotorDriverV2::setServo(uint8_t index, int angle, int maxAngle) {
  if (index >= 4 || maxAngle <= 0) {
    return;
  }
  angle = static_cast<int>(clampValue<float>(angle, 0.0f, static_cast<float>(maxAngle)));
  const int normalized = static_cast<int>(angle * 180.0f / maxAngle);
  write16(kServoRegs[index], static_cast<int16_t>(normalized));
}

int MotorDriverV2::findEncoderIndex(uint8_t motors) const {
  if (motors & E1) return 0;
  if (motors & E2) return 1;
  return -1;
}

int32_t MotorDriverV2::getEncoder(uint8_t motors) {
  uint8_t buffer[8] = {0};
  if (!readBytes(kRegEncoder1, buffer, sizeof(buffer))) {
    return 0;
  }

  const int32_t e1 = readInt32LE(buffer);
  const int32_t e2 = readInt32LE(buffer + 4);
  const int index = findEncoderIndex(motors);
  if (motors == ALL) {
    return e1;
  }
  if (index == 0) return e1;
  if (index == 1) return e2;
  return 0;
}

void MotorDriverV2::resetEncoder(uint8_t motors) {
  write8(kRegResetEnc, motors);
}

void MotorDriverV2::reverseEncoder(uint8_t motors) {
  write8(kRegReverse, motors);
}

int16_t MotorDriverV2::getSpeed(uint8_t motors) {
  uint8_t buffer[4] = {0};
  if (!readBytes(kRegSpeedE1, buffer, sizeof(buffer))) {
    return 0;
  }

  const int16_t s1 = readInt16LE(buffer);
  const int16_t s2 = readInt16LE(buffer + 2);
  const int index = findEncoderIndex(motors);
  if (motors == ALL) {
    return s1;
  }
  if (index == 0) return s1;
  if (index == 1) return s2;
  return 0;
}

}  // namespace robotics
}  // namespace ohstem
