#ifndef OHSTEM_MOTOR_DRIVERS_H
#define OHSTEM_MOTOR_DRIVERS_H

#include <Arduino.h>
#include <Wire.h>

#include "RoboticsConstants.h"
#include "RoboticsUtils.h"

namespace ohstem {
namespace robotics {

class MotorDriverBase {
 public:
  virtual ~MotorDriverBase() = default;

  virtual bool begin() = 0;
  virtual void setMotors(uint8_t motors, int speed) = 0;
  virtual void stop(uint8_t motors = ALL) = 0;
  virtual void brake(uint8_t motors = ALL) = 0;
  virtual void setServo(uint8_t index, int angle, int maxAngle = 180) {
    (void)index;
    (void)angle;
    (void)maxAngle;
  }
  virtual int32_t getEncoder(uint8_t motors = ALL) {
    (void)motors;
    return 0;
  }
  virtual void resetEncoder(uint8_t motors = ALL) { (void)motors; }
  virtual void reverseEncoder(uint8_t motors) { (void)motors; }
  virtual int16_t getSpeed(uint8_t motors = ALL) {
    (void)motors;
    return 0;
  }
  virtual bool hasEncoderSupport() const { return false; }
  virtual bool hasServoSupport() const { return false; }
};

class MotorDriverV1 : public MotorDriverBase {
 public:
  static constexpr uint8_t kDefaultAddress = 0x30;

  explicit MotorDriverV1(TwoWire& wire = Wire, uint8_t address = kDefaultAddress);

  bool begin() override;
  void setMotors(uint8_t motors, int speed) override;
  void stop(uint8_t motors = ALL) override;
  void brake(uint8_t motors = ALL) override;

  void stepperSpeed(uint8_t stepper, uint16_t stepsPerRev, uint16_t speed,
                    uint8_t style = 2);
  void stepperStep(uint8_t stepper, int steps, uint8_t style = 1);

 private:
  void writeCommand(uint8_t reg, const uint8_t* data, size_t len);

  TwoWire* wire_;
  uint8_t address_;
  uint16_t stepsPerRev_ = 200;
};

class MotorDriverV2 : public MotorDriverBase {
 public:
  static constexpr uint8_t kDefaultAddress = 0x54;

  MotorDriverV2(TwoWire& wire = Wire, uint8_t address = kDefaultAddress,
                int m3In1Pin = -1, int m3In2Pin = -1, int m4In1Pin = -1,
                int m4In2Pin = -1);

  bool begin() override;
  String firmwareVersion();
  float battery();

  void setMotors(uint8_t motors, int speed) override;
  void stop(uint8_t motors = ALL) override;
  void brake(uint8_t motors = ALL) override;
  void setServo(uint8_t index, int angle, int maxAngle = 180) override;
  int32_t getEncoder(uint8_t motors = ALL) override;
  void resetEncoder(uint8_t motors = ALL) override;
  void reverseEncoder(uint8_t motors) override;
  int16_t getSpeed(uint8_t motors = ALL) override;
  bool hasEncoderSupport() const override { return true; }
  bool hasServoSupport() const override { return true; }

 private:
  static constexpr uint8_t kRegResetEnc = 0;
  static constexpr uint8_t kRegMotorIndex = 16;
  static constexpr uint8_t kRegMotorSpeed = 18;
  static constexpr uint8_t kRegMotorBrake = 22;
  static constexpr uint8_t kRegReverse = 23;
  static constexpr uint8_t kRegServo1 = 24;
  static constexpr uint8_t kRegFwVersion = 40;
  static constexpr uint8_t kRegWhoAmI = 42;
  static constexpr uint8_t kRegBattery = 43;
  static constexpr uint8_t kRegEncoder1 = 44;
  static constexpr uint8_t kRegSpeedE1 = 52;

  void write8(uint8_t reg, uint8_t value);
  void write16(uint8_t reg, int16_t value);
  void write16Array(uint8_t reg, const int16_t* data, size_t len);
  uint8_t read8(uint8_t reg);
  bool readBytes(uint8_t reg, uint8_t* buffer, size_t len);
  void setLocalMotor(uint8_t index, int speed);
  void brakeLocalMotor(uint8_t index);
  int findEncoderIndex(uint8_t motors) const;

  TwoWire* wire_;
  uint8_t address_;

  int m3In1Pin_;
  int m3In2Pin_;
  int m4In1Pin_;
  int m4In2Pin_;
  bool localMotorsReady_ = false;
};

}  // namespace robotics
}  // namespace ohstem

#endif
