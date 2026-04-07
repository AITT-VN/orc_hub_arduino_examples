#ifndef OHSTEM_DC_MOTOR_H
#define OHSTEM_DC_MOTOR_H

#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <esp32-hal-ledc.h>
#if defined(__has_include)
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#endif
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define OHSTEM_ESP32_LEDC_PIN_API 1
#else
#define OHSTEM_ESP32_LEDC_PIN_API 0
#endif
#else
#include <Servo.h>
#endif

#include "MotorDrivers.h"
#include "RoboticsConstants.h"
#include "RoboticsUtils.h"

namespace ohstem {
namespace robotics {

class DCMotorBase {
 public:
  explicit DCMotorBase(uint8_t port = 0) : port_(port) {}
  virtual ~DCMotorBase() = default;

  virtual void run(float speed) = 0;
  virtual void brake() = 0;
  virtual void stop() = 0;

  // Flip encoder counting direction without changing the motor output direction.
  virtual void reverseEncoder() {}

  // Toggle the motor output direction in software. This is useful when a motor
  // is mounted or wired in the opposite orientation.
  // Example:
  //   void setup() {
  //     leftMotor.reverse();
  //   }
  void reverse();
  void setEncoder(float rpm, uint16_t ppr, uint16_t gears);
  void stallTolerances(float speedRatio, uint32_t timeMs);

  void runTime(float speed, uint32_t timeMs, StopAction thenAction = STOP);
  void runAngle(float speed, float angle, StopAction thenAction = BRAKE);
  void runRotation(float speed, float rotation, StopAction thenAction = BRAKE);
  void runUntilStalled(float speed, StopAction thenAction = STOP);

  float angle();
  void resetAngle();
  int32_t encoderTicks();
  float speed();

  uint8_t port() const { return port_; }
  bool encoderEnabled() const { return encoderEnabled_; }

 protected:
  virtual int32_t rawEncoderTicks() const { return 0; }
  virtual void resetEncoderInternal() {}
  virtual float rawSpeedPps() const { return 0.0f; }
  void applyStopAction(StopAction thenAction);
  int8_t directionMultiplier() const { return reversed_; }

  uint8_t port_ = 0;
  bool encoderEnabled_ = false;
  float rpm_ = 0.0f;
  uint16_t ppr_ = 0;
  uint16_t gears_ = 0;
  float ticksPerRev_ = 0.0f;
  float maxPps_ = 0.0f;
  float stalledSpeedRatio_ = 0.05f;
  uint32_t stalledTimeMs_ = 1000;
  int8_t reversed_ = 1;
};

class DCMotor : public DCMotorBase {
 public:
  // Set reversed=true only when this individual motor needs to spin opposite
  // to the default software direction.
  // Example:
  //   DCMotor leftMotor(motorDriver, E1, true);
  DCMotor(MotorDriverBase& driver, uint8_t port, bool reversed = false);

  void run(float speed) override;
  void brake() override;
  void stop() override;
  void reverseEncoder() override;
  MotorDriverBase& driver() { return driver_; }

 protected:
  int32_t rawEncoderTicks() const override;
  void resetEncoderInternal() override;
  float rawSpeedPps() const override;

 private:
  MotorDriverBase& driver_;
};

class DCMotor2Pin : public DCMotorBase {
 public:
  // Set reversed=true only when this individual motor needs to spin opposite
  // to the default software direction.
  DCMotor2Pin(uint8_t in1Pin, uint8_t in2Pin, bool reversed = false);

  void run(float speed) override;
  void brake() override;
  void stop() override;

 private:
  uint8_t in1Pin_;
  uint8_t in2Pin_;
};

class DCMotor3Pin : public DCMotorBase {
 public:
  // Set reversed=true only when this individual motor needs to spin opposite
  // to the default software direction.
  DCMotor3Pin(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin,
              int stbyPin = -1, bool reversed = false);

  void run(float speed) override;
  void brake() override;
  void stop() override;

 private:
  uint8_t in1Pin_;
  uint8_t in2Pin_;
  uint8_t pwmPin_;
  int stbyPin_;
};

class RobotServo {
 public:
  RobotServo(MotorDriverV2& driver, uint8_t port, int maxAngle = 180);
  RobotServo(uint8_t pin, int maxAngle = 180);

  bool begin();
  void limit(int minAngle, int maxAngle);
  void setAngle(int angleValue);
  void angle(int angleValue) { setAngle(angleValue); }
  void runAngle(int angleValue, int speed = 100);
  void runSteps(int steps, int speed = 100);
  void spin(int speed);
  int currentAngle() const { return currentAngle_; }

 private:
  int angleToMicroseconds(int angleValue) const;

  MotorDriverV2* driver_ = nullptr;
  uint8_t port_ = 0;
  int outputPin_ = -1;
  bool attached_ = false;
#if !defined(ARDUINO_ARCH_ESP32)
  ::Servo directServo_;
#elif !OHSTEM_ESP32_LEDC_PIN_API
  uint8_t pwmChannel_ = 255;
  static uint8_t nextPwmChannel_;
#endif
  int maxAngle_ = 180;
  int currentAngle_ = -1;
  int limitMin_ = 0;
  int limitMax_ = 180;
};

}  // namespace robotics
}  // namespace ohstem

#endif
