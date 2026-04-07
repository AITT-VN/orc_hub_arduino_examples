#include "DCMotor.h"

#include <math.h>

namespace ohstem {
namespace robotics {

#if defined(ARDUINO_ARCH_ESP32) && !OHSTEM_ESP32_LEDC_PIN_API
uint8_t RobotServo::nextPwmChannel_ = 0;
#endif

void DCMotorBase::reverse() { reversed_ = reversed_ == 1 ? -1 : 1; }

void DCMotorBase::setEncoder(float rpm, uint16_t ppr, uint16_t gears) {
  if (rpm <= 0 || ppr == 0 || gears == 0) {
    return;
  }
  // Convert the gearbox and encoder configuration into a usable ticks/rev value.
  encoderEnabled_ = true;
  rpm_ = rpm;
  ppr_ = ppr;
  gears_ = gears;
  ticksPerRev_ = static_cast<float>(ppr_) * 4.0f * static_cast<float>(gears_);
  maxPps_ = rpm_ * ticksPerRev_ / 60.0f;
  resetAngle();
}

void DCMotorBase::stallTolerances(float speedRatio, uint32_t timeMs) {
  stalledSpeedRatio_ = speedRatio;
  stalledTimeMs_ = timeMs;
}

void DCMotorBase::applyStopAction(StopAction thenAction) {
  if (thenAction == STOP) {
    stop();
  } else if (thenAction == BRAKE) {
    brake();
  }
}

void DCMotorBase::runTime(float speedValue, uint32_t timeMs, StopAction thenAction) {
  if (timeMs == 0) {
    return;
  }
  // Keep the motor running for a fixed duration, then apply the requested stop mode.
  const uint32_t startMs = millis();
  run(speedValue);
  while (elapsedMillis(startMs) < timeMs) {
    delay(10);
  }
  applyStopAction(thenAction);
}

void DCMotorBase::runAngle(float speedValue, float angleValue, StopAction thenAction) {
  if (!encoderEnabled_ || angleValue == 0.0f) {
    return;
  }

  // Convert a target shaft angle into encoder ticks and block until it is reached.
  const float targetTicks = fabs(angleValue) * ticksPerRev_ / 360.0f;
  const int32_t startTicks = encoderTicks();
  const float signedSpeed = angleValue < 0 ? -fabs(speedValue) : fabs(speedValue);
  run(signedSpeed);

  while (fabs(static_cast<float>(encoderTicks() - startTicks)) < targetTicks) {
    delay(10);
  }
  applyStopAction(thenAction);
}

void DCMotorBase::runRotation(float speedValue, float rotation, StopAction thenAction) {
  if (rotation == 0.0f) {
    return;
  }
  runAngle(speedValue, rotation * 360.0f, thenAction);
}

void DCMotorBase::runUntilStalled(float speedValue, StopAction thenAction) {
  if (!encoderEnabled_) {
    return;
  }

  // Treat the motor as stalled if it stays below a small percentage of max speed
  // for longer than the configured timeout.
  const float thresholdRpm = (maxPps_ * stalledSpeedRatio_ * 60.0f) / ticksPerRev_;
  bool stalled = false;
  uint32_t stalledStartMs = 0;

  run(speedValue);
  while (true) {
    if (fabs(speed()) <= thresholdRpm) {
      if (!stalled) {
        stalled = true;
        stalledStartMs = millis();
      }
    } else {
      stalled = false;
    }

    if (stalled && elapsedMillis(stalledStartMs) > stalledTimeMs_) {
      break;
    }
    delay(50);
  }

  applyStopAction(thenAction);
}

float DCMotorBase::angle() {
  if (!encoderEnabled_ || ticksPerRev_ == 0.0f) {
    return 0.0f;
  }
  return (static_cast<float>(rawEncoderTicks()) * 360.0f * reversed_) / ticksPerRev_;
}

void DCMotorBase::resetAngle() {
  if (!encoderEnabled_) {
    return;
  }
  resetEncoderInternal();
}

int32_t DCMotorBase::encoderTicks() {
  if (!encoderEnabled_) {
    return 0;
  }
  return rawEncoderTicks();
}

float DCMotorBase::speed() {
  if (!encoderEnabled_ || ticksPerRev_ == 0.0f) {
    return 0.0f;
  }
  return rawSpeedPps() * 60.0f / ticksPerRev_;
}

DCMotor::DCMotor(MotorDriverBase& driver, uint8_t port, bool reversed)
    : DCMotorBase(port), driver_(driver) {
  if (reversed) {
    reverse();
  }
}

void DCMotor::run(float speedValue) {
  const int speed = static_cast<int>(clampValue(speedValue, -100.0f, 100.0f));
  // reversed_ lets the drive base fix left/right orientation without rewiring.
  driver_.setMotors(port_, speed * reversed_);
}

void DCMotor::brake() { driver_.brake(port_); }

void DCMotor::stop() { driver_.stop(port_); }

void DCMotor::reverseEncoder() { driver_.reverseEncoder(port_); }

int32_t DCMotor::rawEncoderTicks() const {
  return const_cast<MotorDriverBase&>(driver_).getEncoder(port_);
}

void DCMotor::resetEncoderInternal() { driver_.resetEncoder(port_); }

float DCMotor::rawSpeedPps() const {
  return const_cast<MotorDriverBase&>(driver_).getSpeed(port_);
}

DCMotor2Pin::DCMotor2Pin(uint8_t in1Pin, uint8_t in2Pin, bool reversed)
    : DCMotorBase(0), in1Pin_(in1Pin), in2Pin_(in2Pin) {
  pinMode(in1Pin_, OUTPUT);
  pinMode(in2Pin_, OUTPUT);
  if (reversed) {
    reverse();
  }
}

void DCMotor2Pin::run(float speedValue) {
  const int speed =
      static_cast<int>(clampValue(speedValue, -100.0f, 100.0f)) * directionMultiplier();
  const int pwm = static_cast<int>(translateValue(abs(speed), 0, 100, 0, 255));

  // A 2-pin motor is driven by applying PWM to one side at a time.
  if (speed > 0) {
    analogWrite(in1Pin_, pwm);
    analogWrite(in2Pin_, 0);
  } else if (speed < 0) {
    analogWrite(in1Pin_, 0);
    analogWrite(in2Pin_, pwm);
  } else {
    analogWrite(in1Pin_, 0);
    analogWrite(in2Pin_, 0);
  }
}

void DCMotor2Pin::stop() { run(0); }

void DCMotor2Pin::brake() {
  analogWrite(in1Pin_, 255);
  analogWrite(in2Pin_, 255);
}

DCMotor3Pin::DCMotor3Pin(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin, int stbyPin,
                         bool reversed)
    : DCMotorBase(0),
      in1Pin_(in1Pin),
      in2Pin_(in2Pin),
      pwmPin_(pwmPin),
      stbyPin_(stbyPin) {
  pinMode(in1Pin_, OUTPUT);
  pinMode(in2Pin_, OUTPUT);
  pinMode(pwmPin_, OUTPUT);
  if (stbyPin_ >= 0) {
    pinMode(stbyPin_, OUTPUT);
    digitalWrite(stbyPin_, HIGH);
  }
  if (reversed) {
    reverse();
  }
}

void DCMotor3Pin::run(float speedValue) {
  const int speed =
      static_cast<int>(clampValue(speedValue, -100.0f, 100.0f)) * directionMultiplier();
  const int pwm = static_cast<int>(translateValue(abs(speed), 0, 100, 0, 255));

  // Direction is set on IN1/IN2 while PWM controls the effective voltage.
  if (speed > 0) {
    digitalWrite(in1Pin_, HIGH);
    digitalWrite(in2Pin_, LOW);
  } else if (speed < 0) {
    digitalWrite(in1Pin_, LOW);
    digitalWrite(in2Pin_, HIGH);
  } else {
    digitalWrite(in1Pin_, LOW);
    digitalWrite(in2Pin_, LOW);
  }
  analogWrite(pwmPin_, pwm);
}

void DCMotor3Pin::stop() { run(0); }

void DCMotor3Pin::brake() {
  digitalWrite(in1Pin_, HIGH);
  digitalWrite(in2Pin_, HIGH);
  analogWrite(pwmPin_, 0);
}

RobotServo::RobotServo(MotorDriverV2& driver, uint8_t port, int maxAngle)
    : driver_(&driver),
      port_(port),
      maxAngle_(maxAngle > 0 ? (maxAngle == 360 ? 180 : maxAngle) : 180) {
  limitMax_ = maxAngle_;
}

RobotServo::RobotServo(uint8_t pin, int maxAngle)
    : outputPin_(pin),
      maxAngle_(maxAngle > 0 ? (maxAngle == 360 ? 180 : maxAngle) : 180) {
  limitMax_ = maxAngle_;
}

bool RobotServo::begin() {
  if (driver_ != nullptr) {
    return true;
  }
  if (!attached_ && outputPin_ >= 0) {
#if defined(ARDUINO_ARCH_ESP32)
    // ESP32 can generate servo pulses through LEDC instead of the Servo library.
#if OHSTEM_ESP32_LEDC_PIN_API
    attached_ = ledcAttach(static_cast<uint8_t>(outputPin_), 50, 16);
#else
    if (pwmChannel_ == 255) {
      if (nextPwmChannel_ >= 16) {
        return false;
      }
      pwmChannel_ = nextPwmChannel_++;
    }
    ledcSetup(pwmChannel_, 50, 16);
    ledcAttachPin(static_cast<uint8_t>(outputPin_), pwmChannel_);
    attached_ = true;
#endif
#else
    directServo_.attach(outputPin_);
    attached_ = true;
#endif
  }
  return attached_;
}

void RobotServo::limit(int minAngle, int maxAngle) {
  if (minAngle < 0 || maxAngle < minAngle || maxAngle > maxAngle_) {
    return;
  }
  limitMin_ = minAngle;
  limitMax_ = maxAngle;
}

int RobotServo::angleToMicroseconds(int angleValue) const {
  const float ratio = static_cast<float>(angleValue) / static_cast<float>(maxAngle_);
  return static_cast<int>(500.0f + ratio * 2000.0f);
}

void RobotServo::setAngle(int angleValue) {
  angleValue = clampValue(angleValue, limitMin_, limitMax_);
  if (driver_ != nullptr) {
    // When a MotorDriverV2 is present, the servo pulse is produced by the driver board.
    driver_->setServo(port_, angleValue, maxAngle_);
  } else if (begin()) {
#if defined(ARDUINO_ARCH_ESP32)
    const uint32_t maxDuty = 65535;
    // Map the requested angle to a 0.5 ms .. 2.5 ms servo pulse inside a 20 ms frame.
    const uint32_t duty = static_cast<uint32_t>(
        (static_cast<uint64_t>(angleToMicroseconds(angleValue)) * maxDuty) / 20000ULL);
#if OHSTEM_ESP32_LEDC_PIN_API
    ledcWrite(static_cast<uint8_t>(outputPin_), duty);
#else
    ledcWrite(pwmChannel_, duty);
#endif
#else
    directServo_.writeMicroseconds(angleToMicroseconds(angleValue));
#endif
  }
  currentAngle_ = angleValue;
}

void RobotServo::runAngle(int angleValue, int speedValue) {
  angleValue = clampValue(angleValue, limitMin_, limitMax_);
  speedValue = clampValue(speedValue, 0, 100);

  if (speedValue >= 100 || currentAngle_ < 0) {
    setAngle(angleValue);
    return;
  }

  // For lower speeds, move one degree at a time to create a simple easing effect.
  const int stepDelay = static_cast<int>(translateValue(speedValue, 0, 100, 10, 0));
  if (currentAngle_ > angleValue) {
    for (int angleStep = currentAngle_; angleStep >= angleValue; --angleStep) {
      setAngle(angleStep);
      delay(stepDelay);
    }
  } else {
    for (int angleStep = currentAngle_; angleStep <= angleValue; ++angleStep) {
      setAngle(angleStep);
      delay(stepDelay);
    }
  }
}

void RobotServo::runSteps(int steps, int speedValue) {
  if (currentAngle_ < 0) {
    currentAngle_ = 0;
  }
  runAngle(currentAngle_ + steps, speedValue);
}

void RobotServo::spin(int speedValue) {
  speedValue = clampValue(speedValue, -100, 100);
  const int angleValue = static_cast<int>(90.0f - (speedValue / 100.0f) * 90.0f);
  setAngle(angleValue);
}

}  // namespace robotics
}  // namespace ohstem
