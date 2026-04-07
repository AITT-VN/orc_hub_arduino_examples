#ifndef OHSTEM_DRIVE_BASE_H
#define OHSTEM_DRIVE_BASE_H

#include <Arduino.h>

#include "DCMotor.h"
#include "Gamepad.h"
#include "LineSensor.h"
#include "PIDController.h"
#include "RoboticsConstants.h"

namespace ohstem {
namespace robotics {

class AngleSensor;

class DriveBase {
 public:
  DriveBase(DriveMode driveMode, DCMotorBase* m1, DCMotorBase* m2,
            DCMotorBase* m3 = nullptr, DCMotorBase* m4 = nullptr);

  void speed(float speedValue, float minSpeed = -1.0f);
  float speed() const { return speed_; }
  void lineSensor(LineSensor* sensor) { lineSensor_ = sensor; }
  void angleSensor(AngleSensor* sensor) { angleSensor_ = sensor; }
  void size(float wheelDiameterMm, float widthMm);
  void useGyro(bool enabled) { useGyro_ = enabled; }
  void pid(float kp, float ki, float kd) { pid_.setTunings(kp, ki, kd); }
  void speedRatio(float left, float right);
  void setAutoMode(bool enabled) { modeAuto_ = enabled; }
  bool autoMode() const { return modeAuto_; }
  void setSideMoveMode(uint8_t mode) { sideMoveMode_ = mode; }
  void onTeleopCommand(GamepadButton button, void (*callback)());
  void updateTeleop(Gamepad& gamepad, uint8_t accelSteps = 5);
  void updateTeleop(const GamepadState& state, uint8_t accelSteps = 5);

  void forward();
  void backward();
  void turnLeft();
  void turnRight();
  void moveLeft();
  void moveRight();

  void forwardFor(float amount, MoveUnit unit = SECOND, StopAction thenAction = STOP);
  void backwardFor(float amount, MoveUnit unit = SECOND, StopAction thenAction = STOP);
  void turnLeftFor(float amount, MoveUnit unit = SECOND, StopAction thenAction = STOP);
  void turnRightFor(float amount, MoveUnit unit = SECOND, StopAction thenAction = STOP);
  void moveLeftFor(float amount, MoveUnit unit = SECOND, StopAction thenAction = STOP);
  void moveRightFor(float amount, MoveUnit unit = SECOND, StopAction thenAction = STOP);
  void straight(float speedValue, float amount, MoveUnit unit = SECOND,
                StopAction thenAction = STOP);
  void turn(float steering, float amount = 0.0f, MoveUnit unit = SECOND,
            StopAction thenAction = STOP);

  void run(Direction dir, float speedValue = NAN);
  void runSpeed(float leftSpeed, float rightSpeed = NAN);
  void stop();
  void brake();
  void stopThen(StopAction thenAction);

  float distance();
  float angle();
  void resetAngle();

  void followLine(bool backward = true, LineState lineState = LINE_END);
  void followLineUntilEnd(StopAction thenAction = STOP);
  void followLineUntilCross(StopAction thenAction = STOP);
  void followLineByTime(float seconds, StopAction thenAction = STOP);
  void turnUntilLineDetected(float steering, StopAction thenAction = STOP);

 private:
  void tickAngleSensor();
  float calcSpeed(float speedValue, float distanceValue, float drivenDistance,
                  float lastDrivenDistance) const;
  void calibSpeed(float speedValue, float& left, float& right);
  void calcSteering(float speedValue, float steering, float& left,
                    float& right) const;

  DriveMode driveMode_;
  DCMotorBase* motors_[4] = {nullptr, nullptr, nullptr, nullptr};
  DCMotorBase* left_[2] = {nullptr, nullptr};
  DCMotorBase* right_[2] = {nullptr, nullptr};
  uint8_t leftCount_ = 0;
  uint8_t rightCount_ = 0;
  DCMotorBase* leftEncoder_ = nullptr;
  DCMotorBase* rightEncoder_ = nullptr;

  float speed_ = 75.0f;
  float minSpeed_ = 40.0f;
  float wheelDiameter_ = 80.0f;
  float width_ = 300.0f;
  float wheelCircumference_ = 80.0f * PI;

  LineSensor* lineSensor_ = nullptr;
  AngleSensor* angleSensor_ = nullptr;
  bool useGyro_ = false;
  bool modeAuto_ = true;
  uint8_t sideMoveMode_ = JOYSTICK;
  LineState lastLineState_ = LINE_CENTER;
  PIDController pid_ = PIDController(5.0f, 0.15f, 0.1f, 0.0f, 0.0f, -10.0f, 10.0f);

  float speedRatioLeft_ = 1.0f;
  float speedRatioRight_ = 1.0f;
  int8_t lastTeleopDir_ = -1;
  float teleopSpeed_ = 0.0f;
  float teleopTurnSpeed_ = 0.0f;
  void (*teleopHandlers_[static_cast<size_t>(GamepadButton::Count)])() = {};
  bool lastTeleopButtons_[static_cast<size_t>(GamepadButton::Count)] = {};
};

}  // namespace robotics
}  // namespace ohstem

#endif
