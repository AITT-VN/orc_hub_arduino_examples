#include "DriveBase.h"

#include <math.h>

#include "AngleSensor.h"

namespace ohstem {
namespace robotics {

DriveBase::DriveBase(DriveMode driveMode, DCMotorBase* m1, DCMotorBase* m2,
                     DCMotorBase* m3, DCMotorBase* m4)
    : driveMode_(driveMode) {
  motors_[0] = m1;
  motors_[1] = m2;
  motors_[2] = m3;
  motors_[3] = m4;

  if (m1 != nullptr) {
    m1->reverse();
    left_[leftCount_++] = m1;
    if (m1->port() == E1 || m1->port() == E2) {
      leftEncoder_ = m1;
    }
  }
  if (m3 != nullptr) {
    m3->reverse();
    left_[leftCount_++] = m3;
    if (m3->port() == E1 || m3->port() == E2) {
      leftEncoder_ = m3;
    }
  }
  if (m2 != nullptr) {
    right_[rightCount_++] = m2;
    if (m2->port() == E1 || m2->port() == E2) {
      rightEncoder_ = m2;
    }
  }
  if (m4 != nullptr) {
    right_[rightCount_++] = m4;
    if (m4->port() == E1 || m4->port() == E2) {
      rightEncoder_ = m4;
    }
  }
}

bool DriveBase::hasDistanceFeedback() const {
  return leftEncoder_ != nullptr && rightEncoder_ != nullptr &&
         leftEncoder_->encoderEnabled() && rightEncoder_->encoderEnabled();
}

bool DriveBase::hasTurnFeedback(float steering) const {
  if (useGyro_ && angleSensor_ != nullptr) {
    return true;
  }
  if (steering > 0.0f) {
    return leftEncoder_ != nullptr && leftEncoder_->encoderEnabled();
  }
  return rightEncoder_ != nullptr && rightEncoder_->encoderEnabled();
}

void DriveBase::speed(float speedValue, float minSpeed) {
  speed_ = speedValue;
  minSpeed_ = (minSpeed >= 0.0f) ? minSpeed : speedValue / 2.0f;
}

void DriveBase::size(float wheelDiameterMm, float widthMm) {
  if (wheelDiameterMm <= 0.0f || widthMm <= 0.0f) {
    return;
  }
  wheelDiameter_ = wheelDiameterMm;
  width_ = widthMm;
  wheelCircumference_ = wheelDiameter_ * PI;
}

void DriveBase::speedRatio(float left, float right) {
  speedRatioLeft_ = left;
  speedRatioRight_ = right;
}

void DriveBase::onTeleopCommand(GamepadButton button, void (*callback)()) {
  teleopHandlers_[static_cast<size_t>(button)] = callback;
}

void DriveBase::forward() { run(DIR_FW); }

void DriveBase::backward() { run(DIR_BW); }

void DriveBase::turnLeft() { run(DIR_L); }

void DriveBase::turnRight() { run(DIR_R); }

void DriveBase::moveLeft() {
  if (driveMode_ != MODE_MECANUM) {
    turnLeft();
  } else {
    run(DIR_SL);
  }
}

void DriveBase::moveRight() {
  if (driveMode_ != MODE_MECANUM) {
    turnRight();
  } else {
    run(DIR_SR);
  }
}

void DriveBase::forwardFor(float amount, MoveUnit unit, StopAction thenAction) {
  straight(speed_, amount, unit, thenAction);
}

void DriveBase::backwardFor(float amount, MoveUnit unit, StopAction thenAction) {
  straight(-speed_, amount, unit, thenAction);
}

void DriveBase::turnLeftFor(float amount, MoveUnit unit, StopAction thenAction) {
  turn(-100.0f, amount, unit, thenAction);
}

void DriveBase::turnRightFor(float amount, MoveUnit unit, StopAction thenAction) {
  turn(100.0f, amount, unit, thenAction);
}

void DriveBase::moveLeftFor(float amount, MoveUnit unit, StopAction thenAction) {
  if (driveMode_ != MODE_MECANUM) {
    turnLeftFor(amount, unit, thenAction);
    return;
  }
  if (unit != SECOND || amount <= 0.0f) {
    return;
  }

  const float durationMs = amount * 1000.0f;
  const uint32_t startMs = millis();
  float lastDriven = 0.0f;
  while (true) {
    const float driven = static_cast<float>(elapsedMillis(startMs));
    if (driven >= durationMs) {
      break;
    }
    const float adjusted = calcSpeed(fabs(speed_), durationMs, driven, lastDriven);
    run(DIR_SL, adjusted);
    lastDriven = driven;
    delay(10);
  }
  stopThen(thenAction);
}

void DriveBase::moveRightFor(float amount, MoveUnit unit, StopAction thenAction) {
  if (driveMode_ != MODE_MECANUM) {
    turnRightFor(amount, unit, thenAction);
    return;
  }
  if (unit != SECOND || amount <= 0.0f) {
    return;
  }

  const float durationMs = amount * 1000.0f;
  const uint32_t startMs = millis();
  float lastDriven = 0.0f;
  while (true) {
    const float driven = static_cast<float>(elapsedMillis(startMs));
    if (driven >= durationMs) {
      break;
    }
    const float adjusted = calcSpeed(fabs(speed_), durationMs, driven, lastDriven);
    run(DIR_SR, adjusted);
    lastDriven = driven;
    delay(10);
  }
  stopThen(thenAction);
}

void DriveBase::tickAngleSensor() {
  if (angleSensor_ != nullptr) {
    angleSensor_->update();
  }
}

void DriveBase::straight(float speedValue, float amount, MoveUnit unit,
                         StopAction thenAction) {
  if (amount <= 0.0f) {
    stopThen(thenAction);
    return;
  }
  if ((unit == CM || unit == INCH) && !hasDistanceFeedback()) {
    stopThen(thenAction);
    return;
  }

  resetAngle();
  float targetDistance = 0.0f;
  float driven = 0.0f;
  float lastDriven = 0.0f;
  const float speedDir = speedValue < 0.0f ? -1.0f : 1.0f;
  pid_.reset();

  const uint32_t startMs = millis();
  if (unit == CM) {
    targetDistance = fabs(amount) * 10.0f;
  } else if (unit == INCH) {
    targetDistance = fabs(amount) * 25.4f;
  } else {
    targetDistance = fabs(amount) * 1000.0f;
  }

  while (true) {
    tickAngleSensor();
    driven = unit == SECOND ? static_cast<float>(elapsedMillis(startMs))
                            : fabs(distance());

    if (driven >= targetDistance) {
      break;
    }

    float expectedSpeed = speedValue;
    if (!((unit == SECOND && amount < 2.0f) || (unit == CM && amount < 10.0f) ||
          (unit == INCH && amount < 4.0f))) {
      expectedSpeed =
          speedDir * calcSpeed(fabs(speedValue), targetDistance, driven, lastDriven);
    }

    float leftSpeed = expectedSpeed;
    float rightSpeed = expectedSpeed;
    calibSpeed(expectedSpeed, leftSpeed, rightSpeed);
    runSpeed(leftSpeed, rightSpeed);
    lastDriven = driven;
    delay(5);
  }

  stopThen(thenAction);
}

void DriveBase::turn(float steering, float amount, MoveUnit unit,
                     StopAction thenAction) {
  if (amount == 0.0f) {
    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;
    calcSteering(speed_, steering, leftSpeed, rightSpeed);
    runSpeed(leftSpeed, rightSpeed);
    return;
  }
  if (amount < 0.0f) {
    amount = fabs(amount);
  }
  if (unit == DEGREE && !hasTurnFeedback(steering)) {
    stopThen(thenAction);
    return;
  }

  float target = 0.0f;
  float driven = 0.0f;
  float lastDriven = 0.0f;
  const uint32_t startMs = millis();

  if (unit == DEGREE) {
    if (useGyro_ && angleSensor_ != nullptr) {
      target = min(fabs(amount), 359.0f);
      angleSensor_->reset();
    } else {
      target = fabs((PI * width_) * (amount / 360.0f));
      resetAngle();
    }
  } else {
    target = fabs(amount) * 1000.0f;
  }

  const float wheelCircDegree = wheelCircumference_ / 360.0f;

  while (true) {
    tickAngleSensor();

    if (unit == SECOND) {
      driven = static_cast<float>(elapsedMillis(startMs));
    } else if (useGyro_ && angleSensor_ != nullptr) {
      driven = fabs(angleSensor_->heading());
    } else if (steering > 0.0f && leftEncoder_ != nullptr) {
      driven = fabs(leftEncoder_->angle()) * wheelCircDegree;
    } else if (rightEncoder_ != nullptr) {
      driven = fabs(rightEncoder_->angle()) * wheelCircDegree;
    } else {
      driven = 0.0f;
    }

    float expectedSpeed = speed_;
    if (!((unit == SECOND && amount < 1.0f) || (unit == DEGREE && amount < 45.0f))) {
      expectedSpeed = calcSpeed(speed_, target, driven, lastDriven);
    }

    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;
    calcSteering(expectedSpeed, steering, leftSpeed, rightSpeed);
    runSpeed(leftSpeed, rightSpeed);
    lastDriven = driven;

    if (driven >= target) {
      break;
    }
    delay(5);
  }

  stopThen(thenAction);
}

void DriveBase::run(Direction dir, float speedValue) {
  const float s = isnan(speedValue) ? speed_ : fabs(speedValue);

  if (driveMode_ == MODE_MECANUM) {
    const float factors[10][4] = {
        {1, 1, 1, 1},
        {1, 0, 0, 1},
        {1, -1, 1, -1},
        {0, -1, -1, 0},
        {-1, -1, -1, -1},
        {-1, 0, 0, -1},
        {-1, 1, -1, 1},
        {0, 1, 1, 0},
        {-1.2f, 1.2f, 1.2f, -1.2f},
        {1.2f, -1.2f, -1.2f, 1.2f}};

    if (motors_[0]) motors_[0]->run(s * factors[dir][0] * speedRatioLeft_);
    if (motors_[1]) motors_[1]->run(s * factors[dir][1] * speedRatioRight_);
    if (motors_[2]) motors_[2]->run(s * factors[dir][2] * speedRatioLeft_);
    if (motors_[3]) motors_[3]->run(s * factors[dir][3] * speedRatioRight_);
    return;
  }

  switch (dir) {
    case DIR_FW:
      runSpeed(s, s);
      break;
    case DIR_BW:
      runSpeed(-s, -s);
      break;
    case DIR_L:
      runSpeed(-s, s);
      break;
    case DIR_R:
      runSpeed(s, -s);
      break;
    case DIR_RF:
      runSpeed(s, s / 2.0f);
      break;
    case DIR_LF:
      runSpeed(s / 2.0f, s);
      break;
    case DIR_RB:
      runSpeed(-s, -s / 2.0f);
      break;
    case DIR_LB:
      runSpeed(-s / 2.0f, -s);
      break;
    default:
      stop();
      break;
  }
}

void DriveBase::runSpeed(float leftSpeed, float rightSpeed) {
  if (isnan(rightSpeed)) {
    rightSpeed = leftSpeed;
  }
  for (uint8_t i = 0; i < leftCount_; ++i) {
    if (left_[i] != nullptr) {
      left_[i]->run(leftSpeed * speedRatioLeft_);
    }
  }
  for (uint8_t i = 0; i < rightCount_; ++i) {
    if (right_[i] != nullptr) {
      right_[i]->run(rightSpeed * speedRatioRight_);
    }
  }
}

void DriveBase::stop() {
  for (auto* motor : motors_) {
    if (motor != nullptr) {
      motor->stop();
    }
  }
}

void DriveBase::brake() {
  for (auto* motor : motors_) {
    if (motor != nullptr) {
      motor->brake();
    }
  }
}

void DriveBase::stopThen(StopAction thenAction) {
  if (thenAction == BRAKE) {
    brake();
    delay(500);
    stop();
  } else if (thenAction == STOP) {
    stop();
  }
}

float DriveBase::distance() {
  if (!hasDistanceFeedback()) {
    return 0.0f;
  }
  const float angleValue =
      (fabs(leftEncoder_->angle()) + fabs(rightEncoder_->angle())) / 2.0f;
  return (angleValue * wheelCircumference_) / 360.0f;
}

float DriveBase::angle() {
  tickAngleSensor();
  return angleSensor_ != nullptr ? angleSensor_->heading() : 0.0f;
}

void DriveBase::resetAngle() {
  if (angleSensor_ != nullptr) {
    angleSensor_->reset();
  }
  for (auto* motor : motors_) {
    if (motor != nullptr) {
      motor->resetAngle();
    }
  }
}

void DriveBase::updateTeleop(Gamepad& gamepad, uint8_t accelSteps) {
  gamepad.update();
  updateTeleop(gamepad.state(), accelSteps);
}

void DriveBase::updateTeleop(const GamepadState& state, uint8_t accelSteps) {
  if (modeAuto_) {
    return;
  }

  int8_t dir = -1;
  GamepadButton activeButton = GamepadButton::Count;

  if (state.leftDistance > 50) {
    dir = state.leftDirection;
    if (driveMode_ == MODE_MECANUM && sideMoveMode_ == JOYSTICK) {
      if (dir == DIR_L) {
        dir = DIR_SL;
      } else if (dir == DIR_R) {
        dir = DIR_SR;
      }
    }
  } else if (state.up && state.left) {
    dir = DIR_LF;
    activeButton = GamepadButton::Up;
  } else if (state.up && state.right) {
    dir = DIR_RF;
    activeButton = GamepadButton::Up;
  } else if (state.down && state.left) {
    dir = DIR_LB;
    activeButton = GamepadButton::Down;
  } else if (state.down && state.right) {
    dir = DIR_RB;
    activeButton = GamepadButton::Down;
  } else if (state.up) {
    dir = DIR_FW;
    activeButton = GamepadButton::Up;
  } else if (state.down) {
    dir = DIR_BW;
    activeButton = GamepadButton::Down;
  } else if (state.left) {
    dir = (driveMode_ == MODE_MECANUM && sideMoveMode_ == DPAD) ? DIR_SL : DIR_L;
    activeButton = GamepadButton::Left;
  } else if (state.right) {
    dir = (driveMode_ == MODE_MECANUM && sideMoveMode_ == DPAD) ? DIR_SR : DIR_R;
    activeButton = GamepadButton::Right;
  } else if (state.l1) {
    activeButton = GamepadButton::L1;
  } else if (state.r1) {
    activeButton = GamepadButton::R1;
  } else if (state.triangle) {
    activeButton = GamepadButton::Triangle;
  } else if (state.square) {
    activeButton = GamepadButton::Square;
  } else if (state.cross) {
    activeButton = GamepadButton::Cross;
  } else if (state.circle) {
    activeButton = GamepadButton::Circle;
  } else if (state.l2) {
    activeButton = GamepadButton::L2;
  } else if (state.r2) {
    activeButton = GamepadButton::R2;
  } else if (state.m1) {
    activeButton = GamepadButton::M1;
  } else if (state.m2) {
    activeButton = GamepadButton::M2;
  } else if (state.thumbL) {
    activeButton = GamepadButton::ThumbL;
  } else if (state.thumbR) {
    activeButton = GamepadButton::ThumbR;
  }

  if (dir != lastTeleopDir_) {
    teleopSpeed_ = minSpeed_;
    teleopTurnSpeed_ = minSpeed_;
  } else {
    teleopSpeed_ = min(teleopSpeed_ + accelSteps, speed_);
    teleopTurnSpeed_ = min(teleopTurnSpeed_ + static_cast<float>(accelSteps) / 2.0f, speed_);
  }

  bool handledCommand = false;
  if (activeButton != GamepadButton::Count) {
    const size_t idx = static_cast<size_t>(activeButton);
    const bool pressed = state.pressed(activeButton);
    if (teleopHandlers_[idx] != nullptr && pressed && !lastTeleopButtons_[idx]) {
      teleopHandlers_[idx]();
      handledCommand = true;
    }
  }

  if (!handledCommand) {
    if (dir == DIR_FW || dir == DIR_BW || dir == DIR_SL || dir == DIR_SR) {
      run(static_cast<Direction>(dir), teleopSpeed_);
    } else if (dir == DIR_L || dir == DIR_R || dir == DIR_LF || dir == DIR_RF ||
               dir == DIR_LB || dir == DIR_RB) {
      run(static_cast<Direction>(dir), teleopTurnSpeed_);
    } else if (activeButton == GamepadButton::Count) {
      stop();
    }
  }

  for (size_t i = 0; i < static_cast<size_t>(GamepadButton::Count); ++i) {
    lastTeleopButtons_[i] = state.pressed(static_cast<GamepadButton>(i));
  }
  lastTeleopDir_ = dir;
}

float DriveBase::calcSpeed(float speedValue, float distanceValue, float drivenDistance,
                           float lastDrivenDistance) const {
  (void)lastDrivenDistance;
  const float startSpeed = minSpeed_;
  const float maxSpeed = speedValue;
  const float endSpeed = startSpeed;
  const float accelDistance = 0.3f * distanceValue;
  const float decelDistance = 0.7f * distanceValue;

  if (drivenDistance <= 0.0f) {
    return startSpeed;
  }
  if (fabs(drivenDistance) < fabs(accelDistance)) {
    return startSpeed + (maxSpeed - startSpeed) * drivenDistance / accelDistance;
  }
  if (fabs(drivenDistance) > fabs(decelDistance)) {
    return maxSpeed - (maxSpeed - endSpeed) * (drivenDistance - decelDistance) /
                          (distanceValue - decelDistance);
  }
  return speedValue;
}

void DriveBase::calibSpeed(float speedValue, float& left, float& right) {
  float angleError = 0.0f;
  if (useGyro_) {
    tickAngleSensor();
    if (angleSensor_ == nullptr) {
      left = speedValue;
      right = speedValue;
      return;
    }
    angleError = angleSensor_->heading();
  } else {
    const float leftTicks = leftEncoder_ != nullptr ? fabs(leftEncoder_->encoderTicks()) : 0.0f;
    const float rightTicks =
        rightEncoder_ != nullptr ? fabs(rightEncoder_->encoderTicks()) : 0.0f;
    angleError = speedValue > 0 ? leftTicks - rightTicks : rightTicks - leftTicks;
  }

  const float correction = pid_.compute(angleError);
  left = speedValue + correction;
  right = speedValue - correction;
}

void DriveBase::calcSteering(float speedValue, float steering, float& left,
                             float& right) const {
  if (steering > 0.0f) {
    left = speedValue;
    right = -2.0f * (speedValue / 100.0f) * steering + speedValue;
  } else if (steering < 0.0f) {
    right = speedValue;
    left = -2.0f * (speedValue / 100.0f) * fabs(steering) + speedValue;
  } else {
    left = speedValue;
    right = speedValue;
  }
}

void DriveBase::followLine(bool backward, LineState lineState) {
  if (lineSensor_ == nullptr) {
    return;
  }
  if (lineState == LINE_END) {
    lineState = lineSensor_->check();
  }

  if (lineState == LINE_END) {
    if (backward) {
      run(DIR_BW, minSpeed_);
    }
  } else if (lineState == LINE_CENTER) {
    if (lastLineState_ == LINE_CENTER) {
      forward();
    } else {
      run(DIR_FW, minSpeed_);
    }
  } else if (lineState == LINE_CROSS) {
    run(DIR_FW, minSpeed_);
  } else if (lineState == LINE_RIGHT) {
    runSpeed(minSpeed_, minSpeed_ * 1.25f);
  } else if (lineState == LINE_RIGHT2) {
    runSpeed(0, minSpeed_);
  } else if (lineState == LINE_RIGHT3) {
    while (lineState != LINE_CENTER && lineState != LINE_LEFT) {
      runSpeed(-minSpeed_, minSpeed_);
      lineState = lineSensor_->check();
    }
  } else if (lineState == LINE_LEFT) {
    runSpeed(minSpeed_ * 1.25f, minSpeed_);
  } else if (lineState == LINE_LEFT2) {
    runSpeed(minSpeed_, 0);
  } else if (lineState == LINE_LEFT3) {
    while (lineState != LINE_CENTER && lineState != LINE_RIGHT) {
      runSpeed(minSpeed_, -minSpeed_);
      lineState = lineSensor_->check();
    }
  }

  lastLineState_ = lineState;
}

void DriveBase::followLineUntilEnd(StopAction thenAction) {
  if (lineSensor_ == nullptr) {
    return;
  }
  int count = 2;
  while (true) {
    const LineState lineState = lineSensor_->check();
    if (lineState == LINE_END) {
      --count;
      if (count == 0) {
        break;
      }
    }
    followLine(false, lineState);
    delay(10);
  }
  stopThen(thenAction);
}

void DriveBase::followLineUntilCross(StopAction thenAction) {
  if (lineSensor_ == nullptr) {
    return;
  }

  int status = 1;
  int count = 0;
  while (true) {
    const LineState lineState = lineSensor_->check();
    if (status == 1) {
      if (lineState != LINE_CROSS) {
        status = 2;
      }
    } else if (status == 2) {
      if (lineState == LINE_CROSS) {
        ++count;
        if (count == 2) {
          break;
        }
      }
    }
    followLine(true, lineState);
    delay(status == 2 && count == 1 ? 20 : 10);
  }
  stopThen(thenAction);
}

void DriveBase::followLineByTime(float seconds, StopAction thenAction) {
  if (lineSensor_ == nullptr || seconds <= 0.0f) {
    return;
  }

  const uint32_t startMs = millis();
  const uint32_t durationMs = static_cast<uint32_t>(seconds * 1000.0f);
  while (elapsedMillis(startMs) < durationMs) {
    followLine(true, lineSensor_->check());
    delay(10);
  }
  stopThen(thenAction);
}

void DriveBase::turnUntilLineDetected(float steering, StopAction thenAction) {
  if (lineSensor_ == nullptr) {
    return;
  }

  int counter = 3;
  int status = 0;
  turn(steering);

  while (true) {
    const LineState lineState = lineSensor_->check();
    if (status == 0) {
      if (lineState == LINE_END) {
        status = 1;
      }
    } else if (status == 1) {
      if (lineState != LINE_END) {
        turn(steering * 0.75f);
        --counter;
        if (counter <= 0) {
          break;
        }
      }
    }
    delay(10);
  }
  stopThen(thenAction);
}

}  // namespace robotics
}  // namespace ohstem
