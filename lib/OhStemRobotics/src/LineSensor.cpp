#include "LineSensor.h"

namespace ohstem {
namespace robotics {

bool LineSensor2P::begin() {
  pinMode(s1Pin_, INPUT);
  pinMode(s2Pin_, INPUT);
  return true;
}

int LineSensor2P::read(uint8_t index) {
  if (index == 0) return digitalRead(s1Pin_);
  if (index == 1) return digitalRead(s2Pin_);
  return 0;
}

LineState LineSensor2P::check() {
  const int s1 = digitalRead(s1Pin_);
  const int s2 = digitalRead(s2Pin_);
  if (s1 == 0 && s2 == 0) return LINE_CENTER;
  if (s1 == 0 && s2 == 1) return LINE_LEFT2;
  if (s1 == 1 && s2 == 0) return LINE_RIGHT2;
  return LINE_CROSS;
}

bool LineSensor3P::begin() {
  pinMode(s1Pin_, INPUT);
  pinMode(s2Pin_, INPUT);
  pinMode(s3Pin_, INPUT);
  return true;
}

int LineSensor3P::read(uint8_t index) {
  if (index == 0) return digitalRead(s1Pin_);
  if (index == 1) return digitalRead(s2Pin_);
  if (index == 2) return digitalRead(s3Pin_);
  return 0;
}

LineState LineSensor3P::check() {
  const int s1 = digitalRead(s1Pin_);
  const int s2 = digitalRead(s2Pin_);
  const int s3 = digitalRead(s3Pin_);

  if (s1 == 1 && s2 == 1 && s3 == 1) return LINE_CROSS;
  if (s1 == 0 && s2 == 0 && s3 == 0) return LINE_END;
  if (s1 == 1 && s2 == 1 && s3 == 0) return LINE_RIGHT;
  if (s1 == 0 && s2 == 1 && s3 == 1) return LINE_LEFT;
  if (s1 == 1 && s2 == 0 && s3 == 0) return LINE_RIGHT2;
  if (s1 == 0 && s2 == 0 && s3 == 1) return LINE_LEFT2;
  return LINE_CENTER;
}

bool LineSensorI2C::begin() { return pcf_.begin(); }

int LineSensorI2C::read(uint8_t index) { return pcf_.pin(index); }

LineState LineSensorI2C::check() {
  const int s0 = read(0);
  const int s1 = read(1);
  const int s2 = read(2);
  const int s3 = read(3);

  if (s0 == 0 && s1 == 0 && s2 == 0 && s3 == 0) return LINE_END;
  if (s0 == 1 && s1 == 1 && s2 == 1 && s3 == 1) return LINE_CROSS;
  if ((s1 == 1 && s2 == 1) || (s0 == 1 && s1 == 0 && s2 == 0 && s3 == 1)) {
    return LINE_CENTER;
  }
  if (s0 == 1 && s1 == 1) return LINE_RIGHT2;
  if (s2 == 1 && s3 == 1) return LINE_LEFT2;
  if (s0 == 0 && s1 == 0 && s2 == 1 && s3 == 0) return LINE_RIGHT;
  if (s0 == 0 && s1 == 1 && s2 == 0 && s3 == 0) return LINE_LEFT;
  if (s1 == 1) return LINE_RIGHT2;
  if (s2 == 1) return LINE_LEFT2;
  if (s0 == 1) return LINE_RIGHT3;
  if (s3 == 1) return LINE_LEFT3;
  return LINE_END;
}

}  // namespace robotics
}  // namespace ohstem
