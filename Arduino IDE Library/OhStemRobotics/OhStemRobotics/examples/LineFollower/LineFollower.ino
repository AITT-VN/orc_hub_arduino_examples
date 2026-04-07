#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Two drive motors and one I2C line sensor are enough for a simple 2WD
// line-following robot.
MotorDriverV2 motorDriver(Wire);
DCMotor leftMotor(motorDriver, E1, true);
DCMotor rightMotor(motorDriver, E2, true);
DriveBase robot(MODE_2WD, &leftMotor, &rightMotor);
LineSensorI2C lineSensor(Wire, 0x23);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!motorDriver.begin()) {
    Serial.println("MotorDriverV2 not found");
    while (true) {
      delay(100);
    }
  }

  if (!lineSensor.begin()) {
    Serial.println("Line sensor not found");
    while (true) {
      delay(100);
    }
  }

  leftMotor.setEncoder(600, 11, 34);
  rightMotor.setEncoder(600, 11, 34);

  // The line-follow helper uses the attached sensor plus min/max drive speed.
  robot.speed(55, 35);
  robot.lineSensor(&lineSensor);

  Serial.println("Line follower example started");
}

void loop() {
  // Read the current line state and let the drive base decide the correction.
  robot.followLine(true, lineSensor.check());
  delay(10);
}
