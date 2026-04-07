#include <OhStemRobotics.h>

using namespace ohstem::robotics;

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

  robot.speed(55, 35);
  robot.lineSensor(&lineSensor);

  Serial.println("Line follower example started");
}

void loop() {
  robot.followLine(true, lineSensor.check());
  delay(10);
}
