#include <OhStemRobotics.h>

using namespace ohstem::robotics;

MotorDriverV2 motorDriver(Wire);
DCMotor motor1(motorDriver, M1);
DCMotor motor2(motorDriver, M2);
DCMotor motor3(motorDriver, E1, true);
DCMotor motor4(motorDriver, E2, true);
DriveBase robot(MODE_MECANUM, &motor1, &motor2, &motor3, &motor4);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!motorDriver.begin()) {
    Serial.println("MotorDriverV2 not found");
    while (true) {
      delay(100);
    }
  }

  motor3.setEncoder(600, 11, 34);
  motor4.setEncoder(600, 11, 34);

  robot.size(80, 300);
  robot.speed(60, 45);

  Serial.println("BasicDrive example started");
}

void loop() {
  robot.forwardFor(40, CM, BRAKE);
  delay(1000);
  robot.backwardFor(40, CM, BRAKE);
  delay(1000);
  robot.turnLeftFor(90, DEGREE, BRAKE);
  delay(1000);
  robot.turnRightFor(90, DEGREE, BRAKE);
  delay(1500);
}
