#include <OhStemRobotics.h>

using namespace ohstem::robotics;

MotorDriverV2 motorDriver(Wire);
DCMotor motor1(motorDriver, M1);
DCMotor motor2(motorDriver, M2);
DCMotor motor3(motorDriver, E1);
DCMotor motor4(motorDriver, E2);

// Matches the common Yolo Uno mapping used in the Python runtime examples.
DriveBase robot(MODE_MECANUM, &motor2, &motor1, &motor3, &motor4);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!motorDriver.begin()) {
    Serial.println("MotorDriverV2 not found");
    while (true) {
      delay(100);
    }
  }

  motor3.setEncoder(350, 11, 34);
  motor4.setEncoder(350, 11, 34);

  robot.size(80, 300);
  robot.speed(60, 40);

  Serial.println("BasicDrive example started");
}

void loop() {
  Serial.println("Forward 40 cm");
  robot.forwardFor(40, CM, BRAKE);
  delay(1000);

  Serial.println("Backward 40 cm");
  robot.backwardFor(40, CM, BRAKE);
  delay(1000);

  Serial.println("Turn left 90 deg");
  robot.turnLeftFor(90, DEGREE, BRAKE);
  delay(1000);

  Serial.println("Turn right 90 deg");
  robot.turnRightFor(90, DEGREE, BRAKE);
  delay(1000);

  Serial.println("Strafe left 0.8 s");
  robot.moveLeftFor(0.8f, SECOND, BRAKE);
  delay(1000);

  Serial.println("Strafe right 0.8 s");
  robot.moveRightFor(0.8f, SECOND, BRAKE);
  delay(1500);
}
