#include <OhStemRobotics.h>

using namespace ohstem::robotics;

MotorDriverV2 motorDriver(Wire);
DCMotor leftMotor(motorDriver, E1);
DCMotor rightMotor(motorDriver, E2);
DriveBase robot(MODE_2WD, &leftMotor, &rightMotor);

void printEncoderReport(const char* label) {
  Serial.print(label);
  Serial.print(" distance(mm): ");
  Serial.print(robot.distance(), 1);
  Serial.print(" left angle(deg): ");
  Serial.print(leftMotor.angle(), 1);
  Serial.print(" right angle(deg): ");
  Serial.println(rightMotor.angle(), 1);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!motorDriver.begin()) {
    Serial.println("MotorDriverV2 not found");
    while (true) {
      delay(100);
    }
  }

  leftMotor.setEncoder(350, 11, 34);
  rightMotor.setEncoder(350, 11, 34);

  robot.size(80, 300);
  robot.speed(55, 35);
  robot.pid(4.5f, 0.12f, 0.05f);

  Serial.println("EncoderDistance example started");
}

void loop() {
  Serial.println("Forward 50 cm");
  robot.forwardFor(50, CM, BRAKE);
  printEncoderReport("After forward");
  delay(1000);

  Serial.println("Backward 30 cm");
  robot.backwardFor(30, CM, BRAKE);
  printEncoderReport("After backward");
  delay(1000);

  Serial.println("Turn left 90 deg using encoder feedback");
  robot.turnLeftFor(90, DEGREE, BRAKE);
  printEncoderReport("After left turn");
  delay(1500);
}
