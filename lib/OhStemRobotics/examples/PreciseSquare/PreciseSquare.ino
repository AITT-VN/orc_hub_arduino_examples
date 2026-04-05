#include <OhStemRobotics.h>

using namespace ohstem::robotics;

MotorDriverV2 motorDriver(Wire);
DCMotor leftMotor(motorDriver, E1);
DCMotor rightMotor(motorDriver, E2);
DriveBase robot(MODE_2WD, &leftMotor, &rightMotor);

MPU6050Sensor imu(Wire);
AngleSensor angleSensor(imu);

void driveEdge(uint8_t edgeIndex) {
  Serial.print("Edge ");
  Serial.println(edgeIndex);

  robot.forwardFor(40, CM, BRAKE);
  Serial.print("  distance(mm): ");
  Serial.println(robot.distance(), 1);
  delay(400);

  robot.turnRightFor(90, DEGREE, BRAKE);
  Serial.print("  heading(deg): ");
  Serial.println(robot.angle(), 1);
  delay(400);
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

  if (!imu.begin()) {
    Serial.println("MPU6050 not found");
    while (true) {
      delay(100);
    }
  }

  leftMotor.setEncoder(350, 11, 34);
  rightMotor.setEncoder(350, 11, 34);

  robot.size(80, 300);
  robot.speed(55, 35);
  robot.pid(4.5f, 0.12f, 0.05f);
  robot.angleSensor(&angleSensor);
  robot.useGyro(true);

  Serial.println("Keep the robot still for MPU6050 calibration...");
  angleSensor.calibrate(300);
  Serial.println("PreciseSquare example started");
}

void loop() {
  for (uint8_t edge = 1; edge <= 4; ++edge) {
    driveEdge(edge);
  }

  Serial.println("Square finished");
  delay(3000);
}
