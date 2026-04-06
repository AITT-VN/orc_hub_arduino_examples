#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Uses an MPU6050 and AngleSensor to make turn commands more accurate.
MotorDriverV2 motorDriver(Wire);
DCMotor leftMotor(motorDriver, E1);
DCMotor rightMotor(motorDriver, E2);
DriveBase robot(MODE_2WD, &leftMotor, &rightMotor);

MPU6050Sensor imu(Wire);
AngleSensor angleSensor(imu);

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
  robot.speed(50, 30);
  robot.angleSensor(&angleSensor);
  robot.useGyro(true);

  Serial.println("Keep the robot still for MPU6050 calibration...");
  angleSensor.calibrate(300);
  Serial.println("GyroTurns example started");
}

void loop() {
  Serial.println("Turn left 90 deg with gyro");
  robot.turnLeftFor(90, DEGREE, BRAKE);
  Serial.println(angleSensor.printData());
  delay(1000);

  Serial.println("Turn right 180 deg with gyro");
  robot.turnRightFor(180, DEGREE, BRAKE);
  Serial.println(angleSensor.printData());
  delay(1500);
}
