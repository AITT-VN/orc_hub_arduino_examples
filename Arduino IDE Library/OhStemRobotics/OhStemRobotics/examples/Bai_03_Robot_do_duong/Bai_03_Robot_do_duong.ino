#include <Wire.h>
#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Lesson 03 - Line following
// The default I2C line sensor address is 0x23.
MotorDriverV2 motorDriver(Wire);
DCMotor leftMotor(motorDriver, E1);
DCMotor rightMotor(motorDriver, E2);
DriveBase robot(MODE_2WD, &leftMotor, &rightMotor);
LineSensorI2C lineSensor(Wire, 0x23);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!motorDriver.begin()) {
    Serial.println("Khong tim thay MotorDriverV2");
    while (true) {
      delay(100);
    }
  }

  if (!lineSensor.begin()) {
    Serial.println("Khong tim thay cam bien line I2C");
    while (true) {
      delay(100);
    }
  }

  robot.speed(50, 35);
  robot.lineSensor(&lineSensor);

  Serial.println("Bai 03 - Robot do duong");
}

void loop() {
  // followLine() uses the current line pattern to steer back to the track.
  robot.followLine(true, lineSensor.check());
  delay(10);
}
