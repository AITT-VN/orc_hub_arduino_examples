#include <Wire.h>
#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Lesson 01 - Basic 4WD robot movement
// Change the motor ports below if your 4-wheel robot uses a different layout.
// DriveBase automatically reverses the left side internally.
// If your chassis wiring is unusual, you can set reversed=true on a single
// motor or call motor.reverse() inside setup() before driving.
// Syntax to reverse only one motor:
//   DCMotor frontLeftMotor(motorDriver, M1, true);
// or:
//   void setup() {
//     frontLeftMotor.reverse();
//   }
MotorDriverV2 motorDriver(Wire);
DCMotor frontLeftMotor(motorDriver, M1);
DCMotor frontRightMotor(motorDriver, M2);
DCMotor rearLeftMotor(motorDriver, E1);
DCMotor rearRightMotor(motorDriver, E2);
DriveBase robot(MODE_4WD, &frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Example for reversing exactly one motor in software:
  // frontLeftMotor.reverse();

  if (!motorDriver.begin()) {
    Serial.println("Khong tim thay MotorDriverV2");
    while (true) {
      delay(100);
    }
  }

  robot.speed(60, 35);
  Serial.println("Bai 01 - Robot 4 banh di chuyen");
}

void loop() {
  // Demonstrate the same movement sequence on a 4-wheel drive base.
  robot.forwardFor(1.5f, SECOND, BRAKE);
  delay(600);

  robot.backwardFor(1.0f, SECOND, BRAKE);
  delay(600);

  robot.turnLeftFor(0.8f, SECOND, BRAKE);
  delay(600);

  robot.turnRightFor(0.8f, SECOND, BRAKE);
  delay(1200);
}
