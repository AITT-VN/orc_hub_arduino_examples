#include <Wire.h>
#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Lesson 01 - Basic robot movement
// Change the motor ports below if your hardware wiring is different.
// DriveBase automatically reverses the left side internally.
// If a custom build still spins the wrong way, you can use:
// DCMotor leftMotor(motorDriver, E1, true);
// or call leftMotor.reverse() inside setup() before the robot starts moving.
// Syntax to reverse only one motor:
//   DCMotor leftMotor(motorDriver, E1, true);
// or:
//   void setup() {
//     leftMotor.reverse();
//   }
MotorDriverV2 motorDriver(Wire);
DCMotor leftMotor(motorDriver, E1);
DCMotor rightMotor(motorDriver, E2);
DriveBase robot(MODE_2WD, &leftMotor, &rightMotor);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Example for reversing exactly one motor in software:
  // leftMotor.reverse();

  if (!motorDriver.begin()) {
    Serial.println("Khong tim thay MotorDriverV2");
    while (true) {
      delay(100);
    }
  }

  robot.speed(60, 35);
  Serial.println("Bai 01 - Robot di chuyen");
}

void loop() {
  // Demonstrate the four most common movement commands.
  robot.forwardFor(1.5f, SECOND, BRAKE);
  delay(600);

  robot.backwardFor(1.0f, SECOND, BRAKE);
  delay(600);

  robot.turnLeftFor(0.8f, SECOND, BRAKE);
  delay(600);

  robot.turnRightFor(0.8f, SECOND, BRAKE);
  delay(1200);
}
