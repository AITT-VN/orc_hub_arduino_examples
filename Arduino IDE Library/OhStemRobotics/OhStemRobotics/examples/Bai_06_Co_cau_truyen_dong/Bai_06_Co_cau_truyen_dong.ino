#include <Wire.h>
#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Lesson 06 - Mechanism control
// This demo uses one DC motor and one servo connected to the same driver.
MotorDriverV2 motorDriver(Wire);
DCMotor mechanismMotor(motorDriver, M1);
RobotServo gateServo(motorDriver, S1, 180);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!motorDriver.begin()) {
    Serial.println("Khong tim thay MotorDriverV2");
    while (true) {
      delay(100);
    }
  }

  gateServo.begin();
  gateServo.limit(20, 130);
  gateServo.setAngle(30);

  Serial.println("Bai 06 - Co cau truyen dong");
}

void loop() {
  // Open the gate, run the mechanism forward, then close and reverse it.
  gateServo.runAngle(30, 80);
  delay(300);
  mechanismMotor.runTime(70, 1800, BRAKE);
  delay(600);

  gateServo.runAngle(110, 80);
  delay(300);
  mechanismMotor.runTime(-60, 1200, BRAKE);
  delay(1500);
}
