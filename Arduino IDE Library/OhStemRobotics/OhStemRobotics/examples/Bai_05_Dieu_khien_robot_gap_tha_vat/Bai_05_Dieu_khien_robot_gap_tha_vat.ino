#include <Wire.h>
#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Lesson 05 - Pick and place with two servos
// S1 controls the arm and S2 controls the claw.
MotorDriverV2 motorDriver(Wire);
DCMotor leftMotor(motorDriver, E1);
DCMotor rightMotor(motorDriver, E2);
DriveBase robot(MODE_2WD, &leftMotor, &rightMotor);
RobotServo armServo(motorDriver, S1, 180);
RobotServo clawServo(motorDriver, S2, 180);

void pickObject() {
  // Open -> lower -> close -> lift.
  clawServo.runAngle(105, 70);
  delay(400);
  armServo.runAngle(130, 70);
  delay(400);
  clawServo.runAngle(45, 70);
  delay(500);
  armServo.runAngle(70, 70);
  delay(400);
}

void dropObject() {
  // Lower -> open -> return to the travel position.
  armServo.runAngle(120, 70);
  delay(400);
  clawServo.runAngle(105, 70);
  delay(500);
  armServo.runAngle(80, 70);
  delay(400);
  clawServo.runAngle(45, 70);
  delay(300);
  armServo.runAngle(125, 70);
  delay(400);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!motorDriver.begin()) {
    Serial.println("Khong tim thay MotorDriverV2");
    while (true) {
      delay(100);
    }
  }

  robot.speed(45, 30);
  armServo.begin();
  clawServo.begin();

  armServo.limit(60, 140);
  clawServo.limit(35, 110);

  // Start in a safe transport pose.
  armServo.setAngle(125);
  clawServo.setAngle(45);

  Serial.println("Bai 05 - Dieu khien robot gap tha vat");
}

void loop() {
  robot.forwardFor(1.0f, SECOND, BRAKE);
  delay(500);

  pickObject();
  delay(500);

  robot.backwardFor(1.0f, SECOND, BRAKE);
  delay(500);

  dropObject();
  delay(1500);
}
