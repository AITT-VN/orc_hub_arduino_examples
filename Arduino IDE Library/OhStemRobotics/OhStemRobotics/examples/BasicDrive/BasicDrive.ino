#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Four motors are combined into a mecanum drive base so the sketch can
// demonstrate straight moves and in-place turns with one high-level API.
// DriveBase automatically reverses the left side internally.
// If one wheel still spins the wrong way in a custom build, set reversed=true
// on that motor or call motor.reverse() inside setup().
MotorDriverV2 motorDriver(Wire);
DCMotor motor1(motorDriver, M1);
DCMotor motor2(motorDriver, M2);
DCMotor motor3(motorDriver, E1, true);
DCMotor motor4(motorDriver, E2, true);
DriveBase robot(MODE_MECANUM, &motor1, &motor2, &motor3, &motor4);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Example:
  // motor1.reverse();

  if (!motorDriver.begin()) {
    Serial.println("MotorDriverV2 not found");
    while (true) {
      delay(100);
    }
  }

  motor3.setEncoder(600, 11, 34);
  motor4.setEncoder(600, 11, 34);

  // These geometry values let distance/angle helpers convert motion targets
  // into encoder travel.
  robot.size(80, 300);
  robot.speed(60, 45);

  Serial.println("BasicDrive example started");
}

void loop() {
  // Run a short demo sequence repeatedly.
  robot.forwardFor(40, CM, BRAKE);
  delay(1000);
  robot.backwardFor(40, CM, BRAKE);
  delay(1000);
  robot.turnLeftFor(90, DEGREE, BRAKE);
  delay(1000);
  robot.turnRightFor(90, DEGREE, BRAKE);
  delay(1500);
}
