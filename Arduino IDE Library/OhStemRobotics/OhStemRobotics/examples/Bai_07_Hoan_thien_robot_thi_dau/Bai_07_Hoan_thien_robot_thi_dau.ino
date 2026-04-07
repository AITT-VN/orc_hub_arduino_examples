#include <Wire.h>
#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Lesson 07 - Competition robot
// M1 toggles between auto line mode and manual mode.
// L1/R1 open and close the claw.
// Triangle/Cross move the arm up and down.
MotorDriverV2 motorDriver(Wire);
DCMotor leftMotor(motorDriver, E1);
DCMotor rightMotor(motorDriver, E2);
DriveBase robot(MODE_2WD, &leftMotor, &rightMotor);
LineSensorI2C lineSensor(Wire, 0x23);
PS4GamepadReceiver ps4Receiver(Wire, 0x55);
Gamepad gamepad(&ps4Receiver);
RobotServo armServo(motorDriver, S1, 180);
RobotServo clawServo(motorDriver, S2, 180);
bool lastM1Pressed = false;
bool lastL1Pressed = false;
bool lastR1Pressed = false;
bool lastTrianglePressed = false;
bool lastCrossPressed = false;

void toggleMode() {
  robot.setAutoMode(!robot.autoMode());
  robot.stop();
  Serial.println(robot.autoMode() ? "AUTO_LINE" : "MANUAL");
}

void openClaw() { clawServo.runAngle(105, 70); }

void closeClaw() { clawServo.runAngle(45, 70); }

void armUp() { armServo.runAngle(125, 70); }

void armDown() { armServo.runAngle(80, 70); }

void handleButtons(const GamepadState& state) {
  // Trigger each action only on the rising edge of the button press.
  if (state.m1 && !lastM1Pressed) {
    toggleMode();
  }
  if (state.l1 && !lastL1Pressed) {
    openClaw();
  }
  if (state.r1 && !lastR1Pressed) {
    closeClaw();
  }
  if (state.triangle && !lastTrianglePressed) {
    armUp();
  }
  if (state.cross && !lastCrossPressed) {
    armDown();
  }

  lastM1Pressed = state.m1;
  lastL1Pressed = state.l1;
  lastR1Pressed = state.r1;
  lastTrianglePressed = state.triangle;
  lastCrossPressed = state.cross;
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

  if (!lineSensor.begin()) {
    Serial.println("Khong tim thay cam bien line I2C");
    while (true) {
      delay(100);
    }
  }

  if (!gamepad.begin()) {
    Serial.println("Khong tim thay bo nhan PS4 I2C");
  }

  armServo.begin();
  clawServo.begin();
  armServo.limit(60, 140);
  clawServo.limit(35, 110);
  armServo.setAngle(125);
  clawServo.setAngle(45);

  robot.speed(60, 35);
  robot.lineSensor(&lineSensor);
  robot.setAutoMode(true);

  Serial.println("Bai 07 - Hoan thien robot cho thi dau");
}

void loop() {
  // Update auxiliary actions first so mode changes take effect immediately.
  gamepad.update();
  handleButtons(gamepad.state());

  if (robot.autoMode()) {
    // In auto mode the robot follows the line continuously.
    robot.followLine(true, lineSensor.check());
  } else {
    // In manual mode the same drive base is controlled from the gamepad.
    robot.updateTeleop(gamepad.state(), 4);
  }
  delay(10);
}
