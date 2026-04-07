#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// A mecanum base can translate left/right, so this example keeps the joystick
// side-move mode enabled.
// DriveBase automatically reverses the left side internally.
// Only add reversed=true to a motor when your hardware still needs it, or
// call motor.reverse() inside setup().
MotorDriverV2 motorDriver(Wire);
DCMotor motor1(motorDriver, M1);
DCMotor motor2(motorDriver, M2);
DCMotor motor3(motorDriver, E1, true);
DCMotor motor4(motorDriver, E2, true);
DriveBase robot(MODE_MECANUM, &motor1, &motor2, &motor3, &motor4);

PS4GamepadReceiver ps4Receiver(Wire, 0x55);
Gamepad gamepad(&ps4Receiver);

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

  gamepad.begin();
  robot.speed(80, 40);
  robot.setAutoMode(false);
  robot.setSideMoveMode(JOYSTICK);
}

void loop() {
  // updateTeleop() reads the controller, applies acceleration smoothing,
  // and maps buttons/sticks to drive commands.
  robot.updateTeleop(gamepad, 3);
  delay(10);
}
