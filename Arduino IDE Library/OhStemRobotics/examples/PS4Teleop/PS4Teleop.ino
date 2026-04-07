#include <OhStemRobotics.h>

#include <OhStemRobotics.h>

using namespace ohstem::robotics;

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
  robot.updateTeleop(gamepad, 3);
  delay(10);
}
