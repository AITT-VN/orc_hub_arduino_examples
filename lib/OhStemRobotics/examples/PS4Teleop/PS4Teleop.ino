#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Basic teleop example that can use the I2C PS4 receiver or another Gamepad data source.
MotorDriverV2 motorDriver(Wire);
DCMotor motor1(motorDriver, M1);
DCMotor motor2(motorDriver, M2);
DCMotor motor3(motorDriver, E1);
DCMotor motor4(motorDriver, E2);
DriveBase robot(MODE_MECANUM, &motor2, &motor1, &motor3, &motor4);

PS4GamepadReceiver ps4Receiver(Wire, 0x55);
Gamepad gamepad(&ps4Receiver);
bool receiverReady = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!motorDriver.begin()) {
    Serial.println("MotorDriverV2 not found");
    while (true) {
      delay(100);
    }
  }

  receiverReady = gamepad.begin();
  robot.speed(80, 40);
  robot.setAutoMode(false);
  robot.setSideMoveMode(JOYSTICK);

  if (receiverReady) {
    Serial.println("PS4 receiver found");
  } else {
    gamepad.attachReceiver(nullptr);
    Serial.println("Gamepad receiver not found");
    Serial.println("You can still feed Gamepad::applyNameValue(...) from BLE/app code.");
  }
}

void loop() {
  if (receiverReady) {
    gamepad.update();
  }
  robot.updateTeleop(gamepad.state(), 3);
  delay(10);
}
