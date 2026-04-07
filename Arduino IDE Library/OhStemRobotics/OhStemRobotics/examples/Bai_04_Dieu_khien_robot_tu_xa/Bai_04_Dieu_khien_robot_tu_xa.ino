#include <Wire.h>
#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Lesson 04 - Remote control with a PS4 receiver over I2C.
// DriveBase automatically reverses the left side internally.
// Only use reversed=true on a motor if your custom wiring still needs it.
MotorDriverV2 motorDriver(Wire);
DCMotor leftMotor(motorDriver, E1);
DCMotor rightMotor(motorDriver, E2);
DriveBase robot(MODE_2WD, &leftMotor, &rightMotor);

PS4GamepadReceiver ps4Receiver(Wire, 0x55);
Gamepad gamepad(&ps4Receiver);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!motorDriver.begin()) {
    Serial.println("Khong tim thay MotorDriverV2");
    while (true) {
      delay(100);
    }
  }

  if (!gamepad.begin()) {
    Serial.println("Khong tim thay bo nhan PS4 I2C");
  }

  robot.speed(80, 40);
  robot.setAutoMode(false);

  Serial.println("Bai 04 - Dieu khien robot tu xa");
}

void loop() {
  // The drive base reads the gamepad state and turns it into motor commands.
  robot.updateTeleop(gamepad, 4);
  delay(10);
}
