#include <Wire.h>
#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Lesson 04 - Remote control for a 4WD robot over PS4 I2C.
// DriveBase automatically reverses the left side internally.
// Only use reversed=true on a motor if your custom wiring still needs it.
MotorDriverV2 motorDriver(Wire);
DCMotor frontLeftMotor(motorDriver, M1);
DCMotor frontRightMotor(motorDriver, M2);
DCMotor rearLeftMotor(motorDriver, E1);
DCMotor rearRightMotor(motorDriver, E2);
DriveBase robot(MODE_4WD, &frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

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

  Serial.println("Bai 04 - Robot 4 banh tu xa");
}

void loop() {
  // The drive base reads the gamepad state and turns it into motor commands.
  robot.updateTeleop(gamepad, 4);
  delay(10);
}
