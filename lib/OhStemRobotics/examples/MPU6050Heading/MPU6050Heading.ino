#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Prints heading, pitch, and roll so users can validate IMU orientation first.
MPU6050Sensor imu(Wire);
AngleSensor angleSensor(imu);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!imu.begin()) {
    Serial.println("MPU6050 not found");
    while (true) {
      delay(100);
    }
  }

  Serial.println("Keep the robot still for MPU6050 calibration...");
  angleSensor.calibrate(300);
  Serial.println("MPU6050Heading example started");
}

void loop() {
  if (angleSensor.update()) {
    Serial.println(angleSensor.printData());
  } else {
    Serial.println("MPU6050 update failed");
  }
  delay(100);
}
