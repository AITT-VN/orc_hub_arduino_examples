#include <Wire.h>
#include <OhStemRobotics.h>

using namespace ohstem::robotics;

// Lesson 02 - 4WD obstacle avoidance
// Change the TRIG/ECHO pins if your ultrasonic sensor is wired differently.
const uint8_t TRIG_PIN = 5;
const uint8_t ECHO_PIN = 18;
const int SAFE_DISTANCE_CM = 20;

MotorDriverV2 motorDriver(Wire);
DCMotor frontLeftMotor(motorDriver, M1);
DCMotor frontRightMotor(motorDriver, M2);
DCMotor rearLeftMotor(motorDriver, E1);
DCMotor rearRightMotor(motorDriver, E2);
DriveBase robot(MODE_4WD, &frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

float readDistanceCm() {
  // Send a short trigger pulse, then measure the echo return time.
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) {
    return -1.0f;
  }
  return duration * 0.0343f / 2.0f;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  if (!motorDriver.begin()) {
    Serial.println("Khong tim thay MotorDriverV2");
    while (true) {
      delay(100);
    }
  }

  robot.speed(55, 35);
  Serial.println("Bai 02 - Robot 4 banh ne vat can");
}

void loop() {
  const float distanceCm = readDistanceCm();
  Serial.print("Khoang cach: ");
  Serial.println(distanceCm);

  if (distanceCm > 0 && distanceCm < SAFE_DISTANCE_CM) {
    // Back away and pivot when the robot gets too close to an obstacle.
    robot.brake();
    delay(200);
    robot.backwardFor(0.4f, SECOND, BRAKE);
    delay(200);
    robot.turnRightFor(0.6f, SECOND, BRAKE);
    delay(200);
  } else {
    // Keep moving forward while the path is clear.
    robot.forward();
    delay(30);
  }
}
