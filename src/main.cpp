#include <OhStemRobotics.h>

#include "AppConfig.h"

using namespace ohstem::robotics;

MotorDriverV2 mdV2(Wire, MOTOR_DRIVER_ADDR);

// Keep each motor tied to its physical driver port so direction fixes stay easy to follow.
DCMotor motor1(mdV2, M1, MOTOR_1_REVERSED);
DCMotor motor2(mdV2, M2, MOTOR_2_REVERSED);
DCMotor motor3(mdV2, E1, MOTOR_3_REVERSED);
DCMotor motor4(mdV2, E2, MOTOR_4_REVERSED);
DCMotor motor5(mdV2, M3, MOTOR_5_REVERSED);

// Keep the same mapping style as the Python version.
DriveBase robot(MODE_MECANUM, &motor1, &motor2, &motor3, &motor4);

ServoMotor servo1(mdV2, S1, SERVO_1_MAX_ANGLE);
ServoMotor servo2(mdV2, S2, SERVO_2_MAX_ANGLE);

PS4GamepadReceiver ps4Receiver(Wire, PS4_RECEIVER_ADDR);
Gamepad gamepad(&ps4Receiver);

bool motorDriverReady = false;
bool ps4ReceiverReady = false;

uint32_t lastMotorDriverRetryMs = 0;
uint32_t lastAuxTaskMs = 0;

// Wait briefly for native USB CDC so the first boot logs are visible in Serial Monitor.
void waitForSerial(uint32_t timeoutMs = 3000) {
  const uint32_t startMs = millis();
  while (!Serial && (millis() - startMs) < timeoutMs) {
    delay(10);
  }
}

// Run a simple I2C scan at boot to help users confirm board wiring and device addresses.
void scanI2C() {
  Wire.begin();

  Serial.println("Scanning I2C bus...");
  Serial.printf("Default I2C pins: SDA=%d, SCL=%d\r\n", SDA, SCL);

  bool foundAny = false;
  for (uint8_t address = 1; address < 0x7F; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() != 0) {
      continue;
    }

    foundAny = true;
    Serial.printf("  Found device at 0x%02X\r\n", address);
  }

  if (!foundAny) {
    Serial.println("  No I2C devices found");
  }

  Serial.printf("Expected motor driver: 0x%02X\r\n", MOTOR_DRIVER_ADDR);
  Serial.printf("Expected PS4 receiver: 0x%02X\r\n", PS4_RECEIVER_ADDR);
  Serial.printf("Expected line sensor: 0x%02X\r\n", LINE_SENSOR_ADDR);
  Serial.println();
}

// Print the active reverse flags so wheel-direction tweaks are easy to verify.
void printMotorConfig() {
  Serial.printf("Motor reverse: M1=%d M2=%d M3=%d M4=%d M5=%d\r\n",
                MOTOR_1_REVERSED, MOTOR_2_REVERSED, MOTOR_3_REVERSED,
                MOTOR_4_REVERSED, MOTOR_5_REVERSED);
}

// Map shoulder buttons to fixed servo presets for quick mechanism control.
void onCmdBtnL1() {
  if (!ENABLE_SERVOS) {
    return;
  }
  servo1.runAngle(SERVO_1_L1_ANGLE, SERVO_SPEED);
}

void onCmdBtnL2() {
  if (!ENABLE_SERVOS) {
    return;
  }
  servo1.runAngle(SERVO_1_L2_ANGLE, SERVO_SPEED);
}

void onCmdBtnR1() {
  if (!ENABLE_SERVOS) {
    return;
  }
  servo2.runAngle(SERVO_2_R1_ANGLE, SERVO_SPEED); 
}

void onCmdBtnR2() {
  if (!ENABLE_SERVOS) {
    return;
  }
  servo2.runAngle(SERVO_2_R2_ANGLE, SERVO_SPEED);
}

void onCmdBtnThumbR() {
  motor5.stop();
}

// Register the extra button actions that sit on top of basic teleop driving.
void bindTeleopCommands() {
  robot.onTeleopCommand(GamepadButton::L1, onCmdBtnL1);
  robot.onTeleopCommand(GamepadButton::L2, onCmdBtnL2);
  robot.onTeleopCommand(GamepadButton::R1, onCmdBtnR1);
  robot.onTeleopCommand(GamepadButton::R2, onCmdBtnR2);
  robot.onTeleopCommand(GamepadButton::ThumbR, onCmdBtnThumbR);
}

// Apply the shared geometry, speed, encoder, and servo settings after hardware is ready.
void setupRobot() {
  motor3.setEncoder(ENCODER_RPM, ENCODER_PPR, ENCODER_GEARS);
  motor4.setEncoder(ENCODER_RPM, ENCODER_PPR, ENCODER_GEARS);

  robot.size(WHEEL_DIAMETER_MM, ROBOT_WIDTH_MM);
  robot.speed(DEFAULT_SPEED, DEFAULT_MIN_SPEED);

  if (ENABLE_SERVOS) {
    servo1.limit(SERVO_1_MIN_LIMIT, SERVO_1_MAX_LIMIT);
    servo2.limit(SERVO_2_MIN_LIMIT, SERVO_2_MAX_LIMIT);
    servo1.setAngle(SERVO_1_L2_ANGLE);
    servo2.setAngle(SERVO_2_R2_ANGLE);
  }
}

// Bring up the detected hardware and choose teleop or autonomous mode for the demo.
bool beginRobot() {
  scanI2C();

  if (!mdV2.begin()) {
    Serial.println("MotorDriverV2 not found");
    Serial.println("Check motor driver power, address 0x54, and I2C pins.");
    return false;
  }

  setupRobot();
  printMotorConfig();

  if (ENABLE_PS4_RECEIVER) {
    ps4ReceiverReady = ps4Receiver.begin();
    gamepad.attachReceiver(ps4ReceiverReady ? &ps4Receiver : nullptr);
    robot.setAutoMode(false);
    robot.setSideMoveMode(JOYSTICK);
    bindTeleopCommands();
    if (ps4ReceiverReady) {
      Serial.println("PS4 receiver found");
    } else {
      Serial.println("Gamepad receiver not found");
    }
    Serial.println("Robot gamepad demo ready");
  } else {
    robot.setAutoMode(true);
    Serial.println("Robot autonomous demo ready");
  }

  Serial.println();
  return true;
}

// Simple timed movement demo used when teleop is disabled in AppConfig.
void runAutonomousDemo() {
  Serial.println("Move: forward");
  robot.forwardFor(1.2f, SECOND, BRAKE);
  delay(600);

  Serial.println("Move: backward");
  robot.backwardFor(1.2f, SECOND, BRAKE);
  delay(600);

  Serial.println("Move: turn left");
  robot.turnLeftFor(0.8f, SECOND, BRAKE);
  delay(600);

  Serial.println("Move: turn right");
  robot.turnRightFor(0.8f, SECOND, BRAKE);
  delay(1000);
}

// Use face buttons for small servo trims and right stick X for the auxiliary motor.
void updateExtraGamepadActions() {
  if ((millis() - lastAuxTaskMs) < AUX_TASK_INTERVAL_MS) {
    return;
  }
  lastAuxTaskMs = millis();

  const GamepadState& state = gamepad.state();

  if (ENABLE_SERVOS) {
    if (state.triangle) {
      servo1.runSteps(SERVO_STEP, SERVO_SPEED);
    }
    if (state.cross) {
      servo1.runSteps(-SERVO_STEP, SERVO_SPEED);
    }
    if (state.square) {
      servo2.runSteps(SERVO_STEP, SERVO_SPEED);
    }
    if (state.circle) {
      servo2.runSteps(-SERVO_STEP, SERVO_SPEED);
    }
  }

  if (state.rightX < -AUX_MOTOR_DEADZONE) {
    motor5.run(-AUX_MOTOR_SPEED);
  } else if (state.rightX > AUX_MOTOR_DEADZONE) {
    motor5.run(AUX_MOTOR_SPEED);
  } else {
    motor5.stop();
  }
}

// Keep teleop working even if the external I2C receiver is missing and BLE feeds GamepadState.
void updateGamepadTeleop() {
  if (ps4ReceiverReady) {
    gamepad.update();
  }

  // If no I2C receiver is present, BLE/app code can still feed gamepad.state().
  robot.updateTeleop(gamepad.state(), TELEOP_ACCEL_STEPS);
  updateExtraGamepadActions();
  delay(10);
}

// Retry the motor driver instead of freezing the whole app when power or wiring is late.
void retryMotorDriver() {
  if ((millis() - lastMotorDriverRetryMs) < MOTOR_DRIVER_RETRY_MS) {
    delay(50);
    return;
  }

  lastMotorDriverRetryMs = millis();
  Serial.println("Retrying motor driver...");
  motorDriverReady = beginRobot();
}

void setup() {
  Serial.begin(115200);
  waitForSerial();

  Serial.println();
  Serial.println("Yolo Uno robot controller");

  motorDriverReady = beginRobot();
}

void loop() {
  if (!motorDriverReady) {
    retryMotorDriver();
    return;
  }

  if (ENABLE_PS4_RECEIVER) {
    updateGamepadTeleop();
    return;
  }

  if (ENABLE_AUTONOMOUS_DEMO) {
    runAutonomousDemo();
    return;
  }

  robot.stop();
  motor5.stop();
  delay(20);
}
