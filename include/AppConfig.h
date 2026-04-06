#pragma once

#include <Arduino.h>

// I2C addresses used by the default Yolo Uno add-on boards.
constexpr uint8_t MOTOR_DRIVER_ADDR = 0x54;
constexpr uint8_t PS4_RECEIVER_ADDR = 0x55;
constexpr uint8_t LINE_SENSOR_ADDR = 0x23;

// Toggle these flags when a wheel or auxiliary motor spins the wrong direction.
// This keeps direction fixes in one place instead of scattering them across the app.
constexpr bool MOTOR_1_REVERSED = false;
constexpr bool MOTOR_2_REVERSED = true;
constexpr bool MOTOR_3_REVERSED = true;
constexpr bool MOTOR_4_REVERSED = true;
constexpr bool MOTOR_5_REVERSED = false;

// Feature switches for optional peripherals and demo modes.
constexpr bool ENABLE_PS4_RECEIVER = true;
constexpr bool ENABLE_IMU = true;
constexpr bool ENABLE_LINE_SENSOR = false;
constexpr bool ENABLE_AUTONOMOUS_DEMO = false;
constexpr bool USE_GYRO_FOR_TURNS = true;
constexpr bool ENABLE_SERVOS = true;

// Runtime timing values used by teleop polling and retry loops.
constexpr uint16_t IMU_CALIB_SAMPLES = 100;
constexpr uint8_t TELEOP_ACCEL_STEPS = 3;
constexpr uint32_t MOTOR_DRIVER_RETRY_MS = 2000;
constexpr uint32_t GAMEPAD_STATUS_REPORT_MS = 1000;
constexpr uint32_t AUX_TASK_INTERVAL_MS = 75;

// Default drive settings and robot geometry for DriveBase calculations.
constexpr float DEFAULT_SPEED = 80.0f;
constexpr float DEFAULT_MIN_SPEED = 40.0f;
constexpr float WHEEL_DIAMETER_MM = 80.0f;
constexpr float ROBOT_WIDTH_MM = 300.0f;

// Encoder setup used by precise distance and turn helpers.
constexpr float ENCODER_RPM = 350.0f;
constexpr uint16_t ENCODER_PPR = 11;
constexpr uint16_t ENCODER_GEARS = 34;

// Servo limits and preset angles used by the gamepad shortcuts.
constexpr int SERVO_1_MAX_ANGLE = 180;
constexpr int SERVO_2_MAX_ANGLE = 180;
constexpr int SERVO_1_MIN_LIMIT = 0;
constexpr int SERVO_1_MAX_LIMIT = 180;
constexpr int SERVO_2_MIN_LIMIT = 0;
constexpr int SERVO_2_MAX_LIMIT = 80;
constexpr int SERVO_SPEED = 100;
constexpr int SERVO_STEP = 5;
constexpr int SERVO_1_L1_ANGLE = 180;
constexpr int SERVO_1_L2_ANGLE = 0;
constexpr int SERVO_2_R1_ANGLE = 80;
constexpr int SERVO_2_R2_ANGLE = 0;

// Auxiliary motor settings used by the right-stick control path.
constexpr int AUX_MOTOR_SPEED = 85;
constexpr int AUX_MOTOR_DEADZONE = 50;
