#pragma once

#include <Arduino.h>

constexpr uint8_t MOTOR_DRIVER_ADDR = 0x54;
constexpr uint8_t PS4_RECEIVER_ADDR = 0x55;
constexpr uint8_t LINE_SENSOR_ADDR = 0x23;

// Keep motor reverse flags here so wiring tweaks feel similar to the Python API.
constexpr bool MOTOR_1_REVERSED = false;
constexpr bool MOTOR_2_REVERSED = true;
constexpr bool MOTOR_3_REVERSED = true;
constexpr bool MOTOR_4_REVERSED = true;
constexpr bool MOTOR_5_REVERSED = false;

constexpr bool ENABLE_PS4_RECEIVER = true;
constexpr bool ENABLE_IMU = true;
constexpr bool ENABLE_LINE_SENSOR = false;
constexpr bool ENABLE_AUTONOMOUS_DEMO = false;
constexpr bool USE_GYRO_FOR_TURNS = true;
constexpr bool ENABLE_SERVOS = true;

constexpr uint16_t IMU_CALIB_SAMPLES = 100;
constexpr uint8_t TELEOP_ACCEL_STEPS = 3;
constexpr uint32_t MOTOR_DRIVER_RETRY_MS = 2000;
constexpr uint32_t GAMEPAD_STATUS_REPORT_MS = 1000;
constexpr uint32_t AUX_TASK_INTERVAL_MS = 75;

constexpr float DEFAULT_SPEED = 80.0f;
constexpr float DEFAULT_MIN_SPEED = 40.0f;
constexpr float WHEEL_DIAMETER_MM = 80.0f;
constexpr float ROBOT_WIDTH_MM = 300.0f;

constexpr float ENCODER_RPM = 350.0f;
constexpr uint16_t ENCODER_PPR = 11;
constexpr uint16_t ENCODER_GEARS = 34;

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

constexpr int AUX_MOTOR_SPEED = 85;
constexpr int AUX_MOTOR_DEADZONE = 50;
