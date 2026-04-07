# OhStemRobotics

Arduino/PlatformIO port of the original `yolouno_extension_robotics` MicroPython runtime.

## What is included

- `MotorDriverV1`, `MotorDriverV2`
- `DCMotor`, `DCMotor2Pin`, `DCMotor3Pin`
- `RobotServo`
- `LineSensor2P`, `LineSensor3P`, `LineSensorI2C`
- `DriveBase`
- `Gamepad`, `PS4GamepadReceiver`
- `PIDController`
- `MPU6050Sensor`, `MPU9250Sensor`
- `AngleSensor`

## Main behavioral changes from the Python version

- Async `await ...` methods are converted to blocking C++ methods.
- IMU fusion is updated by calling `angleSensor.update()` inside `loop()` or while the robot is moving.
- Yolo Uno specific runtime modules such as `ble`, `utility`, `setting`, and task scheduling are replaced by standard Arduino APIs.
- PS4 receiver over I2C is bundled.
- BLE app integration is exposed through `Gamepad::applyNameValue(...)` so you can plug in your own BLE transport.

## Python to C++ API mapping

- `DCMotor.run_time(...)` -> `DCMotor::runTime(...)`
- `DCMotor.run_angle(...)` -> `DCMotor::runAngle(...)`
- `DCMotor.run_rotation(...)` -> `DCMotor::runRotation(...)`
- `Servo.run_angle(...)` -> `RobotServo::runAngle(...)`
- `Servo.run_steps(...)` -> `RobotServo::runSteps(...)`
- `DriveBase.forward_for(...)` -> `DriveBase::forwardFor(...)`
- `DriveBase.turn_left_for(...)` -> `DriveBase::turnLeftFor(...)`
- `DriveBase.follow_line_until_cross(...)` -> `DriveBase::followLineUntilCross(...)`
- `DriveBase.run_teleop(...)` -> `DriveBase::updateTeleop(...)` called from `loop()`

## Arduino IDE

1. Copy `arduino/OhStemRobotics` into your Arduino `libraries` folder.
2. Restart Arduino IDE.
3. Open one of the examples in `examples/`.

## PlatformIO

Add the local library folder to your project, for example:

```ini
lib_deps =
  symlink://../path/to/arduino/OhStemRobotics
```

Or copy `arduino/OhStemRobotics` into your project's `lib/` folder.

## Notes

- `MotorDriverV2` can optionally drive local `M3/M4` pins directly if you pass those pin numbers to the constructor.
- `DriveBase` automatically reverses the left-side motors, matching the original Python implementation.
- For gyroscope-based turns, call `robot.useGyro(true)` and attach an `AngleSensor`.
- `MPU9250Sensor` defaults its magnetometer address to `0x23` to mirror the Python source. Pass `0x0C` if your module uses the standard AK8963 address.
