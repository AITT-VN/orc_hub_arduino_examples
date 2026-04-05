# OhStemRobotics

## English

Arduino/PlatformIO port of the original `yolouno_extension_robotics` MicroPython runtime.

### Included

- `MotorDriverV1`, `MotorDriverV2`
- `DCMotor`, `DCMotor2Pin`, `DCMotor3Pin`
- `RobotServo`
- `LineSensor2P`, `LineSensor3P`, `LineSensorI2C`
- `DriveBase`
- `Gamepad`, `PS4GamepadReceiver`
- `PIDController`
- `MPU6050Sensor`, `MPU9250Sensor`
- `AngleSensor`

These are the main building blocks included in the library.

### Main Behavioral Changes From The Python Version

- Async `await ...` methods are converted to blocking C++ methods.
- IMU fusion is updated by calling `angleSensor.update()` inside `loop()` or while the robot is moving.
- Yolo Uno specific runtime modules such as `ble`, `utility`, `setting`, and task scheduling are replaced by standard Arduino APIs.
- PS4 receiver over I2C is bundled.
- BLE app integration is exposed through `Gamepad::applyNameValue(...)` so you can plug in your own BLE transport.

### Python To C++ API Mapping

- `DCMotor.run_time(...)` -> `DCMotor::runTime(...)`
- `DCMotor.run_angle(...)` -> `DCMotor::runAngle(...)`
- `DCMotor.run_rotation(...)` -> `DCMotor::runRotation(...)`
- `Servo.run_angle(...)` -> `RobotServo::runAngle(...)`
- `Servo.run_steps(...)` -> `RobotServo::runSteps(...)`
- `DriveBase.forward_for(...)` -> `DriveBase::forwardFor(...)`
- `DriveBase.turn_left_for(...)` -> `DriveBase::turnLeftFor(...)`
- `DriveBase.follow_line_until_cross(...)` -> `DriveBase::followLineUntilCross(...)`
- `DriveBase.run_teleop(...)` -> `DriveBase::updateTeleop(...)` called from `loop()`

### DriveBase Quick Guide

- `DriveBase::speed(speed, minSpeed)` sets the cruise speed and the ramp start speed.
- `DriveBase::size(wheelDiameterMm, widthMm)` is required for precise `CM`, `INCH`, and `DEGREE` moves.
- `DriveBase::forward()`, `backward()`, `turnLeft()`, `turnRight()`, `moveLeft()`, and `moveRight()` run continuously until you call `stop()` or `brake()`.
- `DriveBase::forwardFor(...)` and `backwardFor(...)` support `SECOND`, `CM`, and `INCH`.
- `DriveBase::turnLeftFor(...)` and `turnRightFor(...)` support `SECOND` and `DEGREE`.
- `DriveBase::moveLeftFor(...)` and `moveRightFor(...)` are for mecanum strafing and currently support `SECOND`.
- `DriveBase::distance()` needs both left and right encoder motors configured with `setEncoder(...)`.
- `DriveBase::angle()` and gyro-based turns need `robot.angleSensor(&angleSensor)` plus `robot.useGyro(true)`.
- `DriveBase` automatically reverses the left side to match the Python runtime, so start examples with `reversed=false` and only flip a motor if a wheel still spins the wrong way.
- If you call a precise move without the required feedback sensor, the function now returns safely instead of blocking forever.

### Examples

- `examples/BasicDrive`: mecanum basics with forward, backward, turning, and strafing
- `examples/EncoderDistance`: straight distance moves and encoder readback
- `examples/MPU6050Heading`: raw heading, pitch, and roll from an MPU6050 plus `AngleSensor`
- `examples/GyroTurns`: accurate left and right turns using an MPU6050
- `examples/PreciseSquare`: combines encoder straight moves with gyro turns for a repeatable square path
- `examples/LineFollower`: line-following with the I2C line sensor
- `examples/PS4Teleop`: teleop with the I2C PS4 receiver or any custom BLE/app transport that feeds `Gamepad::applyNameValue(...)`

### Arduino IDE

1. Copy `arduino/OhStemRobotics` into your Arduino `libraries` folder.
2. Restart Arduino IDE.
3. Open one of the examples in `examples/`.

### PlatformIO

Add the local library folder to your project, for example:

```ini
lib_deps =
  symlink://../path/to/arduino/OhStemRobotics
```

Or copy `arduino/OhStemRobotics` into your project's `lib/` folder.

### Notes

- `MotorDriverV2` can optionally drive local `M3/M4` pins directly if you pass those pin numbers to the constructor.
- `DriveBase` automatically reverses the left-side motors, matching the original Python implementation.
- For gyroscope-based turns, call `robot.useGyro(true)` and attach an `AngleSensor`.
- `MPU9250Sensor` defaults its magnetometer address to `0x23` to mirror the Python source. Pass `0x0C` if your module uses the standard AK8963 address.

## Tiếng Việt

Bản port cho Arduino/PlatformIO của runtime MicroPython `yolouno_extension_robotics` gốc.

### Thành Phần Có Sẵn

- `MotorDriverV1`, `MotorDriverV2`
- `DCMotor`, `DCMotor2Pin`, `DCMotor3Pin`
- `RobotServo`
- `LineSensor2P`, `LineSensor3P`, `LineSensorI2C`
- `DriveBase`
- `Gamepad`, `PS4GamepadReceiver`
- `PIDController`
- `MPU6050Sensor`, `MPU9250Sensor`
- `AngleSensor`

Đây là các thành phần chính đã được đóng gói sẵn trong thư viện.

### Khác Biệt Chính So Với Bản Python

- Các hàm async `await ...` được chuyển thành hàm C++ dạng blocking.
- Bộ hợp nhất IMU được cập nhật bằng cách gọi `angleSensor.update()` trong `loop()` hoặc trong lúc robot đang chạy.
- Các module runtime riêng của Yolo Uno như `ble`, `utility`, `setting` và scheduler được thay bằng API Arduino chuẩn.
- PS4 receiver giao tiếp qua I2C đã được tích hợp sẵn.
- Tích hợp BLE app được mở qua `Gamepad::applyNameValue(...)` để anh có thể tự nối vào transport BLE riêng.

### Ánh Xạ API Từ Python Sang C++

- `DCMotor.run_time(...)` -> `DCMotor::runTime(...)`
- `DCMotor.run_angle(...)` -> `DCMotor::runAngle(...)`
- `DCMotor.run_rotation(...)` -> `DCMotor::runRotation(...)`
- `Servo.run_angle(...)` -> `RobotServo::runAngle(...)`
- `Servo.run_steps(...)` -> `RobotServo::runSteps(...)`
- `DriveBase.forward_for(...)` -> `DriveBase::forwardFor(...)`
- `DriveBase.turn_left_for(...)` -> `DriveBase::turnLeftFor(...)`
- `DriveBase.follow_line_until_cross(...)` -> `DriveBase::followLineUntilCross(...)`
- `DriveBase.run_teleop(...)` -> `DriveBase::updateTeleop(...)` gọi trong `loop()`

### Hướng Dẫn Nhanh Cho DriveBase

- `DriveBase::speed(speed, minSpeed)` đặt tốc độ chạy và tốc độ khởi động để ramp.
- `DriveBase::size(wheelDiameterMm, widthMm)` cần thiết cho các lệnh di chuyển chính xác theo `CM`, `INCH` và `DEGREE`.
- `DriveBase::forward()`, `backward()`, `turnLeft()`, `turnRight()`, `moveLeft()` và `moveRight()` sẽ chạy liên tục cho đến khi gọi `stop()` hoặc `brake()`.
- `DriveBase::forwardFor(...)` và `backwardFor(...)` hỗ trợ `SECOND`, `CM` và `INCH`.
- `DriveBase::turnLeftFor(...)` và `turnRightFor(...)` hỗ trợ `SECOND` và `DEGREE`.
- `DriveBase::moveLeftFor(...)` và `moveRightFor(...)` dùng cho mecanum strafe và hiện tại hỗ trợ `SECOND`.
- `DriveBase::distance()` cần cả encoder bên trái và bên phải đã được cấu hình bằng `setEncoder(...)`.
- `DriveBase::angle()` và các lệnh quay bằng gyro cần `robot.angleSensor(&angleSensor)` kèm `robot.useGyro(true)`.
- `DriveBase` tự động đảo chiều bên trái để giống runtime Python, nên anh nên bắt đầu example với `reversed=false` và chỉ đảo thêm motor nào vẫn quay sai chiều.
- Nếu gọi lệnh di chuyển chính xác mà thiếu cảm biến feedback cần thiết, hàm sẽ thoát an toàn thay vì bị treo mãi.

### Ví Dụ

- `examples/BasicDrive`: căn bản mecanum với tiến, lùi, quay và strafe
- `examples/EncoderDistance`: chạy thẳng theo khoảng cách và đọc encoder
- `examples/MPU6050Heading`: đọc heading, pitch và roll từ MPU6050 và `AngleSensor`
- `examples/GyroTurns`: quay trái phải chính xác bằng MPU6050
- `examples/PreciseSquare`: kết hợp encoder và gyro để chạy hình vuông lặp lại ổn định
- `examples/LineFollower`: bám line bằng cảm biến line I2C
- `examples/PS4Teleop`: teleop với PS4 receiver I2C hoặc BLE/app tự tùy biến đưa dữ liệu vào `Gamepad::applyNameValue(...)`

### Arduino IDE

1. Sao chép `arduino/OhStemRobotics` vào thư mục `libraries` của Arduino.
2. Khởi động lại Arduino IDE.
3. Mở một example trong thư mục `examples/`.

### PlatformIO

Thêm thư mục thư viện local vào project, ví dụ:

```ini
lib_deps =
  symlink://../path/to/arduino/OhStemRobotics
```

Hoặc copy `arduino/OhStemRobotics` vào thư mục `lib/` của project.

### Lưu Ý

- `MotorDriverV2` có thể điều khiển trực tiếp chân local `M3/M4` nếu anh truyền các pin đó vào constructor.
- `DriveBase` tự động đảo chiều các motor bên trái để giống với bản Python gốc.
- Để quay bằng gyroscope, hãy gọi `robot.useGyro(true)` và gắn thêm `AngleSensor`.
- `MPU9250Sensor` mặc định dùng địa chỉ magnetometer `0x23` để giống source Python. Truyền `0x0C` nếu module của anh dùng địa chỉ AK8963 chuẩn.
