# OhStemRoboticsDemo

## Instruction:

PlatformIO sample project for the C++ port of the OhStem robotics runtime.

### Files

- `platformio.ini`: default Yolo Uno ESP32-S3 environment
- `boards/yolo_uno.json`: local PlatformIO board definition for the custom Yolo Uno board
- `variants/yolo_uno/pins_arduino.h`: default board pin mapping, including `Wire` I2C pins
- `include/AppConfig.h`: app-level addresses and feature toggles
- `src/main.cpp`: demo app using motor driver, drivebase, servo, IMU, and PS4 teleop
- `lib/OhStemRobotics`: local copy of the ported library

### Quick Start

1. Open this folder in VS Code with PlatformIO.
2. The project already targets the custom `yolo_uno` board definition bundled in `boards/yolo_uno.json`.
3. Adjust feature flags in `include/AppConfig.h` if needed.
4. Build and upload.

### Notes

- `Serial` is configured for native USB CDC on Yolo Uno via `ARDUINO_USB_CDC_ON_BOOT=1`.
- Yolo Uno default I2C pins are defined in `variants/yolo_uno/pins_arduino.h`, so examples can use plain `Wire.begin()`.
- `ENABLE_PS4_RECEIVER = true` expects the I2C PS4 receiver at address `0x55`.
- `ENABLE_IMU = true` uses `MPU6050Sensor`.
- The sample keeps the robot in teleop mode when the PS4 receiver is enabled.
- If you want a pure movement demo, set `ENABLE_PS4_RECEIVER = false` and `ENABLE_AUTONOMOUS_DEMO = true`.
- Library examples live in `lib/OhStemRobotics/examples`, including `BasicDrive`, `EncoderDistance`, `MPU6050Heading`, `GyroTurns`, `PreciseSquare`, `LineFollower`, and `PS4Teleop`.

## Hướng dẫn cơ bản:

Dự án mẫu PlatformIO cho bản C++ của runtime robotics OhStem.

### Tập Tin

- `platformio.ini`: môi trường mặc định cho Yolo Uno ESP32-S3
- `boards/yolo_uno.json`: định nghĩa board local cho bo Yolo Uno custom
- `variants/yolo_uno/pins_arduino.h`: map chân mặc định của board, bao gồm chân I2C cho `Wire`
- `include/AppConfig.h`: địa chỉ và các công tắc tính năng của app
- `src/main.cpp`: chương trình demo dùng motor driver, drivebase, servo, IMU và PS4 teleop
- `lib/OhStemRobotics`: bản sao local của thư viện đã được port

### Bắt Đầu Nhanh

1. Mở folder này trong VS Code với PlatformIO.
2. Project đã trỏ sẵn đến board `yolo_uno` được đóng gói trong `boards/yolo_uno.json`.
3. Nếu cần thì chỉnh các feature flag trong `include/AppConfig.h`.
4. Build và nạp firmware.

### Lưu Ý

- `Serial` được cấu hình cho native USB CDC trên Yolo Uno thông qua `ARDUINO_USB_CDC_ON_BOOT=1`.
- Chân I2C mặc định của Yolo Uno được định nghĩa trong `variants/yolo_uno/pins_arduino.h`, nên example có thể dùng `Wire.begin()` trực tiếp.
- `ENABLE_PS4_RECEIVER = true` sẽ kỳ vọng PS4 receiver I2C ở địa chỉ `0x55`.
- `ENABLE_IMU = true` sẽ dùng `MPU6050Sensor`.
- Mẫu demo này giữ robot ở chế độ teleop khi PS4 receiver được bật.
- Nếu muốn demo di chuyển tự động đơn giản, hãy đặt `ENABLE_PS4_RECEIVER = false` và `ENABLE_AUTONOMOUS_DEMO = true`.
- Các example của thư viện nằm trong `lib/OhStemRobotics/examples`, gồm `BasicDrive`, `EncoderDistance`, `MPU6050Heading`, `GyroTurns`, `PreciseSquare`, `LineFollower` và `PS4Teleop`.
