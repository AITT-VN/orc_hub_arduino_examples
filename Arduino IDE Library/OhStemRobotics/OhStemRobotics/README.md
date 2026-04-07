# OhStemRobotics

Thu vien robotics cho Arduino IDE va PlatformIO.

## Tinh nang chinh

- `MotorDriverV1`, `MotorDriverV2`
- `DCMotor`, `DCMotor2Pin`, `DCMotor3Pin`
- `RobotServo`
- `LineSensor2P`, `LineSensor3P`, `LineSensorI2C`
- `DriveBase`
- `Gamepad`, `PS4GamepadReceiver`
- `PIDController`
- `MPU6050Sensor`, `MPU9250Sensor`
- `AngleSensor`

## Cai dat tren Arduino IDE

### Cach 1: Copy thu muc thu vien

1. Dong Arduino IDE.
2. Copy toan bo thu muc `OhStemRobotics` vao thu muc:

```text
Documents/Arduino/libraries/
```

3. Mo lai Arduino IDE.
4. Vao `File > Examples > OhStemRobotics`.

### Cach 2: Add ZIP Library

1. Nen thu muc `OhStemRobotics` thanh file `.zip`.
2. Trong Arduino IDE chon `Sketch > Include Library > Add .ZIP Library...`.
3. Chon file zip vua tao.

## Cau truc vi du theo bai hoc

Thu vien da duoc sap xep lai examples theo tung bai:

- `Bai_01_Robot_di_chuyen`
- `Bai_02_Robot_ne_vat_can`
- `Bai_03_Robot_do_duong`
- `Bai_04_Dieu_khien_robot_tu_xa`
- `Bai_05_Dieu_khien_robot_gap_tha_vat`
- `Bai_06_Co_cau_truyen_dong`
- `Bai_07_Hoan_thien_robot_thi_dau`
- `Bai_01_Robot_4_banh_di_chuyen`
- `Bai_02_Robot_4_banh_ne_vat_can`
- `Bai_03_Robot_4_banh_do_duong`
- `Bai_04_Robot_4_banh_tu_xa`
- `Bai_07_Robot_4_banh_thi_dau`

Moi bai deu co:

- Phan khai bao phan cung ngay dau file
- Cac thong so de sua nhanh trong Arduino IDE
- Chu thich ngan gon de dung trong giao an hoac bai hoc

## Luu y tuong thich

- Thu vien theo dung cau truc Arduino library (`src/`, `examples/`, `library.properties`).
- Da bo sung tuong thich servo PWM cho ESP32 core 2.x va 3.x trong Arduino IDE.
- Neu dung servo truc tiep tren board khong phai ESP32, can dam bao thu vien `Servo` co san trong Arduino IDE.
- Cac bai hoc mac dinh uu tien demo de hieu, de sua, va de mo truc tiep trong IDE.

## Ghi chu phan cung

- `MotorDriverV2` mac dinh dung I2C dia chi `0x54`.
- Cam bien line I2C mac dinh dung dia chi `0x23`.
- Bo nhan PS4 I2C mac dinh dung dia chi `0x55`.
- Thu vien da dat I2C mac dinh voi `SDA = 11` va `SCL = 12` tren ESP32, nhung van giu duoc cach goi quen thuoc `Wire.begin()`.
- Neu can doi chan mac dinh, co the override `OHSTEM_I2C_SDA` va `OHSTEM_I2C_SCL` truoc khi include thu vien.
- Cac bai hoc su dung cong `E1`, `E2`, `S1`, `S2` theo bo kit thong dung cua robot OhStem. Neu phan cung cua anh khac, chi can sua o dau moi file `.ino`.
- Cac bai `Robot_4_banh_*` mac dinh dung `M1`, `M2`, `E1`, `E2` cho 4 dong co cua robot 4WD.
