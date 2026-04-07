#include "MPU6050Sensor.h"

#include <math.h>

#include "RoboticsUtils.h"

namespace ohstem {
namespace robotics {

namespace {

constexpr uint8_t kAddress0 = 0x68;
constexpr uint8_t kAddress1 = 0x69;
constexpr uint8_t kWhoAmIReg = 0x75;
constexpr uint8_t kPowerReg = 0x6B;
constexpr uint8_t kIntBypassReg = 0x37;
constexpr uint8_t kUserCtrlReg = 0x6A;
constexpr uint8_t kSampleRateReg = 0x19;
constexpr uint8_t kFilterReg = 0x1A;
constexpr uint8_t kGyroRangeReg = 0x1B;
constexpr uint8_t kAccelRangeReg = 0x1C;
constexpr uint8_t kAccelDataReg = 0x3B;
constexpr uint8_t kTempDataReg = 0x41;
constexpr uint8_t kGyroDataReg = 0x43;

}  // namespace

MPU6050Sensor::MPU6050Sensor(TwoWire& wire, int deviceAddress,
                             const AxisTransform& transform)
    : wire_(&wire), preferredAddress_(deviceAddress), transform_(transform) {
  configureDefaultWirePins(wire);
}

bool MPU6050Sensor::writeByte(uint8_t address, uint8_t reg, uint8_t value) {
  wire_->beginTransmission(address);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission() == 0;
}

bool MPU6050Sensor::readBytes(uint8_t address, uint8_t reg, uint8_t* buffer,
                              size_t len) {
  wire_->beginTransmission(address);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) {
    return false;
  }
  const size_t count =
      wire_->requestFrom(static_cast<int>(address), static_cast<int>(len));
  if (count != len) {
    while (wire_->available()) {
      wire_->read();
    }
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    buffer[i] = wire_->read();
  }
  return true;
}

bool MPU6050Sensor::readByte(uint8_t address, uint8_t reg, uint8_t& value) {
  return readBytes(address, reg, &value, 1);
}

Vec3f MPU6050Sensor::applyTransform(float x, float y, float z) const {
  const float raw[3] = {x, y, z};
  Vec3f out;
  out.x = raw[transform_.order[0]] * transform_.scale[0];
  out.y = raw[transform_.order[1]] * transform_.scale[1];
  out.z = raw[transform_.order[2]] * transform_.scale[2];
  return out;
}

bool MPU6050Sensor::begin() {
  configureDefaultWirePins(*wire_);
  wire_->begin();
  delay(200);

  uint8_t candidate = 0;
  if (preferredAddress_ >= 0) {
    candidate = static_cast<uint8_t>(preferredAddress_);
  } else {
    uint8_t who = 0;
    if (readByte(kAddress0, kWhoAmIReg, who)) {
      candidate = kAddress0;
    } else if (readByte(kAddress1, kWhoAmIReg, who)) {
      candidate = kAddress1;
    }
  }

  if (candidate == 0) {
    return false;
  }

  resolvedAddress_ = candidate;
  uint8_t chipId = 0;
  if (!readByte(resolvedAddress_, kWhoAmIReg, chipId)) {
    return false;
  }

  writeByte(resolvedAddress_, kPowerReg, 0x01);
  setPassthrough(true);
  setAccelRange(1);
  setGyroRange(1);
  return chipId != 0;
}

bool MPU6050Sensor::readAccel(Vec3f& accel) {
  uint8_t data[6] = {0};
  if (!readBytes(resolvedAddress_, kAccelDataReg, data, sizeof(data))) {
    return false;
  }
  const int16_t rawX = readInt16BE(data);
  const int16_t rawY = readInt16BE(data + 2);
  const int16_t rawZ = readInt16BE(data + 4);
  static const float kScale[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
  accel = applyTransform(rawX / kScale[accelRange_], rawY / kScale[accelRange_],
                         rawZ / kScale[accelRange_]);
  return true;
}

bool MPU6050Sensor::readGyro(Vec3f& gyro) {
  uint8_t data[6] = {0};
  if (!readBytes(resolvedAddress_, kGyroDataReg, data, sizeof(data))) {
    return false;
  }
  const int16_t rawX = readInt16BE(data);
  const int16_t rawY = readInt16BE(data + 2);
  const int16_t rawZ = readInt16BE(data + 4);
  static const float kScale[] = {131.0f, 65.5f, 32.8f, 16.4f};
  gyro = applyTransform(rawX / kScale[gyroRange_], rawY / kScale[gyroRange_],
                        rawZ / kScale[gyroRange_]);
  return true;
}

float MPU6050Sensor::temperatureC() {
  uint8_t data[2] = {0};
  if (!readBytes(resolvedAddress_, kTempDataReg, data, sizeof(data))) {
    return 0.0f;
  }
  return readInt16BE(data) / 340.0f + 35.0f;
}

void MPU6050Sensor::setSampleRate(uint8_t rate) {
  writeByte(resolvedAddress_, kSampleRateReg, rate);
}

void MPU6050Sensor::setFilterRange(uint8_t range) {
  writeByte(resolvedAddress_, kFilterReg, range & 0x07);
}

void MPU6050Sensor::setAccelRange(uint8_t range) {
  static const uint8_t kValues[] = {0x00, 0x08, 0x10, 0x18};
  accelRange_ = range > 3 ? 3 : range;
  writeByte(resolvedAddress_, kAccelRangeReg, kValues[accelRange_]);
}

void MPU6050Sensor::setGyroRange(uint8_t range) {
  static const uint8_t kValues[] = {0x00, 0x08, 0x10, 0x18};
  gyroRange_ = range > 3 ? 3 : range;
  writeByte(resolvedAddress_, kGyroRangeReg, kValues[gyroRange_]);
}

void MPU6050Sensor::setPassthrough(bool enabled) {
  writeByte(resolvedAddress_, kIntBypassReg, enabled ? 0x02 : 0x00);
  writeByte(resolvedAddress_, kUserCtrlReg, 0x00);
}

}  // namespace robotics
}  // namespace ohstem
