#include "MPU9250Sensor.h"

#include "RoboticsUtils.h"

namespace ohstem {
namespace robotics {

MPU9250Sensor::MPU9250Sensor(TwoWire& wire, int deviceAddress,
                             uint8_t magnetometerAddress,
                             const AxisTransform& transform)
    : MPU6050Sensor(wire, deviceAddress, transform),
      magnetometerAddress_(magnetometerAddress) {}

bool MPU9250Sensor::begin() {
  if (!MPU6050Sensor::begin()) {
    return false;
  }
  return setupMagnetometer();
}

bool MPU9250Sensor::setupMagnetometer() {
  uint8_t asa[3] = {0};
  if (!writeByte(magnetometerAddress_, 0x0A, 0x0F)) {
    return false;
  }
  if (!readBytes(magnetometerAddress_, 0x10, asa, sizeof(asa))) {
    return false;
  }
  writeByte(magnetometerAddress_, 0x0A, 0x00);
  delay(10);
  writeByte(magnetometerAddress_, 0x0A, 0x16);

  magCorrection_[0] = ((0.5f * (asa[0] - 128)) / 128.0f) + 1.0f;
  magCorrection_[1] = ((0.5f * (asa[1] - 128)) / 128.0f) + 1.0f;
  magCorrection_[2] = ((0.5f * (asa[2] - 128)) / 128.0f) + 1.0f;
  return true;
}

bool MPU9250Sensor::readMag(Vec3f& mag) {
  uint8_t status1 = 0;
  if (!readByte(magnetometerAddress_, 0x02, status1)) {
    return false;
  }
  if ((status1 & 0x01) == 0) {
    ++magStaleCount_;
    return false;
  }

  uint8_t data[7] = {0};
  if (!readBytes(magnetometerAddress_, 0x03, data, sizeof(data))) {
    return false;
  }
  if (data[6] & 0x08) {
    ++magStaleCount_;
    return false;
  }

  const float y = static_cast<float>(readInt16LE(data));
  const float x = static_cast<float>(readInt16LE(data + 2));
  const float z = -static_cast<float>(readInt16LE(data + 4));
  const float scale = 0.15f;
  mag = applyTransform(x * magCorrection_[0] * scale, y * magCorrection_[1] * scale,
                       z * magCorrection_[2] * scale);
  magStaleCount_ = 0;
  return true;
}

}  // namespace robotics
}  // namespace ohstem
