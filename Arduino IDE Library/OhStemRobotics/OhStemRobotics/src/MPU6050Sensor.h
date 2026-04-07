#ifndef OHSTEM_MPU6050_SENSOR_H
#define OHSTEM_MPU6050_SENSOR_H

#include <Arduino.h>
#include <Wire.h>

#include "IMUBase.h"

namespace ohstem {
namespace robotics {

class MPU6050Sensor : public IMUBase {
 public:
  explicit MPU6050Sensor(TwoWire& wire = Wire, int deviceAddress = -1,
                         const AxisTransform& transform = AxisTransform());

  bool begin() override;
  bool readAccel(Vec3f& accel) override;
  bool readGyro(Vec3f& gyro) override;
  bool hasMagnetometer() const override { return false; }

  float temperatureC();
  void setSampleRate(uint8_t rate);
  void setFilterRange(uint8_t range);
  void setAccelRange(uint8_t range);
  void setGyroRange(uint8_t range);
  void setPassthrough(bool enabled);

 protected:
  bool writeByte(uint8_t address, uint8_t reg, uint8_t value);
  bool readBytes(uint8_t address, uint8_t reg, uint8_t* buffer, size_t len);
  bool readByte(uint8_t address, uint8_t reg, uint8_t& value);
  Vec3f applyTransform(float x, float y, float z) const;
  uint8_t resolvedAddress() const { return resolvedAddress_; }
  TwoWire& wire() { return *wire_; }

 private:
  TwoWire* wire_;
  int preferredAddress_;
  uint8_t resolvedAddress_ = 0;
  AxisTransform transform_;
  uint8_t accelRange_ = 1;
  uint8_t gyroRange_ = 1;
};

}  // namespace robotics
}  // namespace ohstem

#endif
