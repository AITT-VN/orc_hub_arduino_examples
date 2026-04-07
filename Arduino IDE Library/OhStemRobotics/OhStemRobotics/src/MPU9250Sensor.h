#ifndef OHSTEM_MPU9250_SENSOR_H
#define OHSTEM_MPU9250_SENSOR_H

#include "MPU6050Sensor.h"

namespace ohstem {
namespace robotics {

class MPU9250Sensor : public MPU6050Sensor {
 public:
  explicit MPU9250Sensor(TwoWire& wire = Wire, int deviceAddress = -1,
                         uint8_t magnetometerAddress = 0x23,
                         const AxisTransform& transform = AxisTransform());

  bool begin() override;
  bool readMag(Vec3f& mag) override;
  bool hasMagnetometer() const override { return true; }
  uint32_t magStaleCount() const { return magStaleCount_; }

 private:
  bool setupMagnetometer();

  uint8_t magnetometerAddress_;
  float magCorrection_[3] = {1.0f, 1.0f, 1.0f};
  uint32_t magStaleCount_ = 0;
};

}  // namespace robotics
}  // namespace ohstem

#endif
