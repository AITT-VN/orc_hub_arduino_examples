#ifndef OHSTEM_IMU_BASE_H
#define OHSTEM_IMU_BASE_H

#include <Arduino.h>

namespace ohstem {
namespace robotics {

struct Vec3f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  Vec3f() = default;
  Vec3f(float xValue, float yValue, float zValue) : x(xValue), y(yValue), z(zValue) {}
};

struct AxisTransform {
  uint8_t order[3] = {0, 1, 2};
  int8_t scale[3] = {1, 1, 1};
};

class IMUBase {
 public:
  virtual ~IMUBase() = default;
  virtual bool begin() = 0;
  virtual bool readAccel(Vec3f& accel) = 0;
  virtual bool readGyro(Vec3f& gyro) = 0;
  virtual bool readMag(Vec3f& mag) {
    (void)mag;
    return false;
  }
  virtual bool hasMagnetometer() const { return false; }
};

}  // namespace robotics
}  // namespace ohstem

#endif
