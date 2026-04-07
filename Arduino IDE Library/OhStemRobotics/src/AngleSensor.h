#ifndef OHSTEM_ANGLE_SENSOR_H
#define OHSTEM_ANGLE_SENSOR_H

#include <Arduino.h>

#include "IMUBase.h"

namespace ohstem {
namespace robotics {

class AngleSensor {
 public:
  explicit AngleSensor(IMUBase& imu);

  void setDeclination(float degrees) { declination_ = degrees; }
  void calibrate(uint16_t samples = 500);
  void reset();
  bool update();

  float heading() const { return heading_; }
  float pitch() const { return pitch_; }
  float roll() const { return roll_; }
  String printData() const;

 private:
  float computeDeltaTime();
  bool updateNoMag(const Vec3f& accel, const Vec3f& gyro, float dt);
  bool updateWithMag(const Vec3f& accel, const Vec3f& gyro, const Vec3f& mag,
                     float dt);

  IMUBase& imu_;
  Vec3f magBias_;
  Vec3f gyroBias_;

  float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float beta_ = 0.6045998f;
  float pitch_ = 0.0f;
  float heading_ = 0.0f;
  float roll_ = 0.0f;
  float declination_ = 0.0f;
  uint32_t lastUpdateUs_ = 0;
};

}  // namespace robotics
}  // namespace ohstem

#endif
