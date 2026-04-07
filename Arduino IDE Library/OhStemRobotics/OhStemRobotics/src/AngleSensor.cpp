#include "AngleSensor.h"

#include <math.h>

namespace ohstem {
namespace robotics {

namespace {

inline float toRadians(float degrees) { return degrees * PI / 180.0f; }
inline float toDegrees(float radians) { return radians * 180.0f / PI; }

}  // namespace

AngleSensor::AngleSensor(IMUBase& imu) : imu_(imu) {}

void AngleSensor::reset() {
  q_[0] = 1.0f;
  q_[1] = 0.0f;
  q_[2] = 0.0f;
  q_[3] = 0.0f;
  pitch_ = 0.0f;
  heading_ = 0.0f;
  roll_ = 0.0f;
  lastUpdateUs_ = 0;
}

void AngleSensor::calibrate(uint16_t samples) {
  Vec3f accel;
  Vec3f gyro;
  Vec3f mag;

  Vec3f magMin = {0, 0, 0};
  Vec3f magMax = {0, 0, 0};
  bool hasMag = imu_.hasMagnetometer() && imu_.readMag(mag);
  if (hasMag) {
    magMin = mag;
    magMax = mag;
  }

  gyroBias_ = {0, 0, 0};
  // Average several samples to estimate gyro drift, and track magnetometer
  // min/max values for a simple hard-iron bias correction.
  for (uint16_t i = 0; i < samples; ++i) {
    imu_.readAccel(accel);
    if (imu_.readGyro(gyro)) {
      gyroBias_.x += gyro.x;
      gyroBias_.y += gyro.y;
      gyroBias_.z += gyro.z;
    }

    if (hasMag && imu_.readMag(mag)) {
      magMin.x = min(magMin.x, mag.x);
      magMin.y = min(magMin.y, mag.y);
      magMin.z = min(magMin.z, mag.z);
      magMax.x = max(magMax.x, mag.x);
      magMax.y = max(magMax.y, mag.y);
      magMax.z = max(magMax.z, mag.z);
    }
    delay(2);
  }

  if (samples > 0) {
    gyroBias_.x /= samples;
    gyroBias_.y /= samples;
    gyroBias_.z /= samples;
  }

  if (hasMag) {
    magBias_.x = (magMin.x + magMax.x) * 0.5f;
    magBias_.y = (magMin.y + magMax.y) * 0.5f;
    magBias_.z = (magMin.z + magMax.z) * 0.5f;
  }

  reset();
}

float AngleSensor::computeDeltaTime() {
  const uint32_t nowUs = micros();
  if (lastUpdateUs_ == 0) {
    lastUpdateUs_ = nowUs;
    return 0.0001f;
  }
  // The sensor fusion step needs the elapsed time between updates.
  const float dt = static_cast<float>(nowUs - lastUpdateUs_) / 1000000.0f;
  lastUpdateUs_ = nowUs;
  return dt > 0.0f ? dt : 0.0001f;
}

bool AngleSensor::update() {
  Vec3f accel;
  Vec3f gyro;
  if (!imu_.readAccel(accel) || !imu_.readGyro(gyro)) {
    return false;
  }

  gyro.x -= gyroBias_.x;
  gyro.y -= gyroBias_.y;
  gyro.z -= gyroBias_.z;

  const float dt = computeDeltaTime();
  Vec3f mag;
  // Use the magnetometer when available for better heading stability.
  if (imu_.hasMagnetometer() && imu_.readMag(mag)) {
    mag.x -= magBias_.x;
    mag.y -= magBias_.y;
    mag.z -= magBias_.z;
    return updateWithMag(accel, gyro, mag, dt);
  }
  return updateNoMag(accel, gyro, dt);
}

bool AngleSensor::updateNoMag(const Vec3f& accel, const Vec3f& gyro, float dt) {
  float ax = accel.x;
  float ay = accel.y;
  float az = accel.z;
  const float gx = toRadians(gyro.x);
  const float gy = toRadians(gyro.y);
  const float gz = toRadians(gyro.z);

  float q1 = q_[0];
  float q2 = q_[1];
  float q3 = q_[2];
  float q4 = q_[3];

  const float normAcc = sqrtf(ax * ax + ay * ay + az * az);
  if (normAcc == 0.0f) {
    return false;
  }
  ax /= normAcc;
  ay /= normAcc;
  az /= normAcc;

  const float _2q1 = 2.0f * q1;
  const float _2q2 = 2.0f * q2;
  const float _2q3 = 2.0f * q3;
  const float _2q4 = 2.0f * q4;
  const float _4q1 = 4.0f * q1;
  const float _4q2 = 4.0f * q2;
  const float _4q3 = 4.0f * q3;
  const float _8q2 = 8.0f * q2;
  const float _8q3 = 8.0f * q3;
  const float q1q1 = q1 * q1;
  const float q2q2 = q2 * q2;
  const float q3q3 = q3 * q3;
  const float q4q4 = q4 * q4;

  float s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
  float s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 +
             _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
  float s3 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 +
             _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
  float s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;

  const float stepNorm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
  if (stepNorm == 0.0f) {
    return false;
  }
  s1 /= stepNorm;
  s2 /= stepNorm;
  s3 /= stepNorm;
  s4 /= stepNorm;

  const float qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta_ * s1;
  const float qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta_ * s2;
  const float qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta_ * s3;
  const float qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta_ * s4;

  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;
  q4 += qDot4 * dt;

  const float quatNorm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  if (quatNorm == 0.0f) {
    return false;
  }
  q_[0] = q1 / quatNorm;
  q_[1] = q2 / quatNorm;
  q_[2] = q3 / quatNorm;
  q_[3] = q4 / quatNorm;

  // Convert the quaternion result into Euler angles for easier application use.
  heading_ = toDegrees(-atan2f(2.0f * (q_[1] * q_[2] + q_[0] * q_[3]),
                               q_[0] * q_[0] + q_[1] * q_[1] - q_[2] * q_[2] -
                                   q_[3] * q_[3]));
  pitch_ = toDegrees(-asinf(2.0f * (q_[1] * q_[3] - q_[0] * q_[2])));
  roll_ = toDegrees(atan2f(2.0f * (q_[0] * q_[1] + q_[2] * q_[3]),
                           q_[0] * q_[0] - q_[1] * q_[1] - q_[2] * q_[2] +
                               q_[3] * q_[3]));
  return true;
}

bool AngleSensor::updateWithMag(const Vec3f& accel, const Vec3f& gyro,
                                const Vec3f& mag, float dt) {
  float ax = accel.x;
  float ay = accel.y;
  float az = accel.z;
  float mx = mag.x;
  float my = mag.y;
  float mz = mag.z;
  const float gx = toRadians(gyro.x);
  const float gy = toRadians(gyro.y);
  const float gz = toRadians(gyro.z);

  float q1 = q_[0];
  float q2 = q_[1];
  float q3 = q_[2];
  float q4 = q_[3];

  const float accNorm = sqrtf(ax * ax + ay * ay + az * az);
  const float magNorm = sqrtf(mx * mx + my * my + mz * mz);
  if (accNorm == 0.0f || magNorm == 0.0f) {
    return false;
  }
  ax /= accNorm;
  ay /= accNorm;
  az /= accNorm;
  mx /= magNorm;
  my /= magNorm;
  mz /= magNorm;

  const float _2q1 = 2.0f * q1;
  const float _2q2 = 2.0f * q2;
  const float _2q3 = 2.0f * q3;
  const float _2q4 = 2.0f * q4;
  const float _2q1q3 = 2.0f * q1 * q3;
  const float _2q3q4 = 2.0f * q3 * q4;
  const float q1q1 = q1 * q1;
  const float q1q2 = q1 * q2;
  const float q1q3 = q1 * q3;
  const float q1q4 = q1 * q4;
  const float q2q2 = q2 * q2;
  const float q2q3 = q2 * q3;
  const float q2q4 = q2 * q4;
  const float q3q3 = q3 * q3;
  const float q3q4 = q3 * q4;
  const float q4q4 = q4 * q4;

  const float _2q1mx = 2.0f * q1 * mx;
  const float _2q1my = 2.0f * q1 * my;
  const float _2q1mz = 2.0f * q1 * mz;
  const float _2q2mx = 2.0f * q2 * mx;
  const float hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 +
                   _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  const float hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 -
                   my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  const float _2bx = sqrtf(hx * hx + hy * hy);
  const float _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 -
                     mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  const float _4bx = 2.0f * _2bx;
  const float _4bz = 2.0f * _2bz;

  float s1 = (-_2q3 * (2.0f * q2q4 - _2q1q3 - ax) +
              _2q2 * (2.0f * q1q2 + _2q3q4 - ay) -
              _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) +
                           _2bz * (q2q4 - q1q3) - mx) +
              (-_2bx * q4 + _2bz * q2) *
                  (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
              _2bx * q3 *
                  (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz));

  float s2 = (_2q4 * (2.0f * q2q4 - _2q1q3 - ax) +
              _2q1 * (2.0f * q1q2 + _2q3q4 - ay) -
              4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) +
              _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) +
                           _2bz * (q2q4 - q1q3) - mx) +
              (_2bx * q3 + _2bz * q1) *
                  (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
              (_2bx * q4 - _4bz * q2) *
                  (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz));

  float s3 = (-_2q1 * (2.0f * q2q4 - _2q1q3 - ax) +
              _2q4 * (2.0f * q1q2 + _2q3q4 - ay) -
              4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) +
              (-_4bx * q3 - _2bz * q1) *
                  (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
              (_2bx * q2 + _2bz * q4) *
                  (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
              (_2bx * q1 - _4bz * q3) *
                  (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz));

  float s4 = (_2q2 * (2.0f * q2q4 - _2q1q3 - ax) +
              _2q3 * (2.0f * q1q2 + _2q3q4 - ay) +
              (-_4bx * q4 + _2bz * q2) *
                  (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
              (-_2bx * q1 + _2bz * q3) *
                  (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
              _2bx * q2 *
                  (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz));

  const float stepNorm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
  if (stepNorm == 0.0f) {
    return false;
  }
  s1 /= stepNorm;
  s2 /= stepNorm;
  s3 /= stepNorm;
  s4 /= stepNorm;

  const float qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta_ * s1;
  const float qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta_ * s2;
  const float qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta_ * s3;
  const float qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta_ * s4;

  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;
  q4 += qDot4 * dt;

  const float quatNorm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  if (quatNorm == 0.0f) {
    return false;
  }
  q_[0] = q1 / quatNorm;
  q_[1] = q2 / quatNorm;
  q_[2] = q3 / quatNorm;
  q_[3] = q4 / quatNorm;

  // The magnetometer helps lock the yaw/heading estimate to the earth field.
  heading_ =
      declination_ +
      toDegrees(atan2f(2.0f * (q_[1] * q_[2] + q_[0] * q_[3]),
                       q_[0] * q_[0] + q_[1] * q_[1] - q_[2] * q_[2] -
                           q_[3] * q_[3]));
  pitch_ = toDegrees(-asinf(2.0f * (q_[1] * q_[3] - q_[0] * q_[2])));
  roll_ = toDegrees(atan2f(2.0f * (q_[0] * q_[1] + q_[2] * q_[3]),
                           q_[0] * q_[0] - q_[1] * q_[1] - q_[2] * q_[2] +
                               q_[3] * q_[3]));
  return true;
}

String AngleSensor::printData() const {
  String out = "Heading: ";
  out += String(heading_, 0);
  out += " Pitch: ";
  out += String(pitch_, 0);
  out += " Roll: ";
  out += String(roll_, 0);
  return out;
}

}  // namespace robotics
}  // namespace ohstem
