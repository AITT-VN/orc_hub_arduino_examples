#ifndef OHSTEM_PID_CONTROLLER_H
#define OHSTEM_PID_CONTROLLER_H

#include <Arduino.h>
#include <math.h>

#include "RoboticsUtils.h"

namespace ohstem {
namespace robotics {

class PIDController {
 public:
  PIDController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f,
                float setpoint = 0.0f, float sampleTime = 0.01f,
                float outputMin = -INFINITY, float outputMax = INFINITY)
      : kp_(kp),
        ki_(ki),
        kd_(kd),
        setpoint_(setpoint),
        sampleTime_(sampleTime),
        outputMin_(outputMin),
        outputMax_(outputMax) {
    reset();
  }

  float compute(float input, float dt = -1.0f) {
    if (!autoMode_) {
      return lastOutput_;
    }

    const uint32_t nowUs = micros();
    if (dt < 0.0f) {
      dt = lastTimeUs_ == 0 ? 0.0001f : static_cast<float>(nowUs - lastTimeUs_) / 1000000.0f;
      if (dt <= 0.0f) {
        dt = 0.0001f;
      }
    }

    if (sampleTime_ > 0.0f && dt < sampleTime_ && hasOutput_) {
      return lastOutput_;
    }

    float error = setpoint_ - input;
    const float dInput = hasInput_ ? (input - lastInput_) : 0.0f;
    const float dError = hasError_ ? (error - lastError_) : 0.0f;

    if (!proportionalOnMeasurement_) {
      proportional_ = kp_ * error;
    } else {
      proportional_ -= kp_ * dInput;
    }

    integral_ += ki_ * error * dt;
    integral_ = clampValue(integral_, outputMin_, outputMax_);

    if (differentialOnMeasurement_) {
      derivative_ = -kd_ * dInput / dt;
    } else {
      derivative_ = kd_ * dError / dt;
    }

    lastOutput_ = proportional_ + integral_ + derivative_;
    lastOutput_ = clampValue(lastOutput_, outputMin_, outputMax_);

    lastInput_ = input;
    lastError_ = error;
    lastTimeUs_ = nowUs;
    hasOutput_ = true;
    hasInput_ = true;
    hasError_ = true;
    return lastOutput_;
  }

  void setTunings(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void setSetpoint(float setpoint) { setpoint_ = setpoint; }

  void setSampleTime(float sampleTime) { sampleTime_ = sampleTime; }

  void setOutputLimits(float minOutput, float maxOutput) {
    outputMin_ = minOutput;
    outputMax_ = maxOutput;
    integral_ = clampValue(integral_, outputMin_, outputMax_);
    if (hasOutput_) {
      lastOutput_ = clampValue(lastOutput_, outputMin_, outputMax_);
    }
  }

  void setAutoMode(bool enabled, float lastOutput = 0.0f) {
    if (enabled && !autoMode_) {
      reset();
      integral_ = clampValue(lastOutput, outputMin_, outputMax_);
    }
    autoMode_ = enabled;
  }

  void setProportionalOnMeasurement(bool enabled) {
    proportionalOnMeasurement_ = enabled;
  }

  void setDifferentialOnMeasurement(bool enabled) {
    differentialOnMeasurement_ = enabled;
  }

  void reset() {
    proportional_ = 0.0f;
    integral_ = 0.0f;
    derivative_ = 0.0f;
    lastInput_ = 0.0f;
    lastError_ = 0.0f;
    lastOutput_ = 0.0f;
    lastTimeUs_ = micros();
    hasOutput_ = false;
    hasInput_ = false;
    hasError_ = false;
  }

 private:
  float kp_;
  float ki_;
  float kd_;
  float setpoint_;
  float sampleTime_;
  float outputMin_;
  float outputMax_;

  bool autoMode_ = true;
  bool proportionalOnMeasurement_ = false;
  bool differentialOnMeasurement_ = true;

  float proportional_ = 0.0f;
  float integral_ = 0.0f;
  float derivative_ = 0.0f;

  float lastInput_ = 0.0f;
  float lastError_ = 0.0f;
  float lastOutput_ = 0.0f;
  uint32_t lastTimeUs_ = 0;

  bool hasOutput_ = false;
  bool hasInput_ = false;
  bool hasError_ = false;
};

}  // namespace robotics
}  // namespace ohstem

#endif
