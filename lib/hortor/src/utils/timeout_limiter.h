// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "hortor.h"

namespace hortor::utils {

class TimeoutLimiter : public hortor::Noncopyable {
 public:
  void  set_threshold(float threshold);
  float threshold() const;
  void  set_timeout_duration(float timeout_duration);
  float timeout_duration() const;

  bool Process(float current_value, float dt);
  void Reset();

 private:
  float threshold_         = 0.0f;
  float timeout_duration_  = 0.0f;
  float exceeded_duration_ = 0.0f;
};

}  // namespace hortor::utils

namespace hortor::utils {

inline void TimeoutLimiter::set_threshold(float threshold) {
  threshold_ = threshold;
}

inline float TimeoutLimiter::threshold() const {
  return threshold_;
}

inline void TimeoutLimiter::set_timeout_duration(float timeout_duration) {
  timeout_duration_ = timeout_duration;
}

inline float TimeoutLimiter::timeout_duration() const {
  return timeout_duration_;
}

inline bool TimeoutLimiter::Process(float current_value, float dt) {
  if (current_value > threshold_) {
    exceeded_duration_ += dt;
  } else {
    exceeded_duration_ = 0.0f;
  }
  return exceeded_duration_ >= timeout_duration_;
}

inline void TimeoutLimiter::Reset() {
  exceeded_duration_ = 0.0f;
}

}  // namespace hortor::utils
