// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

namespace hortor::utils {

class TimeoutLimiter {
 public:
  /** @brief 设置阈值 */
  void set_threshold(float threshold) { threshold_ = threshold; }
  /** @brief 获取阈值 */
  float threshold() const { return threshold_; }
  /** @brief 设置超时时间 */
  void set_timeout_duration(float timeout_duration) {
    timeout_duration_ = timeout_duration;
  }
  /** @brief 获取超时时间 */
  float timeout_duration() const { return timeout_duration_; }

  /** @brief 处理 */
  bool Process(float current_value, float dt) {
    if (current_value > threshold_) {
      exceeded_duration_ += dt;
    } else {
      exceeded_duration_ = 0.0f;
    }
    return exceeded_duration_ >= timeout_duration_;
  }

  /** @brief 重置 */
  void Reset() { exceeded_duration_ = 0.0f; }

 private:
  /** @brief 阈值 */
  float threshold_ = 0.0f;
  /** @brief 超时时间 */
  float timeout_duration_ = 0.0f;
  /** @brief 超出时间 */
  float exceeded_duration_ = 0.0f;
};
}  // namespace hortor::utils
