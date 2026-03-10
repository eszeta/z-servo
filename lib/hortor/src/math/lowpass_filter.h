// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "math.h"
#include "noncopyable.h"

namespace hortor::math {

/**
 * @brief 低通滤波器类
 */
class LowPassFilter : public hortor::Noncopyable {
 public:
  /**
   * @brief 构造函数
   * @param time_constant - 低通滤波器时间常数(秒)，必须大于0
   */
  explicit LowPassFilter(float time_constant = 0.1f);

  /**
   * @brief 计算滤波值
   * @param x - 输入值
   * @param dt - 时间间隔(秒)，必须大于0
   * @return 滤波值
   */
  float Compute(float x, float dt);

  /**
   * @brief 设置时间常数
   * @param time_constant - 时间常数(秒)，必须大于0
   */
  void set_time_constant(float time_constant);

  /**
   * @brief 获取时间常数
   * @return 时间常数(秒)
   */
  float time_constant() const;

  /**
   * @brief 重置滤波器状态
   */
  void Reset();

 protected:
  float time_constant_;  // 低通滤波器时间常数(秒)
  float y_prev_;         // 上一次滤波值
};

}  // namespace hortor::math

namespace hortor::math {

inline LowPassFilter::LowPassFilter(float time_constant) {
  set_time_constant(time_constant);
  y_prev_ = 0.0f;
}

inline float LowPassFilter::Compute(float x, float dt) {
  if (dt <= 0.0f || time_constant_ <= 0.0f) {
    return x;
  }

  const float alpha           = time_constant_ / (time_constant_ + dt);
  const float one_minus_alpha = 1.0f - alpha;
  const float y               = alpha * y_prev_ + one_minus_alpha * x;
  y_prev_                     = y;
  return y;
}

inline void LowPassFilter::set_time_constant(float time_constant) {
  if (time_constant > 0.0f) {
    time_constant_ = time_constant;
  }
}

inline float LowPassFilter::time_constant() const {
  return time_constant_;
}

inline void LowPassFilter::Reset() {
  y_prev_ = 0.0f;
}

}  // namespace hortor::math
