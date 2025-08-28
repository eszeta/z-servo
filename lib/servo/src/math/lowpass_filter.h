// Copyright 2025 ES_ZETA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Arduino.h>

#include "math/math.h"

namespace hortor_servo {
/**
 * @brief 低通滤波器类
 */
class LowPassFilter {
 public:
  /**
   * @brief 构造函数
   * @param time_constant - 低通滤波器时间常数(秒)，必须大于0
   */
  explicit LowPassFilter(float time_constant = 0.1f) {
    SetTimeConstant(time_constant);
    y_prev_ = 0.0f;
  }

  /**
   * @brief 计算滤波值
   * @param x - 输入值
   * @param dt - 时间间隔(秒)，必须大于0
   * @return 滤波值
   */
  float Compute(float x, float dt) {
    if (dt <= 0.0f || time_constant_ <= 0.0f) {
      return x;  // 如果参数无效，直接返回输入值
    }

    const float alpha = time_constant_ / (time_constant_ + dt);
    const float one_minus_alpha = 1.0f - alpha;
    const float y = alpha * y_prev_ + one_minus_alpha * x;
    y_prev_ = y;
    return y;
  }

  /**
   * @brief 设置时间常数
   * @param time_constant - 时间常数(秒)，必须大于0
   */
  void SetTimeConstant(float time_constant) {
    if (time_constant > 0.0f) {
      time_constant_ = time_constant;
    }
  }

  /**
   * @brief 获取时间常数
   * @return 时间常数(秒)
   */
  float GetTimeConstant() const { return time_constant_; }

  /**
   * @brief 重置滤波器状态
   */
  void Reset() { y_prev_ = 0.0f; }

 protected:
  float time_constant_;  // 低通滤波器时间常数(秒)
  float y_prev_;         // 上一次滤波值
};
}  // namespace hortor_servo
