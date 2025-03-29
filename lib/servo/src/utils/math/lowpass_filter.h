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

#include "../../core/object_interface.h"
#include "./math_types.h"

namespace hortor_servo {
/**
 * @brief 低通滤波器类
 */
class LowPassFilter : public ObjectInterface {
 public:
  /**
   * @brief 构造函数
   * @param time_constant - 低通滤波器时间常数
   */
  explicit LowPassFilter();

  /**
   * @brief 计算滤波值
   * @param x - 输入值
   * @param dt - 时间间隔(微秒)
   * @return 滤波值
   */
  float Compute(float x, uint32_t dt);

  /**
   * @brief 设置时间常数
   * @param time_constant - 时间常数(秒)
   */
  void SetTimeConstant(float time_constant) { time_constant_ = time_constant; }

  /**
   * @brief 获取时间常数
   * @return 时间常数(秒)
   */
  float GetTimeConstant() const { return time_constant_; }

 protected:
  float time_constant_;  // 低通滤波器时间常数(秒)
  float y_prev_;         // 上一次滤波值
};
}  // namespace hortor_servo
