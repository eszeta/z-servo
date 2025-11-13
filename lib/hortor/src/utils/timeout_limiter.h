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

namespace hortor::utils {

class TimeoutLimiter {
 public:
  /** @brief 设置阈值 */
  void SetThreshold(float threshold) { threshold_ = threshold; }
  /** @brief 获取阈值 */
  float GetThreshold() const { return threshold_; }
  /** @brief 设置超时时间 */
  void SetTimeoutDuration(float timeout_duration) {
    timeout_duration_ = timeout_duration;
  }
  /** @brief 获取超时时间 */
  float GetTimeoutDuration() const { return timeout_duration_; }

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
