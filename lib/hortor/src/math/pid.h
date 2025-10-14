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

namespace hortor::math {

/**
 * @brief PID控制器类
 */
class Pid {
 public:
  struct Config {
    float proportional_gain;   // 比例增益 Kp
    float integral_gain;       // 积分增益 Ki
    float derivative_gain;     // 微分增益 Kd
    float antiwindup_gain;     // 抗饱和增益 Ka，用于抑制积分饱和
    float output_limit;        // 输出限幅值
  };

  explicit Pid(const Config& config);
  Pid(const Pid&) = delete;
  Pid& operator=(const Pid&) = delete;

  /**
   * @brief 计算PID控制器输出（无前馈）
   * @param error - 控制误差
   * @param dt - 时间间隔(秒)
   * @return 控制输出
   */
  float Compute(const float error,
                const float dt,
                const float feedforward = 0.0f);

  /**
   * @brief 重置PID控制器状态
   * 清除积分项和上一次的状态
   */
  void Reset();

  // 参数设置和获取接口
  void SetProportionalGain(float gain) { config_.proportional_gain = gain; }
  float GetProportionalGain() const { return config_.proportional_gain; }

  void SetIntegralGain(float gain) { config_.integral_gain = gain; }
  float GetIntegralGain() const { return config_.integral_gain; }

  void SetDerivativeGain(float gain) { config_.derivative_gain = gain; }
  float GetDerivativeGain() const { return config_.derivative_gain; }

  void SetAntiwindupGain(float gain) { config_.antiwindup_gain = gain; }
  float GetAntiwindupGain() const { return config_.antiwindup_gain; }

  void SetOutputLimit(float limit) { config_.output_limit = limit; }
  float GetOutputLimit() const { return config_.output_limit; }

 private:
  // 控制器参数
  Config config_;

  // 控制器状态
  float previous_error_ = 0.0f;        // 上一次的控制误差，用于计算微分项
  float integral_accumulator_ = 0.0f;   // 积分累加值，用于消除静态误差
  float antiwindup_feedback_ = 0.0f;    // 抗饱和反馈值，为输出饱和时被削减的部分
};

}  // namespace hortor::math
