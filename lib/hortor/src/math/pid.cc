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

#include "pid.h"

#include <Arduino.h>

namespace hortor::math {

Pid::Pid(const Config& config) : config_(config) {}

/**
 * @brief 计算PID控制器输出
 */
float Pid::Compute(const float error, const float dt, const float feedforward) {
  // 比例项：直接响应当前误差
  const float proportional = config_.proportional_gain * error;

  // 积分项：使用梯形积分法，并应用抗饱和反馈
  const float integral = integral_accumulator_ +
                         config_.integral_gain * dt * 0.5f * (error + previous_error_) -
                         config_.antiwindup_gain * antiwindup_feedback_;

  // 微分项：使用误差变化率来抑制超调
  const float derivative = config_.derivative_gain * (error - previous_error_) / dt;

  // 综合所有控制项和前馈
  const float output = proportional + integral + derivative + feedforward;

  // 限幅处理
  const float output_clamped = constrain(output, -config_.output_limit, config_.output_limit);

  // 计算饱和误差，用于下次抗饱和反馈
  const float saturation_error = output - output_clamped;

  // 状态更新
  integral_accumulator_ = integral;
  previous_error_ = error;
  antiwindup_feedback_ = saturation_error;
  return output_clamped;
}

void Pid::Reset() {
  integral_accumulator_ = 0.0f;
  antiwindup_feedback_ = 0.0f;
  previous_error_ = 0.0f;
}

}  // namespace hortor::math
