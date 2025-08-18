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

struct PIDParam {
  float kp;
  float ki;
  float kd;
  float ff;
  float i_limit;
  float limit;
};
/**
 * @brief PID控制器类
 */
class PidController {
 public:
  explicit PidController(const PIDParam& param);

  /**
   * @brief 计算PID控制器输出
   * @param error - 控制误差
   * @param feed - 前馈输入
   * @param dt - 时间间隔(秒)
   * @return 控制输出
   */
  float Compute(const float& error, const float& dt, const float& feed);

  /**
   * @brief 计算PID控制器输出（无前馈）
   * @param error - 控制误差
   * @param dt - 时间间隔(秒)
   * @return 控制输出
   */
  float Compute(const float& error, const float& dt);

  /**
   * @brief 重置PID控制器状态
   * 清除积分项和上一次的状态
   */
  void Reset();

  // 参数设置和获取接口
  void SetKp(float kp) { param_.kp = kp; }
  float GetKp() const { return param_.kp; }

  void SetKi(float ki) { param_.ki = ki; }
  float GetKi() const { return param_.ki; }

  void SetKd(float kd) { param_.kd = kd; }
  float GetKd() const { return param_.kd; }

  void SetFf(float ff) { param_.ff = ff; }
  float GetFf() const { return param_.ff; }

  void SetLimit(float limit) { param_.limit = limit; }
  float GetLimit() const { return param_.limit; }

  void SetIlimit(float ilimit) { param_.i_limit = ilimit; }
  float GetIlimit() const { return param_.i_limit; }

 private:
  // 控制器参数
  PIDParam param_;

  // 控制器状态
  float error_prev_ = 0.0f;    // 上一次误差，用于计算微分项
  float output_prev_ = 0.0f;   // 上一次输出，用于变化率限制
  float integral_sum_ = 0.0f;  // 积分累加值，用于消除静态误差
};
}  // namespace hortor_servo
