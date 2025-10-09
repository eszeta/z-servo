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

#include "core/types.h"
#include "resolution.h"

namespace hortor_servo {

class EncoderPll {
 public:
  explicit EncoderPll(uint8_t resolution_bits) : kResolution(resolution_bits) {}
  Error Init(uint16_t pos) {
    pos_estimate_counts_ = static_cast<float>(pos);
    vel_estimate_counts_ = 0.0f;
    return Error::kOk;
  }

  /**
   * @brief 获取估计位置
   * @return 估计位置（单位：计数）
   */
  float GetPosition() { return pos_estimate_counts_; }

  /**
   * @brief 获取估计速度
   * @return 估计速度（单位：计数/秒）
   */
  float GetVelocity() { return vel_estimate_counts_; }

  Error Process(float dt, int32_t pos_counts) {
    // PLL 预测步骤：使用当前速度估计预测下一个位置
    pos_estimate_counts_ += dt * vel_estimate_counts_;

    // PLL 误差计算
    float error = static_cast<float>(pos_counts) - pos_estimate_counts_;

    // PLL 校正步骤：用比例-积分反馈校正估计值
    pos_estimate_counts_ += dt * kPllKp * error;
    vel_estimate_counts_ += dt * kPllKi * error;

    // 零速度对齐：防止静止时的速度抖动
    const float threshold = 0.5f * dt * kPllKi;
    if (fabs(vel_estimate_counts_) < threshold) {
      vel_estimate_counts_ = 0.0f;
    }

    return Error::kOk;
  }

 private:
  const Resolution kResolution;

  /** @brief PLL带宽 [Hz] */
  const float kPllBandwidth = 200.0f;
  /** @brief PLL比例增益 */
  const float kPllKp = 2.0f * kPllBandwidth;
  /** @brief PLL积分增益（临界阻尼） */
  const float kPllKi = 0.25f * kPllKp * kPllKp;

  // PLL 状态变量
  /** @brief 线性位置估计（平滑后） */
  float pos_estimate_counts_ = 0.0f;
  /** @brief 速度估计 [counts/s] */
  float vel_estimate_counts_ = 0.0f;
};
}  // namespace hortor_servo