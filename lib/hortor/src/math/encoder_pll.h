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

#include "hortor.h"
#include "resolution.h"
#include "servo/types.h"

namespace hortor::math {

template <uint8_t Bits>
class EncoderPll {
 public:
  explicit EncoderPll() {}
  Error Init(uint16_t pos) {
    pos_ = static_cast<float>(pos);
    velocity_ = 0.0f;
    return Error::kOk;
  }

  /**
   * @brief 获取估计位置
   * @return 估计位置（单位：pulse）
   */
  float Pos() const { return pos_; }

  /**
   * @brief 获取估计速度
   * @return 估计速度（单位：pulse/秒）
   */
  float GetVelocity() const { return velocity_; }

  /**
   * @brief 获取估计速度
   * @return 估计速度（单位：RPM）
   */
  float GetRpm() const { return rpm_; }
  /**
   * @brief 处理编码器数据
   * @param dt 时间间隔(秒)
   * @param pos_counts 编码器计数值
   * @param encoder_bits 编码器分辨率
   * @return 错误码
   */
  Error Process(float dt, int32_t pos_counts, uint8_t encoder_bits) {
    const auto mapped =
        math::mapResolution(pos_counts, encoder_bits, kResolution.kBits);
    // PLL 预测步骤：使用当前速度估计预测下一个位置
    pos_ += dt * velocity_;

    // PLL 误差计算
    float error = static_cast<float>(mapped) - pos_;

    // PLL 校正步骤：用比例-积分反馈校正估计值
    pos_ += dt * kPllKp * error;
    velocity_ += dt * kPllKi * error;

    // 零速度对齐：防止静止时的速度抖动
    const float threshold = 0.5f * dt * kPllKi;
    if (fabs(velocity_) < threshold) {
      velocity_ = 0.0f;
    }
    rpm_ = (velocity_ / kResolution.kEncoderCpr) * 60.0f;
    return Error::kOk;
  }

 private:
  static constexpr Resolution<Bits> kResolution{};

  /** @brief PLL带宽 [Hz] */
  const float kPllBandwidth = 200.0f;
  /** @brief PLL比例增益 */
  const float kPllKp = 2.0f * kPllBandwidth;
  /** @brief PLL积分增益（临界阻尼） */
  const float kPllKi = 0.25f * kPllKp * kPllKp;

  // PLL 状态变量
  /** @brief 线性位置估计（平滑后） */
  float pos_ = 0.0f;
  /** @brief 速度估计 [pulse/s] */
  float velocity_ = 0.0f;
  /** @brief 速度估计 [RPM] */
  float rpm_ = 0.0f;
};

}  // namespace hortor::math