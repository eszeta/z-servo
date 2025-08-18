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

#include "math/lowpass_filter.h"
#include "math/math.h"
#include "math/resolution.h"

namespace hortor_servo {
/**
 * @brief 传感器基类，提供编码器传感器的通用接口和功能实现
 *
 * 该类实现了编码器传感器的基本功能，包括计数值读取、速度计算和圈数统计。
 * 子类需要实现GetRaw()方法以提供特定传感器的原始计数值。
 */
class Sensor {
 public:
  explicit Sensor(uint8_t resolution_bits) : kResolution(resolution_bits) {}

  /**
   * @brief 获取原始计数值
   * @return 当前原始计数值（未考虑圈数）
   */
  virtual uint16_t GetRawCounts() { return raw_; }

  /**
   * @brief 获取总累积计数值
   * @return 当前总累积计数值（原始计数值 + 圈数 * 满量程）
   */
  virtual int32_t GetTotalCounts() {
    return full_rotations_ * kResolution.kEncoderCpr + raw_;
  }

  /**
   * @brief 获取角速度
   * @return 当前角速度（单位：计数/秒）
   */
  virtual float GetVelocity() { return velocity_; }

  /**
   * @brief 获取圈数
   * @return 圈数（正值表示顺时针，负值表示逆时针）
   */
  virtual int32_t GetFullRotations() { return full_rotations_; }

  /**
   * @brief 初始化传感器
   *
   * 执行传感器初始化操作，包括初始读取和变量初始化。
   * 子类可以重写此方法以添加特定的初始化步骤。
   */
  virtual void Init();

  /**
   * @brief 更新传感器数据
   * @param dt 时间间隔(秒)
   * 读取最新的传感器值并计算相关参数，包括圈数和角速度。
   * 此方法应在主循环中定期调用以保持数据更新。
   */
  void Process(float dt);

  /** @brief 传感器分辨率（位数），决定了传感器的精度和量程 */
  const Resolution kResolution;

 protected:
  /**
   * @brief 获取原始计数值
   * @return 传感器的原始计数值
   *
   * 子类必须实现此方法以提供特定传感器的原始计数值读取功能。
   */
  virtual uint16_t GetRaw() = 0;

  /** @brief 当前角速度值 */
  float velocity_ = 0.0f;
  /** @brief 当前原始计数值 */
  uint16_t raw_ = 0;
  /** @brief 上次更新的原始计数值 */
  uint16_t raw_prev_ = 0;
  /** @brief 圈数计数器 */
  int32_t full_rotations_ = 0;
  /** @brief 上次速度计算时的圈数 */
  int32_t full_rotations_prev_ = 0;
};
}  // namespace hortor_servo
