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
#include "types.h"

namespace hortor_servo {
/**
 * @brief 传感器基类，提供编码器传感器的通用接口和功能实现
 *
 * 该类实现了编码器传感器的基本功能，包括计数值读取、速度计算和圈数统计。
 * 子类需要实现GetRaw()方法以提供特定传感器的原始计数值。
 */
class Encoder {
 public:
  explicit Encoder(uint8_t resolution_bits) : kResolution(resolution_bits) {}

  /**
   * @brief 获取原始计数值
   * @return 当前原始计数值
   */
  uint16_t GetRewPos() { return rew_pos_; }

  /**
   * @brief 获取总累积计数值
   * @return 当前总累积计数值
   */
  int32_t GetPosCounts() { return pos_counts_; }

  /**
   * @brief 获取圈数
   * @return 圈数（正值表示顺时针，负值表示逆时针）
   */
  int32_t GetRevolutions() { return pos_counts_ / kResolution.kEncoderCpr; }

  /**
   * @brief 初始化传感器
   *
   * 执行传感器初始化操作，包括初始读取和变量初始化。
   * 子类可以重写此方法以添加特定的初始化步骤。
   */
  virtual Error Init();

  /**
   * @brief 更新传感器数据
   * @param dt 时间间隔(秒)
   * 读取最新的传感器值并计算相关参数，包括圈数和角速度。
   * 此方法应在主循环中定期调用以保持数据更新。
   */
  Error Process(float dt);

  /** @brief 传感器分辨率（位数），决定了传感器的精度和量程 */
  const Resolution kResolution;

 protected:
  /**
   * @brief 获取原始计数值
   * @return 传感器的原始计数值
   *
   * 子类必须实现此方法以提供特定传感器的原始计数值读取功能。
   */
  virtual Error GetRaw(uint16_t &out_raw) = 0;

  // ========== 状态变量 ==========
  /** @brief 原始值 [0, CPR-1] */
  uint16_t rew_pos_ = 0;
  /** @brief 线性累加位置 [-∞, +∞] */
  int32_t pos_counts_ = 0;
};

}  // namespace hortor_servo
