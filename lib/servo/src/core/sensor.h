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

#include "../utils/math/lowpass_filter.h"
#include "../utils/math/math.h"
#include "./object_interface.h"

namespace hortor_servo {
/**
 * @brief 传感器基类，提供角度传感器的通用接口和功能实现
 *
 * 该类实现了角度传感器的基本功能，包括角度读取、速度计算和圈数统计。
 * 子类需要实现GetRaw()方法以提供特定传感器的原始角度值。
 */
class Sensor : public ObjectInterface {
 public:
  /**
   * @brief 构造函数
   * @param resolution 传感器分辨率（位数），决定了传感器的精度和量程
   */
  explicit Sensor(const uint8_t resolution) : kResolution(resolution) {
    velocity_lpf_.SetTimeConstant(0.01f);
    pos_lpf_.SetTimeConstant(0.01f);
  }

  /**
   * @brief 初始化传感器
   *
   * 执行传感器初始化操作，包括初始读取和变量初始化。
   * 子类可以重写此方法以添加特定的初始化步骤。
   */
  virtual void Init();

  /**
   * @brief 获取机械角度
   * @return 当前机械角度（原始值，未考虑圈数）
   */
  virtual uint16_t GetRawAngle() { return raw_; }

  /**
   * @brief 获取累积角度
   * @return 当前累积角度（机械角度 + 圈数 * 满量程）
   */
  virtual int32_t GetAngle() {
    return full_rotations_ * kEncoderCpr + raw_filtered_;
  }

  /**
   * @brief 获取角速度
   * @return 当前角速度（单位：计数/秒）
   */
  virtual float GetVelocity() { return velocity_filtered_; }

  /**
   * @brief 获取圈数
   * @return 圈数（正值表示顺时针，负值表示逆时针）
   */
  virtual int32_t GetFullRotations() { return full_rotations_; }

  /** @brief 获取角速度滤波器 */
  LowPassFilter &GetVelocityLpf() { return velocity_lpf_; }
  
  /** @brief 获取位置滤波器 */
  LowPassFilter &GetPosLpf() { return pos_lpf_; }

  /**
   * @brief 更新传感器数据
   * @param dt 时间间隔(秒)
   * 读取最新的传感器值并计算相关参数，包括圈数和角速度。
   * 此方法应在主循环中定期调用以保持数据更新。
   */
  virtual void Process(float dt);

  /** @brief 传感器分辨率（位数），决定了传感器的精度和量程 */
  const uint8_t kResolution;
  /** @brief 目标分辨率（位数），决定了传感器的精度和量程 */
  const uint8_t kTargetResolution = 11;
  /** @brief Counts Per Revolution */
  const uint16_t kEncoderCpr = (1 << kTargetResolution);
  /** @brief 溢出检测阈值，用于检测角度是否发生了溢出（通常为满量程的80%） */
  const float kOverflowTh = 0.8f * kEncoderCpr;
  /**
   *  @brief
   * 角度到原始值的转换系数，用于将角度转换为原始值
   */
  const float kAngleToRaw = kEncoderCpr / 360.0f;
  /**
   *  @brief 弧度到原始值的转换系数，用于将弧度转换为原始值
   */
  const float kRadianToRaw = kEncoderCpr / TWO_PI;
  /**
   * @brief 原始值到角度的转换系数，用于将原始值转换为角度
   */
  const float kRawToAngle = 360.0f / kEncoderCpr;
  /** @brief 原始值到弧度的转换系数，用于将原始值转换为弧度
   */
  const float kRawToRadian = TWO_PI / kEncoderCpr;

 protected:
  /**
   * @brief 获取原始角度值
   * @return 传感器的原始角度值
   *
   * 子类必须实现此方法以提供特定传感器的原始角度读取功能。
   */
  virtual uint16_t GetRaw() = 0;

  /** @brief 最小采样时间间隔（秒），固定为100微秒（10kHz） */
  static constexpr float kMinElapsedTime = 100.0f * kMicroToSec;

  /** @brief 当前角速度值 */
  float velocity_ = 0.0f;
  /** @brief 当前滤波后的角速度值 */
  float velocity_filtered_ = 0.0f;
  /** @brief 当前原始角度值 */
  uint16_t raw_ = 0;
  /** @brief 当前滤波后的原始角度值 */
  uint16_t raw_filtered_ = 0;
  /** @brief 上次更新的原始角度值 */
  uint16_t raw_prev_ = 0;
  /** @brief 圈数计数器 */
  int32_t full_rotations_ = 0;
  /** @brief 上次速度计算时的圈数 */
  int32_t full_rotations_prev_ = 0;
  /** @brief 累计时间间隔（微秒） */
  uint32_t accumulated_dt_ = 0;

  LowPassFilter velocity_lpf_;
  LowPassFilter pos_lpf_;
};
}  // namespace hortor_servo
