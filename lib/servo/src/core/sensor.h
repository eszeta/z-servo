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
  explicit Sensor(const uint8_t resolution) : kResolution(resolution) {}

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
  virtual uint16_t GetMechanicalAngle();

  /**
   * @brief 获取累积角度
   * @return 当前累积角度（机械角度 + 圈数 * 满量程）
   */
  virtual uint16_t GetAngle();

  /**
   * @brief 获取角速度
   * @return 当前角速度（单位：计数/秒）
   */
  virtual float GetVelocity();

  /**
   * @brief 获取圈数
   * @return 圈数（正值表示顺时针，负值表示逆时针）
   * @note 由于使用uint16_t类型，实际圈数范围受限
   */
  virtual int32_t GetFullRotations();

  /**
   * @brief 更新传感器数据
   * @param dt 时间间隔(微秒)
   * 读取最新的传感器值并计算相关参数，包括圈数和角速度。
   * 此方法应在主循环中定期调用以保持数据更新。
   */
  virtual void Process(uint32_t dt);

  /** @brief 传感器分辨率（位数），决定了传感器的精度和量程 */
  const uint8_t kResolution;
  /** @brief 传感器满量程值 */
  const uint16_t kFullScale = (1 << kResolution);
  /** @brief 溢出检测阈值，用于检测角度是否发生了溢出（通常为满量程的80%） */
  const float kOverflowTh = 0.8f * kFullScale;
  /**
   *  @brief
   * 角度到原始值的转换系数，用于将角度（0-360度）转换为原始值（0-16383）
   */
  const float kAngleToRaw = kFullScale / 360.0f;
  /**
   *  @brief 弧度到原始值的转换系数，用于将弧度（0-2π）转换为原始值（0-16383）
   */
  const float kRadianToRaw = kFullScale / TWO_PI;
  /**
   * @brief 原始值到角度的转换系数，用于将原始值（0-16383）转换为角度（0-360度）
   */
  const float kRawToAngle = 360.0f / kFullScale;
  /** @brief 原始值到弧度的转换系数，用于将原始值（0-16383）转换为弧度（0-2π）
   */
  const float kRawToRadian = TWO_PI / kFullScale;

 protected:
  /**
   * @brief 获取原始角度值
   * @return 传感器的原始角度值
   *
   * 子类必须实现此方法以提供特定传感器的原始角度读取功能。
   */
  virtual uint16_t GetRaw() = 0;

  /**
   * @brief 计算角速度
   * @param dt 时间间隔(微秒)
   *
   * 基于当前角度和上次记录的角度计算角速度。
   * 考虑了时间间隔和圈数的变化。
   * 计算结果单位为：计数/秒
   */
  void CalculateVelocity(uint32_t dt);
  /**
   * @brief 计算圈数
   *
   * 通过检测角度变化中的溢出来计算圈数。
   * 当角度变化超过阈值时，认为发生了一次完整旋转。
   */
  void CalculateFullRotations();
  /** @brief 最小采样时间间隔（微秒），固定为100微秒（10kHz） */
  static constexpr float kMinElapsedTime = 100.0f;

  /** @brief 当前角速度值 */
  float velocity_ = 0.0f;
  /** @brief 当前原始角度值 */
  uint16_t raw_val_ = 0;
  /** @brief 上次更新的原始角度值，用于检测溢出 */
  uint16_t raw_prev_ = 0;
  /** @brief 上次速度计算时的原始角度值 */
  uint16_t vel_raw_prev_ = 0;
  /** @brief 圈数计数器 */
  int32_t full_rotations_ = 0;
  /** @brief 上次速度计算时的圈数 */
  int32_t vel_full_rotations_ = 0;
  /** @brief 累计时间间隔（微秒） */
  uint32_t accumulated_dt_ = 0;
};
}  // namespace hortor_servo
