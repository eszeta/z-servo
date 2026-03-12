// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "math.h"
#include "resolution.h"
#include "servo/types.h"

namespace hortor::math {

template <typename EncoderType, uint8_t Bits>
class EncoderPll : public hortor::Noncopyable {
 public:
  Error Init(EncoderType* encoder);

  EncoderType* encoder() const;

  /**
   * @brief 获取估计位置
   * @return 估计位置（单位：pulse）
   */
  float pos() const;

  /**
   * @brief 获取估计速度
   * @return 估计速度（单位：pulse/秒）
   */
  float velocity() const;

  /**
   * @brief 获取估计速度
   * @return 估计速度（单位：RPM）
   */
  float rpm() const;

  /**
   * @brief 处理编码器数据
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error Process(float dt);

 private:
  static constexpr Resolution<Bits> kResolution{};

  /** @brief PLL带宽 [Hz] */
  static constexpr float kPllBandwidth = 200.0f;
  /** @brief PLL比例增益 */
  static constexpr float kPllKp = 2.0f * kPllBandwidth;
  /** @brief PLL积分增益（临界阻尼） */
  static constexpr float kPllKi = 0.25f * kPllKp * kPllKp;

  // PLL 状态变量
  /** @brief 线性位置估计（平滑后） */
  float pos_ = 0.0f;
  /** @brief 速度估计 [pulse/s] */
  float velocity_ = 0.0f;
  /** @brief 速度估计 [RPM] */
  float rpm_ = 0.0f;

  /** @brief 角度传感器 */
  EncoderType* encoder_ = nullptr;
};

}  // namespace hortor::math

namespace hortor::math {

template <typename EncoderType, uint8_t Bits>
Error EncoderPll<EncoderType, Bits>::Init(EncoderType* encoder) {
  encoder_                = encoder;
  const auto encoder_pos  = encoder->pos();
  const auto encoder_bits = encoder->kResolution.kBits;
  const auto init_pos     = math::mapResolution(encoder_pos, encoder_bits, kResolution.kBits);
  pos_                    = static_cast<float>(init_pos);
  velocity_               = 0.0f;
  return Error::kOk;
}

template <typename EncoderType, uint8_t Bits>
EncoderType* EncoderPll<EncoderType, Bits>::encoder() const {
  return encoder_;
}

template <typename EncoderType, uint8_t Bits>
float EncoderPll<EncoderType, Bits>::pos() const {
  return pos_;
}

template <typename EncoderType, uint8_t Bits>
float EncoderPll<EncoderType, Bits>::velocity() const {
  return velocity_;
}

template <typename EncoderType, uint8_t Bits>
float EncoderPll<EncoderType, Bits>::rpm() const {
  return rpm_;
}

template <typename EncoderType, uint8_t Bits>
Error EncoderPll<EncoderType, Bits>::Process(float dt) {
  CHECK(encoder_->Process(dt));
  const auto encoder_pos  = encoder_->pos();
  const auto encoder_bits = encoder_->kResolution.kBits;
  const auto mapped       = math::mapResolution(encoder_pos, encoder_bits, kResolution.kBits);
  pos_ += dt * velocity_;

  float error = static_cast<float>(mapped) - pos_;

  pos_ += dt * kPllKp * error;
  velocity_ += dt * kPllKi * error;

  /** @brief 速度死区：低于此值视为静止，防止数值噪声产生虚假速度 [pulse/s] */
  static constexpr float kVelocityDeadband = 1.0f;
  if (fabs(velocity_) < kVelocityDeadband) {
    velocity_ = 0.0f;
  }
  rpm_ = (velocity_ / kResolution.kEncoderCpr) * 60.0f;
  return Error::kOk;
}

}  // namespace hortor::math