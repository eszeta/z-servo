// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include <algorithm>

#include "math/math.h"
#include "math/resolution.h"
#include "servo/types.h"

namespace hortor::servo {

/**
 * @brief 传感器基类，提供编码器传感器的通用接口和功能实现
 */
template <typename DerivedType, uint8_t Bits>
class Encoder : public hortor::Noncopyable {
 public:
  struct Config {
    int32_t homing_offset;
    Reverse reverse;
  };

  /** @brief 传感器分辨率（编译期常量，存储在Flash） */
  static constexpr math::Resolution<Bits> kResolution{};

  /** @brief Recalibrate 末端区宽度（counts），pos_with_offset > CPR - kEdgeThreshold 时走末端分支；1° 对应 ceil(CPR/360)，至少 1 */
  static constexpr int32_t kEdgeThreshold =
      std::max(INT32_C(1), static_cast<int32_t>((kResolution.kEncoderCpr + 359) / 360));

  /**
   * @brief 获取原始计数值
   * @return 当前原始计数值
   */
  uint32_t raw_pos() const;

  /**
   * @brief 获取总累积计数值
   * @return 当前总累积计数值
   */
  int32_t pos() const;

  /**
   * @brief 获取圈数
   * @return 圈数
   */
  int32_t revolutions() const;

  /**
   * @brief 获取反转
   * @return 反转
   */
  Reverse reverse() const;
  void    set_reverse(const Reverse reverse);

  /** @brief 归零偏移 */
  int32_t homing_offset() const;
  void    set_homing_offset(const int32_t homing_offset);

  /**
   * @brief 初始化传感器
   *
   * 执行传感器初始化操作，包括初始读取和变量初始化。
   * 子类可以重写此方法以添加特定的初始化步骤。
   */
  Error Init(const Config& config);

  /**
   * @brief 更新传感器数据
   * @param dt 时间间隔(秒)
   * 读取最新的传感器值并计算相关参数，包括圈数和角速度。
   * 此方法应在主循环中定期调用以保持数据更新。
   */
  Error Process(float dt);

  /**
   * @brief 对齐到目标位置
   *
   * 将当前位置对齐到目标位置，通过调整 homing_offset_ 实现。
   * 调用此方法后，GetPos() 将返回目标位置。
   */
  Error AlignToPosition(uint32_t target);

 protected:
  /**
   * @brief 获取原始计数值
   * @return 传感器的原始计数值
   *
   * 子类必须实现此方法以提供特定传感器的原始计数值读取功能。
   */
  Error ReadRaw(uint32_t& out_raw);

  Error Recalibrate();

  // ========== 状态变量 ==========
  /** @brief 原始值 [0, CPR-1] */
  uint32_t raw_pos_ = 0;
  /** @brief 线性累加位置 [-∞, +∞] */
  int32_t pos_ = 0;
  /** @brief 反转 */
  Reverse reverse_ = Reverse::kNormal;
  /** @brief 归零偏移 */
  int32_t homing_offset_ = 0;
};

}  // namespace hortor::servo

namespace hortor::servo {

template <typename DerivedType, uint8_t Bits>
uint32_t Encoder<DerivedType, Bits>::raw_pos() const {
  return raw_pos_;
}

template <typename DerivedType, uint8_t Bits>
int32_t Encoder<DerivedType, Bits>::pos() const {
  return pos_;
}

template <typename DerivedType, uint8_t Bits>
int32_t Encoder<DerivedType, Bits>::revolutions() const {
  return pos() / kResolution.kEncoderCpr;
}

template <typename DerivedType, uint8_t Bits>
Reverse Encoder<DerivedType, Bits>::reverse() const {
  return reverse_;
}

template <typename DerivedType, uint8_t Bits>
void Encoder<DerivedType, Bits>::set_reverse(const Reverse reverse) {
  reverse_ = reverse;
  Recalibrate();
}

template <typename DerivedType, uint8_t Bits>
int32_t Encoder<DerivedType, Bits>::homing_offset() const {
  return homing_offset_;
}

template <typename DerivedType, uint8_t Bits>
void Encoder<DerivedType, Bits>::set_homing_offset(const int32_t homing_offset) {
  const auto delta_offset = homing_offset - homing_offset_;
  pos_ += delta_offset;
  homing_offset_ = homing_offset;
}

template <typename DerivedType, uint8_t Bits>
Error Encoder<DerivedType, Bits>::Init(const Config& config) {
  homing_offset_ = config.homing_offset;
  reverse_       = config.reverse;

  return Recalibrate();
}

template <typename DerivedType, uint8_t Bits>
Error Encoder<DerivedType, Bits>::Process(float dt) {
  (void)dt;
  uint32_t raw_new;
  CHECK(ReadRaw(raw_new));

  const int32_t delta = static_cast<int32_t>(raw_new) - static_cast<int32_t>(raw_pos_);

  const auto delta_enc = math::wrap_pm(delta, kResolution.kEncoderCpr);

  pos_ += delta_enc * static_cast<int32_t>(reverse_);
  raw_pos_ = raw_new;
  return Error::kOk;
}

template <typename DerivedType, uint8_t Bits>
Error Encoder<DerivedType, Bits>::AlignToPosition(uint32_t target) {
  const auto delta_to_target = static_cast<int32_t>(target) - pos();
  set_homing_offset(homing_offset_ + delta_to_target);
  return Error::kOk;
}

template <typename DerivedType, uint8_t Bits>
Error Encoder<DerivedType, Bits>::ReadRaw(uint32_t& out_raw) {
  return static_cast<DerivedType*>(this)->ReadRawImpl(out_raw);
}

template <typename DerivedType, uint8_t Bits>
Error Encoder<DerivedType, Bits>::Recalibrate() {
  CHECK(ReadRaw(raw_pos_));
  const auto reverse_val     = static_cast<int32_t>(reverse_);
  const auto local_pos       = static_cast<int32_t>(raw_pos_) * reverse_val;
  const auto normal_pos      = math::mod(local_pos, kResolution.kEncoderCpr);
  const auto pos_with_offset = math::mod(normal_pos + homing_offset_, kResolution.kEncoderCpr);

  if (pos_with_offset > kResolution.kEncoderCpr - kEdgeThreshold) {
    pos_ = pos_with_offset - kResolution.kEncoderCpr;
  } else {
    pos_ = pos_with_offset;
  }
  return Error::kOk;
}

}  // namespace hortor::servo
