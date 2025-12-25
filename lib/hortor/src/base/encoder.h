// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "hortor.h"
#include "math/lowpass_filter.h"
#include "math/math.h"
#include "math/resolution.h"
#include "types.h"

namespace hortor::servo {

/**
 * @brief 传感器基类，提供编码器传感器的通用接口和功能实现
 */
template <typename Derived, uint8_t Bits>
class Encoder {
 public:
  struct Config {
    int32_t homing_offset;
    Reverse reverse;
  };

  /** @brief 传感器分辨率（编译期常量，存储在Flash） */
  static constexpr math::Resolution<Bits> kResolution{};

  /**
   * @brief 获取原始计数值
   * @return 当前原始计数值
   */
  uint32_t rew_pos() const { return rew_pos_; }

  /**
   * @brief 获取总累积计数值
   * @return 当前总累积计数值
   */
  int32_t pos() const { return pos_; }

  /**
   * @brief 获取圈数
   * @return 圈数
   */
  int32_t revolutions() const { return pos() / kResolution.kEncoderCpr; }

  /**
   * @brief 获取反转
   * @return 反转
   */
  Reverse reverse() const { return reverse_; }
  void set_reverse(const Reverse reverse) { reverse_ = reverse; }

  /** @brief 归零偏移 */
  int32_t homing_offset() const { return homing_offset_; }
  void set_homing_offset(const int32_t homing_offset) {
    const auto delta_offset = homing_offset - homing_offset_;
    pos_ += delta_offset;
    homing_offset_ = homing_offset;
  }

  /**
   * @brief 初始化传感器
   *
   * 执行传感器初始化操作，包括初始读取和变量初始化。
   * 子类可以重写此方法以添加特定的初始化步骤。
   */
  Error Init(const Config& config) {
    // 读取初始原始值，等待传感器稳定后再次读取
    CHECK(ReadRaw(rew_pos_));
    delay(10);
    CHECK(ReadRaw(rew_pos_));

    // 初始化状态变量
    homing_offset_ = config.homing_offset;
    reverse_ = config.reverse;

    const auto reverse_val = static_cast<int32_t>(reverse_);
    const auto local_pos = static_cast<int32_t>(rew_pos_) * reverse_val;
    const auto normal_pos = math::mod(local_pos, kResolution.kEncoderCpr);
    const auto pos_with_offset =
        math::mod(normal_pos + homing_offset_, kResolution.kEncoderCpr);

    // 启动贴边小窗口，将接近 360° 的读数视为 -0.x°
    constexpr float kEdgeWindowDeg = 1.0f;  // 可按需改为 0.5f 等
    const float edge_counts_f =
        (static_cast<float>(kResolution.kEncoderCpr) / 360.0f) * kEdgeWindowDeg;
    const int32_t edge_threshold = max(ceil(edge_counts_f), 1.0f);
    if (pos_with_offset > kResolution.kEncoderCpr - edge_threshold) {
      pos_ = pos_with_offset - kResolution.kEncoderCpr;
    } else {
      pos_ = pos_with_offset;
    }
    return Error::kOk;
  }

  /**
   * @brief 更新传感器数据
   * @param dt 时间间隔(秒)
   * 读取最新的传感器值并计算相关参数，包括圈数和角速度。
   * 此方法应在主循环中定期调用以保持数据更新。
   */
  Error Process(float dt) {
    // 读取新的原始计数值
    uint32_t raw_new;
    CHECK(ReadRaw(raw_new));

    // 计算位置增量（新位置 - 旧位置）
    const int32_t delta =
        static_cast<int32_t>(raw_new) - static_cast<int32_t>(rew_pos_);

    const auto delta_enc = math::wrap_pm(delta, kResolution.kEncoderCpr);

    // 更新线性累加位置
    pos_ += delta_enc * static_cast<int32_t>(reverse_);

    // 更新原始值记录
    rew_pos_ = raw_new;
    return Error::kOk;
  }

  /**
   * @brief 对齐到目标位置
   *
    * 将当前位置对齐到目标位置，通过调整 homing_offset_ 实现。
    * 调用此方法后，GetPos() 将返回目标位置。
   */
  Error AlignToPosition(uint32_t target) {
    const auto current_normalized = math::mod(pos(), kResolution.kEncoderCpr);
    const auto delta_to_target = target - current_normalized;
    set_homing_offset(homing_offset_ + delta_to_target);
    return Error::kOk;
  }

 protected:
  /**
   * @brief 获取原始计数值
   * @return 传感器的原始计数值
   *
   * 子类必须实现此方法以提供特定传感器的原始计数值读取功能。
   */
  Error ReadRaw(uint32_t& out_raw) {
    return static_cast<Derived*>(this)->ReadRawImpl(out_raw);
  }

  // ========== 状态变量 ==========
  /** @brief 原始值 [0, CPR-1] */
  uint32_t rew_pos_ = 0;
  /** @brief 线性累加位置 [-∞, +∞] */
  int32_t pos_ = 0;
  /** @brief 反转 */
  Reverse reverse_ = Reverse::kNormal;
  /** @brief 归零偏移 */
  int32_t homing_offset_ = 0;
};

}  // namespace hortor::servo
