// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "error.h"
#include "math/resolution.h"
#include "servo/encoder.h"
#include "servo/types.h"

namespace hortor::simulator {

/** @brief 仿真编码器分辨率位数，与 Servo kResolutionBits 一致 */
constexpr uint8_t kSimEncoderBits = 12;

/**
 * @brief 仿真用编码器
 *
 * 不读真实硬件，原始位置由仿真 plant 通过 SetRawPosition 写入，用于算法验证。
 */
class SimulatorEncoder final : public servo::Encoder<SimulatorEncoder, kSimEncoderBits> {
 public:
  /**
   * @brief 设置仿真原始位置 [0, CPR-1]
   *
   * 由仿真 plant 每步调用，供下一拍 Servo::Process 读取。
   */
  void SetRawPosition(uint32_t raw);

  Error ReadRawImpl(uint32_t& out_raw);
};

}  // namespace hortor::simulator

namespace hortor::simulator {

inline void SimulatorEncoder::SetRawPosition(uint32_t raw) {
  constexpr uint32_t kCpr = kResolution.kEncoderCpr;
  raw_pos_                = (raw < kCpr) ? raw : (kCpr - 1);
}

inline Error SimulatorEncoder::ReadRawImpl(uint32_t& out_raw) {
  out_raw = raw_pos_;
  return Error::kOk;
}

}  // namespace hortor::simulator
