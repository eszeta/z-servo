// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#include "error.h"
#include "led.h"

namespace hortor::info_led {

/**
 * @brief LED闪烁模式的基本单元（1字节紧凑编码）
 * bit7: state, bit0-6: duration in 20ms units
 */
struct BlinkUnit {
  uint8_t raw;

  uint16_t duration_ms() const {
    return static_cast<uint16_t>(raw & 0x7Fu) * 20u;
  }
  bool state() const { return (raw & 0x80u) != 0; }

  static constexpr uint8_t kDurationUnitMs = 20;
  static constexpr uint8_t Make(uint16_t duration_ms, bool state) {
    return static_cast<uint8_t>((duration_ms / kDurationUnitMs) |
                                (state ? 0x80u : 0u));
  }
  constexpr BlinkUnit() : raw(0) {}
  constexpr BlinkUnit(uint8_t r) : raw(r) {}
};

/**
 * @brief 信息LED
 * @tparam M LED 输出模式（编译期确定，默认开漏）
 */
template <LedMode M = LedMode::kOpenDrain>
class InfoLED {
 public:
  enum class InfoType { kOk, kWarning, kError, kFatalError, kMax };

  void Init(uint32_t pin);
  void Init(PinName pinName);
  void SetInfo(InfoType type);
  void Stop();
  void ShowErrorCode(uint8_t code);
  void Process(float dt);

 private:
  size_t FillErrorCodePattern(uint8_t code);

  LED<M> led_;
  const BlinkUnit* pattern_ = nullptr;
  size_t pattern_size_ = 0;
  size_t step_ = 0;

  uint32_t elapsed_ms_ = 0;
  // ShowErrorCode 动态模式缓冲区（2*N+1 单元，N=闪烁次数，与 Error 枚举同步）
  static constexpr uint8_t kMaxCode = static_cast<uint8_t>(Error::kMax);
  static constexpr size_t kMaxCodePatternSize = 2 * kMaxCode + 1;
  BlinkUnit error_code_buffer_[kMaxCodePatternSize]{};

  static constexpr BlinkUnit kOkPattern[2] = {
      {BlinkUnit::Make(500, true)},
      {BlinkUnit::Make(500, false)},
  };
  static constexpr BlinkUnit kWarningPattern[2] = {
      {BlinkUnit::Make(200, true)},
      {BlinkUnit::Make(200, false)},
  };
  static constexpr BlinkUnit kErrorPattern[6] = {
      {BlinkUnit::Make(1000, true)},
      {BlinkUnit::Make(500, false)},
      {BlinkUnit::Make(200, true)},
      {BlinkUnit::Make(200, false)},
      {BlinkUnit::Make(200, true)},
      {BlinkUnit::Make(200, false)},
  };
  static constexpr BlinkUnit kFatalErrorPattern[8] = {
      {BlinkUnit::Make(200, true)},
      {BlinkUnit::Make(200, false)},
      {BlinkUnit::Make(200, true)},
      {BlinkUnit::Make(200, false)},
      {BlinkUnit::Make(200, true)},
      {BlinkUnit::Make(200, false)},
      {BlinkUnit::Make(1000, true)},
      {BlinkUnit::Make(1000, false)},
  };
};

}  // namespace hortor::info_led
