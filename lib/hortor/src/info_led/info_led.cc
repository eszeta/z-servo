// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "info_led.h"

#include <Arduino.h>

namespace hortor::info_led {

namespace {

constexpr uint8_t kBlinkOn = BlinkUnit::Make(200, true);
constexpr uint8_t kBlinkOff = BlinkUnit::Make(200, false);
constexpr uint8_t kCodeZeroOn = BlinkUnit::Make(1000, true);
constexpr uint8_t kCycleGap = BlinkUnit::Make(1000, false);

}  // namespace

template <LedMode M>
void InfoLED<M>::Init(uint32_t pin) {
  Init(digitalPinToPinName(pin));
}

template <LedMode M>
void InfoLED<M>::Init(PinName pinName) {
  led_.Init(pinName);
}

template <LedMode M>
void InfoLED<M>::SetInfo(InfoType type) {
  const BlinkUnit* p = nullptr;
  size_t n = 0;
  switch (type) {
    case InfoType::kOk:
      p = kOkPattern;
      n = std::size(kOkPattern);
      break;
    case InfoType::kWarning:
      p = kWarningPattern;
      n = std::size(kWarningPattern);
      break;
    case InfoType::kError:
      p = kErrorPattern;
      n = std::size(kErrorPattern);
      break;
    case InfoType::kFatalError:
      p = kFatalErrorPattern;
      n = std::size(kFatalErrorPattern);
      break;
    default:
      break;
  }
  // 去重：已为此模式时不再重置，避免 loop 中重复 SetInfo 导致 pattern 不断重启
  if (pattern_ == p && pattern_size_ == n) {
    return;
  }
  if (p != nullptr) {
    pattern_ = p;
    pattern_size_ = n;
    step_ = 0;
    elapsed_ms_ = 0;
  }
}

template <LedMode M>
void InfoLED<M>::Stop() {
  pattern_ = nullptr;
  pattern_size_ = 0;
  step_ = 0;
  elapsed_ms_ = 0;
  led_.Turn(false);
}

template <LedMode M>
size_t InfoLED<M>::FillErrorCodePattern(uint8_t code) {
  size_t i = 0;
  const uint8_t n = (code > kMaxCode) ? kMaxCode : code;
  for (uint8_t k = 0; k < n; ++k) {
    error_code_buffer_[i++] = BlinkUnit(kBlinkOn);
    error_code_buffer_[i++] = BlinkUnit(kBlinkOff);
  }
  error_code_buffer_[i++] = BlinkUnit(kCycleGap);
  return i;
}

template <LedMode M>
void InfoLED<M>::ShowErrorCode(uint8_t code) {
  if (code == 0) {
    SetInfo(InfoType::kOk);
    return;
  }
  const auto n = FillErrorCodePattern(code);
  if (n > 0) {
    pattern_ = error_code_buffer_;
    pattern_size_ = n;
    step_ = 0;
    elapsed_ms_ = 0;
  }
}

template <LedMode M>
void InfoLED<M>::Process(float dt) {
  if (pattern_ == nullptr || pattern_size_ == 0) {
    return;
  }

  led_.Turn(pattern_[step_].state());
  elapsed_ms_ += static_cast<uint32_t>(dt * 1000.0f);
  if (elapsed_ms_ < pattern_[step_].duration_ms()) {
    return;
  }

  elapsed_ms_ = 0;
  step_ = (step_ + 1) % pattern_size_;
  led_.Turn(pattern_[step_].state());
}

template class InfoLED<LedMode::kOpenDrain>;
template class InfoLED<LedMode::kPushPull>;

}  // namespace hortor::info_led
