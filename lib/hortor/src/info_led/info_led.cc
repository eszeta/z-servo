// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "info_led.h"

#include <Arduino.h>

namespace hortor::info_led {

// 预定义的信息类型模式定义
const BlinkUnit InfoLED::kOkPattern[2] = {{0.5, true}, {0.5, false}};

const BlinkUnit InfoLED::kWarningPattern[2] = {{0.2, true}, {0.2, false}};

const BlinkUnit InfoLED::kErrorPattern[6] = {{1.0, true},
                                             {0.5, false},
                                             {0.2, true},
                                             {0.2, false},
                                             {0.2, true},
                                             {0.2, false}};

const BlinkUnit InfoLED::kFatalErrorPattern[8] = {{0.2, true},
                                                  {0.2, false},
                                                  {0.2, true},
                                                  {0.2, false},
                                                  {0.2, true},
                                                  {0.2, false},
                                                  {1, true},
                                                  {1, false}};

void InfoLED::Init(const uint32_t pin, const Mode mode) {
  Init(digitalPinToPinName(pin), mode);
}

void InfoLED::Init(const PinName pinName, const Mode mode) {
  current_step_ = 0;
  current_pattern_ = nullptr;
  current_pattern_size_ = 0;
  elapsed_time_ = 0;
  led_.Init(pinName, mode);
}

void InfoLED::SetInfo(InfoType type) {
  switch (type) {
    case InfoType::kOk:
      current_pattern_ = kOkPattern;
      current_pattern_size_ = ARRAY_SIZE(kOkPattern);
      break;
    case InfoType::kWarning:
      current_pattern_ = kWarningPattern;
      current_pattern_size_ = ARRAY_SIZE(kWarningPattern);
      break;
    case InfoType::kError:
      current_pattern_ = kErrorPattern;
      current_pattern_size_ = ARRAY_SIZE(kErrorPattern);
      break;
    case InfoType::kFatalError:
      current_pattern_ = kFatalErrorPattern;
      current_pattern_size_ = ARRAY_SIZE(kFatalErrorPattern);
      break;
    default:
      current_pattern_ = nullptr;
      current_pattern_size_ = 0;
      break;
  }
  current_step_ = 0;
  elapsed_time_ = 0;
  if (current_pattern_ != nullptr) {
    led_.Turn(current_pattern_[current_step_].state);
  }
}

void InfoLED::Stop() {
  current_pattern_ = nullptr;
  current_pattern_size_ = 0;
  elapsed_time_ = 0;
  led_.Turn(false);
}

void InfoLED::Process(float dt) {
  if (current_pattern_ == nullptr) {
    return;
  }

  elapsed_time_ += dt;
  const auto& pattern = current_pattern_[current_step_];
  if (elapsed_time_ >= pattern.duration) {
    current_step_ = (current_step_ + 1) % current_pattern_size_;
    elapsed_time_ = 0;
    led_.Turn(current_pattern_[current_step_].state);
  }
}

}  // namespace hortor::info_led