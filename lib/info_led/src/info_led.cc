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
#include "info_led.h"

#include <Arduino.h>

namespace hortor_servo {
namespace InfoLED {

void InfoLED::Init(const uint32_t pin, const Mode mode) {
  Init(digitalPinToPinName(pin), mode);
}

void InfoLED::Init(const PinName pinName, const Mode mode) {
  current_step_ = 0;
  current_pattern_ = nullptr;
  elapsed_time_ms_ = 0;
  led_.Init(pinName, mode);
}

void InfoLED::SetInfo(InfoType type) {
  current_pattern_ = &patterns_[static_cast<size_t>(type)];
  current_step_ = 0;
  elapsed_time_ms_ = 0;
  led_.Turn((*current_pattern_)[current_step_].state);
}

void InfoLED::Stop() {
  current_pattern_ = nullptr;
  elapsed_time_ms_ = 0;
  led_.Turn(false);
}

void InfoLED::Process(uint32_t dt) {
  elapsed_time_ms_ += dt;
  if (elapsed_time_ms_ >= (*current_pattern_)[current_step_].duration_ms) {
    current_step_ = (current_step_ + 1) % current_pattern_->size();
    elapsed_time_ms_ = 0;
    led_.Turn((*current_pattern_)[current_step_].state);
  }
}

}  // namespace InfoLED
}  // namespace hortor_servo