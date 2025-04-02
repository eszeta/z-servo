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

#include "generic_current.h"

namespace hortor_servo {
namespace generic_current {

void GenericCurrent::Init(const uint8_t pin_adc,
                          const uint16_t shunt_resistor,
                          const float factor) {
  pin_adc_ = pin_adc;
  pinMode(pin_adc_, INPUT);
  CalibrateOffsets();
  gain_ = 1.0f / shunt_resistor / (factor / 1e6f);
}

void GenericCurrent::CalibrateOffsets() {
  offset_ = 0;
  if (!pin_adc_) return;
  constexpr int calibration_rounds = 1000;
  for (int i = 0; i < calibration_rounds; i++) {
    offset_ += ReadADCVoltage();
    delay(1);
  }
  offset_ /= calibration_rounds;
}

float GenericCurrent::GetCurrent() {
  if (!pin_adc_) {
    return 0;
  }
  const float current = ReadADCVoltage();
  return (current - offset_) * gain_;
}

float GenericCurrent::ReadADCVoltage() {
  return static_cast<float>(analogRead(pin_adc_)) * kAdcVoltageConv;
}

}  // namespace generic_current
}  // namespace hortor_servo