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
#include "DRV8231A.h"

#include <Arduino.h>

#include "../../servo_types.h"

namespace hortor_servo {
namespace DRV8231A {

void DRV8231A::InitDriver(const uint8_t pin_a, const uint8_t pin_b) {
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  pinMode(pin_a_, OUTPUT);
  pinMode(pin_b_, OUTPUT);
}

void DRV8231A::InitCurrentSensor(const uint8_t pin_adc, const uint16_t shunt_resistor, const float factor) {
  pin_adc_ = pin_adc;
  pinMode(pin_adc_, INPUT);
  CalibrateOffsets();
  gain_ = 1.0f / shunt_resistor / (factor / 1e6f);
}

void DRV8231A::SetPWM(float pwm) {
  if (pwm > 0.0f) {
    pwm = constrain(pwm, 0.0f, 1.0f);
    analogWrite(pin_a_, 255);
    analogWrite(pin_b_, static_cast<uint32_t>(255 - 255 * pwm));
  } else if (pwm < 0.0f) {
    pwm = constrain(-pwm, 0.0f, 1.0f);
    analogWrite(pin_a_, static_cast<uint32_t>(255 - 255 * pwm));
    analogWrite(pin_b_, 255);
  } else {
    analogWrite(pin_a_, 0);
    analogWrite(pin_b_, 0);
  }
}

void DRV8231A::CalibrateOffsets() {
  offset_ = 0;
  if (!pin_adc_) return;
  constexpr int calibration_rounds = 1000;
  for (int i = 0; i < calibration_rounds; i++) {
    offset_ += ReadADCVoltage();
    delay(1);
  }
  offset_ /= calibration_rounds;
}

float DRV8231A::GetCurrent() {
  if (!pin_adc_) {
    return 0;
  }
  const float current = ReadADCVoltage();
  return (current - offset_) * gain_;
}

float DRV8231A::ReadADCVoltage() { return static_cast<float>(analogRead(pin_adc_)) * kAdcVoltageConv; }
}  // namespace DRV8231A
}  // namespace hortor_servo
