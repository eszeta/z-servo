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

#include "servo/types.h"

namespace hortor::drivers::DRV8231A {

void DRV8231A::Init(const uint8_t pin_a, const uint8_t pin_b) {
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  pinMode(pin_a_, OUTPUT);
  pinMode(pin_b_, OUTPUT);
}

void DRV8231A::SetPWM(float pwm) {
  if (pwm > 0.0f) {
    pwm = constrain(pwm, 0.0f, 1.0f);
    analogWrite(pin_a_, 255 * pwm);
    analogWrite(pin_b_, 0);
  } else if (pwm < 0.0f) {
    pwm = constrain(-pwm, 0.0f, 1.0f);
    analogWrite(pin_a_, 0);
    analogWrite(pin_b_, 255 * pwm);
  } else {
    analogWrite(pin_a_, 0);
    analogWrite(pin_b_, 0);
  }
}

void DRV8231A::Break() {
  analogWrite(pin_a_, 255);
  analogWrite(pin_b_, 255);
}

}  // namespace hortor::drivers::DRV8231A
