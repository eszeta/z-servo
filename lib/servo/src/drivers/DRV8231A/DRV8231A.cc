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

void DRV8231A::Init(const uint8_t pin_a, const uint8_t pin_b) {
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  pinMode(pin_a_, OUTPUT);
  pinMode(pin_b_, OUTPUT);
}

void DRV8231A::SetPWM(float pwm) {
  if (pwm > 0.0f) {
    pwm = constrain(pwm, 0.0f, 1.0f);
    digitalWrite(pin_a_, HIGH);
    analogWrite(pin_b_, static_cast<uint32_t>(255 - 255 * pwm));
  } else if (pwm < 0.0f) {
    pwm = constrain(-pwm, 0.0f, 1.0f);
    analogWrite(pin_a_, static_cast<uint32_t>(255 - 255 * pwm));
    digitalWrite(pin_b_, HIGH);
  } else {
    digitalWrite(pin_a_, HIGH);
    digitalWrite(pin_b_, HIGH);
  }
}

void DRV8231A::Break() {
  digitalWrite(pin_a_, LOW);
  digitalWrite(pin_b_, LOW);
}
}  // namespace DRV8231A
}  // namespace hortor_servo
