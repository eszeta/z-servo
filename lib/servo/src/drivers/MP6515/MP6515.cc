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
#include "MP6515.h"

#include <Arduino.h>

#include "../../servo_types.h"

namespace hortor_servo {
namespace MP6515 {

void MP6515::Init(const uint8_t pin_phase,
                  const uint8_t pin_enbl,
                  const uint8_t pin_brake,
                  const uint8_t pin_sleep) {
  pin_phase_ = pin_phase;
  pin_enbl_ = pin_enbl;
  pin_brake_ = pin_brake;
  pin_sleep_ = pin_sleep;

  pinMode(pin_phase_, OUTPUT);
  pinMode(pin_enbl_, OUTPUT);
  pinMode(pin_brake_, OUTPUT);
  pinMode(pin_sleep_, OUTPUT);

  digitalWrite(pin_phase_, 0);
  analogWrite(pin_enbl_, 0);
  digitalWrite(pin_brake_, 0);
  digitalWrite(pin_sleep_, 1);
}

void MP6515::SetPWM(float pwm) {
  digitalWrite(pin_brake_, LOW);
  if (pwm > 0.0f) {
    pwm = constrain(pwm, 0.0f, 1.0f);
    digitalWrite(pin_phase_, 1);
    analogWrite(pin_enbl_, static_cast<uint32_t>(255 * pwm));
  } else if (pwm < 0.0f) {
    pwm = constrain(-pwm, 0.0f, 1.0f);
    digitalWrite(pin_phase_, 0);
    analogWrite(pin_enbl_, static_cast<uint32_t>(255 * pwm));
  } else {
    digitalWrite(pin_phase_, 0);
    analogWrite(pin_enbl_, 0);
  }
}

void MP6515::Break() { digitalWrite(pin_brake_, HIGH); }
}  // namespace MP6515
}  // namespace hortor_servo
