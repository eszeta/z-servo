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

namespace hortor::drivers::MP6515 {

Error MP6515::Init(const Config& config) {
  if (config.pin_phase == 0 || config.pin_enbl == 0 || config.pin_brake == 0 ||
      config.pin_sleep == 0) {
    return Error::kInvalidParameter;
  }

  pin_phase_ = config.pin_phase;
  pin_enbl_ = config.pin_enbl;
  pin_brake_ = config.pin_brake;
  pin_sleep_ = config.pin_sleep;

  // 配置引脚模式
  pinMode(pin_phase_, OUTPUT);
  pinMode(pin_enbl_, OUTPUT);
  pinMode(pin_brake_, OUTPUT);
  pinMode(pin_sleep_, OUTPUT);

  // 初始状态：滑行模式
  digitalWrite(pin_phase_, LOW);
  digitalWrite(pin_enbl_, LOW);
  digitalWrite(pin_brake_, LOW);
  digitalWrite(pin_sleep_, HIGH);  // 退出睡眠

  initialized_ = true;
  return Error::kOk;
}

void MP6515::SetPWM(float pwm) {
  // 限制 PWM 范围
  pwm = constrain(pwm, -1.0f, 1.0f);

  // 关闭制动
  digitalWrite(pin_brake_, LOW);

  if (pwm > 0.0f) {
    // 正转：PHASE=1, ENABLE=PWM
    digitalWrite(pin_phase_, HIGH);
    analogWrite(pin_enbl_, static_cast<uint32_t>(255 * pwm));
  } else if (pwm < 0.0f) {
    // 反转：PHASE=0, ENABLE=PWM
    digitalWrite(pin_phase_, LOW);
    analogWrite(pin_enbl_, static_cast<uint32_t>(255 * (-pwm)));
  } else {
    // 滑行模式：ENABLE=0
    digitalWrite(pin_phase_, LOW);
    digitalWrite(pin_enbl_, LOW);
  }
}

void MP6515::Brake() {
  // 制动模式：BRAKE=1
  digitalWrite(pin_brake_, HIGH);
}

void MP6515::Coast() {
  // 滑行模式：BRAKE=0, ENABLE=0
  digitalWrite(pin_brake_, LOW);
  digitalWrite(pin_enbl_, LOW);
}

}  // namespace hortor::drivers::MP6515
