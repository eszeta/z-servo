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

#include "drv8231a.h"

#include <Arduino.h>

namespace hortor::drivers::DRV8231A {

Error DRV8231A::Init(const Config& config) {
  if (config.pin_in1 == 0 || config.pin_in2 == 0) {
    return Error::kInvalidParameter;
  }

  pin_in1_ = config.pin_in1;
  pin_in2_ = config.pin_in2;
  pin_nfault_ = config.pin_nfault;
  slow_decay_threshold_ = config.slow_decay_threshold;

  // 配置 IN1 和 IN2 为输出
  pinMode(pin_in1_, OUTPUT);
  pinMode(pin_in2_, OUTPUT);

  // 配置 nFAULT 为输入（如果使用）
  if (pin_nfault_ != 0) {
    pinMode(pin_nfault_, INPUT_PULLUP);
  }

  // 初始状态：滑行模式
  digitalWrite(pin_in1_, LOW);
  digitalWrite(pin_in2_, LOW);

  return Error::kOk;
}

void DRV8231A::SetPWMImpl(float pwm) {
  // 限制 PWM 范围
  pwm = constrain(pwm, -1.0f, 1.0f);

  // 根据 PWM 绝对值选择衰减模式
  const float abs_pwm = abs(pwm);

  if (abs_pwm < slow_decay_threshold_) {
    // 低速：使用慢速衰减模式
    SetPWMSlowDecay(pwm);
  } else {
    // 高速：使用快速衰减模式
    SetPWMFastDecay(pwm);
  }
}

void DRV8231A::SetPWMFastDecay(float pwm) {
  if (pwm > 0.0f) {
    // 正向驱动：IN1=PWM, IN2=0（快速衰减）
    analogWrite(pin_in1_, static_cast<uint32_t>(255 * pwm));
    digitalWrite(pin_in2_, LOW);
  } else if (pwm < 0.0f) {
    // 反向驱动：IN1=0, IN2=PWM（快速衰减）
    digitalWrite(pin_in1_, LOW);
    analogWrite(pin_in2_, static_cast<uint32_t>(255 * (-pwm)));
  } else {
    // 滑行模式：IN1=0, IN2=0
    digitalWrite(pin_in1_, LOW);
    digitalWrite(pin_in2_, LOW);
  }
}

void DRV8231A::SetPWMSlowDecay(float pwm) {
  if (pwm > 0.0f) {
    // 正向驱动：IN1=1, IN2=PWM反相（慢速衰减）
    // PWM 高时制动，PWM 低时驱动，电流缓慢衰减
    digitalWrite(pin_in1_, HIGH);
    analogWrite(pin_in2_, static_cast<uint32_t>(255 * (1.0f - pwm)));
  } else if (pwm < 0.0f) {
    // 反向驱动：IN1=PWM反相, IN2=1（慢速衰减）
    // PWM 高时制动，PWM 低时驱动，电流缓慢衰减
    analogWrite(pin_in1_, static_cast<uint32_t>(255 * (1.0f + pwm)));
    digitalWrite(pin_in2_, HIGH);
  } else {
    // 滑行模式：IN1=0, IN2=0
    digitalWrite(pin_in1_, LOW);
    digitalWrite(pin_in2_, LOW);
  }
}

void DRV8231A::BrakeImpl() {
  // 制动模式：IN1=1, IN2=1（低边导通）
  digitalWrite(pin_in1_, HIGH);
  digitalWrite(pin_in2_, HIGH);
}

void DRV8231A::CoastImpl() {
  // 滑行模式：IN1=0, IN2=0（高阻态）
  digitalWrite(pin_in1_, LOW);
  digitalWrite(pin_in2_, LOW);
}

bool DRV8231A::HasFault() const {
  if (pin_nfault_ == 0) {
    return false;  // 未配置故障检测引脚
  }
  // nFAULT 为低电平时表示有故障
  return digitalRead(pin_nfault_) == LOW;
}

}  // namespace hortor::drivers::DRV8231A
