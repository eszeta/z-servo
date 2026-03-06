// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "base/motor.h"

namespace hortor::drivers::DRV8231A {

class DRV8231A;
using DRV8231ABase = servo::Motor<DRV8231A>;
class DRV8231A final : public DRV8231ABase {
 public:
  struct Config {
    uint8_t pin_in1;                      // IN1 控制引脚
    uint8_t pin_in2;                      // IN2 控制引脚
    uint8_t pin_nfault           = 0;     // nFAULT 引脚（0 表示不使用）
    float   slow_decay_threshold = 0.3f;  // 低速阈值（低于此值使用慢速衰减）
  };

  Error Init(const Config& config);
  void  SetPWMImpl(float pwm);
  void  BrakeImpl();
  void  CoastImpl();
  bool  HasFault() const;

 private:
  void SetPWMFastDecay(float pwm);
  void SetPWMSlowDecay(float pwm);

  uint8_t pin_in1_              = 0;
  uint8_t pin_in2_              = 0;
  uint8_t pin_nfault_           = 0;
  float   slow_decay_threshold_ = 0.3f;
};

}  // namespace hortor::drivers::DRV8231A

namespace hortor::drivers::DRV8231A {

inline Error DRV8231A::Init(const Config& config) {
  VERIFY(config.pin_in1 != 0 && config.pin_in2 != 0, Error::kInvalidArg);

  CHECK(DRV8231ABase::Init());

  pin_in1_              = config.pin_in1;
  pin_in2_              = config.pin_in2;
  pin_nfault_           = config.pin_nfault;
  slow_decay_threshold_ = config.slow_decay_threshold;

  pinMode(pin_in1_, OUTPUT);
  pinMode(pin_in2_, OUTPUT);

  if (pin_nfault_ != 0) {
    pinMode(pin_nfault_, INPUT_PULLUP);
  }

  digitalWrite(pin_in1_, LOW);
  digitalWrite(pin_in2_, LOW);

  return Error::kOk;
}

inline void DRV8231A::SetPWMImpl(float pwm) {
  pwm = pwm * static_cast<int8_t>(reverse_);
  pwm = constrain(pwm, -1.0f, 1.0f);

  const float abs_pwm = abs(pwm);

  if (abs_pwm < slow_decay_threshold_) {
    SetPWMSlowDecay(pwm);
  } else {
    SetPWMFastDecay(pwm);
  }
}

inline void DRV8231A::BrakeImpl() {
  digitalWrite(pin_in1_, HIGH);
  digitalWrite(pin_in2_, HIGH);
}

inline void DRV8231A::CoastImpl() {
  digitalWrite(pin_in1_, LOW);
  digitalWrite(pin_in2_, LOW);
}

inline bool DRV8231A::HasFault() const {
  if (pin_nfault_ == 0) {
    return false;
  }
  return digitalRead(pin_nfault_) == LOW;
}

inline void DRV8231A::SetPWMFastDecay(float pwm) {
  if (pwm > 0.0f) {
    analogWrite(pin_in1_, static_cast<uint32_t>(255 * pwm));
    digitalWrite(pin_in2_, LOW);
  } else if (pwm < 0.0f) {
    digitalWrite(pin_in1_, LOW);
    analogWrite(pin_in2_, static_cast<uint32_t>(255 * (-pwm)));
  } else {
    digitalWrite(pin_in1_, LOW);
    digitalWrite(pin_in2_, LOW);
  }
}

inline void DRV8231A::SetPWMSlowDecay(float pwm) {
  if (pwm > 0.0f) {
    digitalWrite(pin_in1_, HIGH);
    analogWrite(pin_in2_, static_cast<uint32_t>(255 * (1.0f - pwm)));
  } else if (pwm < 0.0f) {
    analogWrite(pin_in1_, static_cast<uint32_t>(255 * (1.0f + pwm)));
    digitalWrite(pin_in2_, HIGH);
  } else {
    digitalWrite(pin_in1_, LOW);
    digitalWrite(pin_in2_, LOW);
  }
}

}  // namespace hortor::drivers::DRV8231A