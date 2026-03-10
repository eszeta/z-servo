// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "servo/motor.h"

namespace hortor::drivers::MP6515 {

class Motor;
using Base = servo::Motor<Motor>;

class Motor final : public Base {
 public:
  struct Config {
    uint8_t pin_phase;  // PHASE 相位引脚
    uint8_t pin_enbl;   // ENABLE 使能引脚（PWM）
    uint8_t pin_brake;  // BRAKE 制动引脚
    uint8_t pin_sleep;  // SLEEP 睡眠引脚
  };

  Error Init(const Config& config);
  void  SetPWMImpl(float pwm);
  void  BrakeImpl();
  void  CoastImpl();

 private:
  uint8_t pin_phase_ = 0;
  uint8_t pin_enbl_  = 0;
  uint8_t pin_brake_ = 0;
  uint8_t pin_sleep_ = 0;
};

}  // namespace hortor::drivers::MP6515

namespace hortor::drivers::MP6515 {

inline Error Motor::Init(const Config& config) {
  VERIFY(config.pin_phase != 0 && config.pin_enbl != 0 && config.pin_brake != 0 &&
             config.pin_sleep != 0,
         Error::kInvalidArg);

  CHECK(Base::Init());

  pin_phase_ = config.pin_phase;
  pin_enbl_  = config.pin_enbl;
  pin_brake_ = config.pin_brake;
  pin_sleep_ = config.pin_sleep;

  pinMode(pin_phase_, OUTPUT);
  pinMode(pin_enbl_, OUTPUT);
  pinMode(pin_brake_, OUTPUT);
  pinMode(pin_sleep_, OUTPUT);

  digitalWrite(pin_phase_, LOW);
  digitalWrite(pin_enbl_, LOW);
  digitalWrite(pin_brake_, LOW);
  digitalWrite(pin_sleep_, HIGH);

  return Error::kOk;
}

inline void Motor::SetPWMImpl(float pwm) {
  pwm = pwm * static_cast<int8_t>(reverse_);
  pwm = constrain(pwm, -1.0f, 1.0f);

  digitalWrite(pin_brake_, LOW);

  if (pwm > 0.0f) {
    digitalWrite(pin_phase_, HIGH);
    analogWrite(pin_enbl_, static_cast<uint32_t>(255 * pwm));
  } else if (pwm < 0.0f) {
    digitalWrite(pin_phase_, LOW);
    analogWrite(pin_enbl_, static_cast<uint32_t>(255 * (-pwm)));
  } else {
    digitalWrite(pin_phase_, LOW);
    digitalWrite(pin_enbl_, LOW);
  }
}

inline void Motor::BrakeImpl() {
  digitalWrite(pin_brake_, HIGH);
}

inline void Motor::CoastImpl() {
  digitalWrite(pin_brake_, LOW);
  digitalWrite(pin_enbl_, LOW);
}

}  // namespace hortor::drivers::MP6515
