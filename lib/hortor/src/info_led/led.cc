// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "led.h"

namespace hortor::info_led {

void LED::Init(const uint32_t pin, const Mode mode) {
  Init(digitalPinToPinName(pin), mode);
}

void LED::Init(const PinName pinName, const Mode mode) {
  mode_ = mode;
  pin_name_ = pinName;
  switch (mode) {
    case Mode::kOpenDrain:
      pinMode(pinName, OUTPUT_OPEN_DRAIN);
      break;
    case Mode::kPushPull:
      pinMode(pinName, OUTPUT);
      break;
  }
  Turn(false);
}

void LED::Turn(const bool value) {
  switch (mode_) {
    case Mode::kOpenDrain:
      digitalWrite(pin_name_, value ? LOW : HIGH);
      break;
    case Mode::kPushPull:
      digitalWrite(pin_name_, value ? HIGH : LOW);
      break;
  }
  state_ = value ? State::kOn : State::kOff;
}

void LED::Toggle() {
  switch (state_) {
    case State::kOff:
      Turn(true);
      break;
    case State::kOn:
      Turn(false);
      break;
  }
}

}  // namespace hortor::info_led