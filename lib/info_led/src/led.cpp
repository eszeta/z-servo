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
#include "led.h"
namespace hortor_servo {
namespace InfoLED {

void LED::Init(const uint32_t pin, const Mode mode) { Init(digitalPinToPinName(pin), mode); }

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
  switch (this->mode_) {
    case Mode::kOpenDrain:
      digitalWrite(this->pin_name_, value ? LOW : HIGH);
      break;
    case Mode::kPushPull:
      digitalWrite(this->pin_name_, value ? HIGH : LOW);
      break;
  }
  state_ = value ? State::kOn : State::kOff;
}

void LED::Toggle() {
  switch (this->state_) {
    case State::kOff:
      Turn(true);
      break;
    case State::kOn:
      Turn(false);
      break;
  }
}
}  // namespace InfoLED
}  // namespace hortor_servo