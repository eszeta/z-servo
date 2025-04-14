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
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "debug_print.h"
#include "drivers/DRV8231A/DRV8231A.h"
#include "drivers/MT6701/MT6701.h"
#include "drivers/generic_current/generic_current.h"
#include "info_led.h"
#include "inst/inst.h"
#include "inst/inst_accessor.h"
#include "inst/inst_i2c_transport.h"
#include "servo.h"

static constexpr auto kInfoLedPin = PB1;
static constexpr auto kTargetLoopRateHz = 500;  // 目标帧率500Hz
static constexpr auto kTargetLoopPeriodUs =
    1000000 / kTargetLoopRateHz;  // 目标周期(微秒)

hortor_servo::InfoLED::InfoLED info_led{};

void setup() {
  info_led.Init(kInfoLedPin, hortor_servo::InfoLED::Mode::kOpenDrain);
  info_led.SetInfo(hortor_servo::InfoLED::InfoType::kOk);
}

void loop() {
  static auto last_time = micros() - kTargetLoopPeriodUs;
  const auto current_time = micros();
  const auto dt = current_time - last_time;
  last_time = current_time;

  info_led.Process(dt);
  // 固定帧率控制
  const auto elapsed_time = micros() - last_time;
  if (elapsed_time < kTargetLoopPeriodUs) {
    delayMicroseconds(kTargetLoopPeriodUs - elapsed_time);
  }
}