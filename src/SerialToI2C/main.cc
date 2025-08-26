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
#include <Wire.h>
#include <core/types.h>
#include <debug_print.h>
#include <info_led.h>
#include <inst/inst_handler_serial.h>
#include <math/math.h>

static constexpr auto kInfoLedPin = PB1;
static constexpr auto kTargetLoopRateHz = 500;  // 目标帧率500Hz
static constexpr auto kTargetLoopPeriodUs =
    1000000 / kTargetLoopRateHz;  // 目标周期(微秒)

hortor_servo::InfoLED::InfoLED info_led{};
HardwareSerial serial(PB4, PB3);
TwoWire wire(PA8, PA9);
hortor_servo::InstHandlerSerial inst_handler{};

hortor_servo::Error Execute(uint8_t *data, size_t size);
// cppcheck-suppress unusedFunction
void setup() {
  info_led.Init(kInfoLedPin, hortor_servo::InfoLED::Mode::kOpenDrain);
  info_led.SetInfo(hortor_servo::InfoLED::InfoType::kOk);
  serial.begin(115200);
  wire.begin();
  inst_handler.Init(&serial);
  inst_handler.SetExecute(&Execute);
}

// cppcheck-suppress unusedFunction
void loop() {
  static auto last_time = micros() - kTargetLoopPeriodUs;
  const auto current_time = micros();
  const auto dt = (current_time - last_time) * hortor_servo::kMicroToSec;
  last_time = current_time;

  info_led.Process(dt);
  inst_handler.Process(dt);
  // 固定帧率控制
  const auto elapsed_time = micros() - last_time;
  if (elapsed_time < kTargetLoopPeriodUs) {
    delayMicroseconds(kTargetLoopPeriodUs - elapsed_time);
  }
}

hortor_servo::Error Execute(uint8_t *data, size_t size) {
  wire.beginTransmission(0x00);
  wire.write(data, size);
  wire.endTransmission(false);
  wire.requestFrom(0x00, 128, 1);
  while (wire.available()) {
    uint8_t data = wire.read();
    serial.write(data);
  }
  return hortor_servo::Error::kOk;
}