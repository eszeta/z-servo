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
#include <core/servo.h>
#include <debug_print.h>
#include <drivers/DRV8231A/DRV8231A.h>
#include <drivers/MT6701/MT6701.h>
#include <drivers/generic_current/generic_current.h>
#include <info_led.h>
#include <inst/inst.h>
#include <inst/inst_accessor.h>
#include <inst/inst_i2c_adapter.h>
#include <math/math.h>

static constexpr auto kInfoLedPin = PA12;
// 目标帧率500Hz
static constexpr auto kTargetLoopRateHz = 500;
// 目标周期(微秒)
static constexpr auto kTargetLoopPeriodUs = 1000000 / kTargetLoopRateHz;

HardwareSerial serial_debug(PB4, PB3);
TwoWire wire_sensor(PA8, PA9);
TwoWire wire_inst(PB7, PA15);

hortor_servo::InfoLED::InfoLED info_led{};
hortor_servo::InstI2cAdapter inst_transport{};
hortor_servo::InstAccessor inst_accessor{};
hortor_servo::Inst inst{};
hortor_servo::DRV8231A::DRV8231A motor_driver{};
hortor_servo::MT6701::MT6701 angle_sensor{};
hortor_servo::generic_current::GenericCurrent current_sensor{};
hortor_servo::Servo servo{};

void receiveEvent(int howMany) { inst_transport.OnReceive(howMany); }
void requestEvent() { inst_transport.OnRequest(); }

void setup() {
  // 设置PWM频率为10kHz
  analogWriteFrequency(10 * 1000);

  serial_debug.begin(921600);
  hortor_servo::DebugEnable(&serial_debug);
  hortor_servo::DebugPrintln(F("setup"));

  info_led.Init(kInfoLedPin, hortor_servo::InfoLED::Mode::kOpenDrain);

  motor_driver.Init(PA0, PA2);
  wire_sensor.begin();
  angle_sensor.Init(&wire_sensor);
  current_sensor.Init(PA3, 1000, 1500);

  servo.LinkDriver(&motor_driver);
  servo.LinkAngleSensor(&angle_sensor);
  servo.LinkCurrentSense(&current_sensor);
  servo.Init();

  inst_accessor.Init();
  inst_transport.Init(&wire_inst);

  inst.LinkAccessor(&inst_accessor);
  inst.LinkTransport(&inst_transport);
  inst.LinkServo(&servo);
  inst.Init();

  inst_accessor.SetMode(hortor_servo::ServoMode::kVelocity);
  inst_accessor.SetTargetVelocity(1000.0f);
  inst_accessor.SetVelPidKp(0.0f);
  inst_accessor.SetVelPidKi(0.0f);

  inst.Refresh();

  info_led.SetInfo(hortor_servo::InfoLED::InfoType::kOk);
}

void loop() {
  static auto last_time = micros() - kTargetLoopPeriodUs;
  const auto current_time = micros();
  const auto dt = (current_time - last_time) * hortor_servo::kMicroToSec;
  last_time = current_time;

  info_led.Process(dt);
  inst.Process(dt);
  servo.Process(dt);

  hortor_servo::DebugPrint(F(">dt:"));
  hortor_servo::DebugPrintln(dt);
  hortor_servo::DebugPrint(F(">pwm:"));
  hortor_servo::DebugPrintln(servo.GetPresentLoad());

  auto present_velocity = servo.GetPresentVelocity();
  hortor_servo::DebugPrint(F(">velocity:"));
  hortor_servo::DebugPrintln(present_velocity);

  auto present_position = servo.GetPresentPosition();
  hortor_servo::DebugPrint(F(">position:"));
  hortor_servo::DebugPrintln(present_position);

  // 固定帧率控制
  const auto elapsed_time = micros() - last_time;
  if (elapsed_time < kTargetLoopPeriodUs) {
    delayMicroseconds(kTargetLoopPeriodUs - elapsed_time);
  }
}