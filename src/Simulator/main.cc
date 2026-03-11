// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include <Arduino.h>

#include "error.h"
#include "servo/servo.h"
#include "servo/types.h"
#include "simulator/current.h"
#include "simulator/encoder.h"
#include "simulator/motor.h"
#include "simulator/plant.h"
#include "utils/task_scheduler.h"

constexpr uint8_t kResolutionBits = 12;

using Motor          = hortor::simulator::SimulatorMotor;
using Encoder        = hortor::simulator::SimulatorEncoder;
using Current        = hortor::simulator::SimulatorCurrent;
using SimulatorPlant = hortor::simulator::SimulatorPlant;
using Servo          = hortor::servo::Servo<Motor, Encoder, Current, kResolutionBits>;
using TaskScheduler  = hortor::utils::TaskScheduler<>;
using EncoderConfig  = Encoder::Config;
using Reverse        = hortor::servo::Reverse;
using Error          = hortor::Error;

Error SystemSetup();
Error SystemLoop();

constexpr uint32_t kMainLoopRateHz = 1000;

Motor          motor{};
Encoder        encoder{};
Current        current{};
Servo          servo{};
SimulatorPlant plant{};
TaskScheduler  scheduler{};

static hortor::Error MainLoopCallback(float dt) {
  CHECK(plant.Process(dt));
  return servo.Process(dt);
}

void setup() {
  SystemSetup();
}

void loop() {
  SystemLoop();
}

Error SystemSetup() {
  EncoderConfig encoder_config{};
  encoder_config.homing_offset = 0;
  encoder_config.reverse       = Reverse::kNormal;

  encoder.Init(encoder_config);
  current.Init();
  motor.Init();

  servo.set_motor(&motor);
  servo.set_encoder(&encoder);
  servo.set_current_sensor(&current);

  plant.set_motor(&motor);
  plant.set_encoder(&encoder);
  plant.set_current_sensor(&current);

  servo.Init();

  scheduler.AddTask(MainLoopCallback, kMainLoopRateHz);
  return Error::kOk;
}

Error SystemLoop() {
  return scheduler.Tick();
}

#if !defined(PIO_UNIT_TESTING)
// 非测试构建时作为入口；测试构建时由 test 的 main 覆盖
__attribute__((weak)) int main() {
  setup();
  for (;;) {
    loop();
  }
  return 0;
}
#endif