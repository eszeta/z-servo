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
#include <drivers/current_mirror/current_mirror.h>
#include <drivers/drv8231a/drv8231a.h>
#include <drivers/mt6701/mt6701.h>
#include <info_led/info_led.h>
#include <math/math.h>
#include <servo/servo.h>
#include <servo_slave/slave.h>
#include <utils/debug_print.h>
#include <utils/monitor.h>
#include <utils/task_scheduler.h>

using hortor::Error;
using hortor::drivers::current_mirror::CurrentMirror;
using hortor::drivers::DRV8231A::DRV8231A;
using hortor::drivers::MT6701::BusType;
using hortor::drivers::MT6701::MT6701;
using hortor::info_led::InfoLED;
using hortor::info_led::Mode;
using hortor::servo::Servo;
using hortor::servo_slave::Slave;
using hortor::utils::DebugEnable;
using hortor::utils::Monitor;
using hortor::utils::TaskScheduler;

// 信息灯引脚
constexpr auto kInfoLedPin = PA12;
// 主控制循环频率 1000Hz
constexpr auto kMainLoopRateHz = 1000;
// 调试输出频率 10Hz
constexpr auto kDebugOutputRateHz = 10;

constexpr auto kResolutionBits = 12;

// 伺服电机类型别名，简化复杂的模板类型
using ServoType =
    Servo<DRV8231A, MT6701<BusType::kI2C>, CurrentMirror, kResolutionBits>;

HardwareSerial serial_debug(PB4, PB3);
TwoWire wire_sensor(PA8, PA9);
TwoWire wire_slave(PB7, PA15);

InfoLED info_led{};
Slave slave{};
ServoType servo{};
Monitor<ServoType> monitor{};

// 集中式任务调度器（固定容量，避免动态分配）
TaskScheduler scheduler{};

// 前向声明
Error MainLoopCallback(float dt);
Error DebugOutputCallback(float dt);

// cppcheck-suppress unusedFunction
void setup() {
  // 设置PWM频率为10kHz
  analogWriteFrequency(10 * 1000);

  serial_debug.begin(115200);
  DebugEnable(&serial_debug);

  wire_sensor.begin();
  wire_slave.begin();

  servo.GetMotor().Init({
      .pin_in1 = PA0,
      .pin_in2 = PA2,
      .pin_nfault = 0,              // 如果硬件连接了 nFAULT，填入引脚号
      .slow_decay_threshold = 0.3f  // 低于 30% 使用慢速衰减
  });
  servo.GetSensor().Init(&wire_sensor);
  servo.GetCurrentSense().Init({.pin_adc = PA3,
                                .ripropi_ohms = 1000.0f,
                                .scaling_factor = 1500.0f,
                                .adc_resolution_bits = 12,
                                .adc_vref_volts = 3.3f,
                                .calibration_samples = 50});
  servo.Init();

  slave.GetRegMap().Init();
  slave.GetPortHandler().Init(&wire_slave);
  // slave.LinkServo(&servo);
  slave.Init();

  monitor.LinkPort(&serial_debug);
  monitor.LinkMotor(&servo);

  info_led.Init(kInfoLedPin, Mode::kOpenDrain);
  info_led.SetInfo(InfoLED::InfoType::kOk);

  // 注册任务：集中式调度
  scheduler.Register(MainLoopCallback, kMainLoopRateHz);        // 500Hz 主控制
  scheduler.Register(DebugOutputCallback, kDebugOutputRateHz);  // 10Hz 调试输出
}

/**
 * @brief 主控制循环回调函数（500Hz）
 * @param dt 距离上次调用的时间间隔（秒）
 */
Error MainLoopCallback(float dt) {
  info_led.Process(dt);
  CHECK(slave.Process(dt));
  CHECK(servo.Process(dt));
  return Error::kOk;
}

/**
 * @brief 调试输出回调函数（10Hz）
 * @param dt 距离上次调用的时间间隔（秒）
 */
Error DebugOutputCallback(float dt) {
  CHECK(monitor.Process(dt));
  return Error::kOk;
}

// cppcheck-suppress unusedFunction
void loop() {
  // 统一调度与睡眠（按最小周期）
  scheduler.Tick();
}