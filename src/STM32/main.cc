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
#include <utils/debug_print.h>
#include <utils/task_scheduler.h>
// #include <protocol/i2c_port_handler.h>
// #include <protocol/slave.h>
// #include <servo_slave/servo_accessor.h>
// #include <servo_slave/types.h>

using hortor::Error;
using hortor::drivers::current_mirror::CurrentMirror;
using hortor::drivers::DRV8231A::DRV8231A;
using hortor::drivers::MT6701::MT6701;
using hortor::info_led::InfoLED;
using hortor::info_led::InfoType;
using hortor::info_led::Mode;
using hortor::servo::Servo;
using hortor::utils::DebugEnable;
using hortor::utils::DebugPrint;
using hortor::utils::DebugPrintln;
using hortor::utils::TaskScheduler;

// 信息灯引脚
constexpr auto kInfoLedPin = PA12;
// 主控制循环频率 500Hz
constexpr auto kMainLoopRateHz = 500;
// 调试输出频率 10Hz
constexpr auto kDebugOutputRateHz = 10;

HardwareSerial serial_debug(PB4, PB3);
TwoWire wire_sensor(PA8, PA9);
// TwoWire wire_inst(PB7, PA15);

InfoLED info_led{};
// InstI2cPortHandler inst_port{};
// ServoAccessor inst_accessor{};
// Slave inst{};
Servo<DRV8231A, MT6701, CurrentMirror, 12> servo{};

// 集中式任务调度器（固定容量，避免动态分配）
TaskScheduler scheduler{};

// 前向声明
Error MainLoopCallback(float dt);
Error DebugOutputCallback(float dt);

// cppcheck-suppress unusedFunction
// void receiveEvent(int howMany) { inst_port.OnReceive(howMany); }
// // cppcheck-suppress unusedFunction
// void requestEvent() { inst_port.OnRequest(); }

// cppcheck-suppress unusedFunction
void setup() {
  // 设置PWM频率为10kHz
  analogWriteFrequency(10 * 1000);

  serial_debug.begin(115200);
  DebugEnable(&serial_debug);

  wire_sensor.begin();

  const DRV8231A::Config motor_config = {
      .pin_in1 = PA0,
      .pin_in2 = PA2,
      .pin_nfault = 0,              // 如果硬件连接了 nFAULT，填入引脚号
      .slow_decay_threshold = 0.3f  // 低于 30% 使用慢速衰减
  };
  const CurrentMirror::Config current_mirror_config = {
      .pin_adc = PA3,
      .ripropi_ohms = 1000.0f,
      .scaling_factor = 1500.0f,
      .adc_resolution_bits = 12,
      .adc_vref_volts = 3.3f,
      .calibration_samples = 50};
  servo.GetMotor().Init(motor_config);
  servo.GetSensor().Init(&wire_sensor);
  servo.GetCurrentSense().Init(current_mirror_config);
  servo.Init();

  // inst_accessor.Init();
  // inst_port.Init(&wire_inst);

  // inst.LinkAccessor(&inst_accessor);
  // inst.LinkPort(&inst_port);
  // inst.LinkServo(&servo);
  // inst.Init();

  // inst_accessor.SetMode(OperatingMode::kVelocity);
  // inst_accessor.SetGoalVelocity(1000.0f);
  // inst_accessor.SetVelPidKp(0.0f);
  // inst_accessor.SetVelPidKi(0.0f);

  // inst_accessor.SetBaudrate(100);

  // inst.LoadEepromConfig();
  // inst.LoadRamConfig();

  info_led.Init(kInfoLedPin, Mode::kOpenDrain);
  info_led.SetInfo(InfoType::kOk);

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
  // inst.Process(dt);
  CHECK(servo.Process(dt));
  return Error::kOk;
}

/**
 * @brief 调试输出回调函数（10Hz）
 * @param dt 距离上次调用的时间间隔（秒）
 */
Error DebugOutputCallback(float dt) {
  // DebugPrint(F(">dt:"));
  // DebugPrintln(dt);
  // DebugPrint(F(">pwm:"));
  // DebugPrintln(servo.GetPresentPwm());

  // auto present_velocity = servo.GetPresentVelocity();
  // DebugPrint(F(">velocity:"));
  // DebugPrintln(present_velocity);

  // auto present_position = servo.GetPresentPosition();
  // DebugPrint(F(">position:"));
  // DebugPrintln(present_position);
  return Error::kOk;
}

// cppcheck-suppress unusedFunction
void loop() {
  // 统一调度与睡眠（按最小周期）
  scheduler.Tick();
}