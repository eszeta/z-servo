// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <base/servo.h>
#include <drivers/current_mirror/current_mirror.h>
#include <drivers/drv8231a/drv8231a.h>
#include <drivers/lsm6dsow/lsm6dsow.h>
#include <drivers/mt6701/mt6701.h>
#include <info_led/info_led.h>
#include <math/math.h>
#include <servo_slave/slave.h>
#include <utils/commander.h>
#include <utils/debug_print.h>
#include <utils/monitor.h>
#include <utils/task_scheduler.h>

using hortor::Error;
using hortor::utils::Commander;
using hortor::utils::DebugEnable;
using hortor::utils::Monitor;
using hortor::utils::TaskScheduler;
using InfoLEDType = hortor::info_led::InfoLED;
using InfoLEDMode = hortor::info_led::Mode;
using InfoLEDInfoType = hortor::info_led::InfoLED::InfoType;
using PortHandlerType = hortor::protocol::I2cPortHandler;
using ReverseType = hortor::servo::Reverse;
using SlaveRegMapType = hortor::servo_slave::RegMap;
using BusType = hortor::drivers::MT6701::BusType;
using EncoderType = hortor::drivers::MT6701::MT6701<BusType::kI2C>;
using EncoderConfig = EncoderType::Config;
using CurrentType = hortor::drivers::current_mirror::CurrentMirror;
using MotorType = hortor::drivers::DRV8231A::DRV8231A;

// 信息灯引脚
constexpr auto kInfoLedPin = PA12;
// 主控制循环频率 1000Hz
constexpr auto kMainLoopRateHz = 1000;
// 调试输出频率 10Hz
constexpr auto kDebugOutputRateHz = 10;

constexpr auto kResolutionBits = 12;

// 伺服电机类型别名，简化复杂的模板类型
using ServoType =
    hortor::servo::Servo<MotorType, EncoderType, CurrentType, kResolutionBits>;

using SlaveType = hortor::servo_slave::Slave<ServoType>;

HardwareSerial serial_debug(PB4, PB3);
TwoWire wire_sensor(PA8, PA9);
TwoWire wire_slave(PB7, PA15);

InfoLEDType info_led{};
SlaveRegMapType regmap{};
PortHandlerType port_handler{};
SlaveType slave{};

MotorType motor_driver{};
EncoderType encoder{};
CurrentType current_sensor{};
ServoType servo{};

Monitor<ServoType> monitor{};
Commander commander{};
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

  motor_driver.Init({
      .pin_in1 = PA0,
      .pin_in2 = PA2,
      .pin_nfault = 0,               // 如果硬件连接了 nFAULT，填入引脚号
      .slow_decay_threshold = 0.3f,  // 低于 30% 使用慢速衰减
  });

  EncoderConfig encoder_config{};
  encoder_config.homing_offset = 0;
  encoder_config.reverse = ReverseType::kNormal;
  encoder_config.wire = &wire_sensor;

  encoder.Init(encoder_config);
  current_sensor.Init({.pin_adc = PA3,
                       .ripropi_ohms = 1000.0f,
                       .scaling_factor = 1500.0f,
                       .adc_resolution_bits = 12,
                       .adc_vref_volts = 3.3f,
                       .calibration_samples = 50});

  servo.set_motor(&motor_driver);
  servo.set_encoder(&encoder);
  servo.set_current_sensor(&current_sensor);
  servo.Init();

  regmap.Init();
  port_handler.Init(&wire_slave);
  slave.set_regmap(&regmap);
  slave.set_port_handler(&port_handler);
  slave.set_servo(&servo);
  slave.Init();

  monitor.set_port(&serial_debug);
  monitor.set_servo(&servo);

  info_led.Init(kInfoLedPin, InfoLEDMode::kOpenDrain);
  info_led.SetInfo(InfoLEDInfoType::kOk);

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