// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include <Arduino.h>
#include <Wire.h>
#include <base/servo.h>
#include <drivers/current_mirror/current_mirror.h>
#include <drivers/drv8231a/drv8231a.h>
#include <drivers/mt6701/mt6701.h>
#include <info_led/info_led.h>
#include <protocol/channel.h>
#include <protocol/transport_i2c.h>
#include <slave/slave.h>
#include <utils/commander.h>
#include <utils/debug_print.h>
#include <utils/monitor.h>
#include <utils/task_scheduler.h>

constexpr auto kResolutionBits = 12;

// 总线与协议
using Transport = hortor::protocol::TransportI2C;
using Channel   = hortor::protocol::ProtocolChannel<Transport>;

// 驱动组件
using Motor   = hortor::drivers::DRV8231A::Motor;
using Encoder = hortor::drivers::MT6701::Encoder<hortor::BusType::kI2C>;
using Current = hortor::drivers::CurrentMirror::Current;

// 伺服与从机
using Servo  = hortor::servo::Servo<Motor, Encoder, Current, kResolutionBits>;
using Slave  = hortor::slave::Slave<Servo, Channel>;
using Regmap = hortor::slave::Regmap;

// 信息灯
using InfoLED =
    hortor::info_led::InfoLED<hortor::info_led::LedMode::kOpenDrain>;
using InfoLEDInfo = InfoLED::InfoType;

// 工具
using Error = hortor::Error;
using hortor::IsFail;
using Monitor       = hortor::utils::Monitor<Servo>;
using Commander     = hortor::utils::Commander;
using TaskScheduler = hortor::utils::TaskScheduler<>;
using hortor::utils::DebugEnable;

// 编码器配置
using EncoderConfig = Encoder::Config;
using Reverse       = hortor::servo::Reverse;

constexpr auto kInfoLedPin        = PA12;
constexpr auto kMainLoopRateHz    = 1000;
constexpr auto kDebugOutputRateHz = 10;

HardwareSerial serial_debug(PB4, PB3);
TwoWire        wire_sensor(PA8, PA9);
TwoWire        wire_slave(PB7, PA15);

InfoLED       led{};
Regmap        regmap{};
Channel       channel{};
Slave         slave{};
Motor         motor_driver{};
Encoder       encoder{};
Current       current_sensor{};
Servo         servo{};
Monitor       monitor{};
Commander     commander{};
TaskScheduler scheduler{};

// 系统初始化
Error SystemSetup();
// 主控制循环回调函数
Error MainLoopCallback(float dt);
// 调试输出回调函数
Error DebugOutputCallback(float dt);
// I2C 从机回调
void OnI2cReceive(int n);
void OnI2cRequest();

// cppcheck-suppress unusedFunction
void setup() {
  const auto error = SystemSetup();
  if (IsFail(error)) {
    led.ShowErrorCode(static_cast<uint8_t>(error));
    scheduler.ClearTasks();
    scheduler.AddTask(DebugOutputCallback,
                      kDebugOutputRateHz);  // 10Hz 调试输出
  }
}

// cppcheck-suppress unusedFunction
void loop() {
  // 统一调度与睡眠（按最小周期）
  // 运行时错误不 Panic，而是点亮故障灯并继续运行（允许通信恢复）
  if (IsFail(scheduler.Tick())) {
    led.SetInfo(InfoLEDInfo::kError);
  }
}

Error SystemSetup() {
  // 设置PWM频率为10kHz
  analogWriteFrequency(10 * 1000);

  serial_debug.begin(115200);
  DebugEnable(&serial_debug);
  hortor::utils::DebugPrintln(F("setup"));

  led.Init(kInfoLedPin);
  led.SetInfo(InfoLEDInfo::kOk);

  wire_sensor.begin();

  CHECK(motor_driver.Init({
      .pin_in1              = PA0,
      .pin_in2              = PA2,
      .pin_nfault           = 0,     // 如果硬件连接了 nFAULT，填入引脚号
      .slow_decay_threshold = 0.3f,  // 低于 30% 使用慢速衰减
  }));

  EncoderConfig encoder_config{};
  encoder_config.homing_offset = 0;
  encoder_config.reverse       = Reverse::kNormal;
  encoder_config.wire          = &wire_sensor;

  CHECK(encoder.Init(encoder_config));
  CHECK(current_sensor.Init({.pin_adc             = PA3,
                             .ripropi_ohms        = 1000.0f,
                             .scaling_factor      = 1500.0f,
                             .adc_resolution_bits = 12,
                             .adc_vref_volts      = 3.3f,
                             .calibration_samples = 50}));

  servo.set_motor(&motor_driver);
  servo.set_encoder(&encoder);
  servo.set_current_sensor(&current_sensor);
  CHECK(servo.Init());
  CHECK(regmap.Init());
  CHECK(channel.Init(&wire_slave));
  slave.set_regmap(&regmap);
  slave.set_channel(&channel);
  slave.set_servo(&servo);
  CHECK(slave.Init());

  wire_slave.begin(slave.id());
  wire_slave.onReceive(OnI2cReceive);
  wire_slave.onRequest(OnI2cRequest);

  monitor.set_port(&serial_debug);
  monitor.set_servo(&servo);

  // 注册任务：集中式调度
  scheduler.AddTask(MainLoopCallback, kMainLoopRateHz);        // 500Hz 主控制
  scheduler.AddTask(DebugOutputCallback, kDebugOutputRateHz);  // 10Hz 调试输出
  return Error::kOk;
}

/**
 * @brief 主控制循环回调函数（500Hz）
 * @param dt 距离上次调用的时间间隔（秒）
 */
Error MainLoopCallback(float dt) {
  CHECK(slave.Process(dt));
  CHECK(servo.Process(dt));
  return Error::kOk;
}

/**
 * @brief 调试输出回调函数（10Hz）
 * @param dt 距离上次调用的时间间隔（秒）
 */
Error DebugOutputCallback(float dt) {
  led.Process(dt);
  CHECK(monitor.Process(dt));
  return Error::kOk;
}

void OnI2cReceive(int n) {
  channel.transport()->OnReceive(n);
}

void OnI2cRequest() {
  channel.transport()->OnRequest();
}
