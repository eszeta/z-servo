// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include <Arduino.h>
#include <Wire.h>
#include <drivers/current_mirror/current_mirror.h>
#include <drivers/drv8231a/drv8231a.h>
#include <drivers/mt6701/mt6701.h>
#include <info_led/info_led.h>
#include <protocol/channel.h>
#include <protocol/transport_i2c.h>
#include <servo/servo.h>
#include <servo/types.h>
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
using Motor      = hortor::drivers::DRV8231A::Motor;
using EncoderBus = hortor::drivers::MT6701::BusType;
using Encoder    = hortor::drivers::MT6701::Encoder<EncoderBus::kI2C>;
using Current    = hortor::drivers::CurrentMirror::Current;

// 伺服与从机
using Servo  = hortor::servo::Servo<Motor, Encoder, Current, kResolutionBits>;
using Slave  = hortor::slave::Slave<Servo, Channel>;
using Regmap = hortor::slave::Regmap;

// 信息灯
using InfoLED     = hortor::info_led::InfoLED<hortor::info_led::LedMode::kOpenDrain>;
using InfoLEDInfo = InfoLED::InfoType;

// 工具
using Error = hortor::Error;
using hortor::IsFail;
using Monitor       = hortor::utils::Monitor<Servo>;
using TaskScheduler = hortor::utils::TaskScheduler<>;
using hortor::utils::DebugEnable;

// 编码器配置
using EncoderConfig = Encoder::Config;
using Reverse       = hortor::servo::Reverse;

// 引脚定义
constexpr auto kPinInfoLed    = PA12;
constexpr auto kPinMotorIn1   = PA0;
constexpr auto kPinMotorIn2   = PA2;
constexpr auto kPinCurrentAdc = PA3;
constexpr auto kPinSensorSda  = PA8;
constexpr auto kPinSensorScl  = PA9;
constexpr auto kPinSlaveSda   = PB7;
constexpr auto kPinSlaveScl   = PA15;
constexpr auto kPinDebugTx    = PB4;
constexpr auto kPinDebugRx    = PB3;

constexpr auto kMainLoopRateHz    = 1000;
constexpr auto kDebugOutputRateHz = 10;

HardwareSerial serial_debug(kPinDebugTx, kPinDebugRx);
TwoWire        wire_sensor(kPinSensorSda, kPinSensorScl);
TwoWire        wire_slave(kPinSlaveSda, kPinSlaveScl);

InfoLED       led{};
Regmap        regmap{};
Channel       channel{};
Slave         slave{};
Motor         motor{};
Encoder       encoder{};
Current       current{};
Servo         servo{};
Monitor       monitor{};
TaskScheduler scheduler{};

// 系统初始化
Error SystemSetup();
// 系统循环
Error SystemLoop();
// 主控制循环回调函数
Error MainLoopCallback(float dt);
// 调试输出回调函数
Error DebugOutputCallback(float dt);
// I2C 从机回调
void OnI2cReceive(int n);
void OnI2cRequest();
// 进入错误模式
void EnterErrorMode(Error error);

// cppcheck-suppress unusedFunction
void setup() {
  const auto error = SystemSetup();
  if (IsFail(error)) {
    EnterErrorMode(error);
  }
}

// cppcheck-suppress unusedFunction
void loop() {
  const auto error = SystemLoop();
  if (IsFail(error)) {
    EnterErrorMode(error);
  }
}

Error SystemSetup() {
  // 设置PWM频率为10kHz
  analogWriteFrequency(10 * 1000);

  serial_debug.begin(115200);
  DebugEnable(&serial_debug);
  hortor::utils::DebugPrintln(F("setup"));

  led.Init(kPinInfoLed);
  led.SetInfo(InfoLEDInfo::kOk);

  wire_sensor.begin();

  CHECK(motor.Init({
      .pin_in1              = kPinMotorIn1,
      .pin_in2              = kPinMotorIn2,
      .pin_nfault           = 0,     // 如果硬件连接了 nFAULT，填入引脚号
      .slow_decay_threshold = 0.3f,  // 低于 30% 使用慢速衰减
  }));

  EncoderConfig encoder_config{};
  encoder_config.homing_offset = 0;
  encoder_config.reverse       = Reverse::kNormal;
  encoder_config.wire          = &wire_sensor;

  CHECK(encoder.Init(encoder_config));
  CHECK(current.Init({.pin_adc             = kPinCurrentAdc,
                      .ripropi_ohms        = 1000.0f,
                      .scaling_factor      = 1500.0f,
                      .adc_resolution_bits = 12,
                      .adc_vref_volts      = 3.3f,
                      .calibration_samples = 50}));

  servo.set_motor(&motor);
  servo.set_encoder(&encoder);
  servo.set_current_sensor(&current);
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

Error SystemLoop() {
  return scheduler.Tick();
}

// 进入错误模式
void EnterErrorMode(Error error) {
  led.ShowErrorCode(static_cast<uint8_t>(error));
  scheduler.ClearTasks();
  scheduler.AddTask(DebugOutputCallback, kDebugOutputRateHz);  // 10Hz 调试输出
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
