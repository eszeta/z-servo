// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include <Arduino.h>
#include <Wire.h>
#include <info_led/info_led.h>
#include <math/resolution.h>
#include <protocol/protocol.h>
#include <slave/types.h>
#include <utils/commander.h>
#include <utils/task_scheduler.h>

#include <cstring>

using InfoLED       = hortor::info_led::InfoLED<hortor::info_led::LedMode::kOpenDrain>;
using InfoLEDInfo   = InfoLED::InfoType;
using Commander     = hortor::utils::Commander;
using TaskScheduler = hortor::utils::TaskScheduler<>;
using Error         = hortor::Error;
using hortor::IsFail;

// Slave 编码器分辨率（与 STM32/main.cc 保持一致）
constexpr uint8_t kResolutionBits = 12;
using Resolution                  = hortor::math::Resolution<kResolutionBits>;

// ── 引脚常量 ──────────────────────────────────────────────────────────────────
constexpr auto kInfoLedPin     = PB1;
constexpr auto kEncPinA        = PB6;
constexpr auto kEncPinB        = PB7;
constexpr auto kMainLoopRateHz = 500u;
constexpr auto kLedOnlyRateHz  = 50u;
// 编码器每格步进 1°，转换为 pulse（有据可查：Resolution::kAngleToRaw）
constexpr float kDegPerClick  = 1.0f;
constexpr auto  kStepPerClick = static_cast<int32_t>(kDegPerClick * Resolution::kAngleToRaw);

// ── 全局对象 ──────────────────────────────────────────────────────────────────
HardwareSerial serial(PB4, PB3);
TwoWire        wire(PA8, PA9);

InfoLED       led{};
Commander     commander{};
TaskScheduler scheduler{};

uint8_t target_id = 1;  // 当前控制的 Slave ID（`i` 命令可修改）

// ── 编码器状态（中断共享，volatile） ──────────────────────────────────────────
volatile int32_t enc_count = 0;
volatile bool    enc_dirty = false;

// ── 串口行缓冲 ────────────────────────────────────────────────────────────────
static char    line_buf[64]{};
static uint8_t line_len = 0;

// ── 函数声明 ──────────────────────────────────────────────────────────────────
Error SystemSetup();
Error MainLoopCallback(float dt);
Error LedCallback(float dt);
void  EnterErrorMode(Error error);
void  OnEncA();

/**
 * @brief 构建 DYNAMIXEL WRITE 包并经 I2C 发送给指定 Slave
 *
 * @tparam T    寄存器数据类型（uint8_t / uint16_t / int32_t …）
 * @param id       目标 Slave I2C 地址 / ID
 * @param reg_addr 寄存器起始地址
 * @param value    写入值（按类型大小字节序原样写入）
 */
template <typename T>
void WriteRegI2C(uint8_t id, uint8_t reg_addr, T value) {
  using namespace hortor::protocol;
  InstPacket pkt{};
  pkt.header1            = kHeaderByte;
  pkt.header2            = kHeaderByte;
  pkt.id                 = id;
  pkt.instructionOrError = Instruction::kWriteData;
  pkt.parameter[0]       = reg_addr;
  memcpy(&pkt.parameter[1], &value, sizeof(T));
  pkt.SetParameterSize(1u + static_cast<uint8_t>(sizeof(T)));
  pkt.SetChecksum(pkt.CalculateChecksum());
  wire.beginTransmission(id);
  wire.write(pkt.buffer, pkt.GetBufferSize());
  wire.endTransmission();
}

/**
 * @brief 将物理增益值转换为 raw uint16 并写入对应寄存器
 *
 * 地址和量纲转换均取自 slave::ControlTable 的寄存器定义，不含手写魔法数字。
 *
 * @tparam RegType  slave::ControlTable 中的寄存器结构（需有 kAddress 和 converter_t）
 */
template <typename RegType>
void WriteGain(uint8_t id, float val) {
  const uint16_t raw = RegType::converter_t::template ToRaw<uint16_t>(val);
  WriteRegI2C(id, RegType::kAddress, raw);
}

// cppcheck-suppress unusedFunction
void setup() {
  const auto error = SystemSetup();
  if (IsFail(error)) {
    EnterErrorMode(error);
  }
}

// cppcheck-suppress unusedFunction
void loop() {
  const auto error = scheduler.Tick();
  if (IsFail(error)) {
    EnterErrorMode(error);
  }
}

void EnterErrorMode(Error error) {
  led.ShowErrorCode(static_cast<uint8_t>(error));
  scheduler.ClearTasks();
  scheduler.AddTask(LedCallback, kLedOnlyRateHz);
}

Error SystemSetup() {
  serial.begin(115200);
  wire.begin();
  led.Init(kInfoLedPin);
  led.SetInfo(InfoLEDInfo::kOk);

  // 编码器 A/B 相引脚，A 相上升/下降沿触发正交解码
  pinMode(kEncPinA, INPUT_PULLUP);
  pinMode(kEncPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kEncPinA), OnEncA, CHANGE);

  // p <angle>  目标位置（单位：度，经 Resolution::kAngleToRaw 转换为 pulse）
  commander.add('p', [](int argc, char** argv) {
    if (argc < 1)
      return;
    const float   angle = atof(argv[0]);
    const int32_t pos   = static_cast<int32_t>(angle * Resolution::kAngleToRaw);
    WriteRegI2C(target_id, 0x98, pos);
  });

  // t <0|1>  力矩使能（TorqueEnable 0x80）
  commander.add('t', [](int argc, char** argv) {
    if (argc < 1)
      return;
    const uint8_t en = static_cast<uint8_t>(atoi(argv[0]));
    WriteRegI2C(target_id, 0x80, en);
  });

  // i <id>  切换目标 Slave ID
  commander.add('i', [](int argc, char** argv) {
    if (argc < 1)
      return;
    target_id = static_cast<uint8_t>(atoi(argv[0]));
  });

  // g <sub> <value>  PID / 前馈增益设置
  // 子标识: pp pi pd vp vi f1 f2
  // 地址与量纲转换来自 slave::ControlTable 寄存器定义，不含手写魔法数字
  commander.add('g', [](int argc, char** argv) {
    namespace CT = hortor::slave::ControlTable;
    if (argc < 2)
      return;
    const float val = atof(argv[1]);
    const char* sub = argv[0];
    if (strcmp(sub, "pp") == 0)
      WriteGain<CT::kPositionPGain>(target_id, val);
    else if (strcmp(sub, "pi") == 0)
      WriteGain<CT::kPositionIGain>(target_id, val);
    else if (strcmp(sub, "pd") == 0)
      WriteGain<CT::kPositionDGain>(target_id, val);
    else if (strcmp(sub, "vp") == 0)
      WriteGain<CT::kVelocityPGain>(target_id, val);
    else if (strcmp(sub, "vi") == 0)
      WriteGain<CT::kVelocityIGain>(target_id, val);
    else if (strcmp(sub, "f1") == 0)
      WriteGain<CT::kFeedforward1stGain>(target_id, val);
    else if (strcmp(sub, "f2") == 0)
      WriteGain<CT::kFeedforward2ndGain>(target_id, val);
  });

  CHECK(scheduler.AddTask(MainLoopCallback, kMainLoopRateHz));
  return Error::kOk;
}

/**
 * @brief 编码器 A 相中断：正交解码，A==B 时正向计数，否则反向
 */
void OnEncA() {
  enc_count += (digitalRead(kEncPinA) == digitalRead(kEncPinB)) ? 1 : -1;
  enc_dirty = true;
}

/**
 * @brief 主循环（500Hz）：串口 Commander 解析 + 编码器本地位置控制
 */
Error MainLoopCallback(float dt) {
  led.Process(dt);

  // --- 串口：逐字符接收，按行触发 Commander ---
  while (serial.available() > 0) {
    const char c = static_cast<char>(serial.read());
    if (c == '\n' || c == '\r') {
      if (line_len > 0) {
        line_buf[line_len] = '\0';
        commander.run(line_buf);
        line_len = 0;
      }
    } else if (line_len < sizeof(line_buf) - 1) {
      line_buf[line_len++] = c;
    }
  }

  // --- 编码器：有新增量则更新 GoalPosition ---
  noInterrupts();
  const bool    dirty = enc_dirty;
  const int32_t cnt   = enc_count;
  enc_dirty           = false;
  interrupts();

  if (dirty) {
    const int32_t goal = constrain(cnt * kStepPerClick, 0, 4095);
    WriteRegI2C(target_id, 0x98, goal);
  }

  return Error::kOk;
}

Error LedCallback(float dt) {
  led.Process(dt);
  return Error::kOk;
}
