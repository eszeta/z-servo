// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include <functional>

#include "regmap_i2c_bus.h"
#include "servo/types.h"
#include "types.h"

namespace hortor::drivers::MT6701 {

/**
 * @brief MT6701控制器类（CRTP模式）
 * @note 该类负责与MT6701传感器进行通信，并提供读取角度值和配置传感器的功能
 *
 * Controller类封装了MT6701磁性角度传感器的所有操作，包括：
 * - 角度读取（原始值、角度值、弧度值）
 * - 传感器配置（工作模式、分辨率、零位等）
 * - 输出设置（模拟输出、PWM输出）
 *
 * 通过 CRTP 继承自 RegMapI2CBus，实现编译期静态多态。
 */
// todo：除了ReadRaw,其它函数都没测试过
template <typename BusImpl>
class RegMap : public BusImpl {
 public:
  /**
   * @brief 读取原始位置和状态值
   * @param angle_raw 原始角度指针 [0...16383]
   * @param field_status 磁场状态指针
   * @param button_pushed 按钮状态指针，如果按钮被按下则为true，否则为false
   * @param track_loss 跟踪丢失状态指针，如果检测到丢失则为true，否则为false
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 从MT6701传感器读取完整的状态信息，包括原始角度值和各种状态标志。
   * 参数可以为nullptr，表示不需要该信息。
   * 注意：I2C模式下只支持角度读取，其他状态信息不可用。
   */
  Error ReadRaw(uint16_t& angle_raw,
                Status& field_status,
                bool& button_pushed,
                bool& track_loss) {
    // 直接调用 BusImpl 的 ReadRaw 实现
    return BusImpl::ReadRaw(angle_raw, field_status, button_pushed, track_loss);
  }

  /**
   * @brief 读取磁场状态
   * @return 磁场状态枚举值
   *
   * 读取当前磁场状态，用于诊断磁铁位置问题。
   * 返回值指示磁场是否正常、过强或过弱。
   */
  Status ReadFieldStatus() {
    Status status = Status::kFieldError;
    uint16_t angle_raw = 0;
    bool button_pushed = false;
    bool track_loss = false;
    ReadRaw(angle_raw, status, button_pushed, track_loss);
    (void)angle_raw;
    (void)button_pushed;
    (void)track_loss;
    return status;
  }

  /**
   * @brief 设置UVW模式
   * @param pairs 极对数 [1...16]
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 配置MT6701传感器工作在UVW模式，适用于无刷电机控制。
   * 极对数参数应与电机的极对数匹配。
   */
  Error WriteUvmMode(const uint8_t pairs) {
    CHECK(WriteUVWPolePair(pairs));
    CHECK(WriteMode(Mode::kUVW));
    return Error::kOk;
  }

  /**
   * @brief 设置ABZ模式
   * @param pulses_per_round 每圈脉冲数 [1...1024]
   * @param z_pulse_width Z脉冲宽度，定义Z信号的脉冲宽度
   * @param hysteresis 迟滞设置，用于抑制抖动
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 配置MT6701传感器工作在ABZ模式，提供增量式编码器输出。
   * 可以设置分辨率、Z信号宽度和迟滞参数。
   */
  Error WriteAbzMode(const uint16_t pulses_per_round,
                     const PulseWidth z_pulse_width,
                     const Hyst hysteresis) {
    CHECK(WritePulseWidth(z_pulse_width));
    CHECK(WriteHyst(hysteresis));
    CHECK(WriteABZPulsePerRound(pulses_per_round));
    CHECK(WriteMode(Mode::kABZ));
    return Error::kOk;
  }

  /**
   * @brief 启用-a-b-z UVW模式（仅适用于QFN封装）
   * @param enable 是否启用负逻辑输出
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 配置MT6701传感器的UVW输出使用负逻辑（-a-b-z而不是abz）。
   * 注意：此功能仅在QFN封装的MT6701芯片上可用。
   */
  Error WriteNaNbNzEnable(bool enable) {  // 仅适用于QFN版本
    CHECK(this->WriteRegField(MT6701Regs::kUVM_MUX, enable));
    return Error::kOk;
  }

  /**
   * @brief 设置模拟输出模式
   * @param start 起始角度 [0...360.0)度
   * @param stop 停止角度 [0...360.0)度
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 配置MT6701传感器工作在模拟输出模式，并设置输出范围。
   * 输出电压将在起始角度和停止角度之间线性变化。
   */
  Error WriteAnalogMode(const float start = 0.0f, const float stop = 360.0f) {
    CHECK(WriteStartStop(start, stop));
    CHECK(WriteOutMode(OutMode::kAnalog));
    return Error::kOk;
  }

  /**
   * @brief 设置PWM输出模式
   * @param frequency 频率选项
   * @param polarity 极性选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 配置MT6701传感器工作在PWM输出模式，并设置频率和极性。
   * PWM占空比将随角度线性变化。
   */
  Error WritePwmMode(const PwmFreq frequency = PwmFreq::kPWMFreq497_2,
                     const PwmPol polarity = PwmPol::kHigh) {
    CHECK(WriteOutMode(OutMode::kPWM));
    CHECK(WritePwmFreq(frequency));
    CHECK(WritePwmPolarity(polarity));
    return Error::kOk;
  }

  /**
   * @brief 设置旋转方向
   * @param direction 方向选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器的角度计数方向。
   * 可以选择顺时针或逆时针方向作为角度增加的方向。
   */
  Error WriteDirection(const Direction direction) {
    CHECK(
        this->WriteRegField(MT6701Regs::kDIR, static_cast<uint8_t>(direction)));
    return Error::kOk;
  }
  /**
   * @brief 获取旋转方向
   * @param direction 方向选项的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 获取MT6701传感器的当前旋转方向。
   */
  Error GetDirection(Direction& direction) {
    uint8_t value;
    CHECK(this->ReadRegField(MT6701Regs::kDIR, value));
    direction = static_cast<Direction>(value);
    return Error::kOk;
  }
  /**
   * @brief 将当前设置保存到EEPROM
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 将当前的所有配置参数保存到MT6701传感器的内部EEPROM中。
   * 保存后的设置在传感器断电后仍然有效。
   * 注意：EEPROM写入操作有寿命限制，不应频繁调用此方法。
   */
  Error ProgramEEPROM() {
    CHECK(this->Write(0x09, 0xB3));
    CHECK(this->Write(0x0A, 0x05));
    delay(600);
    return Error::kOk;
  }

  /**
   * @brief 设置ABZ模式的分辨率
   * @param pulses 每圈脉冲数 [1...1024]
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器在ABZ模式下的分辨率（每圈脉冲数）。
   * 此参数决定了A和B信号的精度。
   */
  Error WriteABZPulsePerRound(uint16_t pulses) {
    pulses--;
    if (pulses >= 1024) {
      return Error::kOutOfRange;
    }
    CHECK(this->WriteRegField(
        MT6701Regs::kABZ_RES_8, MT6701Regs::kABZ_RES_0, pulses));
    return Error::kOk;
  }

  /**
   * @brief 设置UVW模式的极对数
   * @param pairs 极对数 [1...16]
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器在UVW模式下的极对数。
   * 此参数应与电机的极对数匹配，以确保正确的换相。
   */
  Error WriteUVWPolePair(uint8_t pairs) {
    pairs--;
    if (pairs >= 16) {
      return Error::kOutOfRange;
    }
    CHECK(this->WriteRegField(MT6701Regs::kUVM_RES_0, pairs));
    return Error::kOk;
  }

  /**
   * @brief 设置工作模式
   * @param mode 模式选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器的基本工作模式。
   * 可以选择UVW模式（适用于无刷电机）或ABZ模式（增量式编码器）。
   */
  Error WriteMode(Mode mode) {
    CHECK(
        this->WriteRegField(MT6701Regs::kABZ_MUX, static_cast<uint8_t>(mode)));
    return Error::kOk;
  }

  /**
   * @brief 设置12位位置值的零偏移
   * @param zero 原始角度值 [0...4096]
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 使用原始值设置MT6701传感器的零位偏移。
   * 此方法使用12位分辨率的原始值（0-4096范围）。
   */
  Error WriteZeroRaw(uint16_t zero) {
    CHECK(this->WriteRegField(MT6701Regs::kZERO_8, MT6701Regs::kZERO_0, zero));
    return Error::kOk;
  }

  /**
   * @brief 设置浮点值的零偏移
   * @param zero 角度 [0...360.0)度
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 使用角度值设置MT6701传感器的零位偏移。
   * 此方法使用浮点角度值（0-360度范围）。
   */
  Error WriteZero(float zero) {
    return WriteZeroRaw(static_cast<uint16_t>(zero * 4096 / 360.0f));
  }

  /**
   * @brief 设置迟滞参数
   * @param hysteresis 迟滞选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器的迟滞参数，用于抑制角度输出的抖动。
   * 较大的迟滞值可以减少噪声，但会降低灵敏度。
   */
  Error WriteHyst(Hyst hysteresis) {
    CHECK(this->WriteRegField(MT6701Regs::kHYST_2,
                              MT6701Regs::kHYST_0,
                              static_cast<uint16_t>(hysteresis)));
    return Error::kOk;
  }

  /**
   * @brief 设置模拟输出的起始和停止角度（原始值）
   * @param start 起始角度原始值 [0...4096]
   * @param stop 停止角度原始值 [0...4096]
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 使用原始值设置MT6701传感器模拟输出模式的角度范围。
   * 此方法使用12位分辨率的原始值（0-4096范围）。
   * 注意：停止角度必须大于起始角度。
   */
  Error WriteStartStopRaw(uint16_t start, uint16_t stop) {
    // 检查参数范围
    if (start >= 4096 || stop >= 4096) {
      return Error::kOutOfRange;
    }

    // 检查起始和结束角度的合理性
    if (stop <= start) {
      return Error::kInvalidParameter;
    }

    CHECK(this->WriteRegField(
        MT6701Regs::kA_START_8, MT6701Regs::kA_START_0, start));
    CHECK(this->WriteRegField(
        MT6701Regs::kA_STOP_8, MT6701Regs::kA_STOP_0, stop));
    return Error::kOk;
  }

  /**
   * @brief 设置模拟输出的起始和停止角度（浮点值）
   * @param start 起始角度 [0...360.0)度
   * @param stop 停止角度 [0...360.0)度
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 使用角度值设置MT6701传感器模拟输出模式的角度范围。
   * 此方法使用浮点角度值（0-360度范围）。
   * 注意：停止角度必须大于起始角度。
   */
  Error WriteStartStop(float start, float stop) {
    uint16_t start_u16 = static_cast<uint16_t>(start * 4096 / 360.0f);
    uint16_t stop_u16 = static_cast<uint16_t>(stop * 4096 / 360.0f);
    start_u16 = start_u16 >= 4096 ? 4095 : start_u16;
    stop_u16 = stop_u16 >= 4096 ? 4095 : stop_u16;
    return WriteStartStopRaw(start_u16, stop_u16);
  }

  /**
   * @brief 设置Z信号的脉冲宽度
   * @param width 脉冲宽度选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器在ABZ模式下Z信号的脉冲宽度。
   */
  Error WritePulseWidth(PulseWidth width) {
    CHECK(this->WriteRegField(MT6701Regs::kPULSE_WIDTH,
                              static_cast<uint8_t>(width)));
    return Error::kOk;
  }
  /**
   * @brief 获取Z信号的脉冲宽度
   * @param width 脉冲宽度选项的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 获取MT6701传感器的当前Z信号脉冲宽度。
   */
  Error ReadPulseWidth(PulseWidth& width) {
    uint8_t value;
    CHECK(this->ReadRegField(MT6701Regs::kPULSE_WIDTH, value));
    width = static_cast<PulseWidth>(value);
    return Error::kOk;
  }
  /**
   * @brief 设置PWM输出频率
   * @param freq 频率选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器PWM输出模式的频率。
   */
  Error WritePwmFreq(PwmFreq freq) {
    CHECK(
        this->WriteRegField(MT6701Regs::kPWM_FREQ, static_cast<uint8_t>(freq)));
    return Error::kOk;
  }
  /**
   * @brief 获取PWM输出频率
   * @param freq 频率选项的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 获取MT6701传感器的当前PWM输出频率。
   */
  Error ReadPwmFreq(PwmFreq& freq) {
    uint8_t value;
    CHECK(this->ReadRegField(MT6701Regs::kPWM_FREQ, value));
    freq = static_cast<PwmFreq>(value);
    return Error::kOk;
  }
  /**
   * @brief 设置PWM输出极性
   * @param polarity 极性选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器PWM输出的极性。
   * 高电平有效时，占空比随角度增加而增加；
   * 低电平有效时，占空比随角度增加而减少。
   */
  Error WritePwmPolarity(PwmPol polarity) {
    CHECK(this->WriteRegField(MT6701Regs::kPWM_POL,
                              static_cast<uint8_t>(polarity)));
    return Error::kOk;
  }
  /**
   * @brief 获取PWM输出极性
   * @param polarity 极性选项的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 获取MT6701传感器的当前PWM输出极性。
   */
  Error ReadPwmPolarity(PwmPol& polarity) {
    uint8_t value;
    CHECK(this->ReadRegField(MT6701Regs::kPWM_POL, value));
    polarity = static_cast<PwmPol>(value);
    return Error::kOk;
  }
  /**
   * @brief 设置输出模式
   * @param mode 输出模式选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器的输出信号类型。
   * 可以选择模拟输出（电压随角度变化）或PWM输出（占空比随角度变化）。
   */
  Error WriteOutMode(const OutMode mode) {
    CHECK(
        this->WriteRegField(MT6701Regs::kOUT_MODE, static_cast<uint8_t>(mode)));
    return Error::kOk;
  }
  /**
   * @brief 获取输出模式
   * @param mode 输出模式选项的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 获取MT6701传感器的当前输出模式。
   */
  Error ReadOutMode(OutMode& mode) {
    uint8_t value;
    CHECK(this->ReadRegField(MT6701Regs::kOUT_MODE, value));
    mode = static_cast<OutMode>(value);
    return Error::kOk;
  }
};

}  // namespace hortor::drivers::MT6701
