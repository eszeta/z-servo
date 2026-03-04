// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "base/types.h"
#include "hortor.h"
#include "regmap/i2c_plain.h"
#include "regmap/spi_plain.h"
#include "types.h"
namespace hortor::drivers::MT6701 {

template <PlainType PLAIN_TYPE>
class RegMapBase;

template <>
class RegMapBase<PlainType::kI2C>
    : public hortor::regmap::RegMap<hortor::regmap::I2CPlain> {
 public:
  Error Init(TwoWire* wire, const int address) {
    CHECK(plain_.Init(wire, address));
    return Error::kOk;
  }

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
    using kANGLE_6 = MT6701Regs::kANGLE_6;
    using kANGLE_0 = MT6701Regs::kANGLE_0;
    CHECK(ReadField<uint16_t, kANGLE_6, kANGLE_0>(angle_raw));
    (void)field_status;
    (void)button_pushed;
    (void)track_loss;
    return Error::kOk;
  }
};

template <>
class RegMapBase<PlainType::kSPI> : public regmap::RegMap<regmap::SpiPlain> {
 public:
  Error Init(SPIClass* spi, int cs_pin, const SPISettings& spi_settings) {
    CHECK(plain_.Init(spi, cs_pin, spi_settings));
    return Error::kOk;
  }

  Error ReadRaw(uint16_t& angle_raw,
                Status& field_status,
                bool& button_pushed,
                bool& track_loss) {
    SPIClass* spi = plain_.spi();
    const int cs_pin = plain_.cs_pin();
    const SPISettings spi_settings = plain_.spi_settings();
    if (!spi) {
      return Error::kInvalidArg;
    }

    uint8_t data[3];
    digitalWrite(cs_pin, LOW);
    spi->beginTransaction(spi_settings);

    data[0] = spi->transfer(0xFF);
    data[1] = spi->transfer(0xFF);
    data[2] = spi->transfer(0xFF);

    spi->endTransaction();
    digitalWrite(cs_pin, HIGH);

    struct {
      uint16_t angle : 14;
      uint8_t status : 2;
      uint8_t button : 1;
      uint8_t track : 1;
    } __packed* raw = reinterpret_cast<decltype(raw)>(data);

    angle_raw = raw->angle;
    field_status = static_cast<Status>(raw->status);
    button_pushed = raw->button;
    track_loss = raw->track;

    return Error::kOk;
  }

  Error Write(const uint8_t address, const uint8_t data) {
    return Error::kErr;
  }

  Error Write(const uint8_t address, const uint8_t* data, const size_t size) {
    return Error::kErr;
  }

  Error Read(const uint8_t address, uint8_t* data) {
    return Error::kErr;
  }

  Error Read(const uint8_t address, const size_t size, uint8_t* data) {
    return Error::kErr;
  }
};

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
template <PlainType PLAIN_TYPE>
class RegMap : public RegMapBase<PLAIN_TYPE> {
 public:
  Error ReadFieldStatus(Status& status) {
    uint16_t angle_raw = 0;
    bool button_pushed = false;
    bool track_loss = false;
    CHECK(this->ReadRaw(angle_raw, status, button_pushed, track_loss));
    (void)angle_raw;
    (void)button_pushed;
    (void)track_loss;
    return Error::kOk;
  }

  Error WriteUvmMode(const uint8_t pairs) {
    CHECK(WriteUVWPolePair(pairs));
    CHECK(WriteMode(Mode::kUVW));
    return Error::kOk;
  }

  Error WriteAbzMode(const uint16_t pulses_per_round,
                     const PulseWidth z_pulse_width,
                     const Hyst hysteresis) {
    CHECK(WritePulseWidth(z_pulse_width));
    CHECK(WriteHyst(hysteresis));
    CHECK(WriteABZPulsePerRound(pulses_per_round));
    CHECK(WriteMode(Mode::kABZ));
    return Error::kOk;
  }

  Error WriteNaNbNzEnable(bool enable) {
    CHECK(this->template WriteField<MT6701Regs::kUVM_MUX>(enable));
    return Error::kOk;
  }

  Error WriteAnalogMode(const float start = 0.0f, const float stop = 360.0f) {
    CHECK(WriteStartStop(start, stop));
    CHECK(WriteOutMode(OutMode::kAnalog));
    return Error::kOk;
  }

  Error WritePwmMode(const PwmFreq frequency = PwmFreq::kPWMFreq497_2,
                     const PwmPol polarity = PwmPol::kHigh) {
    CHECK(WriteOutMode(OutMode::kPWM));
    CHECK(WritePwmFreq(frequency));
    CHECK(WritePwmPolarity(polarity));
    return Error::kOk;
  }

  Error WriteDirection(const Direction direction) {
    CHECK(this->template WriteField<MT6701Regs::kDIR>(
        static_cast<uint8_t>(direction)));
    return Error::kOk;
  }

  Error GetDirection(Direction& direction) {
    uint8_t value;
    CHECK(this->template ReadField<MT6701Regs::kDIR>(value));
    direction = static_cast<Direction>(value);
    return Error::kOk;
  }

  Error ProgramEEPROM() {
    CHECK(this->Write(0x09, 0xB3));
    CHECK(this->Write(0x0A, 0x05));
    delay(600);
    return Error::kOk;
  }

  Error WriteABZPulsePerRound(uint16_t pulses) {
    pulses--;
    if (pulses >= 1024) {
      return Error::kOutOfRange;
    }
    using kABZ_RES_8 = MT6701Regs::kABZ_RES_8;
    using kABZ_RES_0 = MT6701Regs::kABZ_RES_0;
    CHECK(this->template WriteField<uint16_t, kABZ_RES_8, kABZ_RES_0>(pulses));
    return Error::kOk;
  }

  Error WriteUVWPolePair(uint8_t pairs) {
    pairs--;
    if (pairs >= 16) {
      return Error::kOutOfRange;
    }
    using kUVM_RES_0 = MT6701Regs::kUVM_RES_0;
    CHECK(this->template WriteField<kUVM_RES_0>(pairs));
    return Error::kOk;
  }

  Error WriteMode(Mode mode) {
    using kABZ_MUX = MT6701Regs::kABZ_MUX;
    CHECK(this->template WriteField<kABZ_MUX>(static_cast<uint8_t>(mode)));
    return Error::kOk;
  }

  Error WriteZeroRaw(uint16_t zero) {
    using kZERO_8 = MT6701Regs::kZERO_8;
    using kZERO_0 = MT6701Regs::kZERO_0;
    CHECK(this->template WriteField<uint16_t, kZERO_8, kZERO_0>(zero));
    return Error::kOk;
  }

  Error WriteZero(float zero) {
    return WriteZeroRaw(static_cast<uint16_t>(zero * 4096 / 360.0f));
  }

  Error WriteHyst(Hyst hysteresis) {
    using kHYST_2 = MT6701Regs::kHYST_2;
    using kHYST_0 = MT6701Regs::kHYST_0;
    CHECK(this->template WriteField<uint8_t, kHYST_2, kHYST_0>(
        static_cast<uint8_t>(hysteresis)));
    return Error::kOk;
  }

  Error WriteStartStopRaw(uint16_t start, uint16_t stop) {
    if (start >= 4096 || stop >= 4096) {
      return Error::kOutOfRange;
    }
    if (stop <= start) {
      return Error::kInvalidArg;
    }
    using kA_START_8 = MT6701Regs::kA_START_8;
    using kA_START_0 = MT6701Regs::kA_START_0;
    using kA_STOP_8 = MT6701Regs::kA_STOP_8;
    using kA_STOP_0 = MT6701Regs::kA_STOP_0;
    CHECK(this->template WriteField<uint16_t, kA_START_8, kA_START_0>(start));
    CHECK(this->template WriteField<uint16_t, kA_STOP_8, kA_STOP_0>(stop));
    return Error::kOk;
  }

  Error WriteStartStop(float start, float stop) {
    uint16_t start_u16 = static_cast<uint16_t>(start * 4096 / 360.0f);
    uint16_t stop_u16 = static_cast<uint16_t>(stop * 4096 / 360.0f);
    start_u16 = start_u16 >= 4096 ? 4095 : start_u16;
    stop_u16 = stop_u16 >= 4096 ? 4095 : stop_u16;
    return WriteStartStopRaw(start_u16, stop_u16);
  }

  Error WritePulseWidth(PulseWidth width) {
    using kPULSE_WIDTH = MT6701Regs::kPULSE_WIDTH;
    CHECK(this->template WriteField<kPULSE_WIDTH>(static_cast<uint8_t>(width)));
    return Error::kOk;
  }

  Error ReadPulseWidth(PulseWidth& width) {
    uint8_t value;
    using kPULSE_WIDTH = MT6701Regs::kPULSE_WIDTH;
    CHECK(this->template ReadField<kPULSE_WIDTH>(value));
    width = static_cast<PulseWidth>(value);
    return Error::kOk;
  }

  Error WritePwmFreq(PwmFreq freq) {
    using kPWM_FREQ = MT6701Regs::kPWM_FREQ;
    CHECK(this->template WriteField<kPWM_FREQ>(static_cast<uint8_t>(freq)));
    return Error::kOk;
  }

  Error ReadPwmFreq(PwmFreq& freq) {
    uint8_t value;
    using kPWM_FREQ = MT6701Regs::kPWM_FREQ;
    CHECK(this->template ReadField<kPWM_FREQ>(value));
    freq = static_cast<PwmFreq>(value);
    return Error::kOk;
  }

  Error WritePwmPolarity(PwmPol polarity) {
    using kPWM_POL = MT6701Regs::kPWM_POL;
    CHECK(this->template WriteField<kPWM_POL>(static_cast<uint8_t>(polarity)));
    return Error::kOk;
  }

  Error ReadPwmPolarity(PwmPol& polarity) {
    uint8_t value;
    using kPWM_POL = MT6701Regs::kPWM_POL;
    CHECK(this->template ReadField<kPWM_POL>(value));
    polarity = static_cast<PwmPol>(value);
    return Error::kOk;
  }

  Error WriteOutMode(const OutMode mode) {
    using kOUT_MODE = MT6701Regs::kOUT_MODE;
    CHECK(this->template WriteField<kOUT_MODE>(static_cast<uint8_t>(mode)));
    return Error::kOk;
  }

  Error ReadOutMode(OutMode& mode) {
    uint8_t value;
    using kOUT_MODE = MT6701Regs::kOUT_MODE;
    CHECK(this->template ReadField<kOUT_MODE>(value));
    mode = static_cast<OutMode>(value);
    return Error::kOk;
  }
};

}  // namespace hortor::drivers::MT6701
