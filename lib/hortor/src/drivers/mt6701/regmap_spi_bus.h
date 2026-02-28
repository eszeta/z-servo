// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "base/types.h"
#include "regmap/regmap_spi_bus.h"
#include "types.h"

namespace hortor::drivers::MT6701 {

/**
 * @brief SPI通信实现（CRTP模式）
 *
 * 实现通过SPI协议与MT6701传感器通信的功能。
 * SPI模式提供更高的通信速度，但功能有限，主要用于角度读取。
 *
 * 注意：此类目前未集成到主 MT6701 驱动中。
 * MT6701 主驱动使用 I2C 接口（见 mt6701.h）。
 */
class RegMapSpiBus : public regmap::RegMapSpiBus {
 public:
  /**
   * @brief 写寄存器 MT6701不支持SPI写
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @return 错误码, 始终返回kGeneralErr
   */
  Error Write(const uint8_t address, const uint8_t data) {
    return Error::kGeneralErr;
  }

  /**
   * @brief 写寄存器 MT6701不支持SPI写
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @param size 数据长度
   * @return 错误码, 始终返回kGeneralErr
   */
  Error WriteBytes(const uint8_t address,
                   const uint8_t* data,
                   const size_t size) {
    return Error::kGeneralErr;
  }

  /**
   * @brief 读寄存器 MT6701不支持SPI读
   * @param address 寄存器地址
   * @param data 读取数据的存储指针
   * @return 错误码, 始终返回kGeneralErr
   */
  Error Read(const uint8_t address, uint8_t* data) {
    return Error::kGeneralErr;
  }

  /**
   * @brief 读寄存器 MT6701不支持SPI读
   * @param address 寄存器地址
   * @param data 读取数据的存储指针
   * @param size 读取数据的长度
   * @return 错误码, 始终返回kGeneralErr
   */
  Error ReadBytes(const uint8_t address, const size_t size, uint8_t* data) {
    return Error::kGeneralErr;
  }

  /**
   * @brief 读取原始数据
   * @param angle_raw 原始角度值指针，用于存储读取的角度值
   * @param field_status 磁场状态指针，用于存储磁场状态
   * @param button_pushed 按钮状态指针，用于存储按钮是否被按下
   * @param track_loss 跟踪丢失状态指针，用于存储是否丢失跟踪
   * @return 错误码，成功返回OK
   *
   * 通过SPI接口从MT6701传感器读取当前角度值和状态信息。
   * SPI模式支持完整的状态信息读取。
   */
  Error ReadRaw(uint16_t& angle_raw,
                Status& field_status,
                bool& button_pushed,
                bool& track_loss) {
    if (!spi_) {
      return Error::kInvalidParameter;
    }

    uint8_t data[3];
    digitalWrite(cs_pin_, LOW);
    spi_->beginTransaction(spi_settings_);

    data[0] = spi_->transfer(0xFF);
    data[1] = spi_->transfer(0xFF);
    data[2] = spi_->transfer(0xFF);

    spi_->endTransaction();
    digitalWrite(cs_pin_, HIGH);

    struct {
      uint16_t angle : 14;  // 14位角度数据，范围0-16383
      uint8_t status : 2;   // 2位磁场状态，指示磁场强度
      uint8_t button : 1;   // 1位按钮状态，指示按钮是否被按下
      uint8_t track : 1;    // 1位跟踪丢失状态，指示是否丢失跟踪
    } __packed* raw = reinterpret_cast<decltype(raw)>(data);

    angle_raw = raw->angle;
    field_status = static_cast<Status>(raw->status);
    button_pushed = raw->button;
    track_loss = raw->track;

    return Error::kOk;
  }
};

}  // namespace hortor::drivers::MT6701