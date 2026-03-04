// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "regmap/spi_plain.h"

namespace hortor::drivers::MA330 {

/**
 * @brief SPI通信实现
 *
 * 实现通过SPI协议与MA330传感器通信的功能。
 * SPI模式提供更高的通信速度，但功能有限，主要用于角度读取。
 */
class SpiPlain : public regmap::SpiPlain {
 public:
  /**
    * @brief 批量写寄存器 MA330特殊SPI写实现
    * @param address 起始寄存器地址
    * @param data 要写入的数据
    * @param size 数据长度
    * @return 错误码
    */
  Error Write(const uint8_t address, const uint8_t* data, const size_t size) {
    VERIFY(data, Error::kInvalidArg);
    for (size_t i = 0; i < size; ++i) {
      uint16_t cmd = 0x8000 | (((address + i) & 0x1F) << 8) | data[i];
      transfer16(cmd);
      delay(20);  // 20ms delay required per MA330 spec
      transfer16(0x0000);
    }
    return Error::kOk;
  }

  /**
   * @brief 读取多个寄存器实现
   * @param address 寄存器地址
   * @param size 读取数据的长度
   * @param data 读取数据的存储指针
   * @return 错误码，成功返回OK
   */
  Error Read(const uint8_t address, const size_t size, uint8_t* data) {
    VERIFY(data, Error::kInvalidArg);
    for (size_t i = 0; i < size; ++i) {
      uint16_t cmd = 0x4000 | (((address + i) & 0x001F) << 8);
      uint16_t value = transfer16(cmd);
      delayMicroseconds(1);
      value = transfer16(0x0000);
      data[i] = value >> 8;
    }
    return Error::kOk;
  }

  /**
    * @brief 读取原始角度数据
    * @param angle_raw 原始角度值引用，用于存储读取的角度值
    * @return 错误码，成功返回OK
    *
    * 通过SPI接口从MA330传感器读取当前角度值。
    */
  Error ReadRaw(uint16_t& angle_raw) {
    angle_raw = transfer16(0x0000);
    return Error::kOk;
  }

 private:
  uint16_t transfer16(uint16_t outValue) {
    spi_->beginTransaction(spi_settings_);
    if (cs_pin_ >= 0) digitalWrite(cs_pin_, LOW);
    uint16_t value = spi_->transfer16(outValue);
    if (cs_pin_ >= 0) digitalWrite(cs_pin_, HIGH);
    spi_->endTransaction();
    return value;
  }
};

}  // namespace hortor::drivers::MA330
