// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "regmap/regmap_spi_bus.h"

namespace hortor::drivers::MA330 {

/**
 * @brief SPI通信实现
 *
 * 实现通过SPI协议与MA330传感器通信的功能。
 * SPI模式提供更高的通信速度，但功能有限，主要用于角度读取。
 */
class RegMapSpiBus : public regmap::RegMapSpiBus {
 public:
  /**
   * @brief 写寄存器 MA330特殊SPI写实现
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @return 错误码
   */
  Error Write(const uint8_t address, const uint8_t data);

  /**
   * @brief 读寄存器 MA330特殊SPI读实现
   * @param address 寄存器地址
   * @param data 读取数据的存储指针
   * @return 错误码
   */
  Error Read(const uint8_t address, uint8_t* data);

  /**
   * @brief 读取原始角度数据
   * @param angle_raw 原始角度值引用，用于存储读取的角度值
   * @return 错误码，成功返回OK
   *
   * 通过SPI接口从MA330传感器读取当前角度值。
   */
  Error ReadRaw(uint16_t& angle_raw);

 private:
  uint16_t transfer16(uint16_t outValue);
};

}  // namespace hortor::drivers::MA330
