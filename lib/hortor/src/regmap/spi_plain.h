// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <SPI.h>

#include "base/types.h"
#include "hortor.h"
#include "regmap.h"

namespace hortor::regmap {

/**
 * @brief SPI总线寄存器访问实现
 */
class SpiPlain : public hortor::Noncopyable {
 public:
  SPIClass* spi() { return spi_; }
  int cs_pin() const { return cs_pin_; }
  SPISettings spi_settings() const { return spi_settings_; }

  /**
   * @brief 初始化SPI通信
   * @param spi Arduino SPI对象指针
   * @param cs_pin 片选引脚编号
   * @param spi_settings SPI通信设置
   * @return 错误码，成功返回OK
   */
  Error Init(SPIClass* spi, int cs_pin, const SPISettings& spi_settings) {
    VERIFY(spi, Error::kInvalidArg);
    spi_ = spi;
    cs_pin_ = cs_pin;
    spi_settings_ = spi_settings;
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    return Error::kOk;
  }
  /**
   * @brief 写寄存器实现
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @param size 数据长度
   * @return 错误码，成功返回OK
   */
  Error Write(const uint8_t address, const uint8_t* data, const size_t size) {
    VERIFY(spi_ && data, Error::kInvalidArg);
    spi_->beginTransaction(spi_settings_);
    if (cs_pin_ >= 0) digitalWrite(cs_pin_, LOW);
    spi_->transfer(address);
    spi_->transfer(const_cast<uint8_t*>(data), nullptr, size);
    if (cs_pin_ >= 0) digitalWrite(cs_pin_, HIGH);
    spi_->endTransaction();
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
    VERIFY(spi_, Error::kInvalidArg);
    spi_->beginTransaction(spi_settings_);
    if (cs_pin_ >= 0) digitalWrite(cs_pin_, LOW);
    spi_->transfer(0x80 | address);
    spi_->transfer(data, size);
    if (cs_pin_ >= 0) digitalWrite(cs_pin_, HIGH);
    spi_->endTransaction();
    return Error::kOk;
  }

 protected:
  /** @brief SPI通信接口指针 */
  SPIClass* spi_{};
  /** @brief 片选引脚编号 */
  int cs_pin_{};
  /** @brief SPI通信设置 */
  SPISettings spi_settings_;
};
}  // namespace hortor::regmap
