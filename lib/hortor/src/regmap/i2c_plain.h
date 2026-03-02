// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Wire.h>

#include "base/types.h"
#include "hortor.h"
#include "regmap.h"

namespace hortor::regmap {

/**
 * @brief I2C通信实现
 *
 * 实现通过I2C协议与设备通信的功能。
 */
class I2CPlain : public hortor::Noncopyable {
 public:
  /**
    * @brief 初始化I2C通信
    * @param wire Arduino Wire对象指针
    * @param address 从机地址
    * @return 错误码，成功返回OK
    */
  Error Init(TwoWire* wire, const int address) {
    if (!wire) {
      return Error::kInvalidParameter;
    }
    wire_ = wire;
    address_ = address;
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
    if (!wire_ || !data) {
      return Error::kInvalidParameter;
    }
    wire_->beginTransmission(address_);
    wire_->write(address);
    wire_->write(data, size);
    if (wire_->endTransmission() != 0) {
      return Error::kIOErr;
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
    if (!wire_ || !data) {
      return Error::kInvalidParameter;
    }
    wire_->beginTransmission(address_);
    wire_->write(address);
    wire_->endTransmission(false);
    wire_->requestFrom(address_, size);
    for (size_t i = 0; i < size; i++) {
      if (wire_->available()) {
        data[i] = wire_->read();
      } else {
        return Error::kIOErr;
      }
    }
    return Error::kOk;
  }

 protected:
  /** @brief I2C通信接口指针，指向Arduino Wire对象 */
  TwoWire* wire_{};
  /** @brief I2C地址 */
  int address_{};
};

}  // namespace hortor::regmap
