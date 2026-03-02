// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Wire.h>

#include "base/noncopyable.h"
#include "base/types.h"
#include "hortor.h"
#include "regmap.h"

namespace hortor::regmap {

/**
 * @brief 内存映射寄存器访问实现
 *
 * 用于直接访问内存映射的寄存器区域。
 */
class MmioPlain : public hortor::Noncopyable {
 public:
  /**
    * @brief 初始化内存映射访问
    * @param regs 寄存器基地址
    * @param size 寄存器区域大小
    * @return 错误码，成功返回OK
    */
  Error Init(uint8_t* regs, const size_t size) {
    if (!regs || size == 0) {
      return Error::kInvalidParameter;
    }
    regs_ = regs;
    size_ = size;
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
    if (address + size > size_) {
      return Error::kInvalidParameter;
    }
    for (size_t i = 0; i < size; ++i) {
      regs_[address + i] = data[i];
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
    if (address + size > size_) {
      return Error::kInvalidParameter;
    }
    for (size_t i = 0; i < size; ++i) {
      data[i] = regs_[address + i];
    }
    return Error::kOk;
  }

 protected:
  /** @brief 寄存器基地址 */
  uint8_t* regs_{};
  /** @brief 寄存器区域大小 */
  size_t size_{};
};

}  // namespace hortor::regmap
