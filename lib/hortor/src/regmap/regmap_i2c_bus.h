// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Wire.h>

#include "hortor.h"
#include "regmap.h"
#include "servo/types.h"

namespace hortor::regmap {

/**
 * @brief I2C总线寄存器访问实现（CRTP模式）
 *
 * I2C模式支持完整的寄存器读写操作，适用于配置和数据读取。
 * 通信时序：
 * 1. 写操作：START -> 从机地址(写) -> 寄存器地址 -> 数据 -> STOP
 * 2. 读操作：START -> 从机地址(写) -> 寄存器地址 -> START -> 从机地址(读) ->
 * 数据 -> STOP
 *
 * 错误码说明：
 * - kOk: 操作成功
 * - kInvalidParamErr: 参数无效（wire_为空或address_无效）
 * - kIOErr: I2C通信错误（如设备无响应、通信超时等）
 */
class RegMapI2CBus : public RegMap<RegMapI2CBus> {
 public:
  /**
   * @brief 初始化I2C通信
   * @param wire Arduino Wire对象指针
   * @param address 从机地址
   * @return 错误码，成功返回OK
   */
  Error Init(TwoWire* wire, const int address);

  /**
   * @brief 写寄存器实现
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @param size 数据长度
   * @return 错误码，成功返回OK
   */
  Error WriteBytesImpl(const uint8_t address,
                       const uint8_t* data,
                       const size_t size);

  /**
   * @brief 读取多个寄存器实现
   * @param address 寄存器地址
   * @param size 读取数据的长度
   * @param data 读取数据的存储指针
   * @return 错误码，成功返回OK
   */
  Error ReadBytesImpl(const uint8_t address, const size_t size, uint8_t* data);

 protected:
  /** @brief I2C通信接口指针，指向Arduino Wire对象 */
  TwoWire* wire_{};
  /** @brief I2C地址 */
  int address_{};
};

}  // namespace hortor::regmap
