// Copyright 2025 ES_ZETA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Arduino.h>

#include "hortor.h"
#include "regmap/regmap.h"
#include "regmap/regmap_mmio.h"
#include "types.h"

namespace hortor::protocol {

/**
 * @brief 协议寄存器映射基类（CRTP模式）
 *
 * 使用 CRTP 实现编译期静态多态，消除虚函数开销。
 *
 * @tparam Derived 派生类类型
 * @tparam BusImpl 总线实现类型（RegMapMmio、RegMapI2CBus等）
 */
template <typename Derived, typename BusImpl>
class RegMap : public BusImpl {
 protected:
  /**
   * @brief 获取派生类引用
   * @return 派生类引用
   */
  Derived& AsDerived() { return static_cast<Derived&>(*this); }

  /**
   * @brief 获取派生类常量引用
   * @return 派生类常量引用
   */
  const Derived& AsDerived() const {
    return static_cast<const Derived&>(*this);
  }

 public:
  /**
   * @brief 获取伺服ID
   * @param id 伺服ID存储引用
   * @return 错误码
   */
  Error GetServoId(uint8_t& id) { return AsDerived().GetServoIdImpl(id); }

  /**
   * @brief 获取状态
   * @param status 状态存储引用
   * @return 错误码
   */
  Error GetStatus(uint8_t& status) { return AsDerived().GetStatusImpl(status); }

  /**
   * @brief 获取状态包返回级别
   * @param return_level 返回级别存储引用
   * @return 错误码
   *
   * 0	PING Instruction
   *    Returns the Status Packet for PING Instruction only
   *
   * 1	PING Instruction
   *    READ Instruction
   *    Returns the Status Packet for PING and READ Instruction
   *
   * 2	All Instructions
   *    Returns the Status Packet for all Instructions
   */
  Error GetReturnLevel(uint8_t& return_level) {
    return AsDerived().GetReturnLevelImpl(return_level);
  }

  /**
   * @brief 恢复EEPROM
   * @return 错误码
   */
  Error RecoveryEeprom() { return AsDerived().RecoveryEepromImpl(); }

  /**
   * @brief 加载EEPROM
   * @return 错误码
   */
  Error LoadEeprom() { return AsDerived().LoadEepromImpl(); }

  /**
   * @brief 存储EEPROM
   * @return 错误码
   */
  Error StoreEeprom() { return AsDerived().StoreEepromImpl(); }

  /**
   * @brief 存储EEPROM（指定地址和大小）
   * @param address 地址
   * @param size 大小
   * @return 错误码
   */
  Error StoreEeprom(const uint8_t address, const uint8_t size) {
    return AsDerived().StoreEepromImpl(address, size);
  }
};

}  // namespace hortor::protocol