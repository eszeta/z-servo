// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

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
 public:
  /**
   * @brief 读取 ControlTableItem 字段
   * @tparam T 字段类型
   * @param item ControlTableItem
   * @param value 读取值的存储引用
   * @return 错误码
   */
  template <typename T>
  Error ReadRegField(const ControlTableItem<T>& item, T& value) {
    return BusImpl::ReadRegField(item.reg, value);
  }

  /**
   * @brief 写入 ControlTableItem 字段
   * @tparam T 字段类型
   * @param item ControlTableItem
   * @param value 要写入的值
   * @return 错误码
   */
  template <typename T>
  Error WriteRegField(const ControlTableItem<T>& item, const T value) {
    return BusImpl::WriteRegField(item.reg, value);
  }

  /**
   * @brief 写入 ControlTableItem 字段
   * @tparam T 字段类型
   * @param item ControlTableItem
   * @param value 要写入的值
   * @return 错误码
   */
  template <typename T>
  Error WriteRegField(const ControlTableItem<T>& item, const bool value) {
    return BusImpl::WriteRegField(
        item.reg, value ? static_cast<T>(1) : static_cast<T>(0));
  }
};

}  // namespace hortor::protocol