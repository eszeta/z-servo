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

#include "../utils/bit_utils.h"

namespace hortor_servo {

/**
 * @brief 寄存器位域描述结构
 *
 * 定义寄存器位域的地址、起始位置、位宽和掩码。
 * 用于描述寄存器中的特定位域。
 */
struct Register {
  const uint8_t address;  // 寄存器地址
  const uint8_t shift;    // 起始位置
  const uint8_t bits;     // 位宽
  const uint8_t mask;     // 掩码

  /**
   * @brief 构造函数
   * @param address 寄存器地址
   * @param shift 起始位置
   * @param bits 位宽 (1-8)
   */
  constexpr Register(const uint8_t address,
                     const uint8_t shift,
                     const uint8_t bits)
      : address(address),
        shift(shift),
        bits(bits),
        mask(bit_utils::CreateMask8(shift, bits)) {}

  /** @brief 复制构造函数 */
  Register(const Register& other) = delete;

  /** @brief 赋值操作符 */
  Register& operator=(const Register& other) = delete;

  /**
   * @brief 获取寄存器中的特定位域值
   * @param field 寄存器位域描述
   * @param data 寄存器数据
   * @return 特定位域的值
   */
  static constexpr uint8_t GetValue(const Register& field,
                                    const uint8_t data) noexcept {
    return (data & field.mask) >> field.shift;
  }

  /**
   * @brief 设置寄存器中的特定位域值
   * @param field 寄存器位域描述
   * @param value 特定位域的值
   * @param data 寄存器数据
   * @return 更新后的寄存器数据
   */
  static constexpr uint8_t SetValue(const Register& field,
                                    const uint8_t value,
                                    uint8_t& data) noexcept {
    data = (data & ~field.mask) | ((value << field.shift) & field.mask);
    return data;
  }

  /**
   * @brief 设置组合值
   * @param highField 高位字节寄存器位域描述
   * @param lowField 低位字节寄存器位域描述
   * @param value 组合值
   * @param highByte 高位字节
   * @param lowByte 低位字节
   */
  static constexpr void SetCombinedValue(const Register& highField,
                                         const Register& lowField,
                                         const uint16_t value,
                                         uint8_t& highByte,
                                         uint8_t& lowByte) noexcept {
    SetValue(lowField, value, lowByte);
    SetValue(highField, value >> lowField.bits, highByte);
  }

  /**
   * @brief 组合寄存器操作（用于跨越多个字节的值）
   * @param highField 高位字节寄存器位域描述
   * @param lowField 低位字节寄存器位域描述
   * @param highByte 高位字节
   * @param lowByte 低位字节
   * @return 组合后的寄存器数据
   */
  static constexpr uint16_t GetCombinedValue(const Register& highField,
                                             const Register& lowField,
                                             const uint8_t highByte,
                                             const uint8_t lowByte) noexcept {
    const uint16_t high = GetValue(highField, highByte);
    const uint16_t low = GetValue(lowField, lowByte);
    return (high << lowField.bits) | low;
  }
};
}  // namespace hortor_servo
