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

#include "utils/bit_utils.h"

namespace hortor_servo {

// 模板特化实现组合类型选择
template <typename T, size_t Size = sizeof(T)>
struct CombinedTypeImpl;

template <typename T>
struct CombinedTypeImpl<T, 1> {
  using type = uint16_t;
};

template <typename T>
struct CombinedTypeImpl<T, 2> {
  using type = uint32_t;
};

template <typename T>
struct CombinedTypeImpl<T, 4> {
  using type = uint64_t;
};

template <typename T>
using CombinedType = typename CombinedTypeImpl<T>::type;

template <typename T>
struct RegisterField;
typedef RegisterField<uint8_t> Register8;
typedef RegisterField<uint16_t> Register16;
typedef RegisterField<uint32_t> Register32;

/**
 * @brief 寄存器位域描述结构
 *
 * 定义寄存器位域的地址、起始位置、位宽和掩码。
 * 用于描述寄存器中的特定位域。
 */
template <typename T = uint8_t>
struct __packed RegisterField {
 public:
  const uint8_t address;  // 寄存器地址
  const uint8_t shift;    // 起始位置
  const uint8_t bits;     // 位宽

  /**
   * @brief 构造函数
   * @param address 寄存器地址
   * @param shift 起始位置
   * @param bits 位宽 (1-8)
   */
  constexpr RegisterField(const uint8_t address,
                          const uint8_t shift,
                          const uint8_t bits)
      : address(address), shift(shift), bits(bits) {}

  /** @brief 复制构造函数 */
  RegisterField(const RegisterField& other) = delete;

  /** @brief 赋值操作符 */
  RegisterField& operator=(const RegisterField& other) = delete;

  constexpr uint8_t getSize() const { return sizeof(T); }
};

class RegisterUtils {
 public:
  /**
   * @brief 获取寄存器中的特定位域值
   * @param field 寄存器位域描述
   * @param data 寄存器数据
   * @return 特定位域的值
   */
  template <typename T>
  static constexpr T GetValue(const RegisterField<T>& field,
                              const T data) noexcept {
    const auto mask = bit_utils::CreateMask<T>(field.shift, field.bits);
    return (data & mask) >> field.shift;
  }

  /**
   * @brief 设置寄存器中的特定位域值
   * @param field 寄存器位域描述
   * @param data 寄存器数据
   * @param value 特定位域的值
   * @param out 更新后的寄存器数据
   * @return 更新后的寄存器数据
   */
  template <typename T>
  static constexpr void SetValue(const RegisterField<T>& field,
                                 const T data,
                                 const T value,
                                 T& out) noexcept {
    const T mask = bit_utils::CreateMask<T>(field.shift, field.bits);
    out = (data & ~mask) | ((value << field.shift) & mask);
  }

  /**
   * @brief 组合寄存器操作（用于跨越多个字节的值）
   * @param high_field 高位字节寄存器位域描述
   * @param low_field 低位字节寄存器位域描述
   * @param high_data 高位字节
   * @param low_data 低位字节
   * @return 组合后的寄存器数据
   */
  template <typename T>
  static constexpr auto GetCombinedValue(const RegisterField<T>& high_field,
                                         const RegisterField<T>& low_field,
                                         const T high_data,
                                         const T low_data) noexcept {
    static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                  "Unsupported size");
    const auto high =
        static_cast<CombinedType<T>>(GetValue(high_field, high_data));
    const auto low =
        static_cast<CombinedType<T>>(GetValue(low_field, low_data));
    return (high << low_field.bits) | low;
  }

  /**
   * @brief 设置组合值
   * @param high_field 高位字节寄存器位域描述
   * @param low_field 低位字节寄存器位域描述
   * @param high_data 高位字节
   * @param low_data 低位字节
   * @param value 组合值
   * @param out_high_data 更新后的高位字节
   * @param out_low_data 更新后的低位字节
   */
  template <typename T>
  static constexpr void SetCombinedValue(const RegisterField<T>& high_field,
                                         const RegisterField<T>& low_field,
                                         const T high_data,
                                         const T low_data,
                                         const CombinedType<T> value,
                                         T& out_high_data,
                                         T& out_low_data) noexcept {
    static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                  "Unsupported size");
    const auto lowValue = static_cast<T>(value);
    const auto highValue = static_cast<T>(value >> low_field.bits);
    SetValue(low_field, low_data, lowValue, out_low_data);
    SetValue(high_field, high_data, highValue, out_high_data);
  }
};
}  // namespace hortor_servo
