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
#include "reg_field.h"
#include "utils/bit_utils.h"

namespace hortor::regmap {

using hortor::utils::bit_utils::CreateMask;

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
/**
 * @brief 获取寄存器中的特定位域值
 * @param field 寄存器位域描述
 * @param data 寄存器数据
 * @return 特定位域的值
 */
template <typename T>
constexpr T GetValue(const RegField<T>& field, const T data) noexcept {
  const auto mask = CreateMask<T>(field.shift, field.bits);
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
constexpr void SetValue(const RegField<T>& field,
                        const T data,
                        const T value,
                        T& out) noexcept {
  const T mask = CreateMask<T>(field.shift, field.bits);
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
constexpr auto GetCombinedValue(const RegField<T>& high_field,
                                const RegField<T>& low_field,
                                const T high_data,
                                const T low_data) noexcept {
  static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                "Unsupported size");
  const auto high =
      static_cast<CombinedType<T>>(GetValue(high_field, high_data));
  const auto low = static_cast<CombinedType<T>>(GetValue(low_field, low_data));
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
constexpr void SetCombinedValue(const RegField<T>& high_field,
                                const RegField<T>& low_field,
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

}  // namespace hortor::regmap

