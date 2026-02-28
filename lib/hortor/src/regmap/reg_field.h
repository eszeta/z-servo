// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include <type_traits>

#include "hortor.h"
#include "utils/bit_utils.h"

namespace hortor::regmap {

/// 按 sizeof(T) 索引的类型映射表
template <typename T>
struct RegFieldTypes {
  static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                "Unsupported RegField element size");

  static constexpr size_t kIndex =
      sizeof(T) == 1 ? 0 : (sizeof(T) == 2 ? 1 : 2);

  /// 单寄存器存储类型（与 T 同宽，无符号，用于位域操作）
  using Storage =
      std::tuple_element_t<kIndex, std::tuple<uint8_t, uint16_t, uint32_t>>;

  /// 跨寄存器拼接后的值类型（如 high<<8|low），保持 T 的符号性
  using Merged = std::conditional_t<
      std::is_signed_v<T>,
      std::tuple_element_t<kIndex, std::tuple<int16_t, int32_t, int64_t>>,
      std::tuple_element_t<kIndex, std::tuple<uint16_t, uint32_t, uint64_t>>>;
};

template <typename T>
using RegStorage = typename RegFieldTypes<T>::Storage;
template <typename T>
using RegMerged = typename RegFieldTypes<T>::Merged;

template <typename T>
struct RegField;

typedef RegField<uint8_t> RegFieldU08;
typedef RegField<uint16_t> RegFieldU16;
typedef RegField<uint32_t> RegFieldU32;
typedef RegField<int8_t> RegFieldS08;
typedef RegField<int16_t> RegFieldS16;
typedef RegField<int32_t> RegFieldS32;

/**
 * @brief 寄存器位域描述结构
 *
 * 定义寄存器位域的地址、起始位置、位宽和掩码。
 * 用于描述寄存器中的特定位域。
 */
template <typename T = uint8_t>
struct RegField {
 public:
  static constexpr size_t kSize = sizeof(T);
  const uint8_t address;  // 寄存器地址
  const uint8_t shift;    // 起始位置
  const uint8_t bits;     // 位宽

  /**
   * @brief 构造函数
   * @param address 寄存器地址
   * @param shift 起始位置
   * @param bits 位宽 (1-8)
   */
  constexpr RegField(const uint8_t address,
                     const uint8_t shift,
                     const uint8_t bits)
      : address(address), shift(shift), bits(bits) {}

  /**
   * @brief 获取寄存器中的特定位域值（包装函数，自动推导类型）
   * @param field 寄存器位域描述
   * @param data 寄存器数据
   * @return 特定位域的值
   */
  static constexpr T GetValue(const RegField<T>& field,
                              const RegStorage<T> data) noexcept {
    using utils::bit_utils::CreateMask;
    const auto mask = CreateMask<RegStorage<T>>(field.shift, field.bits);
    const auto extracted = (data & mask) >> field.shift;
    if constexpr (std::is_signed_v<T>) {
      constexpr auto target_bits = sizeof(T) * 8;
      const auto shift_amount = target_bits - field.bits;
      return static_cast<T>(extracted << shift_amount) >> shift_amount;
    } else {
      return extracted;
    }
  }

  /**
   * @brief 设置寄存器中的特定位域值（包装函数，自动推导类型）
   * @param field 寄存器位域描述
   * @param data 寄存器数据
   * @param value 特定位域的值
   * @param out 更新后的寄存器数据
   */
  static constexpr void SetValue(const RegField<T>& field,
                                 const T value,
                                 RegStorage<T>& data) noexcept {
    using utils::bit_utils::CreateMask;
    const auto mask = CreateMask<RegStorage<T>>(field.shift, field.bits);
    data = (data & ~mask) | ((value << field.shift) & mask);
  }

  /**
   * @brief 组合寄存器操作（包装函数，自动推导类型）
   * @param high_field 高位字节寄存器位域描述
   * @param low_field 低位字节寄存器位域描述
   * @param high_data 高位字节
   * @param low_data 低位字节
   * @return 组合后的寄存器数据
   */
  static constexpr RegMerged<T> GetCombinedValue(const RegField<T>& high_field,
                                             const RegField<T>& low_field,
                                             const RegStorage<T> high_data,
                                             const RegStorage<T> low_data) noexcept {
    using utils::bit_utils::CreateMask;
    const RegStorage<T> high = GetValue(high_field, high_data);
    const RegStorage<T> low = GetValue(low_field, low_data);
    const auto high_mask = CreateMask<RegStorage<T>>(0, high_field.bits);
    const auto low_mask = CreateMask<RegStorage<T>>(0, low_field.bits);
    const auto extracted =
        ((high & high_mask) << low_field.bits) | (low & low_mask);
    if constexpr (std::is_signed_v<T>) {
      constexpr auto target_bits = sizeof(RegMerged<T>) * 8;
      const auto shift_amount = target_bits - high_field.bits - low_field.bits;
      return static_cast<RegMerged<T>>(extracted << shift_amount) >> shift_amount;
    } else {
      return extracted;
    }
  }

  /**
   * @brief 设置组合值（包装函数，自动推导类型）
   * @param high_field 高位字节寄存器位域描述
   * @param low_field 低位字节寄存器位域描述
   * @param high_data 高位字节
   * @param low_data 低位字节
   * @param value 组合值
   * @param out_high_data 更新后的高位字节
   * @param out_low_data 更新后的低位字节
   */
  static constexpr void SetCombinedValue(const RegField<T>& high_field,
                                         const RegField<T>& low_field,
                                         const RegMerged<T> value,
                                         RegStorage<T>& high_data,
                                         RegStorage<T>& low_data) noexcept {
    const auto lowValue = static_cast<RegStorage<T>>(value);
    const auto highValue = static_cast<RegStorage<T>>(value >> low_field.bits);
    SetValue(low_field, lowValue, low_data);
    SetValue(high_field, highValue, high_data);
  }
};

}  // namespace hortor::regmap
