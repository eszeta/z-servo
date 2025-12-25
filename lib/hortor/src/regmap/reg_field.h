// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include <type_traits>

#include "hortor.h"
#include "utils/bit_utils.h"

namespace hortor::regmap {

template <typename T>
struct CombinedTypeImpl {
  static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                "Unsupported size for CombinedType");

  // 根据大小选择无符号类型
  using UnsignedCombinedType = std::conditional_t<
      sizeof(T) == 1,
      uint16_t,
      std::conditional_t<sizeof(T) == 2, uint32_t, uint64_t>>;

  // 根据大小选择有符号类型
  using SignedCombinedType =
      std::conditional_t<sizeof(T) == 1,
                         int16_t,
                         std::conditional_t<sizeof(T) == 2, int32_t, int64_t>>;

  using UnsignedType = std::conditional_t<
      sizeof(T) == 1,
      uint8_t,
      std::conditional_t<sizeof(T) == 2, uint16_t, uint32_t>>;

  using CombinedType = std::conditional_t<std::is_signed_v<T>,
                                          SignedCombinedType,
                                          UnsignedCombinedType>;
};

template <typename T>
using UCType = typename CombinedTypeImpl<T>::UnsignedCombinedType;
template <typename T>
using CType = typename CombinedTypeImpl<T>::CombinedType;
template <typename T>
using UType = typename CombinedTypeImpl<T>::UnsignedType;

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

  /** @brief 复制构造函数 */
  RegField(const RegField& other) = delete;

  /** @brief 赋值操作符 */
  RegField& operator=(const RegField& other) = delete;

  /**
   * @brief 获取寄存器中的特定位域值
   * @param field 寄存器位域描述
   * @param data 寄存器数据
   * @return 特定位域的值
   */
  static constexpr T GetValue(const RegField<T>& field,
                              const UType<T> data) noexcept {
    using utils::bit_utils::CreateMask;
    const auto mask = CreateMask<UType<T>>(field.shift, field.bits);
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
   * @brief 设置寄存器中的特定位域值
   * @param field 寄存器位域描述
   * @param data 寄存器数据
   * @param value 特定位域的值
   * @param out 更新后的寄存器数据
   * @return 更新后的寄存器数据
   */
  static constexpr void SetValue(const RegField<T>& field,
                                 const T value,
                                 UType<T>& data) noexcept {
    using utils::bit_utils::CreateMask;
    const auto mask = CreateMask<UType<T>>(field.shift, field.bits);
    data = (data & ~mask) | ((value << field.shift) & mask);
  }

  /**
   * @brief 组合寄存器操作（用于跨越多个字节的值）
   * @param high_field 高位字节寄存器位域描述
   * @param low_field 低位字节寄存器位域描述
   * @param high_data 高位字节
   * @param low_data 低位字节
   * @return 组合后的寄存器数据
   */
  static constexpr CType<T> GetCombinedValue(const RegField<T>& high_field,
                                             const RegField<T>& low_field,
                                             const UType<T> high_data,
                                             const UType<T> low_data) noexcept {
    static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                  "Unsupported size");
    using utils::bit_utils::CreateMask;
    const UType<T> high = GetValue(high_field, high_data);
    const UType<T> low = GetValue(low_field, low_data);
    const auto high_mask = CreateMask<UType<T>>(0, high_field.bits);
    const auto low_mask = CreateMask<UType<T>>(0, low_field.bits);
    const auto extracted =
        ((high & high_mask) << low_field.bits) | (low & low_mask);
    if constexpr (std::is_signed_v<T>) {
      constexpr auto target_bits = sizeof(CType<T>) * 8;
      const auto shift_amount = target_bits - high_field.bits - low_field.bits;
      return static_cast<CType<T>>(extracted << shift_amount) >> shift_amount;
    } else {
      return extracted;
    }
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
  static constexpr void SetCombinedValue(const RegField<T>& high_field,
                                         const RegField<T>& low_field,
                                         const CType<T> value,
                                         UType<T>& high_data,
                                         UType<T>& low_data) noexcept {
    static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                  "Unsupported size");
    const auto lowValue = static_cast<UType<T>>(value);
    const auto highValue = static_cast<UType<T>>(value >> low_field.bits);
    SetValue(low_field, lowValue, low_data);
    SetValue(high_field, highValue, high_data);
  }
};

/**
 * @brief 获取寄存器中的特定位域值（包装函数，自动推导类型）
 * @param field 寄存器位域描述
 * @param data 寄存器数据
 * @return 特定位域的值
 */
template <typename T>
constexpr T GetValue(const RegField<T>& field, const UType<T> data) noexcept {
  return RegField<T>::GetValue(field, data);
}

/**
 * @brief 设置寄存器中的特定位域值（包装函数，自动推导类型）
 * @param field 寄存器位域描述
 * @param data 寄存器数据
 * @param value 特定位域的值
 * @param out 更新后的寄存器数据
 */
template <typename T>
constexpr void SetValue(const RegField<T>& field,
                        const T value,
                        UType<T>& data) noexcept {
  RegField<T>::SetValue(field, value, data);
}

/**
 * @brief 组合寄存器操作（包装函数，自动推导类型）
 * @param high_field 高位字节寄存器位域描述
 * @param low_field 低位字节寄存器位域描述
 * @param high_data 高位字节
 * @param low_data 低位字节
 * @return 组合后的寄存器数据
 */
template <typename T>
constexpr CType<T> GetCombinedValue(const RegField<T>& high_field,
                                    const RegField<T>& low_field,
                                    const UType<T> high_data,
                                    const UType<T> low_data) noexcept {
  return RegField<T>::GetCombinedValue(
      high_field, low_field, high_data, low_data);
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
template <typename T>
constexpr void SetCombinedValue(const RegField<T>& high_field,
                                const RegField<T>& low_field,
                                const CType<T> value,
                                UType<T>& high_data,
                                UType<T>& low_data) noexcept {
  RegField<T>::SetCombinedValue(
      high_field, low_field, value, high_data, low_data);
}

}  // namespace hortor::regmap
