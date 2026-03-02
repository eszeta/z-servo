// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include <type_traits>

#include "base/noncopyable.h"
#include "hortor.h"
#include "utils/bit_utils.h"
namespace hortor::regmap {

template <typename T>
struct RegFieldTypes;
template <typename T>
using StorageType = typename RegFieldTypes<T>::Storage;
template <typename T>
using MergedType = typename RegFieldTypes<T>::Merged;

template <typename T>
struct RegField;

template <typename T, uint8_t Address, uint8_t Shift, uint8_t Bits>
struct Field;
}  // namespace hortor::regmap

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
                              const StorageType<T> data) noexcept {
    const auto mask = hortor::utils::bit_utils::CreateMask(field.shift, field.bits);
    const auto extracted = (data & mask) >> field.shift;
    if constexpr (std::is_signed_v<T>) {
      const auto target_bits = sizeof(T) * 8;
      const auto shift_amount = target_bits - field.bits;
      return static_cast<T>(extracted << shift_amount) >> shift_amount;
    } else {
      return extracted;
    }
  }

  /**
   * @brief 设置寄存器中的特定位域值（包装函数，自动推导类型）
   * @param field 寄存器位域描述
   * @param value 特定位域的值
   * @param data 寄存器数据
   */
  static constexpr void SetValue(const RegField<T>& field,
                                 const T value,
                                 StorageType<T>& data) noexcept {
    const auto mask = hortor::utils::bit_utils::CreateMask(field.shift, field.bits);
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
  static constexpr MergedType<T> GetCombinedValue(
      const RegField<T>& high_field,
      const RegField<T>& low_field,
      const StorageType<T> high_data,
      const StorageType<T> low_data) noexcept {
    const StorageType<T> high = GetValue(high_field, high_data);
    const StorageType<T> low = GetValue(low_field, low_data);
    const auto high_mask = hortor::utils::bit_utils::CreateMask(0, high_field.bits);
    const auto low_mask = hortor::utils::bit_utils::CreateMask(0, low_field.bits);
    const auto extracted =
        ((high & high_mask) << low_field.bits) | (low & low_mask);
    if constexpr (std::is_signed_v<T>) {
      const auto target_bits = sizeof(MergedType<T>) * 8;
      const auto shift_amount = target_bits - high_field.bits - low_field.bits;
      return static_cast<MergedType<T>>(extracted << shift_amount) >>
             shift_amount;
    } else {
      return extracted;
    }
  }
};
template <typename T, uint8_t Address, uint8_t Shift, uint8_t Bits>
struct Field : public hortor::Noncopyable {
 public:
  using Type = T;
  using Storage = typename RegFieldTypes<T>::Storage;

  static constexpr uint8_t kAddress = Address;  // 寄存器地址
  static constexpr uint8_t kShift = Shift;      // 起始位置
  static constexpr uint8_t kBits = Bits;        // 位宽
  static constexpr size_t kMask =
      hortor::utils::bit_utils::CreateMask(Shift, Bits);
  static constexpr size_t kClearMask = ~kMask;
  static constexpr size_t kSize = sizeof(T);

  /**
   * 
   * @brief 获取寄存器中的特定位域值（
   * @param data 寄存器数据
   * @return 特定位域的值
   */
  static constexpr T GetValue(const Storage data) noexcept {
    const auto extracted = (data & kMask) >> kShift;
    if constexpr (std::is_signed_v<T>) {
      const auto target_bits = sizeof(Storage) * 8;
      const auto shift_amount = target_bits - kBits;
      return static_cast<T>(extracted << shift_amount) >> shift_amount;
    } else {
      return extracted;
    }
  }

  /**
   * @brief 设置寄存器中的特定位域值
   * @param data 寄存器数据
   * @param value 特定位域的值
   */
  static constexpr void SetValue(const T value, Storage& data) noexcept {
    data = (data & ~kMask) | ((value << kShift) & kMask);
  }

  /**
   * @brief 清除寄存器中的特定位域值
   * @param data 寄存器数据
   */
  static constexpr void Clear(Storage& data) noexcept {
    data = data & kClearMask;
  }
};

template <typename T, typename High, typename Low>
struct Merged2 : public hortor::Noncopyable {
  using Type = T;
  using Storage = typename RegFieldTypes<T>::Storage;
  using Merged2Base = Merged2<T, High, Low>;
  using HighField = High;
  using LowField = Low;
  using HighType = typename High::Type;
  using LowType = typename Low::Type;
  using HighStorage = typename High::Storage;
  using LowStorage = typename Low::Storage;

  static constexpr T GetValue(const Storage data) noexcept {
    return Merged2Base::GetValue(data);
  }

  static constexpr void SetValue(const T value,
                                 HighStorage& high_data,
                                 LowStorage& low_data) noexcept {
    High::SetValue(value >> Low::kBits, high_data);
    Low::SetValue(value, low_data);
  }

  static constexpr T GetValue(const HighStorage high_data,
                              const LowStorage low_data) noexcept {
    const HighType high_value = High::GetValue(high_data);
    const LowType low_value = Low::GetValue(low_data);
    const auto extracted =
        ((high_value & High::kMask) << Low::kBits) | (low_value & Low::kMask);
    if constexpr (std::is_signed_v<T>) {
      const auto target_bits = sizeof(T) * 8;
      const auto shift_amount = target_bits - High::kBits - Low::kBits;
      return static_cast<T>(extracted << shift_amount) >> shift_amount;
    } else {
      return extracted;
    }
  }
};
}  // namespace hortor::regmap
