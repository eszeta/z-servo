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
};

/**
 * @brief 寄存器位域描述结构
 *
 * 定义寄存器位域的地址、起始位置、位宽和掩码。
 * 用于描述寄存器中的特定位域。
 */
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

template <typename T, typename HIGHT_FIELD, typename LOW_FIELD>
struct Merged2 : public hortor::Noncopyable {
  using Type = T;
  using Storage = typename RegFieldTypes<T>::Storage;
  using Merged2Base = Merged2<T, HIGHT_FIELD, LOW_FIELD>;

  using HighField = HIGHT_FIELD;
  using HighType = typename HIGHT_FIELD::Type;
  using HighStorage = typename HIGHT_FIELD::Storage;

  using LowField = LOW_FIELD;
  using LowType = typename LOW_FIELD::Type;
  using LowStorage = typename LOW_FIELD::Storage;

  static constexpr T GetValue(const Storage data) noexcept {
    return Merged2Base::GetValue(data);
  }

  static constexpr void SetValue(const T value,
                                 HighStorage& high_data,
                                 LowStorage& low_data) noexcept {
    HIGHT_FIELD::SetValue(value >> LOW_FIELD::kBits, high_data);
    LOW_FIELD::SetValue(value, low_data);
  }

  static constexpr T GetValue(const HighStorage high_data,
                              const LowStorage low_data) noexcept {
    const HighType high_value = HIGHT_FIELD::GetValue(high_data);
    const LowType low_value = LOW_FIELD::GetValue(low_data);
    const auto extracted =
        ((high_value & HIGHT_FIELD::kMask) << LOW_FIELD::kBits) |
        (low_value & LOW_FIELD::kMask);
    if constexpr (std::is_signed_v<T>) {
      const auto target_bits = sizeof(T) * 8;
      const auto shift_amount =
          target_bits - HIGHT_FIELD::kBits - LOW_FIELD::kBits;
      return static_cast<T>(extracted << shift_amount) >> shift_amount;
    } else {
      return extracted;
    }
  }
};
}  // namespace hortor::regmap
