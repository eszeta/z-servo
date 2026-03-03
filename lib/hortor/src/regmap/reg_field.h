// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include <tuple>
#include <type_traits>

#include "base/noncopyable.h"
#include "hortor.h"
#include "utils/bit_utils.h"

namespace hortor::regmap {
template <typename VALUE_TYPE, uint8_t ADDRESS, uint8_t SHIFT, uint8_t BITS>
struct Field;

template <typename VALUE_TYPE, typename HIGH_FIELD, typename LOW_FIELD>
struct Merged2;

template <typename VALUE_TYPE>
struct Converter;

// template <typename VALUE_TYPE, int32_t NUMERATOR, int32_t DENOMINATOR>
// struct RatioConverter;

struct Trait;

}  // namespace hortor::regmap

namespace hortor::regmap {

struct Trait {
  template <typename T>
  struct AccessType {
    static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                  "Unsupported RegField element size");

    static constexpr size_t kIndex =
        sizeof(T) == 1 ? 0 : (sizeof(T) == 2 ? 1 : 2);

    /// 单寄存器存储类型（与 T 同宽，无符号，用于位域操作）
    using Type =
        std::tuple_element_t<kIndex, std::tuple<uint8_t, uint16_t, uint32_t>>;
  };

  template <typename OUT_TYPE, uint8_t VALID_BITS, typename RAW_TYPE>
  static constexpr OUT_TYPE SignExtend(const RAW_TYPE raw) noexcept {
    static_assert(std::is_integral_v<OUT_TYPE>,
                  "SignExtend output type must be integral");
    static_assert(VALID_BITS > 0, "SignExtend valid bits must be > 0");

    constexpr uint8_t kTargetBits = sizeof(OUT_TYPE) * 8U;
    static_assert(VALID_BITS <= kTargetBits,
                  "SignExtend valid bits exceed output width");

    using unsigned_out_t = std::make_unsigned_t<OUT_TYPE>;
    constexpr uint8_t kShift = static_cast<uint8_t>(kTargetBits - VALID_BITS);

    // 先左移让符号位对齐到最高位，再算术右移恢复目标位宽。
    const auto shifted =
        static_cast<unsigned_out_t>(static_cast<unsigned_out_t>(raw) << kShift);
    const auto as_signed = static_cast<OUT_TYPE>(shifted);
    return static_cast<OUT_TYPE>(as_signed >> kShift);
  }

  template <typename T, typename = void>
  struct FieldOf {
    using Type = typename T::FieldBase;
  };

  template <typename T, typename = void>
  struct ConverterOf {
    using value_t = typename FieldOf<T>::Type::value_t;
    using Type = ::hortor::regmap::Converter<value_t>;
  };

  template <typename T>
  struct ConverterOf<T, std::void_t<typename T::converter_t>> {
    using Type = typename T::converter_t;
  };

  template <typename T>
  struct ReadValueTypeOf {
    using Type = typename ConverterOf<T>::Type::value_t;
  };

  template <typename T>
  struct WriteValueTypeOf {
    using Type = typename ConverterOf<T>::Type::value_t;
  };

  template <typename T>
  struct EncodePipeline {
    using Field = typename FieldOf<T>::Type;
    using Converter = typename ConverterOf<T>::Type;
    using in_t = typename WriteValueTypeOf<T>::Type;
    using out_t = typename Field::value_t;

    static constexpr out_t Run(const in_t value) noexcept {
      return Converter::template ToRaw<out_t>(value);
    }
  };

  template <typename T>
  struct DecodePipeline {
    using Field = typename FieldOf<T>::Type;
    using Converter = typename ConverterOf<T>::Type;
    using in_t = typename Field::value_t;
    using out_t = typename ReadValueTypeOf<T>::Type;

    static constexpr out_t Run(const in_t raw) noexcept {
      return Converter::template FromRaw<in_t>(raw);
    }
  };
};

/**
 * @brief 寄存器位域描述结构
 *
 * 定义寄存器位域的地址、起始位置、位宽和掩码。
 * 用于描述寄存器中的特定位域。
 */
template <typename VALUE_TYPE, uint8_t ADDRESS, uint8_t SHIFT, uint8_t BITS>
struct Field : public hortor::Noncopyable {
 public:
  using FieldBase = Field<VALUE_TYPE, ADDRESS, SHIFT, BITS>;
  using value_t = VALUE_TYPE;
  using access_t = typename Trait::AccessType<value_t>::Type;

  static constexpr uint8_t kAddress = ADDRESS;
  static constexpr uint8_t kShift = SHIFT;
  static constexpr uint8_t kBits = BITS;
  static constexpr size_t kSize = sizeof(access_t);
  static constexpr size_t kSizeBits = kSize * 8U;
  static constexpr access_t kMask = static_cast<access_t>(
      hortor::utils::bit_utils::CreateMask(kShift, kBits));
  static constexpr access_t kClearMask = static_cast<access_t>(~kMask);

  static_assert(std::is_integral_v<value_t>,
                "Field value type must be integral");
  static_assert(sizeof(value_t) == 1 || sizeof(value_t) == 2 ||
                    sizeof(value_t) == 4,
                "Field value type must be 1/2/4 bytes");
  static_assert(BITS > 0, "Field BITS must be greater than 0");
  static_assert(BITS <= kSizeBits,
                "Field BITS exceeds underlying access width");
  static_assert(SHIFT < kSizeBits,
                "Field SHIFT exceeds underlying access width");
  static_assert(SHIFT + BITS <= kSizeBits,
                "Field range exceeds underlying access width");

  // 从寄存器数据中提取当前位域值；有符号值按位宽执行符号扩展。
  static constexpr value_t GetValue(const access_t data) noexcept {
    const access_t extracted = static_cast<access_t>((data & kMask) >> kShift);
    if constexpr (std::is_signed_v<value_t>) {
      return Trait::template SignExtend<value_t, kBits>(extracted);
    }
    return static_cast<value_t>(extracted);
  }

  // 将 value 写入目标位域，不影响其它位。
  static constexpr void SetValue(const value_t value, access_t& data) noexcept {
    const access_t shifted =
        static_cast<access_t>(static_cast<access_t>(value) << kShift);
    data = static_cast<access_t>((data & kClearMask) | (shifted & kMask));
  }

  static constexpr void Clear(access_t& data) noexcept { data &= kClearMask; }
};

template <typename VALUE_TYPE, typename HIGH_FIELD, typename LOW_FIELD>
struct Merged2 : public hortor::Noncopyable {
  using Merged2Base = Merged2<VALUE_TYPE, HIGH_FIELD, LOW_FIELD>;
  using value_t = VALUE_TYPE;
  using HighField = HIGH_FIELD;
  using LowField = LOW_FIELD;
  using access_t = typename Trait::AccessType<value_t>::Type;
  using high_value_t = typename HighField::value_t;
  using high_access_t = typename HighField::access_t;
  using low_value_t = typename LowField::value_t;
  using low_access_t = typename LowField::access_t;

  static constexpr size_t kValueBits = sizeof(value_t) * 8U;
  static constexpr uint8_t kMergedBits = HighField::kBits + LowField::kBits;

  static_assert(std::is_integral_v<value_t>,
                "Merged2 value type must be integral");
  static_assert(kMergedBits <= kValueBits,
                "Merged2 field width exceeds target value width");

  static constexpr void SetValue(const value_t value,
                                 high_access_t& high_access,
                                 low_access_t& low_access) noexcept {
    HighField::SetValue(static_cast<high_value_t>(value >> LowField::kBits),
                        high_access);
    LowField::SetValue(static_cast<low_value_t>(value), low_access);
  }

  static constexpr value_t GetValue(const high_access_t high_access,
                                    const low_access_t low_access) noexcept {
    constexpr uint8_t kHighBits = HighField::kBits;
    constexpr uint8_t kLowBits = LowField::kBits;

    const access_t high_part =
        static_cast<access_t>(HighField::GetValue(high_access));
    const access_t low_part =
        static_cast<access_t>(LowField::GetValue(low_access));

    constexpr access_t high_mask = static_cast<access_t>(
        hortor::utils::bit_utils::CreateMask(0, kHighBits));
    constexpr access_t low_mask = static_cast<access_t>(
        hortor::utils::bit_utils::CreateMask(0, kLowBits));

    const access_t extracted = static_cast<access_t>(
        ((high_part & high_mask) << kLowBits) | (low_part & low_mask));

    if constexpr (std::is_signed_v<value_t>) {
      return Trait::template SignExtend<value_t, kMergedBits>(extracted);
    }
    return static_cast<value_t>(extracted);
  }
};

template <typename VALUE_TYPE>
struct Converter : public hortor::Noncopyable {
  using value_t = VALUE_TYPE;
  template <typename RAW_TYPE>
  static constexpr value_t FromRaw(const RAW_TYPE raw) noexcept {
    return static_cast<value_t>(raw);
  }

  template <typename RAW_TYPE>
  static constexpr RAW_TYPE ToRaw(const value_t value) noexcept {
    return static_cast<RAW_TYPE>(value);
  }
};

template <typename VALUE_TYPE, int32_t NUMERATOR, int32_t DENOMINATOR = 1>
struct RatioConverter : public Converter<VALUE_TYPE> {
  using value_t = typename regmap::Converter<VALUE_TYPE>::value_t;
  static_assert(NUMERATOR != 0, "NUMERATOR must not be zero");
  static_assert(DENOMINATOR != 0, "DENOMINATOR must not be zero");

  template <typename RAW_TYPE>
  static constexpr value_t FromRaw(const RAW_TYPE raw) noexcept {
    return static_cast<value_t>(raw) * static_cast<value_t>(NUMERATOR) /
           static_cast<value_t>(DENOMINATOR);
  }

  template <typename RAW_TYPE>
  static constexpr RAW_TYPE ToRaw(const value_t value) noexcept {
    return static_cast<RAW_TYPE>(value * static_cast<value_t>(DENOMINATOR) /
                                 static_cast<value_t>(NUMERATOR));
  }
};
}  // namespace hortor::regmap
