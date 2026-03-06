// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "base/types.h"
#include "hortor.h"
#include "regmap/reg_field.h"

namespace hortor::regmap {

/**
 * @brief 寄存器映射基类
 */
template <typename PlainType>
class RegMap : public hortor::Noncopyable {
 public:
  PlainType& plain();

  template <typename T>
  Error Write(const uint8_t address, const T data);

  Error Write(const uint8_t address, const uint8_t* data, const size_t size);

  template <typename T>
  Error Read(const uint8_t address, T& data);

  Error Read(const uint8_t address, const size_t size, uint8_t* data);

  template <typename T>
  Error WriteField(typename Trait::WriteValueTypeOf<T>::Type value);

  template <typename T, typename HighFieldType, typename LowFieldType>
  Error WriteField(T value);

  template <typename T>
  Error ReadField(typename Trait::ReadValueTypeOf<T>::Type& value);

  template <typename T, typename HighFieldType, typename LowFieldType>
  Error ReadField(T& value);

 protected:
  PlainType plain_;
};

}  // namespace hortor::regmap

namespace hortor::regmap {

template <typename PlainType>
PlainType& RegMap<PlainType>::plain() {
  return plain_;
}

template <typename PlainType>
template <typename T>
Error RegMap<PlainType>::Write(const uint8_t address, const T data) {
  return Write(address, reinterpret_cast<const uint8_t*>(&data), sizeof(T));
}

template <typename PlainType>
Error RegMap<PlainType>::Write(const uint8_t address, const uint8_t* data,
                               const size_t size) {
  return plain_.Write(address, data, size);
}

template <typename PlainType>
template <typename T>
Error RegMap<PlainType>::Read(const uint8_t address, T& data) {
  return Read(address, sizeof(T), reinterpret_cast<uint8_t*>(&data));
}

template <typename PlainType>
Error RegMap<PlainType>::Read(const uint8_t address, const size_t size,
                              uint8_t* data) {
  return plain_.Read(address, size, data);
}

template <typename PlainType>
template <typename T>
Error RegMap<PlainType>::WriteField(
    typename Trait::WriteValueTypeOf<T>::Type value) {
  using FieldBase = typename Trait::FieldOf<T>::Type;
  typename FieldBase::access_t access;
  CHECK(Read(FieldBase::kAddress, access));
  const auto raw = Trait::EncodePipeline<T>::Run(value);
  FieldBase::SetValue(raw, access);
  CHECK(Write(FieldBase::kAddress, access));
  return Error::kOk;
}

template <typename PlainType>
template <typename T, typename HighFieldType, typename LowFieldType>
Error RegMap<PlainType>::WriteField(T value) {
  typename HighFieldType::access_t high_access;
  typename LowFieldType::access_t  low_access;
  CHECK(Read(HighFieldType::kAddress, high_access));
  CHECK(Read(LowFieldType::kAddress, low_access));
  Merged2<T, HighFieldType, LowFieldType>::SetValue(value, high_access,
                                                    low_access);
  CHECK(Write(HighFieldType::kAddress, high_access));
  CHECK(Write(LowFieldType::kAddress, low_access));
  return Error::kOk;
}

template <typename PlainType>
template <typename T>
Error RegMap<PlainType>::ReadField(
    typename Trait::ReadValueTypeOf<T>::Type& value) {
  using FieldBase = typename Trait::FieldOf<T>::Type;
  typename FieldBase::access_t access;
  CHECK(Read(FieldBase::kAddress, access));
  const auto raw = FieldBase::GetValue(access);
  value          = Trait::DecodePipeline<T>::Run(raw);
  return Error::kOk;
}

template <typename PlainType>
template <typename T, typename HighFieldType, typename LowFieldType>
Error RegMap<PlainType>::ReadField(T& value) {
  typename HighFieldType::access_t high_access;
  typename LowFieldType::access_t  low_access;
  CHECK(Read(HighFieldType::kAddress, high_access));
  CHECK(Read(LowFieldType::kAddress, low_access));
  value = Merged2<T, HighFieldType, LowFieldType>::GetValue(high_access,
                                                            low_access);
  return Error::kOk;
}

}  // namespace hortor::regmap
