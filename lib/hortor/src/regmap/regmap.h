// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "base/noncopyable.h"
#include "base/types.h"
#include "hortor.h"
#include "regmap/reg_field.h"

namespace hortor::regmap {

/**
 * @brief 寄存器映射基类
 */
template <typename PLAIN>
class RegMap : public hortor::Noncopyable {
 public:
  PLAIN& plain() { return plain_; }

  /**
   * @brief 写寄存器
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @return 错误码，成功返回OK
   */
  template <typename T>
  Error Write(const uint8_t address, const T data) {
    return Write(address, reinterpret_cast<const uint8_t*>(&data), sizeof(T));
  }

  /**
   * @brief 写寄存器
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @param size 数据长度
   * @return 错误码，成功返回OK
   */
  Error Write(const uint8_t address, const uint8_t* data, const size_t size) {
    return plain_.Write(address, data, size);
  }

  /**
   * @brief 读寄存器
   * @param address 寄存器地址
   * @param data 读取数据的存储引用
   * @return 错误码，成功返回OK
   */
  template <typename T>
  Error Read(const uint8_t address, T& data) {
    return Read(address, sizeof(T), reinterpret_cast<uint8_t*>(&data));
  }

  /**
   * @brief 读取多个寄存器
   * @param address 寄存器地址
   * @param size 读取数据的长度
   * @param data 读取数据的存储指针
   * @return 错误码，成功返回OK
   */
  Error Read(const uint8_t address, const size_t size, uint8_t* data) {
    return plain_.Read(address, size, data);
  }

  /**
   * @brief 通用的寄存器读写模板函数
   * @param value 要写入的值
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T>
  Error WriteField(typename Trait::WriteValueTypeOf<T>::Type value) {
    using FieldBase = typename Trait::FieldOf<T>::Type;
    typename FieldBase::access_t access;
    CHECK(Read(FieldBase::kAddress, access));
    const auto raw = Trait::EncodePipeline<T>::Run(value);
    FieldBase::SetValue(raw, access);
    CHECK(Write(FieldBase::kAddress, access));
    return Error::kOk;
  }

  template <typename T, typename HIGH_FIELD, typename LOW_FIELD>
  Error WriteField(T value) {
    typename HIGH_FIELD::access_t high_access;
    typename LOW_FIELD::access_t low_access;
    CHECK(Read(HIGH_FIELD::kAddress, high_access));
    CHECK(Read(LOW_FIELD::kAddress, low_access));
    Merged2<T, HIGH_FIELD, LOW_FIELD>::SetValue(value, high_access, low_access);
    CHECK(Write(HIGH_FIELD::kAddress, high_access));
    CHECK(Write(LOW_FIELD::kAddress, low_access));
    return Error::kOk;
  }

  /**
   * @brief 通用的寄存器读取模板函数
   * @param value 读取值的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T>
  Error ReadField(typename Trait::ReadValueTypeOf<T>::Type& value) {
    using FieldBase = typename Trait::FieldOf<T>::Type;
    typename FieldBase::access_t access;
    CHECK(Read(FieldBase::kAddress, access));
    const auto raw = FieldBase::GetValue(access);
    value = Trait::DecodePipeline<T>::Run(raw);
    return Error::kOk;
  }

  /**
   * @brief 通用的寄存器读写模板函数
   * @param high 高位寄存器字段定义
   * @param low 低位寄存器字段定义
   * @param value 要写入的值
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T, typename HIGH_FIELD, typename LOW_FIELD>
  Error ReadField(T& value) {
    typename HIGH_FIELD::access_t high_access;
    typename LOW_FIELD::access_t low_access;
    CHECK(Read(HIGH_FIELD::kAddress, high_access));
    CHECK(Read(LOW_FIELD::kAddress, low_access));
    value =
        Merged2<T, HIGH_FIELD, LOW_FIELD>::GetValue(high_access, low_access);
    return Error::kOk;
  }

 protected:
  PLAIN plain_;
};

}  // namespace hortor::regmap
