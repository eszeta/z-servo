// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "base/noncopyable.h"
#include "base/types.h"
#include "hortor.h"
#include "regmap/reg_field.h"

namespace hortor::regmap {

/**
 * @brief 寄存器映射基类（CRTP模式）
 *
 * 提供寄存器读写功能。基于 Linux Kernel regmap 子系统设计。
 * 使用 CRTP (Curiously Recurring Template Pattern) 实现编译期静态多态，
 * 消除虚函数和 std::function 的运行时开销。
 *
 * @tparam Derived 派生类类型，必须实现 WriteImpl 和 ReadImpl 方法
 */
template <typename Derived>
class RegMap : public hortor::Noncopyable {
 public:
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
    return static_cast<Derived*>(this)->WriteImpl(address, data, size);
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
    return static_cast<Derived*>(this)->ReadImpl(address, size, data);
  }

  /**
   * @brief 通用的寄存器读写模板函数
   * @param value 要写入的值
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T>
  Error WriteField(typename T::Type value) {
    typename T::Storage data;
    CHECK(Read(T::kAddress, data));
    T::SetValue(value, data);
    CHECK(Write(T::kAddress, data));
    return Error::kOk;
  }

  template <typename T, typename High, typename Low>
  Error WriteField(T value) {
    typename High::Storage high_data;
    typename Low::Storage low_data;
    CHECK(Read(High::kAddress, high_data));
    CHECK(Read(Low::kAddress, low_data));
    Merged2<T, High, Low>::SetValue(value, high_data, low_data);
    CHECK(Write(High::kAddress, high_data));
    CHECK(Write(Low::kAddress, low_data));
    return Error::kOk;
  }

  /**
   * @brief 通用的寄存器读取模板函数
   * @param value 读取值的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T>
  Error ReadField(typename T::Type& value) {
    typename T::Storage data;
    CHECK(Read(T::kAddress, data));
    value = T::GetValue(data);
    return Error::kOk;
  }

  /**
   * @brief 通用的寄存器读写模板函数
   * @param high 高位寄存器字段定义
   * @param low 低位寄存器字段定义
   * @param value 要写入的值
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T, typename High, typename Low>
  Error ReadField(T& value) {
    typename High::Storage high_data;
    typename Low::Storage low_data;
    CHECK(Read(High::kAddress, high_data));
    CHECK(Read(Low::kAddress, low_data));
    value = Merged2<T, High, Low>::GetValue(high_data, low_data);
    return Error::kOk;
  }
};

}  // namespace hortor::regmap
