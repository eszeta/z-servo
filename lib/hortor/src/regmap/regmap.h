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

#include "hortor.h"
#include "regmap/reg_field.h"
#include "servo/types.h"

namespace hortor::regmap {

/**
 * @brief 寄存器映射基类（CRTP模式）
 *
 * 提供寄存器读写功能。基于 Linux Kernel regmap 子系统设计。
 * 使用 CRTP (Curiously Recurring Template Pattern) 实现编译期静态多态，
 * 消除虚函数和 std::function 的运行时开销。
 *
 * @tparam Derived 派生类类型，必须实现 WriteBytesImpl 和 ReadBytesImpl 方法
 */
template <typename Derived>
class RegMap {
 protected:
  /**
   * @brief 获取派生类引用
   * @return 派生类引用
   */
  Derived& AsDerived() { return static_cast<Derived&>(*this); }

  /**
   * @brief 获取派生类常量引用
   * @return 派生类常量引用
   */
  const Derived& AsDerived() const {
    return static_cast<const Derived&>(*this);
  }

 public:
  /**
   * @brief 写寄存器
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @return 错误码，成功返回OK
   */
  template <typename T>
  Error Write(const uint8_t address, const T data) {
    return WriteBytes(
        address, reinterpret_cast<const uint8_t*>(&data), sizeof(T));
  }

  /**
   * @brief 写寄存器
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @param size 数据长度
   * @return 错误码，成功返回OK
   */
  Error WriteBytes(const uint8_t address,
                   const uint8_t* data,
                   const size_t size) {
    return AsDerived().WriteBytesImpl(address, data, size);
  }

  /**
   * @brief 读寄存器
   * @param address 寄存器地址
   * @param data 读取数据的存储引用
   * @return 错误码，成功返回OK
   */
  template <typename T>
  Error Read(const uint8_t address, T& data) {
    return ReadBytes(address, sizeof(T), reinterpret_cast<uint8_t*>(&data));
  }

  /**
   * @brief 读取多个寄存器
   * @param address 寄存器地址
   * @param size 读取数据的长度
   * @param data 读取数据的存储指针
   * @return 错误码，成功返回OK
   */
  Error ReadBytes(const uint8_t address, const size_t size, uint8_t* data) {
    return AsDerived().ReadBytesImpl(address, size, data);
  }

  /**
   * @brief 通用的寄存器读写模板函数
   * @param reg 寄存器字段定义
   * @param value 要写入的值
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T>
  Error WriteRegField(const RegField<T>& reg, T value) {
    UType<T> data;
    CHECK(Read(reg.address, data));
    SetValue(reg, value, data);
    CHECK(Write(reg.address, data));
    return Error::kOk;
  }

  /**
   * @brief 通用的寄存器读写模板函数
   * @param reg 寄存器字段定义
   * @param value 要写入的值
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T>
  Error WriteRegField(const RegField<T>& reg, bool value) {
    return WriteRegField(reg, value ? 1 : 0);
  }

  /**
   * @brief 通用的寄存器读写模板函数
   * @param high 高位寄存器字段定义
   * @param low 低位寄存器字段定义
   * @param value 要写入的值
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T>
  Error WriteRegField(const RegField<T>& high,
                      const RegField<T>& low,
                      uint16_t value) {
    T high_value, low_value;
    CHECK(Read(high.address, high_value));
    CHECK(Read(low.address, low_value));
    SetCombinedValue(high, low, value, high_value, low_value);
    CHECK(Write(high.address, high_value));
    CHECK(Write(low.address, low_value));
    return Error::kOk;
  }

  /**
   * @brief 通用的寄存器读取模板函数
   * @param reg 寄存器字段定义
   * @param value 读取值的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T>
  Error ReadRegField(const RegField<T>& reg, T& value) {
    T data;
    CHECK(Read(reg.address, data));
    value = GetValue(reg, data);
    return Error::kOk;
  }

  /**
   * @brief 通用的寄存器读取模板函数
   * @param reg 寄存器字段定义
   * @param value 读取值的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T>
  Error ReadRegField(const RegField<T>& reg, bool& value) {
    uint8_t data;
    CHECK(ReadRegField(reg, data));
    value = data != 0;
    return Error::kOk;
  }

  /**
   * @brief 通用的寄存器读写模板函数
   * @param high 高位寄存器字段定义
   * @param low 低位寄存器字段定义
   * @param value 要写入的值
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T>
  Error ReadRegField(const RegField<T>& high,
                     const RegField<T>& low,
                     uint16_t& value) {
    T high_value, low_value;
    CHECK(Read(high.address, high_value));
    CHECK(Read(low.address, low_value));
    value = GetCombinedValue(high, low, high_value, low_value);
    return Error::kOk;
  }
};

}  // namespace hortor::regmap
