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

#include <functional>

#include "hortor.h"
#include "register/register_field.h"
#include "servo/types.h"

namespace hortor::reg {

/**
 * @brief 寄存器访问器基类
 *
 * 提供寄存器读写功能。
 */
class RegisterAccessor {
 public:
  /** @brief 多字节写寄存器函数类型 */
  using WriteMultipleFunc = std::function<Error(
      const uint8_t address, const uint8_t* data, const size_t size)>;
  using ReadMultipleFunc = std::function<Error(
      const uint8_t address, const size_t size, uint8_t* data)>;
  /**
   * @brief 写寄存器
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @return 错误码，成功返回OK
   */
  template <typename T>
  Error Write(const uint8_t address, const T data) {
    return WriteMultiple(
        address, reinterpret_cast<const uint8_t*>(&data), sizeof(T));
  }

  /**
   * @brief 写寄存器
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @param size 数据长度
   * @return 错误码，成功返回OK
   */
  Error WriteMultiple(const uint8_t address,
                      const uint8_t* data,
                      const size_t size) {
    return write_multiple_(address, data, size);
  }

  /**
   * @brief 读寄存器
   * @param address 寄存器地址
   * @param data 读取数据的存储引用
   * @return 错误码，成功返回OK
   */
  template <typename T>
  Error Read(const uint8_t address, T& data) {
    return ReadMultiple(address, sizeof(T), reinterpret_cast<uint8_t*>(&data));
  }

  /**
   * @brief 读取多个寄存器
   * @param address 寄存器地址
   * @param size 读取数据的长度
   * @param data 读取数据的存储指针
   * @return 错误码，成功返回OK
   */
  Error ReadMultiple(const uint8_t address, const size_t size, uint8_t* data) {
    return read_multiple_(address, size, data);
  }

  /**
   * @brief 通用的寄存器读写模板函数
   * @param reg 寄存器字段定义
   * @param value 要写入的值
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T>
  Error WriteRegField(const RegisterField<T>& reg, T value) {
    T data;
    CHECK(Read(reg.address, data));
    RegisterUtils::SetValue(reg, data, value, data);
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
  Error WriteRegField(const RegisterField<T>& reg, bool value) {
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
  Error WriteRegField(const RegisterField<T>& high,
                      const RegisterField<T>& low,
                      uint16_t value) {
    T high_value, low_value;
    CHECK(Read(high.address, high_value));
    CHECK(Read(low.address, low_value));
    RegisterUtils::SetCombinedValue(
        high, low, high_value, low_value, value, high_value, low_value);
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
  Error ReadRegField(const RegisterField<T>& reg, T& value) {
    T data;
    CHECK(Read(reg.address, data));
    value = RegisterUtils::GetValue(reg, data);
    return Error::kOk;
  }

  /**
   * @brief 通用的寄存器读取模板函数
   * @param reg 寄存器字段定义
   * @param value 读取值的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  template <typename T>
  Error ReadRegField(const RegisterField<T>& reg, bool& value) {
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
  Error ReadRegField(const RegisterField<T>& high,
                     const RegisterField<T>& low,
                     uint16_t& value) {
    T high_value, low_value;
    CHECK(Read(high.address, high_value));
    CHECK(Read(low.address, low_value));
    value = RegisterUtils::GetCombinedValue(high, low, high_value, low_value);
    return Error::kOk;
  }

  /** @brief 设置多字节写寄存器函数 */
  void SetWriteMultiple(WriteMultipleFunc write_multiple) {
    write_multiple_ = write_multiple;
  }

  /** @brief 设置多字节读寄存器函数 */
  void SetReadMultiple(ReadMultipleFunc read_multiple) {
    read_multiple_ = read_multiple;
  }

 protected:
  /** @brief 多字节写寄存器函数对象 */
  WriteMultipleFunc write_multiple_;
  /** @brief 多字节读寄存器函数对象 */
  ReadMultipleFunc read_multiple_;
};

}  // namespace hortor::reg