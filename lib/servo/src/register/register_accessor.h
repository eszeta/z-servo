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

#include "core/types.h"
#include "register/register.h"
namespace hortor_servo {
/**
 * @brief 寄存器访问器基类
 *
 * 提供寄存器读写功能。
 */
class RegisterAccessor {
 public:
  /** @brief 写寄存器函数类型 */
  using WriteFunc =
      std::function<Error(const uint8_t address, const uint8_t data)>;
  /** @brief 多字节写寄存器函数类型 */
  using WriteMultipleFunc = std::function<Error(
      const uint8_t address, const uint8_t* data, const size_t size)>;
  /** @brief 读寄存器函数类型 */
  using ReadFunc = std::function<Error(const uint8_t address, uint8_t* data)>;
  /** @brief 多字节读寄存器函数类型 */
  using ReadMultipleFunc = std::function<Error(
      const uint8_t address, const size_t size, uint8_t* data)>;
  /**
   * @brief 写寄存器
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @return 错误码，成功返回OK
   */
  Error Write(const uint8_t address, const uint8_t data) {
    return write_(address, data);
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
   * @param data 读取数据的存储指针
   * @return 错误码，成功返回OK
   */
  Error Read(const uint8_t address, uint8_t* data) {
    return read_(address, data);
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
  Error WriteRegField(const Register& reg, uint8_t value);

  /**
   * @brief 通用的寄存器读写模板函数
   * @param high 高位寄存器字段定义
   * @param low 低位寄存器字段定义
   * @param value 要写入的值
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  Error WriteRegField(const Register& high,
                      const Register& low,
                      uint16_t value);

  /**
   * @brief 通用的寄存器读取模板函数
   * @param reg 寄存器字段定义
   * @param value 读取值的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  Error ReadRegField(const Register& reg, uint8_t* value);

  /**
   * @brief 通用的寄存器读取模板函数
   * @param reg 寄存器字段定义
   * @param value 读取值的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  Error ReadRegField(const Register& reg, bool* value);

  /**
   * @brief 通用的寄存器读取模板函数
   * @param high 高位寄存器字段定义
   * @param low 低位寄存器字段定义
   * @param value 读取值的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   */
  Error ReadRegField(const Register& high,
                     const Register& low,
                     uint16_t* value);

  /** @brief 设置写寄存器函数 */
  void SetWrite(WriteFunc write) { write_ = write; }

  /** @brief 设置多字节写寄存器函数 */
  void SetWriteMultiple(WriteMultipleFunc write_multiple) {
    write_multiple_ = write_multiple;
  }

  /** @brief 设置读寄存器函数 */
  void SetRead(ReadFunc read) { read_ = read; }

  /** @brief 设置多字节读寄存器函数 */
  void SetReadMultiple(ReadMultipleFunc read_multiple) {
    read_multiple_ = read_multiple;
  }

 protected:
  /** @brief 写寄存器函数对象 */
  WriteFunc write_;
  /** @brief 多字节写寄存器函数对象 */
  WriteMultipleFunc write_multiple_;
  /** @brief 读寄存器函数对象 */
  ReadFunc read_;
  /** @brief 多字节读寄存器函数对象 */
  ReadMultipleFunc read_multiple_;
};
}  // namespace hortor_servo
