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

#include <Wire.h>

#include "../../core/object_interface.h"
#include "../../servo_types.h"
#include "register_accessor.h"

namespace hortor_servo {

/**
 * @brief I2C通信实现
 *
 * I2C模式支持完整的寄存器读写操作，适用于配置和数据读取。
 */
class RegisterLocalTransport : public ObjectInterface {
 public:
  /**
   * @brief 初始化I2C通信
   * @param wire Arduino Wire对象指针
   * @param address
   * @return 错误码，成功返回OK
   */
  Error Init(uint8_t* regs, const size_t size);

  /**
   * @brief 设置寄存器访问器函数
   * @param accessor 寄存器访问器
   * @return 错误码，成功返回OK
   */
  Error LinkAccessor(RegisterAccessor& accessor);

  /**
   * @brief 写寄存器
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @return 错误码，成功返回OK
   */
  Error Write(const uint8_t address, const uint8_t data);

  /**
   * @brief 写寄存器
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @param size 数据长度
   * @return 错误码，成功返回OK
   */
  Error WriteMultiple(const uint8_t address,
                      const uint8_t* data,
                      const size_t size);

  /**
   * @brief 读寄存器
   * @param address 寄存器地址
   * @param data 读取数据的存储指针
   * @return 错误码，成功返回OK
   */
  Error Read(const uint8_t address, uint8_t* data);

  /**
   * @brief 读取多个寄存器
   * @param address 寄存器地址
   * @param data 读取数据的存储指针
   * @param size 读取数据的长度
   * @return 错误码，成功返回OK
   */
  Error ReadMultiple(const uint8_t address, const size_t size, uint8_t* data);

 protected:
  /** @brief 寄存器地址 */
  volatile uint8_t* regs_{};
  /** @brief 寄存器大小 */
  size_t size_{};
};
}  // namespace hortor_servo
