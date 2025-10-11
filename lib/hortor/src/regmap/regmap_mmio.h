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

#include "hortor.h"
#include "regmap.h"
#include "servo/types.h"

namespace hortor::regmap {

/**
 * @brief 内存映射寄存器访问实现
 *
 * 用于直接访问内存映射的寄存器区域。
 */
class RegMapMmio {
 public:
  /**
   * @brief 初始化内存映射访问
   * @param regs 寄存器基地址
   * @param size 寄存器区域大小
   * @return 错误码，成功返回OK
   */
  Error Init(uint8_t* regs, const size_t size);

  /**
   * @brief 设置寄存器访问器函数
   * @param accessor 寄存器访问器
   * @return 错误码，成功返回OK
   */
  Error LinkAccessor(RegMap& accessor);

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
   * @brief 读取多个寄存器
   * @param address 寄存器地址
   * @param data 读取数据的存储指针
   * @param size 读取数据的长度
   * @return 错误码，成功返回OK
   */
  Error ReadMultiple(const uint8_t address, const size_t size, uint8_t* data);

 protected:
  /** @brief 寄存器基地址 */
  uint8_t* regs_{};
  /** @brief 寄存器区域大小 */
  size_t size_{};
};

}  // namespace hortor::regmap

