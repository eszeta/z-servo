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

#include <Arduino.h>

#include "hortor.h"
#include "utils/bit_utils.h"

namespace hortor::reg {

template <typename T>
struct RegisterField;
typedef RegisterField<uint8_t> Register8;
typedef RegisterField<uint16_t> Register16;
typedef RegisterField<uint32_t> Register32;

/**
 * @brief 寄存器位域描述结构
 *
 * 定义寄存器位域的地址、起始位置、位宽和掩码。
 * 用于描述寄存器中的特定位域。
 */
template <typename T = uint8_t>
struct __packed RegisterField {
 public:
  const uint8_t address;  // 寄存器地址
  const uint8_t shift;    // 起始位置
  const uint8_t bits;     // 位宽

  /**
   * @brief 构造函数
   * @param address 寄存器地址
   * @param shift 起始位置
   * @param bits 位宽 (1-8)
   */
  constexpr RegisterField(const uint8_t address,
                          const uint8_t shift,
                          const uint8_t bits)
      : address(address), shift(shift), bits(bits) {}

  /** @brief 复制构造函数 */
  RegisterField(const RegisterField& other) = delete;

  /** @brief 赋值操作符 */
  RegisterField& operator=(const RegisterField& other) = delete;

  constexpr uint8_t getSize() const { return sizeof(T); }
};
}  // namespace hortor::reg