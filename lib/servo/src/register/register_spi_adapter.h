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

#include <SPI.h>

#include "core/types.h"
#include "register_accessor.h"

namespace hortor_servo {
class RegisterSpiAdapter {
 public:
  /**
   * @brief 初始化SPI通信
   * @param spi Arduino SPI对象指针
   * @param cs_pin 片选引脚编号
   * @param spi_settings SPI通信设置
   * @return 错误码，成功返回OK
   */
  Error Init(SPIClass *spi, int cs_pin, const SPISettings &spi_settings);

  /**
   * @brief 设置寄存器访问器函数
   * @param accessor 寄存器访问器
   * @return 错误码，成功返回OK
   */
  Error LinkAccessor(RegisterAccessor &accessor);

  /**
   * @brief 写寄存器
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @param size 数据长度
   * @return 错误码，成功返回OK
   */
  Error WriteMultiple(const uint8_t address,
                      const uint8_t *data,
                      const size_t size);

  /**
   * @brief 读取多个寄存器
   * @param address 寄存器地址
   * @param data 读取数据的存储指针
   * @param size 读取数据的长度
   * @return 错误码，成功返回OK
   */
  Error ReadMultiple(const uint8_t address, const size_t size, uint8_t *data);

 protected:
  /** @brief SPI通信接口指针 */
  SPIClass *spi_{};
  /** @brief 片选引脚编号 */
  int cs_pin_{};
  /** @brief SPI通信设置 */
  SPISettings spi_settings_;
};
}  // namespace hortor_servo
