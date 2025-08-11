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

#include "../../utils/register/register_spi_adapter.h"
#include "MA330_accessor.h"

namespace hortor_servo {
namespace MA330 {

/**
 * @brief SPI通信实现
 *
 * 实现通过SPI协议与MA330传感器通信的功能。
 * SPI模式提供更高的通信速度，但功能有限，主要用于角度读取。
 */
class MA330SpiAdapter : public RegisterSpiAdapter {
 public:
  /**
   * @brief 设置寄存器访问器函数
   * @param accessor 寄存器访问器
   * @return 错误码
   */
  Error LinkAccessor(MA330Accessor& accessor);

  /**
   * @brief 写寄存器
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @return 错误码, 始终返回kGeneralErr
   */
  Error Write(const uint8_t address, const uint8_t data);

  /**
   * @brief 写寄存器 MA330不支持SPI写数据段
   * @param address 寄存器地址
   * @param data 要写入的数据
   * @param size 数据长度
   * @return 错误码, 始终返回kGeneralErr
   */
  Error WriteMultiple(const uint8_t address,
                      const uint8_t* data,
                      const size_t size) {
    return Error::kGeneralErr;
  }

  /**
   * @brief 读寄存器 MA330不支持SPI读数据
   * @param address 寄存器地址
   * @param data 读取数据的存储指针
   * @return 错误码, 始终返回kGeneralErr
   */
  Error Read(const uint8_t address, uint8_t* data);

  /**
   * @brief 读寄存器 MA330不支持SPI读数据段
   * @param address 寄存器地址
   * @param data 读取数据的存储指针
   * @param size 读取数据的长度
   * @return 错误码, 始终返回kGeneralErr
   */
  Error ReadMultiple(const uint8_t address, const size_t size, uint8_t* data) {
    return Error::kGeneralErr;
  }

  /**
   * @brief 读取原始数据
   * @param angle_raw 原始角度值指针，用于存储读取的角度值
   * @param field_status 磁场状态指针，用于存储磁场状态
   * @param button_pushed 按钮状态指针，用于存储按钮是否被按下
   * @param track_loss 跟踪丢失状态指针，用于存储是否丢失跟踪
   * @return 错误码，成功返回OK
   *
   * 通过SPI接口从MT6701传感器读取当前角度值和状态信息。
   * SPI模式支持完整的状态信息读取。
   */
  Error ReadRaw(uint16_t* angle_raw);

 private:
  uint16_t transfer16(uint16_t outValue);
};

}  // namespace MA330
}  // namespace hortor_servo
