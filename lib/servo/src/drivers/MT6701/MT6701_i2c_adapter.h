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

#include "MT6701_accessor.h"
#include "MT6701_types.h"
#include "core/types.h"
#include "register/register_i2c_adapter.h"

namespace hortor_servo {
namespace MT6701 {

/**
 * @brief I2C通信实现
 *
 * 实现通过I2C协议与MT6701传感器通信的功能。
 * I2C模式支持完整的寄存器读写操作，适用于配置和数据读取。
 */
class MT6701I2cAdapter : public RegisterI2CAdapter {
 public:
  /**
   * @brief 设置寄存器访问器函数
   * @param accessor 寄存器访问器
   * @return 错误码，成功返回OK
   */
  Error LinkAccessor(MT6701Accessor& accessor);

  /**
   * @brief 读取原始数据
   * @param angle_raw 原始角度值指针，用于存储读取的角度值
   * @param field_status 磁场状态指针，用于存储磁场状态（I2C模式下不支持）
   * @param button_pushed
   * 按钮状态指针，用于存储按钮是否被按下（I2C模式下不支持）
   * @param track_loss
   * 跟踪丢失状态指针，用于存储是否丢失跟踪（I2C模式下不支持）
   * @return 错误码，成功返回OK
   *
   * 通过I2C接口从MT6701传感器读取当前角度值。
   * 注意：I2C模式下只支持角度读取，不支持其他状态信息。
   */
  Error ReadRaw(uint16_t* angle_raw,
                Status* field_status,
                bool* button_pushed,
                bool* track_loss);
};

}  // namespace MT6701
}  // namespace hortor_servo
