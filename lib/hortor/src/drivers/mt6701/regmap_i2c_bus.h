// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "base/types.h"
#include "hortor.h"
#include "regmap/regmap_i2c_bus.h"
#include "types.h"

namespace hortor::drivers::MT6701 {

/**
 * @brief I2C通信实现
 *
 * 实现通过I2C协议与MT6701传感器通信的功能。
 * I2C模式支持完整的寄存器读写操作，适用于配置和数据读取。
 */
class RegMapI2CBus : public regmap::RegMapI2CBus {
 public:
  /**
   * @brief 读取原始数据
   * @param angle_raw 原始角度值引用，用于存储读取的角度值
   * @param field_status 磁场状态引用，用于存储磁场状态（I2C模式下不支持）
   * @param button_pushed
   * 按钮状态引用，用于存储按钮是否被按下（I2C模式下不支持）
   * @param track_loss
   * 跟踪丢失状态引用，用于存储是否丢失跟踪（I2C模式下不支持）
   * @return 错误码，成功返回OK
   *
   * 通过I2C接口从MT6701传感器读取当前角度值。
   * 注意：I2C模式下只支持角度读取，不支持其他状态信息。
   */
  Error ReadRaw(uint16_t& angle_raw,
                Status& field_status,
                bool& button_pushed,
                bool& track_loss) {
    using kANGLE_6 = MT6701Regs::kANGLE_6;
    using kANGLE_0 = MT6701Regs::kANGLE_0;
    CHECK(ReadField<uint16_t, kANGLE_6, kANGLE_0>(angle_raw));
    // I2C模式下不支持这些状态读取
    (void)field_status;
    (void)button_pushed;
    (void)track_loss;

    return Error::kOk;
  }
};

}  // namespace hortor::drivers::MT6701