// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "error.h"
#include "noncopyable.h"
namespace hortor {
/**
 * @brief 总线类型枚举
 *
 * 定义MT6701传感器支持的不同通信总线类型。
 */
enum class BusType : uint8_t {
  kI2C,  // I2C总线通信
  kSPI,  // SPI总线通信
};
}  // namespace hortor