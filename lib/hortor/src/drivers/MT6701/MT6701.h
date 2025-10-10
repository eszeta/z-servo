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

#include "MT6701_accessor.h"
#include "MT6701_i2c_adapter.h"
#include "servo/encoder.h"
#include "servo/types.h"
#include "types.h"

namespace hortor::drivers::MT6701 {

/**
 * @brief MT6701磁性角度传感器实现类
 *
 * 该类封装了MT6701磁性角度传感器的功能，继承自通用传感器基类。
 * MT6701是一款高精度、低功耗的磁性角度传感器，提供14位分辨率的角度测量。
 * 本实现使用I2C接口与传感器通信，支持角度读取和状态查询。
 */
class MT6701 final : public servo::Encoder {
 public:
  /**
   * @brief 构造函数
   *
   * 初始化MT6701传感器对象，设置分辨率为14位（0-16383范围）。
   */
  explicit MT6701() : servo::Encoder(14) {}

  /**
   * @brief 初始化传感器
   * @param wire I2C通信接口指针
   * @return 错误码，成功返回OK
   *
   * 配置并初始化MT6701传感器，建立I2C通信，并执行基类初始化。
   * 必须在使用传感器前调用此方法。
   */
  Error Init(TwoWire *wire);

  /**
   * @brief 获取原始角度值
   * @return 传感器的原始角度值（14位分辨率，范围0-16383）
   *
   * 通过I2C接口读取MT6701传感器的当前角度值。
   * 该方法实现了基类的纯虚函数。
   */
  Error GetRaw(uint16_t &out_raw) override;

  /**
   * @brief 获取控制器实例
   * @return 控制器实例指针
   *
   * 返回控制器实例的指针，用于直接操作传感器的配置和状态。
   */
  MT6701Accessor *GetAccessor() { return &accessor_; }

 private:
  /** @brief 访问器实例，负责与传感器的具体通信操作 */
  MT6701Accessor accessor_;

  /** @brief I2C通信实例，提供底层I2C接口 */
  MT6701I2cAdapter i2c_transport_;
};

}  // namespace hortor::drivers::MT6701