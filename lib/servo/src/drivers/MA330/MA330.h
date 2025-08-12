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

#include "MA330_accessor.h"
#include "MA330_spi_adapter.h"
#include "core/sensor.h"
#include "core/types.h"

namespace hortor_servo {
namespace MA330 {

/**
 * @brief MT6701磁性角度传感器实现类
 *
 * 该类封装了MT6701磁性角度传感器的功能，继承自通用传感器基类。
 * MT6701是一款高精度、低功耗的磁性角度传感器，提供14位分辨率的角度测量。
 * 本实现使用I2C接口与传感器通信，支持角度读取和状态查询。
 */
class MA330 final : public Sensor {
 public:
  explicit MA330();
  /**
   * @brief 初始化传感器
   * @param wire I2C通信接口指针
   * @return 错误码，成功返回OK
   *
   * 配置并初始化MT6701传感器，建立I2C通信，并执行基类初始化。
   * 必须在使用传感器前调用此方法。
   */
  Error InitSPI(SPIClass *spi, const uint8_t cs_pin);

  /**
   * @brief 获取原始角度值
   * @return 传感器的原始角度值（14位分辨率，范围0-16383）
   *
   * 通过I2C接口读取MT6701传感器的当前角度值。
   * 该方法实现了基类的纯虚函数。
   */
  uint16_t GetRaw() override;

  /**
   * @brief 获取控制器实例
   * @return 控制器实例指针
   *
   * 返回控制器实例的指针，用于直接操作传感器的配置和状态。
   */
  MA330Accessor *GetAccessor() { return &accessor_; }

 private:
  /** @brief 访问器实例，负责与传感器的具体通信操作 */
  MA330Accessor accessor_;

  /** @brief SPI通信实例，提供底层SPI接口 */
  MA330SpiAdapter spi_transport_;
};

}  // namespace MA330
}  // namespace hortor_servo
