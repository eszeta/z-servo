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

#include "regmap.h"
#include "regmap_spi_bus.h"
#include "servo/encoder.h"
#include "servo/types.h"

namespace hortor::drivers::MA330 {
constexpr uint8_t kResolutionBits = 14;
/**
 * @brief MT6701磁性角度传感器实现类
 *
 * 该类封装了MT6701磁性角度传感器的功能，继承自通用传感器基类。
 * MT6701是一款高精度、低功耗的磁性角度传感器，提供14位分辨率的角度测量。
 * 本实现使用I2C接口与传感器通信，支持角度读取和状态查询。
 */
class MA330 final : public servo::Encoder<MA330, kResolutionBits> {
 public:
  using EncoderBase = servo::Encoder<MA330, kResolutionBits>;

  struct Config : public EncoderBase::Config {
    SPIClass *spi;
    uint8_t cs_pin;
  };
  
  /**
   * @brief 初始化传感器
   * @param wire I2C通信接口指针
   * @return 错误码，成功返回OK
   *
   * 配置并初始化MT6701传感器，建立I2C通信，并执行基类初始化。
   * 必须在使用传感器前调用此方法。
   */
  Error Init(const Config &config) {
    CHECK(regmap_.Init(
        config.spi, config.cs_pin, SPISettings(1000000, MSBFIRST, SPI_MODE3)));
    CHECK((servo::Encoder<MA330, kResolutionBits>::Init(config)));
    return Error::kOk;
  }

  /**
   * @brief 获取原始角度值
   * @return 传感器的原始角度值（14位分辨率，范围0-16383）
   *
   * 通过I2C接口读取MT6701传感器的当前角度值。
   * 该方法实现了基类的纯虚函数。
   */
  Error GetRawImpl(uint16_t &out_raw) {
    CHECK(regmap_.ReadRaw(out_raw));
    return Error::kOk;
  }

  /**
   * @brief 获取寄存器映射实例
   * @return 寄存器映射实例指针
   *
   * 返回寄存器映射实例的指针，用于直接操作传感器的配置和状态。
   */
  RegMap *GetRegMap() { return &regmap_; }

 private:
  /** @brief 寄存器映射实例（包含SPI通信层），负责与传感器的具体通信操作 */
  RegMap regmap_;
};

}  // namespace hortor::drivers::MA330