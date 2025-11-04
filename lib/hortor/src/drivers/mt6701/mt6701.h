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
#include <Wire.h>

#include "regmap.h"
#include "regmap_i2c_bus.h"
#include "regmap_spi_bus.h"
#include "servo/encoder.h"
#include "servo/types.h"
#include "types.h"

namespace hortor::drivers::MT6701 {

constexpr uint8_t kResolutionBits = 14;

/**
 * @brief MT6701磁性角度传感器通用模板声明
 *
 * 使用BusType枚举作为模板参数，通过模板偏特化为不同总线类型提供实现。
 * 支持I2C和SPI两种通信方式，每种方式有独立的Init方法签名。
 */
template <BusType bus_type>
class MT6701;

/**
 * @brief MT6701磁性角度传感器私有基类
 *
 * 包含所有与总线无关的公共代码，避免在特化中重复实现。
 * 使用CRTP模式继承servo::Encoder，提供统一的传感器接口。
 */
template <typename BusImpl>
class MT6701Base : public servo::Encoder<MT6701Base<BusImpl>, kResolutionBits> {
 public:
  using EncoderBase = servo::Encoder<MT6701Base<BusImpl>, kResolutionBits>;
  /**
   * @brief 获取控制器实例
   * @return 控制器实例引用
   *
   * 返回控制器实例的引用，用于直接操作传感器的配置和状态。
   */
  RegMap<BusImpl>& GetRegMap() { return regmap_; }

  /**
   * @brief 获取原始角度值
   * @param out_raw 输出的原始角度值（14位分辨率，范围0-16383）
   * @return 错误码，成功返回OK
   *
   * 通过总线接口读取MT6701传感器的当前角度值。
   * 该方法实现了基类的纯虚函数。
   */
  Error GetRawImpl(uint16_t& out_raw) {
    Status status = Status::kNormal;
    bool button_pushed = false;
    bool track_loss = false;
    CHECK(regmap_.ReadRaw(out_raw, status, button_pushed, track_loss));
    (void)status;
    (void)button_pushed;
    (void)track_loss;
    return Error::kOk;
  }

 protected:
  /** @brief 寄存器映射实例，负责与传感器的具体通信操作 */
  RegMap<BusImpl> regmap_;
};

/**
 * @brief MT6701磁性角度传感器I2C实现
 *
 * 该类封装了MT6701磁性角度传感器的I2C通信功能，继承自私有基类。
 * MT6701是一款高精度、低功耗的磁性角度传感器，提供14位分辨率的角度测量。
 * 本特化使用I2C接口与传感器通信，支持角度读取和状态查询。
 */
template <>
class MT6701<BusType::kI2C> final : public MT6701Base<RegMapI2CBus> {
 public:
  struct Config : public EncoderBase::Config {
    TwoWire* wire;
  };

  /**
   * @brief 初始化传感器（I2C模式）
   * @param wire I2C通信接口指针
   * @return 错误码，成功返回OK
   *
   * 配置并初始化MT6701传感器，建立I2C通信，并执行基类初始化。
   * 必须在使用传感器前调用此方法。
   */
  Error Init(const Config& config) {
    CHECK(regmap_.Init(config.wire, kI2CAddress));
    CHECK((MT6701Base<RegMapI2CBus>::Init(config)));
    return Error::kOk;
  }
};

/**
 * @brief MT6701磁性角度传感器SPI实现
 *
 * 该类封装了MT6701磁性角度传感器的SPI通信功能，继承自私有基类。
 * MT6701是一款高精度、低功耗的磁性角度传感器，提供14位分辨率的角度测量。
 * 本特化使用SPI接口与传感器通信，支持角度读取和状态查询。
 */
template <>
class MT6701<BusType::kSPI> final : public MT6701Base<RegMapSpiBus> {
 public:
  struct Config : public EncoderBase::Config {
    SPIClass* spi;
    int cs_pin;
    SPISettings spi_settings;
  };

  /**
   * @brief 初始化传感器（SPI模式）
   * @param spi SPI通信接口指针
   * @param cs_pin 片选引脚号
   * @param spi_settings SPI通信设置
   * @return 错误码，成功返回OK
   *
   * 配置并初始化MT6701传感器，建立SPI通信，并执行基类初始化。
   * 必须在使用传感器前调用此方法。
   */
  Error Init(const Config& config) {
    CHECK(regmap_.Init(config.spi, config.cs_pin, config.spi_settings));
    CHECK(MT6701Base<RegMapSpiBus>::Init(config));
    return Error::kOk;
  }
};

}  // namespace hortor::drivers::MT6701