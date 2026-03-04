// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "base/types.h"
#include "regmap/i2c_plain.h"
#include "regmap/regmap.h"
#include "types.h"

namespace hortor::drivers::LSM6DSOW {

/**
 * @brief LSM6DSOW控制器类
 */
class RegMap : public regmap::RegMap<regmap::I2CPlain> {
 public:
  /**
   * @brief 初始化控制器
   * @param wire Arduino Wire对象指针
   * @param address 从机地址
   * @return 初始化结果
   */
  Error Init(TwoWire* wire, const int address) {
    CHECK(plain_.Init(wire, address));

    uint8_t value;
    using kWHO_AM_I = LSM6DSOWRegs::kWHO_AM_I;
    CHECK(ReadField<kWHO_AM_I>(value));
    VERIFY(value == 0x6C, Error::kErr);

    // set the gyroscope control register to work at 104 Hz, 2000 dps and in
    // bypass mode
    using kCTRL2_G = LSM6DSOWRegs::kCTRL2_G;
    CHECK(WriteField<kCTRL2_G>(0x4C));

    // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass
    // mode and enable ODR/4 low pass filter (check figure9 of LSM6DSOW's
    // datasheet)
    using kCTRL1_XL = LSM6DSOWRegs::kCTRL1_XL;
    CHECK(WriteField<kCTRL1_XL>(0x4A));

    // set gyroscope power mode to high performance and bandwidth to 16 MHz
    using kCTRL7_G = LSM6DSOWRegs::kCTRL7_G;
    CHECK(WriteField<kCTRL7_G>(0x00));

    // Set the ODR config register to ODR/4
    using kCTRL8_XL = LSM6DSOWRegs::kCTRL8_XL;
    CHECK(WriteField<kCTRL8_XL>(0x09));

    return Error::kOk;
  }

  /**
   * @brief 读取加速度数据
   * @param x 加速度X轴数据
   * @param y 加速度Y轴数据
   * @param z 加速度Z轴数据
   * @return 读取结果
   */
  Error ReadAcceleration(float& x, float& y, float& z) {
    const uint8_t size = 6;
    uint8_t data[size];
    using kOUTX_A = LSM6DSOWRegs::kOUTX_A;
    CHECK(Read(kOUTX_A::kAddress, size, data));
    const int16_t x_raw = static_cast<int16_t>(data[1] << 8 | data[0]);
    const int16_t y_raw = static_cast<int16_t>(data[3] << 8 | data[2]);
    const int16_t z_raw = static_cast<int16_t>(data[5] << 8 | data[4]);
    x = static_cast<float>(x_raw * 4.0 / 32768.0);
    y = static_cast<float>(y_raw * 4.0 / 32768.0);
    z = static_cast<float>(z_raw * 4.0 / 32768.0);
    return Error::kOk;
  }

  /**
   * @brief 检查加速度数据是否可用
   * @return 是否可用
   */
  bool AccelerationAvailable() {
    uint8_t value;
    using kXLDA = LSM6DSOWRegs::kXLDA;
    const Error err = ReadField<kXLDA>(value);
    if (err != Error::kOk) {
      return false;
    }
    return value & 0x01;
  }

  /**
   * @brief 读取陀螺仪数据
   * @param x 陀螺仪X轴数据,单位：度/秒
   * @param y 陀螺仪Y轴数据,单位：度/秒
   * @param z 陀螺仪Z轴数据,单位：度/秒
   * @return 读取结果
   */
  Error ReadGyroscope(float& x, float& y, float& z) {
    const uint8_t size = 6;
    uint8_t data[size];
    using kOUTX_L_G = LSM6DSOWRegs::kOUTX_L_G;
    CHECK(Read(kOUTX_L_G::kAddress, size, data));
    const int16_t x_raw = static_cast<int16_t>(data[1] << 8 | data[0]);
    const int16_t y_raw = static_cast<int16_t>(data[3] << 8 | data[2]);
    const int16_t z_raw = static_cast<int16_t>(data[5] << 8 | data[4]);
    x = static_cast<float>(x_raw * 2000.0 / 32768.0);
    y = static_cast<float>(y_raw * 2000.0 / 32768.0);
    z = static_cast<float>(z_raw * 2000.0 / 32768.0);
    return Error::kOk;
  }

  /**
   * @brief 检查陀螺仪数据是否可用
   * @return 是否可用
   */
  bool GyroscopeAvailable() {
    uint8_t value;
    using kGDA = LSM6DSOWRegs::kGDA;
    const Error err = ReadField<kGDA>(value);
    if (err != Error::kOk) {
      return false;
    }
    return value & 0x02;
  }

  /**
   * @brief 读取温度数据
   * @param temperature_deg 温度数据
   * @return 读取结果
   */
  Error ReadTemperature(float& temperature_deg) {
    const uint8_t size = 2;
    uint8_t data[size];
    using kOUT_TEMP = LSM6DSOWRegs::kOUT_TEMP;
    CHECK(Read(kOUT_TEMP::kAddress, size, data));
    const int16_t value = static_cast<int16_t>(data[1] << 8 | data[0]);
    temperature_deg = (static_cast<float>(value) / 256) + 25;
    return Error::kOk;
  }

  /**
   * @brief 检查温度数据是否可用
   * @return 是否可用
   */
  bool TemperatureAvailable() {
    uint8_t value;
    using kTDA = LSM6DSOWRegs::kTDA;
    const Error err = ReadField<kTDA>(value);
    if (err != Error::kOk) {
      return false;
    }
    return value & 0x04;
  }
};

}  // namespace hortor::drivers::LSM6DSOW
