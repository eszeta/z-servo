// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "base/imu.h"
#include "regmap.h"
#include "types.h"

namespace hortor::drivers::LSM6DSOW {

class IMU;
using Base = servo::IMU<IMU>;

class IMU final : public Base {
 public:
  static constexpr uint8_t kI2CAddress = 0x6A;

  struct Config {
    TwoWire* wire;
  };

  Error Init(const Config& config);
  Error ReadAccelerationImpl(float& x, float& y, float& z);
  bool  AccelerationAvailableImpl();
  Error ReadGyroscopeImpl(float& x, float& y, float& z);
  bool  GyroscopeAvailableImpl();
  Error ReadTemperatureImpl(float& temperature_deg);
  bool  TemperatureAvailableImpl();

 private:
  Regmap regmap_;
};

}  // namespace hortor::drivers::LSM6DSOW

namespace hortor::drivers::LSM6DSOW {

inline Error IMU::Init(const Config& config) {
  CHECK(regmap_.Init(config.wire, kI2CAddress));
  return Error::kOk;
}

inline Error IMU::ReadAccelerationImpl(float& x, float& y, float& z) {
  return regmap_.ReadAcceleration(x, y, z);
}

inline bool IMU::AccelerationAvailableImpl() {
  return regmap_.AccelerationAvailable();
}

inline Error IMU::ReadGyroscopeImpl(float& x, float& y, float& z) {
  return regmap_.ReadGyroscope(x, y, z);
}

inline bool IMU::GyroscopeAvailableImpl() {
  return regmap_.GyroscopeAvailable();
}

inline Error IMU::ReadTemperatureImpl(float& temperature_deg) {
  return regmap_.ReadTemperature(temperature_deg);
}

inline bool IMU::TemperatureAvailableImpl() {
  return regmap_.TemperatureAvailable();
}

}  // namespace hortor::drivers::LSM6DSOW