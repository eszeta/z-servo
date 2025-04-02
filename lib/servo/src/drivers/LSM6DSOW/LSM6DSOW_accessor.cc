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

#include "LSM6DSOW_accessor.h"

namespace hortor_servo {
namespace LSM6DSOW {

using Regs = LSM6DSOWRegs;

Error LSM6DSOW_accessor::Init() {
  uint8_t value;
  CHECK(Read(Regs::kWHO_AM_I.address, &value));
  if (value != 0x6C) {
    return Error::kGeneralErr;
  }

  // set the gyroscope control register to work at 104 Hz, 2000 dps and in
  // bypass mode
  CHECK(Write(Regs::kCTRL2_G.address, 0x4C));

  // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass
  // mode and enable ODR/4 low pass filter (check figure9 of LSM6DSOW's
  // datasheet)
  CHECK(Write(Regs::kCTRL1_XL.address, 0x4A));

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  CHECK(Write(Regs::kCTRL7_G.address, 0x00));

  // Set the ODR config register to ODR/4
  CHECK(Write(Regs::kCTRL8_XL.address, 0x09));

  return Error::kOk;
}

Error LSM6DSOW_accessor::ReadAcceleration(float* x, float* y, float* z) {
  const uint8_t size = 6;
  uint8_t data[size];
  CHECK(ReadMultiple(Regs::kOUTX_A.address, size, data));
  const int16_t x_raw = static_cast<int16_t>(data[1] << 8 | data[0]);
  const int16_t y_raw = static_cast<int16_t>(data[3] << 8 | data[2]);
  const int16_t z_raw = static_cast<int16_t>(data[5] << 8 | data[4]);
  *x = static_cast<float>(x_raw * 4.0 / 32768.0);
  *y = static_cast<float>(y_raw * 4.0 / 32768.0);
  *z = static_cast<float>(z_raw * 4.0 / 32768.0);
  return Error::kOk;
}

bool LSM6DSOW_accessor::AccelerationAvailable() {
  uint8_t value;
  const Error err = ReadRegField(Regs::kXLDA, &value);
  if (err != Error::kOk) {
    return false;
  }
  return value & 0x01;
}

Error LSM6DSOW_accessor::ReadGyroscope(float* x, float* y, float* z) {
  const uint8_t size = 6;
  uint8_t data[size];
  CHECK(ReadMultiple(Regs::kOUTX_L_G.address, size, data));
  const int16_t x_raw = static_cast<int16_t>(data[1] << 8 | data[0]);
  const int16_t y_raw = static_cast<int16_t>(data[3] << 8 | data[2]);
  const int16_t z_raw = static_cast<int16_t>(data[5] << 8 | data[4]);
  *x = static_cast<float>(x_raw * 2000.0 / 32768.0);
  *y = static_cast<float>(y_raw * 2000.0 / 32768.0);
  *z = static_cast<float>(z_raw * 2000.0 / 32768.0);
  return Error::kOk;
}

bool LSM6DSOW_accessor::GyroscopeAvailable() {
  uint8_t value;
  const Error err = ReadRegField(Regs::kGDA, &value);
  if (err != Error::kOk) {
    return false;
  }
  return value & 0x02;
}

Error LSM6DSOW_accessor::ReadTemperature(float* temperature_deg) {
  const uint8_t size = 2;
  uint8_t data[size];
  CHECK(ReadMultiple(Regs::kOUT_TEMP.address, size, data));
  const int16_t value = static_cast<int16_t>(data[1] << 8 | data[0]);
  *temperature_deg = (static_cast<float>(value) / 256) + 25;
  return Error::kOk;
}

bool LSM6DSOW_accessor::TemperatureAvailable() {
  uint8_t value;
  const Error err = ReadRegField(Regs::kTDA, &value);
  if (err != Error::kOk) {
    return false;
  }
  return value & 0x04;
}
}  // namespace LSM6DSOW
}  // namespace hortor_servo
