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

#include "../utils/register/register_accessor.h"
#include "../utils/register/register_local_transport.h"
#include "./inst_types.h"

namespace hortor_servo {

class InstAccessor : public RegisterAccessor {
 public:
  using Regs = ServoRegs;

  Error Init();
  Error RecoveryEeprom();
  Error LoadEEPROM();
  Error StoreEeprom();
  Error ResetRam();

  /**
   * @brief Eeprom是否锁定
   * @return 是否锁定
   */
  bool GetWriteLock() {
    uint8_t lock;
    ReadRegField(Regs::kWriteLock, &lock);
    return lock == 1;
  }

  /**
   * @brief 获取舵机ID
   * @return 舵机ID
   */
  uint8_t GetId() {
    uint8_t id;
    ReadRegField(Regs::kId, &id);
    return id;
  }

  /**
   * @brief 获取固件主版本号
   * @return 固件主版本号
   */
  uint8_t GetFirmwareMajor() {
    uint8_t major;
    ReadRegField(Regs::kFirmwareMajor, &major);
    return major;
  }

  /**
   * @brief 获取固件次版本号
   * @return 固件次版本号
   */
  uint8_t GetFirmwareMinor() {
    uint8_t minor;
    ReadRegField(Regs::kFirmwareMinor, &minor);
    return minor;
  }

  /**
   * @brief 获取舵机状态
   * @return 舵机状态
   */
  uint8_t GetStatus() {
    uint8_t status;
    ReadRegField(Regs::kStatus, &status);
    return status;
  }

  /**
   * @brief 获取响应级别
   * @return 响应级别
   */
  bool GetResponseLevel() {
    uint8_t response_level;
    ReadRegField(Regs::kResponseLevel, &response_level);
    return response_level == 1;
  }

 private:
  RegisterLocalTransport local_transport_;
  uint8_t regs_[RegsBlocks::kTotal.size()] = {};
};
}  // namespace hortor_servo
