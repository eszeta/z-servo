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
#include <Wire.h>

#include "../servo_types.h"
#include "./inst_transport_interface.h"

namespace hortor_servo {

class InstI2cTransport : public InstTransportInterface {
 public:
  /**
   * @brief 初始化
   * @param wire I2C对象
   * @param address I2C地址
   * @return 错误码
   */
  Error Init(TwoWire *wire) {
    wire_ = wire;
    return Error::kOk;
  }

  /**
   * @brief 处理数据
   * @param dt 时间间隔(微秒)
   * @return 错误码
   */
  Error Process(const uint32_t dt) override;

  /**
   * @brief 发送数据
   * @param reply_idx 回复索引
   * @param data 数据
   * @return 错误码
   */
  Error Response(const uint8_t reply_idx, const uint8_t *data) override;

  void OnReceive(int howMany);

  void OnRequest();

 private:
  TwoWire *wire_;
  uint8_t rx_buffer_[128];
  uint8_t tx_buffer_[128];
};
}  // namespace hortor_servo
