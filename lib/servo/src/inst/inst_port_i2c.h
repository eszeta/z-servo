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

#include "inst/inst_port.h"
#include "inst/inst_protocol.h"
#include "inst/inst_types.h"

namespace hortor_servo {

class InstPortI2c : public InstPort {
 public:
  /**
   * @brief 构造函数
   */
  InstPortI2c() = default;
  
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
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error Process(const float dt) override;

  /**
   * @brief 发送数据
   * @param reply_idx 回复索引
   * @param data 数据
   * @return 错误码
   */
  Error Response(const uint8_t reply_idx, const StatusPacket *packet) override;

  Error OnReceive(int howMany);

  Error OnRequest();

 private:
  static constexpr auto kBufferSize = 128;
  TwoWire *wire_ = nullptr;
  InstProtocol protocol_{};
  InstPacket inst_packet_{};
  StatusPacket status_packet_{};
};
}  // namespace hortor_servo
