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

#include "hortor.h"
#include "port_handler.h"
#include "protocol.h"

namespace hortor::protocol {

class I2cPortHandler : public PortHandler<I2cPortHandler> {
 public:
  /**
   * @brief 构造函数
   */
  I2cPortHandler() = default;

  /**
   * @brief 初始化
   * @param wire I2C对象
   * @return 错误码
   */
  Error Init(TwoWire *wire) {
    wire_ = wire;
    return Error::kOk;
  }

  /**
   * @brief 处理数据实现
   * @param protocol 协议处理器
   * @param dt 时间间隔(秒)
   * @param inst_packet 指令包
   * @param is_complete 是否完成
   * @return 错误码
   */
  Error ProcessImpl(InstProtocol &protocol,
                    const float dt,
                    InstPacket &inst_packet,
                    bool &is_complete);

  /**
   * @brief 发送响应数据实现
   * @param packet 状态包
   * @param reply_idx 回复索引
   * @return 错误码
   */
  Error ResponseImpl(const StatusPacket &packet, const uint8_t reply_idx);

  /**
   * @brief I2C 接收事件回调
   * @param howMany 接收的字节数
   * @return 错误码
   */
  Error OnReceive(int howMany);

  /**
   * @brief I2C 请求事件回调
   * @return 错误码
   */
  Error OnRequest();

 private:
  TwoWire *wire_ = nullptr;
};

}  // namespace hortor::protocol