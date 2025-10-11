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

#include "hortor.h"
#include "port_handler.h"
#include "protocol.h"
#include "types.h"

#ifdef ARDUINO_ARCH_STM32
#include <HardwareSerial.h>
#endif

namespace hortor::protocol {

class InstSerialPortHandler : public InstPortHandler<InstSerialPortHandler> {
 public:
  /**
   * @brief 构造函数
   */
  InstSerialPortHandler() = default;

  /**
   * @brief 初始化
   * @param serial 串口
   * @return 错误码
   */
  Error Init(HardwareSerial *serial) {
    serial_ = serial;
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

 private:
  HardwareSerial *serial_ = nullptr;
  uint32_t delay_time_ = 0;
  bool response_pending_ = false;
};

}  // namespace hortor::protocol