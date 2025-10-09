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

#include <functional>

#include "core/types.h"
#include "protocol.h"
#include "types.h"
namespace hortor_servo {

class InstPortHandler {
 public:
  /**
   * @brief 处理数据
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  virtual Error Process(InstProtocol &protocol,
                        const float dt,
                        InstPacket &inst_packet,
                        bool &is_complete) = 0;

  /**
   * @brief 发送数据
   * @param reply_idx 回复索引
   * @param data 数据
   * @return 错误码
   */
  virtual Error Response(const StatusPacket &packet,
                         const uint8_t reply_idx) = 0;

  /**
   * @brief 设置响应延迟
   * 单位: 毫秒
   */
  void SetResponseDelay(const uint16_t response_delay) {
    response_delay_ = response_delay;
  }

 protected:
  uint16_t response_delay_ = 0;
};

}  // namespace hortor_servo
