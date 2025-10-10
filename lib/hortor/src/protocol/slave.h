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
#include "port_table_accessor.h"
#include "protocol.h"
#include "servo/servo.h"
#include "types.h"

namespace hortor::protocol {

class Slave {
 public:
  /**
   * @brief 构造函数
   */
  Slave() = default;

  /**
   * @brief 初始化
   * @return 错误码
   */
  Error Init();

  /**
   * @brief 链接访问器
   * @param accessor 指令访问器
   */
  Error LinkAccessor(PortTableAccessor *accessor);

  /**
   * @brief 链接传输接口
   * @param transport 指令传输接口
   */
  Error LinkPort(InstPortHandler *port);

  /**
   * @brief 处理指令
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error Process(float dt);

 private:
  /**
   * @brief 执行指令
   * @param data 指令数据
   * @return 错误码
   */
  Error Execute(const InstPacket &packet);

  /**
   * @brief 响应指令
   * @param reply_idx 响应索引
   * @param parameter 参数
   * @param parameter_size 参数大小
   * @return 错误码
   */
  Error Response(const uint8_t reply_idx,
                 const uint8_t *parameter,
                 const size_t parameter_size);
  /**
   * @brief 写寄存器
   * @param address 地址
   * @param data 数据
   * @param size 大小
   * @return 错误码
   */
  Error WriteRegs(const uint8_t address,
                  const uint8_t *data,
                  const size_t size);
  /**
   * @brief PING指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error PingHandler(const InstPacket &packet);
  /**
   * @brief 读取数据指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error ReadDataHandler(const InstPacket &packet, const bool response);
  /**
   * @brief 写数据指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error WriteDataHandler(const InstPacket &packet, const bool response);
  /**
   * @brief 写寄存器指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error RegWriteHandler(const InstPacket &packet, const bool response);
  /**
   * @brief 执行指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error ActionHandler(const InstPacket &packet, const bool response);
  /**
   * @brief 同步写指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error SyncWriteHandler(const InstPacket &packet, const bool response);
  /**
   * @brief 同步读指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error BulkReadHandler(const InstPacket &packet, const bool response);
  /**
   * @brief 恢复指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error ResetHandler(const InstPacket &packet, const bool response);

  bool CheckResponse(const uint8_t instruction,
                     const uint8_t return_level) const;
  /**
   * @brief 指令访问器
   */
  PortTableAccessor *accessor_ = nullptr;
  /**
   * @brief 指令传输接口
   */
  InstPortHandler *port_ = nullptr;
  /**
   * @brief 异步写缓冲区
   */
  uint8_t async_write_buffer_[128] = {};
  /**
   * @brief 异步写缓冲区大小
   */
  size_t buffer_size_ = 0;

  InstProtocol protocol_{};
  InstPacket inst_packet_{};
  StatusPacket status_packet_{};
};

}  // namespace hortor::protocol