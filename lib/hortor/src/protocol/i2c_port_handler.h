// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

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
   * @brief 初始化
   * @param wire I2C对象
   * @return 错误码
   */
  Error Init(TwoWire* wire) {
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
  Error ProcessImpl(InstProtocol& protocol, const float dt,
                    InstPacket& inst_packet, bool& is_complete);

  /**
   * @brief 发送响应数据实现
   * @param packet 状态包
   * @param reply_idx 回复索引
   * @return 错误码
   */
  Error ResponseImpl(const StatusPacket& packet, const uint8_t reply_idx);

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
  TwoWire* wire_ = nullptr;
};

}  // namespace hortor::protocol

namespace hortor::protocol {

inline Error I2cPortHandler::ProcessImpl(InstProtocol& protocol, const float dt,
                                         InstPacket& inst_packet,
                                         bool&       is_complete) {
  while (wire_->available()) {
    uint8_t data = wire_->read();
    CHECK(protocol.Process(inst_packet, data, is_complete));
  }
  return Error::kOk;
}

inline Error I2cPortHandler::ResponseImpl(const StatusPacket& packet,
                                          const uint8_t       reply_idx) {
  const size_t size = packet.GetBufferSize();
  memcpy(status_packet_.buffer, packet.buffer, size);
  return Error::kOk;
}

inline Error I2cPortHandler::OnReceive(int howMany) {
  return Error::kOk;
}

inline Error I2cPortHandler::OnRequest() {
  const size_t size = status_packet_.GetBufferSize();
  wire_->write(status_packet_.buffer, size);
  return Error::kOk;
}

}  // namespace hortor::protocol