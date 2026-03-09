// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "hortor.h"
#include "protocol.h"
#include "transport.h"
#include "types.h"

namespace hortor::protocol {

/**
 * @brief 协议通道
 *
 * 组合 Transport + InstProtocol，完成收包解析与发包。
 * 持有 inst_packet、status_packet，供 Slave 执行与响应。
 *
 * @tparam TransportType 传输层类型（TransportI2C / TransportSerial）
 */
template <typename TransportType>
class ProtocolChannel : public hortor::Noncopyable {
 public:
  /**
   * @brief 初始化（设置 Transport 的接收目标并初始化传输层，需在 wire.begin 前调用）
   * @param args 转发给 Transport::Init 的参数（如 TwoWire* 或 HardwareSerial*）
   */
  template <typename... Args>
  Error Init(Args&&... args);

  /**
   * @brief 处理接收与发送
   * @param dt 时间间隔（秒）
   * @param is_complete 是否收到完整指令包
   * @return 错误码
   */
  Error Process(const float dt, bool& is_complete);

  /**
   * @brief 发送响应包
   * @param packet 状态包
   * @param reply_idx 回复索引（用于多回复延迟）
   * @return 错误码
   */
  Error Response(const StatusPacket& packet, const uint8_t reply_idx);

  /**
   * @brief 设置响应延迟（秒，Serial 多从机时使用）
   */
  void SetResponseDelay(const float response_delay);

  /**
   * @brief 获取解析后的指令包（只读）
   */
  const InstPacket& inst_packet() const { return inst_packet_; }

  /**
   * @brief 获取协议解析器（供 Slave 调用 CreateResponse）
   */
  InstProtocol* parser() { return &parser_; }

  TransportType*       transport() { return &transport_; }
  const TransportType* transport() const { return &transport_; }

 private:
  TransportType transport_;
  InstProtocol  parser_;
  InstPacket    inst_packet_{};
  StatusPacket  status_packet_{};

  float response_delay_   = 0.0f;
  float delay_remaining_  = 0.0f;
  bool  response_pending_ = false;
};

}  // namespace hortor::protocol

namespace hortor::protocol {

template <typename TransportType>
template <typename... Args>
Error ProtocolChannel<TransportType>::Init(Args&&... args) {
  transport_.SetReceiver(&parser_, &inst_packet_);
  return transport_.Init(std::forward<Args>(args)...);
}

template <typename TransportType>
Error ProtocolChannel<TransportType>::Process(const float dt, bool& is_complete) {
  is_complete = false;

  if (transport_.ConsumePacket()) {
    is_complete = true;
    return Error::kOk;
  }

  if (response_pending_) {
    delay_remaining_ -= dt;
    if (delay_remaining_ <= 0.0f) {
      response_pending_ = false;
      const size_t size = status_packet_.GetBufferSize();
      CHECK(transport_.Write(status_packet_.buffer, size));
    }
  }

  while (transport_.Available() > 0) {
    uint8_t byte = 0;
    if (!transport_.ReadByte(byte)) {
      break;
    }
    CHECK(parser_.Process(inst_packet_, byte, is_complete));
    if (is_complete) {
      return Error::kOk;
    }
  }
  return Error::kOk;
}

template <typename TransportType>
Error ProtocolChannel<TransportType>::Response(const StatusPacket& packet,
                                               const uint8_t       reply_idx) {
  memcpy(status_packet_.buffer, packet.buffer, packet.GetBufferSize());

  const float delay = response_delay_ * static_cast<float>(reply_idx + 1);
  if (delay <= 0.0f) {
    const size_t size = status_packet_.GetBufferSize();
    return transport_.Write(status_packet_.buffer, size);
  }
  delay_remaining_  = delay;
  response_pending_ = true;
  return Error::kOk;
}

template <typename TransportType>
void ProtocolChannel<TransportType>::SetResponseDelay(const float response_delay) {
  response_delay_ = response_delay;
}

}  // namespace hortor::protocol
