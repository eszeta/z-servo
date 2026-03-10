// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "error.h"
#include "noncopyable.h"
#include "protocol.h"

namespace hortor::protocol {

/**
 * @brief 传输层接口（CRTP 模式）
 *
 * 纯字节读写，无协议解析。派生类实现具体传输（I2C/Serial 等）。
 *
 * @tparam DerivedType 派生类类型
 */
template <typename DerivedType>
class Transport : public hortor::Noncopyable {
 public:
  /**
   * @brief 读取一字节
   * @param byte 输出字节
   * @return 成功返回 true，无数据返回 false
   */
  bool ReadByte(uint8_t& byte) { return static_cast<DerivedType*>(this)->ReadByteImpl(byte); }

  /**
   * @brief 可读字节数
   */
  size_t Available() { return static_cast<DerivedType*>(this)->AvailableImpl(); }

  /**
   * @brief 写入字节
   * @param data 数据指针
   * @param size 字节数
   * @return 错误码
   */
  Error Write(const uint8_t* data, const size_t size) {
    return static_cast<DerivedType*>(this)->WriteImpl(data, size);
  }

  /**
   * @brief 消费已就绪的包（I2C 推模式：返回 true 并清除 packet_ready）
   */
  bool ConsumePacket() { return static_cast<DerivedType*>(this)->ConsumePacketImpl(); }

  /**
   * @brief 设置接收目标（I2C 推模式：OnReceive 中直接解析）
   */
  void SetReceiver(InstProtocol* protocol, InstPacket* inst_packet) {
    static_cast<DerivedType*>(this)->SetReceiverImpl(protocol, inst_packet);
  }
};

}  // namespace hortor::protocol
