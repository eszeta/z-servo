// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include <functional>

#include "hortor.h"
#include "protocol.h"

namespace hortor::protocol {

/**
 * @brief 指令端口处理器基类（CRTP 模式）
 *
 * 使用 CRTP 模式实现静态多态，消除虚函数调用开销。
 * 派生类需要实现 ProcessImpl() 和 ResponseImpl() 方法。
 *
 * @tparam Derived 派生类类型
 */
template <typename DerivedType>
class PortHandler : public hortor::Noncopyable {
 public:
  /**
   * @brief 处理数据
   * @param protocol 协议处理器
   * @param dt 时间间隔(秒)
   * @param inst_packet 指令包
   * @param is_complete 是否完成
   * @return 错误码
   */
  Error Process(InstProtocol& protocol, const float dt, InstPacket& inst_packet,
                bool& is_complete);

  /**
   * @brief 发送响应数据
   * @param packet 状态包
   * @param reply_idx 回复索引
   * @return 错误码
   */
  Error Response(const StatusPacket& packet, const uint8_t reply_idx);

  /**
   * @brief 设置响应延迟
   * @param response_delay 响应延迟时间（秒）
   */
  void SetResponseDelay(const float response_delay);

 protected:
  StatusPacket status_packet_{};        // 状态包缓冲区
  float        response_delay_ = 0.0f;  // 响应延迟（秒）
};

}  // namespace hortor::protocol

namespace hortor::protocol {

template <typename DerivedType>
Error PortHandler<DerivedType>::Process(InstProtocol& protocol, const float dt,
                                        InstPacket& inst_packet,
                                        bool&       is_complete) {
  return static_cast<DerivedType*>(this)->ProcessImpl(protocol, dt, inst_packet,
                                                      is_complete);
}

template <typename DerivedType>
Error PortHandler<DerivedType>::Response(const StatusPacket& packet,
                                         const uint8_t       reply_idx) {
  return static_cast<DerivedType*>(this)->ResponseImpl(packet, reply_idx);
}

template <typename DerivedType>
void PortHandler<DerivedType>::SetResponseDelay(const float response_delay) {
  response_delay_ = response_delay;
}

}  // namespace hortor::protocol
