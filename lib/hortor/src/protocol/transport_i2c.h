// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file transport_i2c.h
 * @brief I2C 从机传输层（OnReceive 解析、OnRequest 发送）
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "protocol.h"
#include "transport.h"

namespace hortor::protocol {

/**
 * @brief I2C 从机传输层
 *
 * OnReceive 中直接解析到 inst_packet，无 rx 缓冲。
 * Write 存储指针，OnRequest 时发送，无 tx 缓冲。
 */
class TransportI2C : public Transport<TransportI2C> {
 public:
  /**
   * @brief 初始化，绑定 I2C 总线
   * @param wire TwoWire 实例（如 &Wire）
   * @return 错误码
   */
  Error Init(TwoWire* wire);

  /**
   * @brief I2C 推模式无轮询读，恒返回 false（CRTP 实现）
   * @param byte 未使用
   * @return 恒为 false
   */
  bool ReadByteImpl(uint8_t& byte);
  /** @brief I2C 推模式无可读字节数概念，恒返回 0（CRTP 实现） */
  size_t AvailableImpl();
  /**
   * @brief 保存发送指针，OnRequest 时写出（CRTP 实现）
   * @param data 待发送数据指针
   * @param size 字节数
   * @return 错误码
   */
  Error WriteImpl(const uint8_t* data, const size_t size);
  /** @brief 清除 packet_ready 标志（CRTP 实现） */
  bool ConsumePacketImpl();
  /**
   * @brief 设置 OnReceive 中解析目标（CRTP 实现）
   * @param protocol 解析器
   * @param inst_packet 指令包缓冲区
   */
  void SetReceiverImpl(InstProtocol* protocol, InstPacket* inst_packet);

  /**
   * @brief I2C 从机接收回调，解析字节并置位 packet_ready
   * @param howMany 本帧接收字节数
   */
  void OnReceive(int howMany);
  /** @brief I2C 从机发送回调，写出 tx_data_ */
  void OnRequest();

 private:
  TwoWire* wire_ = nullptr;  ///< 绑定的 I2C 总线

  InstProtocol* protocol_ptr_    = nullptr;  ///< 解析器
  InstPacket*   inst_packet_ptr_ = nullptr;  ///< 指令包缓冲区
  volatile bool packet_ready_    = false;    ///< 是否收完一整包

  const uint8_t* tx_data_ = nullptr;  ///< 待发送数据指针
  size_t         tx_size_ = 0;        ///< 待发送字节数
};

}  // namespace hortor::protocol

namespace hortor::protocol {

inline Error TransportI2C::Init(TwoWire* wire) {
  wire_ = wire;
  return Error::kOk;
}

inline bool TransportI2C::ReadByteImpl(uint8_t& byte) {
  (void)byte;
  return false;
}

inline size_t TransportI2C::AvailableImpl() {
  return 0;
}

inline Error TransportI2C::WriteImpl(const uint8_t* data, const size_t size) {
  tx_data_ = data;
  tx_size_ = size;
  return Error::kOk;
}

inline bool TransportI2C::ConsumePacketImpl() {
  if (!packet_ready_) {
    return false;
  }
  packet_ready_ = false;
  return true;
}

inline void TransportI2C::SetReceiverImpl(InstProtocol* protocol, InstPacket* inst_packet) {
  protocol_ptr_    = protocol;
  inst_packet_ptr_ = inst_packet;
}

inline void TransportI2C::OnReceive(int howMany) {
  (void)howMany;
  if (wire_ == nullptr || protocol_ptr_ == nullptr || inst_packet_ptr_ == nullptr) {
    return;
  }
  while (wire_->available()) {
    bool is_complete = false;
    protocol_ptr_->Process(*inst_packet_ptr_, wire_->read(), is_complete);
    if (is_complete) {
      packet_ready_ = true;
    }
  }
}

inline void TransportI2C::OnRequest() {
  if (wire_ != nullptr && tx_data_ != nullptr && tx_size_ > 0) {
    wire_->write(tx_data_, tx_size_);
  }
}

}  // namespace hortor::protocol
