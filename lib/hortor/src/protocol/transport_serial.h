// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file transport_serial.h
 * @brief 串口传输层（直接读写 Serial，无缓冲）
 */

#pragma once

#include <Arduino.h>

#include "protocol.h"
#include "transport.h"

namespace hortor::protocol {

/**
 * @brief 串口传输层，响应延迟由 ProtocolChannel 处理
 */
class TransportSerial : public Transport<TransportSerial> {
 public:
  /**
   * @brief 初始化，绑定串口
   * @param serial HardwareSerial 实例（如 &Serial1）
   * @return 错误码
   */
  Error Init(HardwareSerial* serial) {
    serial_ = serial;
    return Error::kOk;
  }

  /**
   * @brief 从串口读取一字节（CRTP 实现）
   * @param byte 输出字节
   * @return 有数据返回 true，否则 false
   */
  bool ReadByteImpl(uint8_t& byte);
  /** @brief 当前可读字节数（CRTP 实现） */
  size_t AvailableImpl();
  /**
   * @brief 写入字节到串口（CRTP 实现）
   * @param data 数据指针
   * @param size 字节数
   * @return 错误码
   */
  Error WriteImpl(const uint8_t* data, const size_t size);
  /// @brief 串口无包边界，恒返回 false（CRTP 实现）
  bool ConsumePacketImpl() { return false; }
  /// @brief 串口无需设置接收目标，空实现（CRTP 实现）
  void SetReceiverImpl(InstProtocol*, InstPacket*) {}

 private:
  HardwareSerial* serial_ = nullptr;  ///< 绑定的串口
};

}  // namespace hortor::protocol

namespace hortor::protocol {

inline bool TransportSerial::ReadByteImpl(uint8_t& byte) {
  if (serial_ == nullptr || serial_->available() == 0) {
    return false;
  }
  byte = serial_->read();
  return true;
}

inline size_t TransportSerial::AvailableImpl() {
  return serial_ != nullptr ? serial_->available() : 0;
}

inline Error TransportSerial::WriteImpl(const uint8_t* data, const size_t size) {
  if (serial_ == nullptr || data == nullptr) {
    return Error::kInvalidArg;
  }
  const size_t written = serial_->write(data, size);
  return written == size ? Error::kOk : Error::kIO;
}

}  // namespace hortor::protocol
