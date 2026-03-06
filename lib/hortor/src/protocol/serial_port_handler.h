// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

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

class SerialPortHandler : public PortHandler<SerialPortHandler> {
 public:
  /**
   * @brief 初始化
   * @param serial 串口
   * @return 错误码
   */
  Error Init(HardwareSerial* serial) {
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
  Error ProcessImpl(InstProtocol& protocol, const float dt,
                    InstPacket& inst_packet, bool& is_complete);

  /**
   * @brief 发送响应数据实现
   * @param packet 状态包
   * @param reply_idx 回复索引
   * @return 错误码
   */
  Error ResponseImpl(const StatusPacket& packet, const uint8_t reply_idx);

 private:
  HardwareSerial* serial_           = nullptr;
  uint32_t        delay_time_       = 0;
  bool            response_pending_ = false;
};

}  // namespace hortor::protocol

namespace hortor::protocol {

inline Error SerialPortHandler::ProcessImpl(InstProtocol& protocol,
                                            const float   dt,
                                            InstPacket&   inst_packet,
                                            bool&         is_complete) {
  delay_time_ -= dt;
  if (delay_time_ <= 0 && response_pending_) {
    response_pending_        = false;
    const size_t buffer_size = status_packet_.GetBufferSize();
    const size_t size = serial_->write(status_packet_.buffer, buffer_size);
    if (size != buffer_size) {
      return Error::kIO;
    }
  }

  while (serial_->available()) {
    uint8_t data = serial_->read();
    CHECK(protocol.Process(inst_packet, data, is_complete));
  }
  return Error::kOk;
}

inline Error SerialPortHandler::ResponseImpl(const StatusPacket& packet,
                                             const uint8_t       reply_idx) {
  const size_t size = packet.GetBufferSize();
  memcpy(status_packet_.buffer, packet.buffer, size);
  delay_time_       = response_delay_ * (reply_idx + 1);
  response_pending_ = true;
  return Error::kOk;
}

}  // namespace hortor::protocol