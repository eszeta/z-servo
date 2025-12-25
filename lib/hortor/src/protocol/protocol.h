// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "hortor.h"
#include "servo/types.h"
#include "types.h"

namespace hortor::protocol {

/**
 * @brief 指令包状态
 * DYNAMIXEL Protocol 1.0
 * https://emanual.robotis.com/docs/en/dxl/protocol1/
 */
enum class PacketState : uint8_t {
  kHeader1,
  kHeader2,
  kId,
  kLength,
  kInstructionOrError,
  kParameter,
  kChecksum,
};

struct __packed InstPacket {
  static constexpr uint8_t kBufferCapacity = 128;
  static constexpr uint8_t kParameterOffset = 5;
  union {
    uint8_t buffer[kBufferCapacity];
    struct {
      uint8_t header1;
      uint8_t header2;
      uint8_t id;
      uint8_t length;
      uint8_t instructionOrError;
      uint8_t parameter[kBufferCapacity - kParameterOffset];
    };
  };
  /**
   * @brief 获取ID
   * @param buffer 指令包
   * @return ID
   */
  uint8_t GetId() const { return buffer[PacketIndex::kId]; }

  /**
   * @brief 获取指令包长度
   * @param buffer 指令包
   * @return 指令包长度（包括ID、长度、指令/错误和参数的总长度）
   */
  uint8_t GetLength() const { return length; }

  /**
   * @brief 获取参数长度
   * @param buffer 指令包
   * @return 参数长度
   */
  uint8_t GetParameterSize() const {
    // 去掉 instruction/error 和 checksum
    return GetLength() - 2;
  }

  /**
   * @brief 设置参数长度
   * @param buffer 指令包
   * @param size 参数长度（不包括指令/错误和校验和）
   */
  void SetParameterSize(const uint8_t size) { length = size + 2; }

  /**
   * @brief 获取校验和
   * @param buffer 指令包
   * @return 校验和
   */
  uint8_t GetChecksum() const {
    const uint8_t parameter_size = GetParameterSize();
    return buffer[PacketIndex::kParameter + parameter_size];
  }

  /**
   * @brief 设置校验和
   * @param buffer 指令包
   */
  void SetChecksum(uint8_t checksum) {
    const uint8_t parameter_size = GetParameterSize();
    buffer[PacketIndex::kParameter + parameter_size] = checksum;
  }

  /**
   * @brief 获取指令包总大小
   * @param buffer 指令包
   * @return 指令包总大小（包括头部、ID、长度、指令/错误、参数和校验和）
   */
  size_t GetBufferSize() const {
    // Header(2Byte) + ID + Length = 4
    return length + 4;
  }

  /**
   * @brief 计算校验和
   * @param buffer 指令包
   * @return 校验和
   */
  // Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)
  uint8_t CalculateChecksum() const {
    uint8_t checksum = id + length + instructionOrError;
    const uint8_t l = GetParameterSize();
    for (uint8_t i = PacketIndex::kId; i < l; i++) {
      checksum += buffer[i];
    }
    return ~(static_cast<uint8_t>(checksum));
  }
};

typedef InstPacket StatusPacket;

class InstProtocol {
 public:
  InstProtocol();
  ~InstProtocol();

  Error Process(InstPacket& packet, const uint8_t recv_data, bool& is_complete);
  Error CreateResponse(const uint8_t id,
                       const StatusErrorBits& status,
                       const uint8_t* parameter,
                       const size_t parameter_size,
                       StatusPacket& packet);

 private:
  uint8_t param_pos_ = 0;
  PacketState packet_state_ = PacketState::kHeader1;
};

}  // namespace hortor::protocol