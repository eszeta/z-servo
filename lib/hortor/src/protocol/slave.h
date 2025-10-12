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
#include "protocol.h"
#include "regmap.h"
#include "servo/servo.h"
#include "types.h"

namespace hortor::protocol {

/**
 * @brief 协议从机类
 *
 * @tparam RegMapType 寄存器映射类型
 * @tparam PortHandlerType 端口处理器类型（CRTP 派生类）
 */
template <typename RegMapType, typename PortHandlerType>
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
   * @brief 链接寄存器映射
   * @param reg_map 寄存器映射指针
   * @return 错误码
   */
  Error LinkRegmap(RegMapType* reg_map);

  /**
   * @brief 链接传输接口
   * @param port 端口处理器指针
   * @return 错误码
   */
  Error LinkPort(PortHandlerType* port);

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
  Error Execute(const InstPacket& packet);

  /**
   * @brief 响应指令
   * @param reply_idx 响应索引
   * @param parameter 参数
   * @param parameter_size 参数大小
   * @return 错误码
   */
  Error Response(const uint8_t reply_idx,
                 const uint8_t* parameter,
                 const size_t parameter_size);
  /**
   * @brief 写寄存器
   * @param address 地址
   * @param data 数据
   * @param size 大小
   * @return 错误码
   */
  Error WriteRegs(const uint8_t address,
                  const uint8_t* data,
                  const size_t size);
  /**
   * @brief PING指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error PingHandler(const InstPacket& packet);
  /**
   * @brief 读取数据指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error ReadDataHandler(const InstPacket& packet, const bool response);
  /**
   * @brief 写数据指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error WriteDataHandler(const InstPacket& packet, const bool response);
  /**
   * @brief 写寄存器指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error RegWriteHandler(const InstPacket& packet, const bool response);
  /**
   * @brief 执行指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error ActionHandler(const InstPacket& packet, const bool response);
  /**
   * @brief 同步写指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error SyncWriteHandler(const InstPacket& packet, const bool response);
  /**
   * @brief 同步读指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error BulkReadHandler(const InstPacket& packet, const bool response);
  /**
   * @brief 恢复指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error ResetHandler(const InstPacket& packet, const bool response);

  bool CheckResponse(const uint8_t instruction,
                     const uint8_t return_level) const;
  /**
   * @brief 寄存器映射指针
   */
  RegMapType* regmap_ = nullptr;
  /**
   * @brief 指令传输接口
   */
  PortHandlerType* port_ = nullptr;
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

// ==================== 模板实现 ====================

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::Init() {
  buffer_size_ = 0;
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::LinkRegmap(RegMapType* regmap) {
  regmap_ = regmap;
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::LinkPort(PortHandlerType* port) {
  port_ = port;
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::Process(float dt) {
  bool is_complete = false;
  CHECK(port_->Process(protocol_, dt, inst_packet_, is_complete));
  if (is_complete) {
    CHECK(Execute(inst_packet_));
  }
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::Response(
    const uint8_t reply_idx,
    const uint8_t* parameter,
    const size_t parameter_size) {
  uint8_t id = 0;
  uint8_t status = 0;
  CHECK(regmap_->ServoId(id));
  CHECK(regmap_->Status(status));
  CHECK(protocol_.CreateResponse(
      id, status, parameter, parameter_size, status_packet_));
  CHECK(port_->Response(status_packet_, reply_idx));
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::WriteRegs(const uint8_t address,
                                                    const uint8_t* data,
                                                    const size_t size) {
  if (data == nullptr || size == 0) {
    return Error::kInvalidParameter;
  }
  CHECK(regmap_->WriteBytes(address, data, size));
  CHECK(regmap_->StoreEeprom(address, size));
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
bool Slave<RegMapType, PortHandlerType>::CheckResponse(
    const uint8_t instruction, const uint8_t return_level) const {
  switch (return_level) {
    case 0:
      return instruction == Instruction::kPing;
    case 1:
      return instruction == Instruction::kPing ||
             instruction == Instruction::kReadData ||
             instruction == Instruction::kBulkRead;
    case 2:
      return true;
  }
  return false;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::Execute(const InstPacket& packet) {
  const auto isBroadcast = packet.id == kBroadcastId;
  uint8_t id = 0;
  CHECK(regmap_->ServoId(id));
  const auto isSelf = packet.id == id;
  if (!isSelf && !isBroadcast) {
    return Error::kOk;
  }
  const auto instruction = packet.instructionOrError;
  uint8_t return_level = 0;
  CHECK(regmap_->ReturnLevel(return_level));
  const auto response = CheckResponse(instruction, return_level);

  switch (instruction) {
    case Instruction::kPing: {
      CHECK(PingHandler(packet));
      break;
    }
    case Instruction::kReadData: {
      CHECK(ReadDataHandler(packet, response));
      break;
    }
    case Instruction::kWriteData: {
      CHECK(WriteDataHandler(packet, response));
      break;
    }
    case Instruction::kRegWrite: {
      CHECK(RegWriteHandler(packet, response));
      break;
    }
    case Instruction::kAction: {
      CHECK(ActionHandler(packet, response));
      break;
    }
    case Instruction::kReset: {
      CHECK(ResetHandler(packet, response));
      break;
    }
    case Instruction::kSyncWrite: {
      CHECK(SyncWriteHandler(packet, response));
      break;
    }
    case Instruction::kBulkRead: {
      CHECK(BulkReadHandler(packet, response));
      break;
    }
    default: {
      return Error::kInvalidInstruction;
    }
  }
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::PingHandler(
    const InstPacket& packet) {
  CHECK(Response(0, nullptr, 0));
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::ReadDataHandler(
    const InstPacket& packet, const bool response) {
  const uint8_t address = packet.parameter[0];
  const uint8_t size = packet.parameter[1];
  uint8_t buffer[128];
  CHECK(regmap_->ReadBytes(address, size, buffer));
  if (response) {
    CHECK(Response(0, buffer, size));
  }
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::WriteDataHandler(
    const InstPacket& packet, const bool response) {
  const uint8_t address = packet.parameter[0];
  const uint8_t size = packet.GetParameterSize() - 1;
  CHECK(WriteRegs(address, packet.parameter + 1, size));
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::RegWriteHandler(
    const InstPacket& packet, const bool response) {
  const size_t size = packet.GetBufferSize();
  if (buffer_size_ + size > sizeof(async_write_buffer_)) {
    return Error::kArrayOutOfRange;
  }
  memcpy(async_write_buffer_ + buffer_size_, packet.buffer, size);
  buffer_size_ += size;
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::ActionHandler(
    const InstPacket& packet, const bool response) {
  const uint8_t* buffer = async_write_buffer_;
  while (buffer_size_ > 0) {
    const InstPacket* const reg_write_packet =
        reinterpret_cast<const InstPacket*>(buffer);
    const size_t packet_size = reg_write_packet->GetBufferSize();
    if (buffer_size_ < packet_size) {
      break;
    }
    const uint8_t address = reg_write_packet->parameter[0];
    const uint8_t size = reg_write_packet->GetParameterSize() - 1;
    CHECK(WriteRegs(address, reg_write_packet->parameter + 1, size));
    buffer_size_ -= packet_size;
    buffer += packet_size;
  }
  buffer_size_ = 0;
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::SyncWriteHandler(
    const InstPacket& packet, const bool response) {
  const uint8_t address = packet.parameter[0];
  const uint8_t data_size = packet.parameter[1];
  const uint8_t parameter_size = packet.GetParameterSize();
  const uint8_t block_size = data_size + 1;
  const uint8_t block_count = (parameter_size - 2) / block_size;
  const uint8_t* parameter = packet.parameter + 2;
  for (uint8_t i = 0; i < block_count; i++) {
    const uint8_t target_id = parameter[0];
    uint8_t id = 0;
    CHECK(regmap_->ServoId(id));
    if (id == target_id) {
      const uint8_t* data = parameter + 1;
      CHECK(WriteRegs(address, data, data_size));
    }
    parameter += block_size;
  }
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::BulkReadHandler(
    const InstPacket& packet, const bool response) {
  const uint8_t address = packet.parameter[0];
  const uint8_t data_size = packet.parameter[1];
  const uint8_t parameter_size = packet.GetParameterSize();
  const uint8_t block_count = parameter_size - 2;
  bool hit = false;
  uint8_t response_idx = 0;
  uint8_t buffer[128];
  const uint8_t* parameter = packet.parameter + 2;
  for (uint8_t i = 0; i < block_count; i++) {
    const uint8_t target_id = parameter[0];
    uint8_t id = 0;
    CHECK(regmap_->ServoId(id));
    if (id == target_id) {
      hit = true;
      response_idx = i;
      CHECK(regmap_->ReadBytes(address, data_size, buffer));
    }
    parameter += 1;
  }
  if (response && hit) {
    CHECK(Response(response_idx, buffer, data_size));
  }
  return Error::kOk;
}

template <typename RegMapType, typename PortHandlerType>
Error Slave<RegMapType, PortHandlerType>::ResetHandler(const InstPacket& packet,
                                                       const bool response) {
  CHECK(regmap_->RecoveryEeprom());
  CHECK(regmap_->StoreEeprom());
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

}  // namespace hortor::protocol