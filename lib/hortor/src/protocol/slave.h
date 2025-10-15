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
template <typename Derived, typename RegMapType, typename PortHandlerType>
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
  Error Init() { return Error::kOk; }

  /**
   * @brief 处理指令
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error Process(float dt) {
    bool is_complete = false;
    CHECK(port_handler_.Process(protocol_, dt, inst_packet_, is_complete));
    if (is_complete) {
      CHECK(Execute(inst_packet_));
    }
    return Error::kOk;
  }

  /**
   * @brief 获取寄存器映射
   * @return 寄存器映射
   */
  RegMapType& GetRegMap() { return regmap_; }
  /**
   * @brief 获取端口处理器
   * @return 端口处理器
   */
  PortHandlerType& GetPortHandler() { return port_handler_; }

  /**
   * @brief 设置ID
   * @param id ID
   */
  void SetId(const uint8_t id) { id_ = id; }
  /**
   * @brief 获取ID
   * @return ID
   */
  uint8_t GetId() const { return id_; }
  /**
   * @brief 设置返回级别
   * @param return_level 返回级别
   */
  void SetReturnLevel(const uint8_t return_level) {
    return_level_ = return_level;
  }
  /**
   * @brief 获取返回级别
   * @return 返回级别
   */
  uint8_t GetReturnLevel() const { return return_level_; }
  /**
   * @brief 设置状态
   * @param status 状态
   */
  void SetStatus(const uint8_t status) { status_ = status; }
  /**
   * @brief 获取状态
   * @return 状态
   */
  uint8_t GetStatus() const { return status_; }

 protected:
  Derived& AsDerived() { return static_cast<Derived&>(*this); }
  const Derived& AsDerived() const {
    return static_cast<const Derived&>(*this);
  }
  /**
   * @brief 执行指令
   * @param data 指令数据
   * @return 错误码
   */
  Error Execute(const InstPacket& packet) {
    const auto isBroadcast = packet.id == kBroadcastId;
    const auto isSelf = packet.id == id_;
    if (!isSelf && !isBroadcast) {
      return Error::kOk;
    }
    const auto instruction = packet.instructionOrError;
    const auto response = CheckResponse(instruction, return_level_);

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

  /**
   * @brief 响应指令
   * @param reply_idx 响应索引
   * @param parameter 参数
   * @param parameter_size 参数大小
   * @return 错误码
   */
  Error Response(const uint8_t reply_idx,
                 const uint8_t* parameter,
                 const size_t parameter_size) {
    CHECK(AsDerived().ResponseImpl(reply_idx, parameter, parameter_size));
    CHECK(protocol_.CreateResponse(
        id_, status_, parameter, parameter_size, status_packet_));
    CHECK(port_handler_.Response(status_packet_, reply_idx));
    return Error::kOk;
  }
  /**
   * @brief 写寄存器
   * @param address 地址
   * @param data 数据
   * @param size 大小
   * @return 错误码
   */
  Error WriteRegs(const uint8_t address,
                  const uint8_t* data,
                  const size_t size) {
    if (data == nullptr || size == 0) {
      return Error::kInvalidParameter;
    }
    CHECK(regmap_.WriteBytes(address, data, size));
    return AsDerived().WriteRegsImpl(address, data, size);
  }
  /**
   * @brief PING指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error PingHandler(const InstPacket& packet) {
    CHECK(Response(0, nullptr, 0));
    return Error::kOk;
  }
  /**
   * @brief 读取数据指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error ReadDataHandler(const InstPacket& packet, const bool response) {
    const uint8_t address = packet.parameter[0];
    const uint8_t size = packet.parameter[1];
    uint8_t buffer[128];
    CHECK(regmap_.ReadBytes(address, size, buffer));
    if (response) {
      CHECK(Response(0, buffer, size));
    }
    return Error::kOk;
  }
  /**
   * @brief 写数据指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error WriteDataHandler(const InstPacket& packet, const bool response) {
    const uint8_t address = packet.parameter[0];
    const uint8_t size = packet.GetParameterSize() - 1;
    CHECK(WriteRegs(address, packet.parameter + 1, size));
    if (response) {
      CHECK(Response(0, nullptr, 0));
    }
    return Error::kOk;
  }
  /**
   * @brief 写寄存器指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error RegWriteHandler(const InstPacket& packet, const bool response) {
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
  /**
   * @brief 执行指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error ActionHandler(const InstPacket& packet, const bool response) {
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
  /**
   * @brief 同步写指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error SyncWriteHandler(const InstPacket& packet, const bool response) {
    const uint8_t address = packet.parameter[0];
    const uint8_t data_size = packet.parameter[1];
    const uint8_t parameter_size = packet.GetParameterSize();
    const uint8_t block_size = data_size + 1;
    const uint8_t block_count = (parameter_size - 2) / block_size;
    const uint8_t* parameter = packet.parameter + 2;
    for (uint8_t i = 0; i < block_count; i++) {
      const uint8_t target_id = parameter[0];
      if (id_ == target_id) {
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
  /**
   * @brief 同步读指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error BulkReadHandler(const InstPacket& packet, const bool response) {
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
      if (id_ == target_id) {
        hit = true;
        response_idx = i;
        CHECK(regmap_.ReadBytes(address, data_size, buffer));
      }
      parameter += 1;
    }
    if (response && hit) {
      CHECK(Response(response_idx, buffer, data_size));
    }
    return Error::kOk;
  }
  /**
   * @brief 恢复指令
   * @param packet 指令包
   * @param response 是否响应
   * @return 错误码
   */
  Error ResetHandler(const InstPacket& packet, const bool response) {
    if (response) {
      CHECK(Response(0, nullptr, 0));
    }
    return AsDerived().ResetImpl();
  }

  bool CheckResponse(const uint8_t instruction,
                     const uint8_t return_level) const {
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

  /**
   * @brief slaveID
   */
  uint8_t id_ = 0;
  /**
   * @brief 返回级别
   */
  uint8_t return_level_ = 0;
  /**
   * @brief 状态
   */
  uint8_t status_ = 0;
  /**
   * @brief 寄存器映射指针
   */
  RegMapType regmap_{};
  /**
   * @brief 指令传输接口
   */
  PortHandlerType port_handler_{};
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