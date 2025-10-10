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

#include "slave.h"

#include <Arduino.h>

#include "port_table_accessor.h"
#include "utils/bit_utils.h"
namespace hortor::protocol {

Error Slave::Init() {
  buffer_size_ = 0;
  return Error::kOk;
}

Error Slave::LinkAccessor(PortTableAccessor *accessor) {
  accessor_ = accessor;
  return Error::kOk;
}

Error Slave::LinkPort(InstPortHandler *port) {
  port_ = port;
  return Error::kOk;
}

Error Slave::Process(float dt) {
  bool is_complete = false;
  CHECK(port_->Process(protocol_, dt, inst_packet_, is_complete));
  if (is_complete) {
    CHECK(Execute(inst_packet_));
  }
  return Error::kOk;
}

Error Slave::Response(const uint8_t reply_idx,
                      const uint8_t *parameter,
                      const size_t parameter_size) {
  uint8_t id = 0;
  uint8_t status = 0;
  CHECK(accessor_->GetServoId(id));
  CHECK(accessor_->GetStatus(status));
  CHECK(protocol_.CreateResponse(
      id, status, parameter, parameter_size, status_packet_));
  CHECK(port_->Response(status_packet_, reply_idx));
  return Error::kOk;
}

Error Slave::WriteRegs(const uint8_t address,
                       const uint8_t *data,
                       const size_t size) {
  if (data == nullptr || size == 0) {
    return Error::kInvalidParameter;
  }
  CHECK(accessor_->WriteMultiple(address, data, size));
  CHECK(accessor_->StoreEeprom(address, size));
  return Error::kOk;
}

bool Slave::CheckResponse(const uint8_t instruction,
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

Error Slave::Execute(const InstPacket &packet) {
  const auto isBroadcast = packet.id == kBroadcastId;
  uint8_t id = 0;
  CHECK(accessor_->GetServoId(id));
  const auto isSelf = packet.id == id;
  if (!isSelf && !isBroadcast) {
    return Error::kOk;
  }
  const auto instruction = packet.instructionOrError;
  uint8_t return_level = 0;
  CHECK(accessor_->GetReturnLevel(return_level));
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

/**
 * @brief PING 指令处理函数
 * 当收到 PING 指令帧时，舵机返回一个包含 ID 号的应答包。
 * @param packet 指令包
 * @return 错误码
 */
Error Slave::PingHandler(const InstPacket &packet) {
  CHECK(Response(0, nullptr, 0));
  return Error::kOk;
}

/**
 * @brief 读取数据指令处理函数
 * @param packet 指令包
 * @return 错误码
 */
Error Slave::ReadDataHandler(const InstPacket &packet, const bool response) {
  const uint8_t address = packet.parameter[0];
  const uint8_t size = packet.parameter[1];
  uint8_t buffer[128];
  CHECK(accessor_->ReadMultiple(address, size, buffer));
  if (response) {
    CHECK(Response(0, buffer, size));
  }
  return Error::kOk;
}

/**
 * @brief 写入数据指令处理函数
 * @param packet 指令包
 * @return 错误码
 */
Error Slave::WriteDataHandler(const InstPacket &packet, const bool response) {
  const uint8_t address = packet.parameter[0];
  const uint8_t size = packet.GetParameterSize() - 1;
  CHECK(WriteRegs(address, packet.parameter + 1, size));
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

/**
 * @brief 寄存器写入指令处理函数
 * REG WRITE 指令相似于 WRITE DATA，只是执行的时间不同。
 * 当收到 REG WRITE 指令帧时，把收到的数据储存在缓冲区备用，并把kAsyncWriteSt
 * 寄存器置 1。 当收到 ACTION指令后，储存的指令最终被执行。
 * @param packet 指令包
 * @return 错误码
 */
Error Slave::RegWriteHandler(const InstPacket &packet, const bool response) {
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
 * @brief 执行指令处理函数
 * @param packet 指令包
 * @return 错误码
 */
Error Slave::ActionHandler(const InstPacket &packet, const bool response) {
  const uint8_t *buffer = async_write_buffer_;
  while (buffer_size_ > 0) {
    const InstPacket *const reg_write_packet =
        reinterpret_cast<const InstPacket *>(buffer);
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
 * @brief 同步写入指令处理函数
 * 一条 SYNC WRITE 指令可一次修改多个舵机的控制表内容，
 * 而 REG WRITE+ACTION 指令是分步做到的。
 * 尽管如此，使用 SYNC WRITE 指令时，写入的数据长度和保存数据的首地址必须相同。
 * @param packet 指令包
 * @return 错误码
 */
Error Slave::SyncWriteHandler(const InstPacket &packet, const bool response) {
  const uint8_t address = packet.parameter[0];
  const uint8_t data_size = packet.parameter[1];
  const uint8_t parameter_size = packet.GetParameterSize();
  const uint8_t block_size = data_size + 1;
  const uint8_t block_count = (parameter_size - 2) / block_size;
  const uint8_t *parameter = packet.parameter + 2;
  for (uint8_t i = 0; i < block_count; i++) {
    const uint8_t target_id = parameter[0];
    uint8_t id = 0;
    CHECK(accessor_->GetServoId(id));
    if (id == target_id) {
      const uint8_t *data = parameter + 1;
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
 * @brief 同步读取指令处理函数
 * 一条 SYNC READ 指令可一次查询多个舵机的控制表内容，
 * 同步读指令中指定了需要查询舵机的 ID，
 * 舵机返回应答包顺序按指令包中 ID 顺序返回，
 * 使用 SYNC READ 指令时，所有查询的数据长度和数据的首地址必须相同。
 * @param packet 指令包
 * @return 错误码
 */
Error Slave::BulkReadHandler(const InstPacket &packet, const bool response) {
  const uint8_t address = packet.parameter[0];
  const uint8_t data_size = packet.parameter[1];
  const uint8_t parameter_size = packet.GetParameterSize();
  const uint8_t block_count = parameter_size - 2;
  bool hit = false;
  uint8_t response_idx = 0;
  uint8_t buffer[128];
  const uint8_t *parameter = packet.parameter + 2;
  for (uint8_t i = 0; i < block_count; i++) {
    const uint8_t target_id = parameter[0];
    uint8_t id = 0;
    CHECK(accessor_->GetServoId(id));
    if (id == target_id) {
      hit = true;
      response_idx = i;
      CHECK(accessor_->ReadMultiple(address, data_size, buffer));
    }
    parameter += 1;
  }
  if (response && hit) {
    CHECK(Response(response_idx, buffer, data_size));
  }
  return Error::kOk;
}

/**
 * @brief 恢复出厂设置指令处理函数
 * @param packet 指令包
 * @return 错误码
 */
Error Slave::ResetHandler(const InstPacket &packet, const bool response) {
  CHECK(accessor_->RecoveryEeprom());
  CHECK(accessor_->StoreEeprom());
  if (response) {
    CHECK(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

}  // namespace hortor::protocol