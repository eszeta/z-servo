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
#include "inst.h"

#include <Arduino.h>

#include "../utils/bit_utils.h"
#include "./inst_accessor.h"
#include "./inst_types.h"
namespace hortor_servo {

Error Inst::Init() {
  buffer_size_ = 0;

  CHECK_ERROR(LoadEepromConfig());
  return Error::kOk;
}

Error Inst::LinkAccessor(InstAccessor *accessor) {
  accessor_ = accessor;
  return Error::kOk;
}

Error Inst::LinkTransport(InstTransportInterface *transport) {
  transport_ = transport;
  CHECK_ERROR(transport_->SetExecute([this](uint8_t *data) -> Error { return this->Execute(data); }));
  return Error::kOk;
}

Error Inst::LinkServo(Servo *servo) {
  servo_ = servo;
  return Error::kOk;
}

Error Inst::Process(uint32_t dt) {
  CHECK_ERROR(transport_->Process(dt));
  CHECK_ERROR(UpdateStatusRegs());
  return Error::kOk;
}

Error Inst::Response(const uint8_t reply_idx, const uint8_t *parameter, const size_t parameter_size) {
  status_packet_.header = 0xffff;
  status_packet_.id = accessor_->GetId();
  if (parameter == nullptr) {
    status_packet_.SetParameterSize(0);
  } else {
    memcpy(status_packet_.parameter, parameter, parameter_size);
    status_packet_.SetParameterSize(parameter_size);
  }
  status_packet_.error = accessor_->GetStatus();
  status_packet_.SetChecksum();
  CHECK_ERROR(transport_->Response(reply_idx, tx_buffer_));
  return Error::kOk;
}

Error Inst::WriteRegs(const uint8_t address, const uint8_t *data, const size_t size) {
  if (data == nullptr || size == 0) {
    return Error::kInvalidParameter;
  }
  CHECK_ERROR(accessor_->WriteMultiple(address, data, size));
  if (RegsBlocks::kEeprom.InBlock(address, size) || RegsBlocks::kInternalEeprom.InBlock(address, size)) {
    if (!accessor_->GetWriteLock()) {
      CHECK_ERROR(accessor_->StoreEeprom());
    }
    CHECK_ERROR(LoadEepromConfig());
  }
  if (RegsBlocks::kNormalRam.InBlock(address, size)) {
    CHECK_ERROR(LoadRamConfig());
  }
  CHECK_ERROR(CheckAction(address, size));
  return Error::kOk;
}

Error Inst::LoadEepromConfig() {
  const auto response_delay = accessor_->GetUint8(ServoRegisters::kResponseDelay) * 2;
  transport_->SetResponseDelay(response_delay);

  const auto mode = static_cast<MotorMode>(accessor_->GetUint8(ServoRegisters::kMode));
  servo_->SetMode(mode);

  const auto min_position = accessor_->GetUint16(ServoRegisters::kMinPositionH, ServoRegisters::kMinPositionL);
  servo_->SetMinPosition(min_position);

  const auto max_position = accessor_->GetUint16(ServoRegisters::kMaxPositionH, ServoRegisters::kMaxPositionL);
  servo_->SetMaxPosition(max_position);

  const auto max_temperature = accessor_->GetUint8(ServoRegisters::kMaxTemperature);
  servo_->SetMaxTemperature(max_temperature);

  const auto max_voltage = accessor_->GetUint8(ServoRegisters::kMaxVoltage) * 0.1f;
  servo_->SetMaxVoltage(max_voltage);

  const auto min_voltage = accessor_->GetUint8(ServoRegisters::kMinVoltage) * 0.1f;
  servo_->SetMinVoltage(min_voltage);

  const auto max_torque = accessor_->GetUint16(ServoRegisters::kMaxTorqueH, ServoRegisters::kMaxTorqueL) * 0.001f;
  servo_->SetMaxTorque(max_torque);

  // todo:kOption
  // todo:kUnloadCondition

  const auto min_startup_force = accessor_->GetUint8(ServoRegisters::kMinStartupForce) * 0.001f;
  servo_->SetMinStartupForce(min_startup_force);

  const auto cw_insensitive_area = accessor_->GetUint8(ServoRegisters::kCWInsensitiveArea);
  servo_->SetCWInsensitiveArea(cw_insensitive_area);

  const auto ccw_insensitive_area = accessor_->GetUint8(ServoRegisters::kCCWInsensitiveArea);
  servo_->SetCCWInsensitiveArea(ccw_insensitive_area);

  const auto current_protection_threshold =
      accessor_->GetUint16(ServoRegisters::kCurrentProtectionThH, ServoRegisters::kCurrentProtectionThL) * 6.5f;
  servo_->SetCurrentProtectionThreshold(current_protection_threshold);

  const auto angular_resolution = accessor_->GetUint8(ServoRegisters::kAngularResolution);
  servo_->SetAngularResolution(angular_resolution);

  const auto position_correction =
      accessor_->GetUint16(ServoRegisters::kPositionCorrectionH, ServoRegisters::kPositionCorrectionL);
  // BIT11为方向位，表示正负方向，其他位可表示范围为0-2047步
  servo_->SetPositionCorrection(bit_utils::SignToTwos(position_correction, 11));

  const auto torque_protection_threshold = accessor_->GetUint8(ServoRegisters::kTorqueProtectionTh) * 0.01f;
  servo_->SetTorqueProtectionThreshold(torque_protection_threshold);

  const auto torque_protection_time = accessor_->GetUint8(ServoRegisters::kTorqueProtectionTime) * 10;
  servo_->SetTorqueProtectionTime(torque_protection_time);

  const auto overload_torque = accessor_->GetUint8(ServoRegisters::kOverloadTorque) * 0.01f;
  servo_->SetOverloadTorque(overload_torque);

  const auto overcurrent_protection_time = accessor_->GetUint8(ServoRegisters::kOvercurrentProtectionTime) * 10;
  servo_->SetOvercurrentProtectionTime(overcurrent_protection_time);

  auto &pos_pid = servo_->GetPosPid();
  pos_pid.SetKp(accessor_->GetUint8(ServoRegisters::kPosProportionalGain) * 0.1f);
  pos_pid.SetKi(accessor_->GetUint8(ServoRegisters::kPosIntegralGain) * 0.1f);
  pos_pid.SetKd(accessor_->GetUint8(ServoRegisters::kPosDerivativeGain) * 0.1f);
  pos_pid.SetFf(accessor_->GetUint8(ServoRegisters::kPosPidFf) * 0.1f);
  pos_pid.SetRamp(accessor_->GetUint8(ServoRegisters::kPosPidRamp) * 0.1f);
  if (mode == MotorMode::kPosition || mode == MotorMode::kStep) {
    pos_pid.SetLimit(accessor_->GetUint8(ServoRegisters::kIntegralLimit) * 4.0f);
  } else {
    pos_pid.SetLimit(0);
  }

  auto &vel_pid = servo_->GetVelPid();
  vel_pid.SetKp(accessor_->GetUint8(ServoRegisters::kVelocityProportionalGain) * 0.1f);
  vel_pid.SetKi(accessor_->GetUint8(ServoRegisters::kVelocityIntegralGain) * 0.1f);

  const auto motor_direction = accessor_->GetUint8(ServoRegisters::kMotorDirection);
  servo_->SetMotorDirection(motor_direction ? Direction::CCW : Direction::CW);

  const auto sensor_direction = accessor_->GetUint8(ServoRegisters::kSensorDirection);
  servo_->SetSensorDirection(sensor_direction ? Direction::CCW : Direction::CW);

  const auto pos_filter = accessor_->GetUint8(ServoRegisters::kPosFilter);
  servo_->GetPosLpf().SetTimeConstant(pos_filter * 0.001f);

  const auto current_filter = accessor_->GetUint8(ServoRegisters::kCurrentFilter);
  servo_->GetCurrentLpf().SetTimeConstant(current_filter * 0.001f);

  const auto velocity_filter = accessor_->GetUint8(ServoRegisters::kVelocityFilter);
  servo_->GetVelocityLpf().SetTimeConstant(velocity_filter * 0.001f);
  return Error::kOk;
}

Error Inst::LoadRamConfig() {
  const auto mode = static_cast<MotorMode>(accessor_->GetUint8(ServoRegisters::kMode));

  const auto torque_enable = accessor_->GetUint8(ServoRegisters::kTorqueEnable);
  servo_->SetTorqueEnable(torque_enable);

  const auto target_acceleration = accessor_->GetUint8(ServoRegisters::kTargetAcceleration) * 100;
  servo_->SetTargetAcceleration(target_acceleration);

  const auto target_position = accessor_->GetUint16(ServoRegisters::kTargetPositionH, ServoRegisters::kTargetPositionL);
  servo_->SetTargetPosition(bit_utils::SignToTwos(target_position, 15));

  const auto target_time = accessor_->GetUint16(ServoRegisters::kTargetTimeH, ServoRegisters::kTargetTimeL);
  if (mode == MotorMode::kPwm) {
    servo_->SetTargetTime(0);
    servo_->SetTargetPwm(bit_utils::SignToTwos(target_time, 10) * 0.01f);
  } else {
    servo_->SetTargetTime(target_time);
    servo_->SetTargetPwm(0);
  }

  const auto target_velocity = accessor_->GetUint16(ServoRegisters::kTargetVelocityH, ServoRegisters::kTargetVelocityL);
  servo_->SetTargetVelocity(bit_utils::SignToTwos(target_velocity, 15));

  const auto torque_limit = accessor_->GetUint16(ServoRegisters::kTorqueLimitH, ServoRegisters::kTorqueLimitL) * 0.001f;
  servo_->SetTorqueLimit(torque_limit);

  return Error::kOk;
}

/**
 * @brief 检查是否需要执行
 * 如果修改了舵机目标位置，则需要执行
 * @param address 寄存器地址
 * @param size 寄存器大小
 * @return 错误码
 */
Error Inst::CheckAction(const uint8_t address, const size_t size) {
  if (RegsBlocks::kAction.InBlock(address, size)) {
    servo_->Action();
  }
  return Error::kOk;
}

Error Inst::UpdateStatusRegs() {
  const auto present_position = servo_->GetPresentPosition();

  accessor_->WriteRegField(ServoRegisters::kPresentPositionH, ServoRegisters::kPresentPositionL,
                           bit_utils::TwosToSign(present_position, 15));

  const auto present_velocity = servo_->GetPresentVelocity();
  accessor_->WriteRegField(ServoRegisters::kPresentVelocityH, ServoRegisters::kPresentVelocityL,
                           bit_utils::TwosToSign(present_velocity, 15));

  const auto present_load = servo_->GetPresentLoad();
  accessor_->WriteRegField(ServoRegisters::kPresentLoadH, ServoRegisters::kPresentLoadL,
                           bit_utils::TwosToSign(present_load, 10));

  const auto present_voltage = servo_->GetPresentVoltage() / 0.1f;
  accessor_->WriteRegField(ServoRegisters::kPresentVoltage, static_cast<uint8_t>(present_voltage));

  const auto present_temperature = servo_->GetPresentTemperature();
  accessor_->WriteRegField(ServoRegisters::kPresentTemperature, static_cast<uint8_t>(present_temperature));

  const auto error_status = servo_->GetErrorStatus();
  accessor_->WriteRegField(ServoRegisters::kStatus, error_status);

  const auto moving = servo_->GetMoving();
  accessor_->WriteRegField(ServoRegisters::kMoving, moving);

  const auto present_current = servo_->GetPresentCurrent() / 6.5f;
  accessor_->WriteRegField(ServoRegisters::kPresentCurrentH, ServoRegisters::kPresentCurrentL,
                           static_cast<uint16_t>(present_current));

  return Error::kOk;
}

Error Inst::Execute(const uint8_t *data) {
  const InstPacket *const packet = reinterpret_cast<const InstPacket *>(data);
  const uint8_t instruction = packet->instruction;
  const uint8_t id = packet->id;
  // 254(0XFE) 表示广播ID
  if (id != accessor_->GetId() && id != 0xfe) {
    return Error::kOk;
  }
  // 若控制器发出的 ID 号为 254，
  // 所有的舵机均接收指令，
  // 除 PING 指令外其它指令均不返回应答信息。
  // (多个舵机连接在总线上不能使用广播 PING 指令)
  const bool response_id = id == accessor_->GetId();
  // 0:除读指令与PING指令外其它指令不返回应答包 1:对所有指令返回应答包
  const bool response_level = accessor_->GetResponseLevel();
  const bool response = response_id && response_level;
  switch (instruction) {
    case Instruction::kPing: {
      CHECK_ERROR(PingHandler(packet, response_id));
      break;
    }
    case Instruction::kReadData: {
      CHECK_ERROR(ReadDataHandler(packet, response));
      break;
    }
    case Instruction::kWriteData: {
      CHECK_ERROR(WriteDataHandler(packet, response));
      break;
    }
    case Instruction::kRegWrite: {
      CHECK_ERROR(RegWriteHandler(packet, response));
      break;
    }
    case Instruction::kAction: {
      CHECK_ERROR(ActionHandler(packet, response));
      break;
    }
    case Instruction::kSyncWrite: {
      CHECK_ERROR(SyncWriteHandler(packet, response));
      break;
    }
    case Instruction::kSyncRead: {
      CHECK_ERROR(SyncReadHandler(packet, response));
      break;
    }
    case Instruction::kRecovery: {
      CHECK_ERROR(RecoveryHandler(packet, response));
      break;
    }
    case Instruction::kReset: {
      CHECK_ERROR(ResetHandler(packet, response));
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
 * @param response 是否响应
 * @return 错误码
 */
Error Inst::PingHandler(const InstPacket *packet, const bool response) {
  if (response) {
    CHECK_ERROR(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

/**
 * @brief 读取数据指令处理函数
 * @param packet 指令包
 * @param response 是否响应
 * @return 错误码
 */
Error Inst::ReadDataHandler(const InstPacket *packet, const bool response) {
  if (response) {
    const uint8_t address = packet->parameter[0];
    const uint8_t size = packet->parameter[1];
    uint8_t buffer[128];
    CHECK_ERROR(accessor_->ReadMultiple(address, size, buffer));
    CHECK_ERROR(Response(0, buffer, size));
  }
  return Error::kOk;
}

/**
 * @brief 写入数据指令处理函数
 * @param packet 指令包
 * @param response 是否响应
 * @return 错误码
 */
Error Inst::WriteDataHandler(const InstPacket *packet, const bool response) {
  const uint8_t address = packet->parameter[0];
  const uint8_t size = packet->GetParameterSize() - 1;
  CHECK_ERROR(WriteRegs(address, packet->parameter + 1, size));
  if (response) {
    CHECK_ERROR(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

/**
 * @brief 寄存器写入指令处理函数
 * REG WRITE 指令相似于 WRITE DATA，只是执行的时间不同。
 * 当收到 REG WRITE 指令帧时，把收到的数据储存在缓冲区备用，并把
 * kAsynchronousWriteSt 寄存器置 1。 当收到 ACTION
 * 指令后，储存的指令最终被执行。
 * @param packet 指令包
 * @param response 是否响应
 * @return 错误码
 */
Error Inst::RegWriteHandler(const InstPacket *packet, const bool response) {
  const size_t size = packet->GetBufferSize();
  if (buffer_size_ + size > sizeof(async_write_buffer_)) {
    return Error::kArrayOutOfRange;
  }
  memcpy(async_write_buffer_ + buffer_size_, tx_buffer_, size);
  buffer_size_ += size;
  CHECK_ERROR(accessor_->Write(ServoRegisters::kAsynchronousWriteSt.address, 1));
  if (response) {
    CHECK_ERROR(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

/**
 * @brief 执行指令处理函数
 * @param packet 指令包
 * @param response 是否响应
 * @return 错误码
 */
Error Inst::ActionHandler(const InstPacket *packet, const bool response) {
  const uint8_t *buffer = async_write_buffer_;
  while (buffer_size_ > 0) {
    const InstPacket *const reg_write_packet = reinterpret_cast<const InstPacket *>(buffer);
    const size_t packet_size = reg_write_packet->GetBufferSize();
    if (buffer_size_ < packet_size) {
      break;
    }
    const uint8_t address = reg_write_packet->parameter[0];
    const uint8_t size = reg_write_packet->GetParameterSize() - 1;
    CHECK_ERROR(WriteRegs(address, reg_write_packet->parameter + 1, size));
    buffer_size_ -= packet_size;
    buffer += packet_size;
  }
  buffer_size_ = 0;
  CHECK_ERROR(accessor_->Write(ServoRegisters::kAsynchronousWriteSt.address, 0));
  if (response) {
    CHECK_ERROR(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

/**
 * @brief 同步写入指令处理函数
 * 一条 SYNC WRITE 指令可一次修改多个舵机的控制表内容，
 * 而 REG WRITE+ACTION 指令是分步做到的。
 * 尽管如此，使用 SYNC WRITE 指令时，写入的数据长度和保存数据的首地址必须相同。
 * @param packet 指令包
 * @param response 是否响应
 * @return 错误码
 */
Error Inst::SyncWriteHandler(const InstPacket *packet, const bool response) {
  const uint8_t address = packet->parameter[0];
  const uint8_t data_size = packet->parameter[1];
  const uint8_t parameter_size = packet->GetParameterSize();
  const uint8_t block_size = data_size + 1;
  const uint8_t block_count = (parameter_size - 2) / block_size;
  bool hit = false;
  uint8_t response_idx = 0;
  const uint8_t *parameter = packet->parameter + 2;
  for (uint8_t i = 0; i < block_count; i++) {
    const uint8_t id = parameter[0];
    if (id == accessor_->GetId()) {
      const uint8_t *data = parameter + 1;
      hit = true;
      response_idx = i;
      CHECK_ERROR(WriteRegs(address, data, data_size));
    }
    parameter += block_size;
  }
  if (response && hit) {
    CHECK_ERROR(Response(response_idx, nullptr, 0));
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
 * @param response 是否响应
 * @return 错误码
 */
Error Inst::SyncReadHandler(const InstPacket *packet, const bool response) {
  const uint8_t address = packet->parameter[0];
  const uint8_t data_size = packet->parameter[1];
  const uint8_t parameter_size = packet->GetParameterSize();
  const uint8_t block_count = parameter_size - 2;
  bool hit = false;
  uint8_t response_idx = 0;
  uint8_t buffer[128];
  const uint8_t *parameter = packet->parameter + 2;
  for (uint8_t i = 0; i < block_count; i++) {
    const uint8_t id = parameter[0];
    if (id == accessor_->GetId()) {
      hit = true;
      response_idx = i;
      CHECK_ERROR(accessor_->ReadMultiple(address, data_size, buffer));
    }
    parameter += 1;
  }
  if (response && hit) {
    CHECK_ERROR(Response(response_idx, buffer, data_size));
  }
  return Error::kOk;
}

/**
 * @brief 恢复出厂设置指令处理函数
 * @param packet 指令包
 * @param response 是否响应
 * @return 错误码
 */
Error Inst::RecoveryHandler(const InstPacket *packet, const bool response) {
  CHECK_ERROR(accessor_->RecoveryEeprom());
  CHECK_ERROR(accessor_->StoreEeprom());
  if (response) {
    CHECK_ERROR(Response(0, nullptr, 0));
  }
  return Error::kOk;
}

/**
 * @brief 重置舵机状态(重置舵机圈数)
 * @param packet 指令包
 * @param response 是否响应
 * @return 错误码
 */
Error Inst::ResetHandler(const InstPacket *packet, const bool response) {
  // todo: 重置舵机圈数
  CHECK_ERROR(accessor_->ResetRam());
  if (response) {
    CHECK_ERROR(Response(0, nullptr, 0));
  }
  return Error::kOk;
}
}  // namespace hortor_servo