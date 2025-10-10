// // Copyright 2025 ES_ZETA
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

// #include "slave.h"

// #include <Arduino.h>

// #include "protocol/accessor.h"
// #include "protocol/inst_types.h"
// #include "utils/bit_utils.h"
// namespace hortor {

// Error Slave::Init() {
//   buffer_size_ = 0;

//   CHECK(LoadEepromConfig());
//   return Error::kOk;
// }

// Error Slave::LinkAccessor(InstAccessor *accessor) {
//   accessor_ = accessor;
//   return Error::kOk;
// }

// Error Slave::LinkPort(InstPortHandler *port) {
//   port_ = port;
//   return Error::kOk;
// }

// Error Slave::LinkServo(Servo *servo) {
//   servo_ = servo;
//   return Error::kOk;
// }

// Error Slave::Process(float dt) {
//   bool is_complete = false;
//   CHECK(port_->Process(protocol_, dt, inst_packet_, is_complete));
//   if (is_complete) {
//     CHECK(Execute(&inst_packet_));
//   }
//   CHECK(UpdateStatusRegs());
//   return Error::kOk;
// }

// Error Slave::Response(const uint8_t reply_idx,
//                      const uint8_t *parameter,
//                      const size_t parameter_size) {
//   const auto id = accessor_->GetId();
//   const auto status = accessor_->GetStatus();
//   CHECK(protocol_.CreateResponse(
//       id, status, parameter, parameter_size, status_packet_));
//   CHECK(port_->Response(status_packet_, reply_idx));
//   return Error::kOk;
// }

// Error Slave::WriteRegs(const uint8_t address,
//                       const uint8_t *data,
//                       const size_t size) {
//   if (data == nullptr || size == 0) {
//     return Error::kInvalidParameter;
//   }
//   CHECK(accessor_->WriteMultiple(address, data, size));
//   if (RegsBlocks::kEeprom.InBlock(address, size) ||
//       RegsBlocks::kInternalEeprom.InBlock(address, size)) {
//     if (!accessor_->GetWriteLock()) {
//       CHECK(accessor_->StoreEeprom());
//     }
//     CHECK(LoadEepromConfig());
//   }
//   if (RegsBlocks::kNormalRam.InBlock(address, size)) {
//     CHECK(LoadRamConfig());
//   }
//   CHECK(CheckAction(address, size));
//   return Error::kOk;
// }

// Error Slave::LoadEepromConfig() {
//   const auto response_delay = accessor_->GetResponseDelay();
//   port_->SetResponseDelay(response_delay);

//   const auto mode = accessor_->GetMode();
//   servo_->SetMode(mode);

//   const auto min_position = accessor_->GetMinPosition();
//   servo_->SetMinPosition(min_position);

//   const auto max_position = accessor_->GetMaxPosition();
//   servo_->SetMaxPosition(max_position);

//   const auto max_temperature = accessor_->GetMaxTemperature();
//   servo_->SetMaxTemperature(max_temperature);

//   const auto max_voltage = accessor_->GetMaxVoltage();
//   servo_->SetMaxVoltage(max_voltage);

//   const auto min_voltage = accessor_->GetMinVoltage();
//   servo_->SetMinVoltage(min_voltage);

//   const auto max_torque = accessor_->GetMaxTorque();
//   servo_->SetMaxTorque(max_torque);

//   // todo:kOption
//   // todo:kUnloadCondition

//   const auto min_startup_force = accessor_->GetMinStartupForce();
//   servo_->SetMinStartupForce(min_startup_force);

//   const auto cw_insensitive_area = accessor_->GetCWInsensitiveArea();
//   servo_->SetCWInsensitiveArea(cw_insensitive_area);

//   const auto ccw_insensitive_area = accessor_->GetCCWInsensitiveArea();
//   servo_->SetCCWInsensitiveArea(ccw_insensitive_area);

//   const auto current_protection_threshold =
//       accessor_->GetCurrentProtectionThreshold();
//   servo_->SetCurrentProtectionThreshold(current_protection_threshold);

//   const auto angular_resolution = accessor_->GetAngularResolution();
//   servo_->SetAngularResolution(angular_resolution);

//   const auto position_correction = accessor_->GetPositionCorrection();
//   servo_->SetPositionCorrection(position_correction);

//   const auto torque_protection_threshold =
//       accessor_->GetTorqueProtectionThreshold();
//   servo_->SetTorqueProtectionThreshold(torque_protection_threshold);

//   const auto torque_protection_time = accessor_->GetTorqueProtectionTime();
//   servo_->SetTorqueProtectionTime(torque_protection_time);

//   const auto overload_torque = accessor_->GetOverloadTorque();
//   servo_->SetOverloadTorque(overload_torque);

//   const auto overcurrent_protection_time =
//       accessor_->GetOvercurrentProtectionTime();
//   servo_->SetOvercurrentProtectionTime(overcurrent_protection_time);

//   auto &pos_pid = servo_->GetPosPid();
//   pos_pid.SetKp(accessor_->GetPosPidKp());
//   pos_pid.SetKi(accessor_->GetPosPidKi());
//   pos_pid.SetKd(accessor_->GetPosPidKd());
//   pos_pid.SetFf(accessor_->GetPosPidFf());
//   pos_pid.SetIlimit(accessor_->GetPosPidILimit());

//   auto &vel_pid = servo_->GetVelPid();
//   vel_pid.SetKp(accessor_->GetVelPidKp());
//   vel_pid.SetKi(accessor_->GetVelPidKi());

//   const auto sensor_direction = accessor_->GetSensorDirection();
//   servo_->SetSensorDirection(sensor_direction);

//   const auto motor_direction = accessor_->GetMotorDirection();
//   servo_->SetMotorDirection(motor_direction);

//   const auto pos_filter = accessor_->GetPosFilter();
//   servo_->GetPosLpf().SetTimeConstant(pos_filter);

//   const auto current_filter = accessor_->GetCurrentFilter();
//   servo_->GetCurrentLpf().SetTimeConstant(current_filter);

//   const auto vel_filter = accessor_->GetVelFilter();
//   servo_->GetVelocityLpf().SetTimeConstant(vel_filter);
//   return Error::kOk;
// }

// Error Slave::LoadRamConfig() {
//   const auto mode = accessor_->GetMode();

//   const auto torque_enable = accessor_->GetTorqueEnable();
//   servo_->SetTorqueEnable(torque_enable);

//   const auto target_acceleration = accessor_->GetGoalAcceleration();
//   servo_->SetGoalAcceleration(target_acceleration);

//   const auto target_position = accessor_->GetGoalPosition();
//   servo_->SetGoalPosition(target_position);

//   if (mode == ServoMode::kPwm) {
//     servo_->SetGoalTime(0);
//     servo_->SetGoalPwm(accessor_->GetGoalPwm());
//   } else {
//     servo_->SetGoalTime(accessor_->GetGoalTime());
//     servo_->SetGoalPwm(0);
//   }

//   const auto target_velocity = accessor_->GetGoalVelocity();
//   servo_->SetGoalVelocity(target_velocity);

//   const auto torque_limit = accessor_->GetTorqueLimit();
//   servo_->SetTorqueLimit(torque_limit);

//   return Error::kOk;
// }

// /**
//  * @brief 检查是否需要执行
//  * 如果修改了舵机目标位置，则需要执行
//  * @param address 寄存器地址
//  * @param size 寄存器大小
//  * @return 错误码
//  */
// Error Slave::CheckAction(const uint8_t address, const size_t size) {
//   if (RegsBlocks::kAction.InBlock(address, size)) {
//     servo_->Action();
//   }
//   return Error::kOk;
// }

// Error Slave::UpdateStatusRegs() {
//   const auto present_position = servo_->GetPresentPosition();
//   accessor_->SetPresentPosition(present_position);

//   const auto present_velocity = servo_->GetPresentVelocity();
//   accessor_->SetPresentVelocity(present_velocity);

//   const auto present_load = servo_->GetPresentLoad();
//   accessor_->SetPresentLoad(present_load);

//   const auto present_voltage = servo_->GetPresentVoltage();
//   accessor_->SetPresentVoltage(present_voltage);

//   const auto present_temperature = servo_->GetPresentTemperature();
//   accessor_->SetPresentTemperature(present_temperature);

//   const auto error_status = servo_->GetErrorStatus();
//   accessor_->SetStatus(error_status);

//   const auto moving = servo_->GetMoving();
//   accessor_->SetMoving(moving);

//   const auto present_current = servo_->GetPresentCurrent();
//   accessor_->SetPresentCurrent(present_current);

//   return Error::kOk;
// }

// Error Slave::Execute(const InstPacket *packet) {
//   const uint8_t instruction = packet->instructionOrError;
//   const uint8_t id = packet->id;
//   // 254(0XFE) 表示广播ID
//   if (id != accessor_->GetId() && id != 0xfe) {
//     return Error::kOk;
//   }
//   // 若控制器发出的 ID 号为 254，
//   // 所有的舵机均接收指令，
//   // 除 PING 指令外其它指令均不返回应答信息。
//   // (多个舵机连接在总线上不能使用广播 PING 指令)
//   const bool response_id = id == accessor_->GetId();
//   // 0:除读指令与PING指令外其它指令不返回应答包 1:对所有指令返回应答包
//   const bool response_level = accessor_->GetResponseLevel();
//   const bool response = response_id && response_level;
//   switch (instruction) {
//     case Instruction::kPing: {
//       CHECK(PingHandler(packet, response_id));
//       break;
//     }
//     case Instruction::kReadData: {
//       CHECK(ReadDataHandler(packet, response));
//       break;
//     }
//     case Instruction::kWriteData: {
//       CHECK(WriteDataHandler(packet, response));
//       break;
//     }
//     case Instruction::kRegWrite: {
//       CHECK(RegWriteHandler(packet, response));
//       break;
//     }
//     case Instruction::kAction: {
//       CHECK(ActionHandler(packet, response));
//       break;
//     }
//     case Instruction::kSyncWrite: {
//       CHECK(SyncWriteHandler(packet, response));
//       break;
//     }
//     case Instruction::kSyncRead: {
//       CHECK(SyncReadHandler(packet, response));
//       break;
//     }
//     case Instruction::kRecovery: {
//       CHECK(RecoveryHandler(packet, response));
//       break;
//     }
//     case Instruction::kReset: {
//       CHECK(ResetHandler(packet, response));
//       break;
//     }
//     default: {
//       return Error::kInvalidInstruction;
//     }
//   }
//   return Error::kOk;
// }

// /**
//  * @brief PING 指令处理函数
//  * 当收到 PING 指令帧时，舵机返回一个包含 ID 号的应答包。
//  * @param packet 指令包
//  * @param response 是否响应
//  * @return 错误码
//  */
// Error Slave::PingHandler(const InstPacket *packet, const bool response) {
//   if (response) {
//     CHECK(Response(0, nullptr, 0));
//   }
//   return Error::kOk;
// }

// /**
//  * @brief 读取数据指令处理函数
//  * @param packet 指令包
//  * @param response 是否响应
//  * @return 错误码
//  */
// Error Slave::ReadDataHandler(const InstPacket *packet, const bool response) {
//   if (response) {
//     const uint8_t address = packet->parameter[0];
//     const uint8_t size = packet->parameter[1];
//     uint8_t buffer[128];
//     CHECK(accessor_->ReadMultiple(address, size, buffer));
//     CHECK(Response(0, buffer, size));
//   }
//   return Error::kOk;
// }

// /**
//  * @brief 写入数据指令处理函数
//  * @param packet 指令包
//  * @param response 是否响应
//  * @return 错误码
//  */
// Error Slave::WriteDataHandler(const InstPacket *packet, const bool response)
// {
//   const uint8_t address = packet->parameter[0];
//   const uint8_t size = packet->GetParameterSize() - 1;
//   CHECK(WriteRegs(address, packet->parameter + 1, size));
//   if (response) {
//     CHECK(Response(0, nullptr, 0));
//   }
//   return Error::kOk;
// }

// /**
//  * @brief 寄存器写入指令处理函数
//  * REG WRITE 指令相似于 WRITE DATA，只是执行的时间不同。
//  * 当收到 REG WRITE 指令帧时，把收到的数据储存在缓冲区备用，并把kAsyncWriteSt
//  * 寄存器置 1。 当收到 ACTION指令后，储存的指令最终被执行。
//  * @param packet 指令包
//  * @param response 是否响应
//  * @return 错误码
//  */
// Error Slave::RegWriteHandler(const InstPacket *packet, const bool response) {
//   const size_t size = packet->GetBufferSize();
//   if (buffer_size_ + size > sizeof(async_write_buffer_)) {
//     return Error::kArrayOutOfRange;
//   }
//   std::copy(packet->buffer,
//             packet->buffer + size,
//             async_write_buffer_ + buffer_size_);
//   buffer_size_ += size;
//   accessor_->SetAsyncWrite(true);
//   if (response) {
//     CHECK(Response(0, nullptr, 0));
//   }
//   return Error::kOk;
// }

// /**
//  * @brief 执行指令处理函数
//  * @param packet 指令包
//  * @param response 是否响应
//  * @return 错误码
//  */
// Error Slave::ActionHandler(const InstPacket *packet, const bool response) {
//   const uint8_t *buffer = async_write_buffer_;
//   while (buffer_size_ > 0) {
//     const InstPacket *const reg_write_packet =
//         reinterpret_cast<const InstPacket *>(buffer);
//     const size_t packet_size = reg_write_packet->GetBufferSize();
//     if (buffer_size_ < packet_size) {
//       break;
//     }
//     const uint8_t address = reg_write_packet->parameter[0];
//     const uint8_t size = reg_write_packet->GetParameterSize() - 1;
//     CHECK(WriteRegs(address, reg_write_packet->parameter + 1, size));
//     buffer_size_ -= packet_size;
//     buffer += packet_size;
//   }
//   buffer_size_ = 0;
//   accessor_->SetAsyncWrite(false);
//   if (response) {
//     CHECK(Response(0, nullptr, 0));
//   }
//   return Error::kOk;
// }

// /**
//  * @brief 同步写入指令处理函数
//  * 一条 SYNC WRITE 指令可一次修改多个舵机的控制表内容，
//  * 而 REG WRITE+ACTION 指令是分步做到的。
//  * 尽管如此，使用 SYNC WRITE
//  指令时，写入的数据长度和保存数据的首地址必须相同。
//  * @param packet 指令包
//  * @param response 是否响应
//  * @return 错误码
//  */
// Error Slave::SyncWriteHandler(const InstPacket *packet, const bool response)
// {
//   const uint8_t address = packet->parameter[0];
//   const uint8_t data_size = packet->parameter[1];
//   const uint8_t parameter_size = packet->GetParameterSize();
//   const uint8_t block_size = data_size + 1;
//   const uint8_t block_count = (parameter_size - 2) / block_size;
//   bool hit = false;
//   uint8_t response_idx = 0;
//   const uint8_t *parameter = packet->parameter + 2;
//   for (uint8_t i = 0; i < block_count; i++) {
//     const uint8_t id = parameter[0];
//     if (id == accessor_->GetId()) {
//       const uint8_t *data = parameter + 1;
//       hit = true;
//       response_idx = i;
//       CHECK(WriteRegs(address, data, data_size));
//     }
//     parameter += block_size;
//   }
//   if (response && hit) {
//     CHECK(Response(response_idx, nullptr, 0));
//   }
//   return Error::kOk;
// }

// /**
//  * @brief 同步读取指令处理函数
//  * 一条 SYNC READ 指令可一次查询多个舵机的控制表内容，
//  * 同步读指令中指定了需要查询舵机的 ID，
//  * 舵机返回应答包顺序按指令包中 ID 顺序返回，
//  * 使用 SYNC READ 指令时，所有查询的数据长度和数据的首地址必须相同。
//  * @param packet 指令包
//  * @param response 是否响应
//  * @return 错误码
//  */
// Error Slave::SyncReadHandler(const InstPacket *packet, const bool response) {
//   const uint8_t address = packet->parameter[0];
//   const uint8_t data_size = packet->parameter[1];
//   const uint8_t parameter_size = packet->GetParameterSize();
//   const uint8_t block_count = parameter_size - 2;
//   bool hit = false;
//   uint8_t response_idx = 0;
//   uint8_t buffer[128];
//   const uint8_t *parameter = packet->parameter + 2;
//   for (uint8_t i = 0; i < block_count; i++) {
//     const uint8_t id = parameter[0];
//     if (id == accessor_->GetId()) {
//       hit = true;
//       response_idx = i;
//       CHECK(accessor_->ReadMultiple(address, data_size, buffer));
//     }
//     parameter += 1;
//   }
//   if (response && hit) {
//     CHECK(Response(response_idx, buffer, data_size));
//   }
//   return Error::kOk;
// }

// /**
//  * @brief 恢复出厂设置指令处理函数
//  * @param packet 指令包
//  * @param response 是否响应
//  * @return 错误码
//  */
// Error Slave::RecoveryHandler(const InstPacket *packet, const bool response) {
//   CHECK(accessor_->RecoveryEeprom());
//   CHECK(accessor_->StoreEeprom());
//   if (response) {
//     CHECK(Response(0, nullptr, 0));
//   }
//   return Error::kOk;
// }

// /**
//  * @brief 重置舵机状态(重置舵机圈数)
//  * @param packet 指令包
//  * @param response 是否响应
//  * @return 错误码
//  */
// Error Slave::ResetHandler(const InstPacket *packet, const bool response) {
//   // todo: 重置舵机圈数
//   CHECK(accessor_->ResetRam());
//   if (response) {
//     CHECK(Response(0, nullptr, 0));
//   }
//   return Error::kOk;
// }

// }  // namespace hortor