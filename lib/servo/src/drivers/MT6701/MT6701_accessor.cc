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

#include "MT6701_accessor.h"

namespace hortor_servo {
namespace MT6701 {

Error MT6701Accessor::ReadRaw(uint16_t* angle_raw, Status* field_status, bool* button_pushed, bool* track_loss) {
  return read_raw_(angle_raw, field_status, button_pushed, track_loss);
}

Status MT6701Accessor::ReadFieldStatus() {
  Status status = Status::kFieldError;
  ReadRaw(nullptr, &status, nullptr, nullptr);
  return status;
}

Error MT6701Accessor::SetUvmMode(const uint8_t pairs) {
  CHECK_ERROR(SetUVWPolePair(pairs));
  CHECK_ERROR(SetMode(Mode::kUVW));
  return Error::kOk;
}

Error MT6701Accessor::SetAbzMode(const uint16_t pulses_per_round, const PulseWidth z_pulse_width,
                                 const Hyst hysteresis) {
  CHECK_ERROR(SetPulseWidth(z_pulse_width));
  CHECK_ERROR(SetHyst(hysteresis));
  CHECK_ERROR(SetABZPulsePerRound(pulses_per_round));
  CHECK_ERROR(SetMode(Mode::kABZ));
  return Error::kOk;
}

Error MT6701Accessor::SetNaNbNzEnable(const bool enable) {
  CHECK_ERROR(WriteRegField(MT6701Regs::kUVM_MUX, enable ? 1 : 0));
  return Error::kOk;
}

Error MT6701Accessor::SetAnalogMode(const float start, const float stop) {
  CHECK_ERROR(SetStartStop(start, stop));
  CHECK_ERROR(SetOutMode(OutMode::kAnalog));
  return Error::kOk;
}

Error MT6701Accessor::SetPwmMode(const PwmFreq frequency, const PwmPol polarity) {
  CHECK_ERROR(SetOutMode(OutMode::kPWM));
  CHECK_ERROR(SetPwmFreq(frequency));
  CHECK_ERROR(SetPwmPolarity(polarity));
  return Error::kOk;
}

Error MT6701Accessor::SetDirection(const Direction direction) {
  CHECK_ERROR(WriteRegField(MT6701Regs::kDIR, static_cast<uint8_t>(direction)));
  return Error::kOk;
}

Error MT6701Accessor::GetDirection(Direction* direction) {
  uint8_t value;
  CHECK_ERROR(ReadRegField(MT6701Regs::kDIR, &value));
  *direction = static_cast<Direction>(value);
  return Error::kOk;
}

Error MT6701Accessor::ProgramEEPROM() {
  CHECK_ERROR(Write(0x09, 0xB3));
  CHECK_ERROR(Write(0x0A, 0x05));
  delay(600);
  return Error::kOk;
}

Error MT6701Accessor::SetABZPulsePerRound(uint16_t pulses) {
  pulses--;
  if (pulses >= 1024) {
    return Error::kOutOfRange;
  }
  CHECK_ERROR(WriteRegField(MT6701Regs::kABZ_RES_8, MT6701Regs::kABZ_RES_0, pulses));
  return Error::kOk;
}

Error MT6701Accessor::SetUVWPolePair(uint8_t pairs) {
  pairs--;
  if (pairs >= 16) {
    return Error::kOutOfRange;
  }

  CHECK_ERROR(WriteRegField(MT6701Regs::kUVM_RES_0, pairs));
  return Error::kOk;
}

Error MT6701Accessor::SetMode(const Mode mode) {
  CHECK_ERROR(WriteRegField(MT6701Regs::kABZ_MUX, static_cast<uint8_t>(mode)));
  return Error::kOk;
}

Error MT6701Accessor::SetZeroRaw(uint16_t zero) {
  CHECK_ERROR(WriteRegField(MT6701Regs::kZERO_8, MT6701Regs::kZERO_0, zero));
  return Error::kOk;
}

Error MT6701Accessor::SetZero(const float zero) { return SetZeroRaw(static_cast<uint16_t>(zero * 4096 / 360.0f)); }

Error MT6701Accessor::SetHyst(const Hyst hysteresis) {
  CHECK_ERROR(WriteRegField(MT6701Regs::kHYST_2, MT6701Regs::kHYST_0, static_cast<uint16_t>(hysteresis)));
  return Error::kOk;
}

Error MT6701Accessor::SetStartStopRaw(uint16_t start, uint16_t stop) {
  // 检查参数范围
  if (start >= 4096 || stop >= 4096) {
    return Error::kOutOfRange;
  }

  // 检查起始和结束角度的合理性
  if (stop <= start) {
    return Error::kInvalidParameter;
  }

  CHECK_ERROR(WriteRegField(MT6701Regs::kA_START_8, MT6701Regs::kA_START_0, start));
  CHECK_ERROR(WriteRegField(MT6701Regs::kA_STOP_8, MT6701Regs::kA_STOP_0, stop));
  return Error::kOk;
}

Error MT6701Accessor::SetStartStop(const float start, const float stop) {
  uint16_t start_u16 = static_cast<uint16_t>(start * 4096 / 360.0f);
  uint16_t stop_u16 = static_cast<uint16_t>(stop * 4096 / 360.0f);
  start_u16 = start_u16 >= 4096 ? 4095 : start_u16;
  stop_u16 = stop_u16 >= 4096 ? 4095 : stop_u16;
  return SetStartStopRaw(start_u16, stop_u16);
}

Error MT6701Accessor::SetPulseWidth(const PulseWidth width) {
  CHECK_ERROR(WriteRegField(MT6701Regs::kPULSE_WIDTH, static_cast<uint8_t>(width)));
  return Error::kOk;
}

Error MT6701Accessor::GetPulseWidth(PulseWidth* width) {
  uint8_t value;
  CHECK_ERROR(ReadRegField(MT6701Regs::kPULSE_WIDTH, &value));
  *width = static_cast<PulseWidth>(value);
  return Error::kOk;
}

Error MT6701Accessor::SetPwmFreq(const PwmFreq freq) {
  CHECK_ERROR(WriteRegField(MT6701Regs::kPWM_FREQ, static_cast<uint8_t>(freq)));
  return Error::kOk;
}

Error MT6701Accessor::GetPwmFreq(PwmFreq* freq) {
  uint8_t value;
  CHECK_ERROR(ReadRegField(MT6701Regs::kPWM_FREQ, &value));
  *freq = static_cast<PwmFreq>(value);
  return Error::kOk;
}

Error MT6701Accessor::SetPwmPolarity(const PwmPol polarity) {
  CHECK_ERROR(WriteRegField(MT6701Regs::kPWM_POL, static_cast<uint8_t>(polarity)));
  return Error::kOk;
}

Error MT6701Accessor::GetPwmPolarity(PwmPol* polarity) {
  uint8_t value;
  CHECK_ERROR(ReadRegField(MT6701Regs::kPWM_POL, &value));
  *polarity = static_cast<PwmPol>(value);
  return Error::kOk;
}

Error MT6701Accessor::SetOutMode(const OutMode mode) {
  CHECK_ERROR(WriteRegField(MT6701Regs::kOUT_MODE, static_cast<uint8_t>(mode)));
  return Error::kOk;
}

Error MT6701Accessor::GetOutMode(OutMode* mode) {
  uint8_t value;
  CHECK_ERROR(ReadRegField(MT6701Regs::kOUT_MODE, &value));
  *mode = static_cast<OutMode>(value);
  return Error::kOk;
}

}  // namespace MT6701
}  // namespace hortor_servo