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

#include "monitor.h"

namespace hortor_servo {
void Monitor::UseMonitoring(Print &serial) { monitorPort_ = &serial; }

void Monitor::LinkMotor(Servo *servo) { servo_ = servo; }

void Monitor::Process(float dt) {
  if (!monitorPort_) return;
  static float accumulated_time = 0;
  accumulated_time += dt;
  if (accumulated_time < period_) return;
  accumulated_time = 0;

  bool printed = false;
  if (variables_ & static_cast<uint8_t>(MonitorBitmap::kTarget)) {
    monitorPort_->print(servo_->GetGoalPosition(), decimals_);
    printed = true;
  }

  if (variables_ & static_cast<uint8_t>(MonitorBitmap::kVoltage)) {
    if (printed) monitorPort_->print(separator_);
    monitorPort_->print(servo_->GetPresentLoad(), decimals_);
    printed = true;
  }

  if (variables_ & static_cast<uint8_t>(MonitorBitmap::kCurrent)) {
    if (printed) monitorPort_->print(separator_);
    monitorPort_->print(servo_->GetPresentCurrent(), decimals_);  // mAmps
    printed = true;
  }

  if (variables_ & static_cast<uint8_t>(MonitorBitmap::kVelocity)) {
    if (printed) monitorPort_->print(separator_);
    monitorPort_->print(servo_->GetPresentVelocity(), decimals_);
    printed = true;
  }

  if (variables_ & static_cast<uint8_t>(MonitorBitmap::kPosition)) {
    if (printed) monitorPort_->print(separator_);
    monitorPort_->print(servo_->GetPresentPosition(), decimals_);
    printed = true;
  }

  if (printed) {
    monitorPort_->println(separator_);
  }
}
}  // namespace hortor_servo
