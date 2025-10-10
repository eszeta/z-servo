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

#include "servo.h"

#include "math/math.h"

namespace hortor::servo {

/**
 * @brief 链接电机驱动器
 * @param driver 电机驱动器指针
 */
void Servo::LinkDriver(Motor *driver) { driver_ = driver; }

/**
 * @brief 链接角度传感器
 * @param sensor 角度传感器指针
 */
void Servo::LinkAngleSensor(Encoder *sensor) { encoder = sensor; }

/**
 * @brief 链接电流传感器
 * @param current_sense 电流传感器指针
 */
void Servo::LinkCurrentSense(Current *current_sense) {
  current_sense_ = current_sense;
}

/**
 * @brief 初始化舵机
 */
void Servo::Init() {}

/**
 * @brief 处理舵机逻辑
 * @param dt 时间间隔(秒)
 */
Error Servo::Process(float dt) {
  CHECK(RefreshPresentVariables(dt));

  if (!enabled_) {
    return Error::kOk;
  }

  switch (mode_) {
    case ServoMode::kPosition: {
      const auto pos_error = goal_position_ - present_position_;

      if (IsPositionReached(pos_error)) {
        moving_ = false;
      }

      if (!torque_enable_ && !moving_) {
        SetPower(0);
        break;
      }

      // 位置环计算目标速度
      const auto position_error = pos_pid_.Compute(pos_error, dt);

      auto next_velocity = position_error;

      // 应用加速度限制
      if (goal_acceleration_ > math::kFloatThreshold) {
        const auto velocity_change = next_velocity - present_velocity_;
        const auto limited_acceleration = constrain(
            velocity_change, -goal_acceleration_ * dt, goal_acceleration_ * dt);
        next_velocity = present_velocity_ + limited_acceleration;
      }

      // 应用最高速度限制
      if (goal_velocity_ > math::kFloatThreshold ||
          goal_velocity_ < -math::kFloatThreshold) {
        const auto limited_velocity = abs(goal_velocity_);
        next_velocity =
            constrain(next_velocity, -limited_velocity, limited_velocity);
      }

      // 速度环计算控制输出
      const auto velocity_error = next_velocity - present_velocity_;
      const auto pwm_set = velocity_pid_.Compute(velocity_error, dt);

      // 输出控制信号（PWM）
      SetPower(pwm_set);
      break;
    }
    case ServoMode::kVelocity: {
      auto next_velocity = goal_velocity_;

      // 加速度限制：限制速度变化率
      if (goal_acceleration_ > math::kFloatThreshold) {
        const auto velocity_change = next_velocity - present_velocity_;
        const auto limited_acceleration = constrain(
            velocity_change, -goal_acceleration_ * dt, goal_acceleration_ * dt);
        next_velocity = present_velocity_ + limited_acceleration;
      }

      const auto velocity_error = next_velocity - present_velocity_;
      const auto pwm_set = velocity_pid_.Compute(velocity_error, dt);
      SetPower(pwm_set);
      break;
    }
    case ServoMode::kPwm: {
      const auto pwm_set = goal_pwm_;
      SetPower(pwm_set);
      break;
    }
    default:
      SetPower(0);
  }
  return Error::kOk;
}

Error Servo::RefreshPresentVariables(float dt) {
  // 处理编码器
  CHECK(encoder->Process(dt));
  // 处理编码器PLL
  const auto encoder_pos_counts = encoder->GetPosCounts();
  const auto encoder_bits = encoder->kResolution.kBits;
  const auto encoder_pos_counts_mapped =
      math::mapResolution(encoder_pos_counts, encoder_bits, kResolution.kBits);
  CHECK(encoder_pll.Process(dt, encoder_pos_counts_mapped));

  // 获取当前位置
  const auto direction = static_cast<float>(encoder_direction_);
  present_position_ =
      encoder_pll.GetPosition() * direction + position_correction_;

  // 获取当前速度
  present_velocity_ = encoder_pll.GetVelocity() * direction;

  // 获取当前电流
  CHECK(current_sense_->GetCurrent(present_current_));
  present_current_ = current_lpf_.Compute(present_current_, dt);
  return Error::kOk;
}

/**
 * @brief 检查是否到达目标位置
 * @param pos_error 位置误差
 * @return 是否到达目标位置
 */
bool Servo::IsPositionReached(int16_t pos_error) {
  if (pos_error > 0 && pos_error > cw_insensitive_area_) {
    return false;
  }
  if (pos_error < 0 && -pos_error > ccw_insensitive_area_) {
    return false;
  }
  return true;
}

/**
 * @brief 设置电机功率
 * @param pwm PWM值
 */
void Servo::SetPower(const float pwm) {
  present_load_ = pwm;
  const auto direction = static_cast<float>(motor_direction_);
  const auto pwm_set = direction * pwm;
  driver_->SetPWM(pwm_set);
}

/**
 * @brief 电机刹车
 */
void Servo::Break() { driver_->Break(); }

}  // namespace hortor::servo
