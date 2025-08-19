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

namespace hortor_servo {

/**
 * @brief 链接电机驱动器
 * @param driver 电机驱动器指针
 */
void Servo::LinkDriver(Motor *driver) { driver_ = driver; }

/**
 * @brief 链接角度传感器
 * @param sensor 角度传感器指针
 */
void Servo::LinkAngleSensor(Sensor *sensor) { angle_sensor_ = sensor; }

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
 * @brief 执行动作
 */
void Servo::Action() {
  if (target_position_ >= min_position_ && target_position_ <= max_position_) {
    moving_ = true;
  }
}

/**
 * @brief 处理舵机逻辑
 * @param dt 时间间隔(秒)
 */
void Servo::Process(float dt) {
  angle_sensor_->Process(dt);
  present_position_ = GetTotalCounts(dt);
  present_velocity_ = GetVelocity(dt);
  present_current_ = GetCurrent(dt);
  if (!enabled_) return;
  switch (mode_) {
    case ServoMode::kPosition: {
      const auto pos_error = target_position_ - present_position_;

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
      if (target_acceleration_ > kFloatThreshold) {
        const auto velocity_change = next_velocity - present_velocity_;
        const auto limited_acceleration = std::clamp(velocity_change,
                                                     -target_acceleration_ * dt,
                                                     target_acceleration_ * dt);
        next_velocity = present_velocity_ + limited_acceleration;
      }

      // 应用最高速度限制
      if (target_velocity_ > kFloatThreshold ||
          target_velocity_ < -kFloatThreshold) {
        const auto limited_velocity = std::abs(target_velocity_);
        next_velocity =
            std::clamp(next_velocity, -limited_velocity, limited_velocity);
      }

      // 速度环计算控制输出
      const auto velocity_error = next_velocity - present_velocity_;
      const auto pwm_set = velocity_pid_.Compute(velocity_error, dt);

      // 输出控制信号（PWM）
      SetPower(pwm_set);
      break;
    }
    case ServoMode::kVelocity: {
      auto next_velocity = target_velocity_;

      // 加速度限制：限制速度变化率
      if (target_acceleration_ > kFloatThreshold) {
        const auto velocity_change = next_velocity - present_velocity_;
        const auto limited_acceleration = std::clamp(velocity_change,
                                                     -target_acceleration_ * dt,
                                                     target_acceleration_ * dt);
        next_velocity = present_velocity_ + limited_acceleration;
      }

      const auto velocity_error = next_velocity - present_velocity_;
      const auto pwm_set = velocity_pid_.Compute(velocity_error, dt);
      SetPower(pwm_set);
      break;
    }
    case ServoMode::kPwm: {
      const auto pwm_set = target_pwm_;
      SetPower(pwm_set);
      break;
    }
    default:
      SetPower(0);
  }
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
 * @brief 获取当前位置（映射后的计数值）
 * @param dt 时间间隔(秒)
 * @return 当前位置值
 */
float Servo::GetTotalCounts(float dt) {
  const auto direction = static_cast<float>(sensor_direction_);
  const auto raw = angle_sensor_->GetTotalCounts();
  const auto raw_mapped =
      mapResolution(raw, angle_sensor_->kResolution.kBits, kResolution.kBits);
  const auto filtered = pos_lpf_.Compute(raw_mapped, dt);
  const auto corrected = direction * filtered + position_correction_;
  return corrected;
}

/**
 * @brief 获取当前速度
 * @param dt 时间间隔(秒)
 * @return 当前速度值
 */
float Servo::GetVelocity(float dt) {
  const auto direction = static_cast<float>(sensor_direction_);
  const auto raw = angle_sensor_->GetVelocity();
  const auto raw_mapped =
      mapResolution(raw, angle_sensor_->kResolution.kBits, kResolution.kBits);
  const auto filtered = velocity_lpf_.Compute(raw_mapped, dt);
  const auto corrected = direction * filtered;
  return corrected;
}

/**
 * @brief 获取当前电流
 * @param dt 时间间隔(秒)
 * @return 当前电流值
 */
float Servo::GetCurrent(float dt) {
  const auto raw = current_sense_->GetCurrent();
  const auto filtered = current_lpf_.Compute(raw, dt);
  return filtered;
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
}  // namespace hortor_servo
