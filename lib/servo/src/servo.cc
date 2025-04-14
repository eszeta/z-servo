#include "servo.h"

#include "utils/math/math.h"
#include "utils/math/math_types.h"

namespace hortor_servo {

void Servo::LinkDriver(MotorDriver *driver) { driver_ = driver; }

void Servo::LinkAngleSensor(Sensor *sensor) { angle_sensor_ = sensor; }

void Servo::LinkCurrentSense(Current *current_sense) {
  current_sense_ = current_sense;
}

void Servo::Init() {}

void Servo::Action() {
  if (target_position_ >= min_position_ && target_position_ <= max_position_) {
    moving_ = true;
  }
}

void Servo::Process(uint32_t dt) {
  angle_sensor_->Process(dt);
  present_position_ = GetAngle(dt);
  present_velocity_ = GetVelocity(dt);
  present_current_ = GetCurrent(dt);
  if (!enabled_) return;
  switch (mode_) {
    case ServoMode::kPosition: {
      const auto pos_error = target_position_ - present_position_;

      if (IsPositionReached(pos_error)) {
        moving_ = false;
      }

      if (torque_enable_ || moving_) {
        // 始终计算位置PID输出作为速度上限
        const auto pos_pid_vel = pos_pid_.Compute(pos_error, dt);

        auto target_vel = pos_pid_vel;
        const auto target_vel_abs = std::abs(target_velocity_);
        if (target_vel_abs > 0.001f) {
          // 目标速度不为0时，取位置PID输出和目标速度中较小的那个
          const auto speed_limit = std::abs(pos_pid_vel);
          const auto limited_speed = std::min(target_vel_abs, speed_limit);
          target_vel = std::copysign(limited_speed, pos_error);
        }

        // 加速度限制
        if (target_acceleration_ > 0.001f) {
          const auto dt_s = dt * kMicroToSec;
          const auto delta_v = std::abs(target_acceleration_ * dt_s);
          target_vel = present_velocity_ + std::copysign(delta_v, pos_error);
        }
        const auto vel_error = target_vel - present_velocity_;
        const auto pwm_set = velocity_pid_.Compute(vel_error, dt);
        SetPower(pwm_set);
      }
      break;
    }
    case ServoMode::kVelocity: {
      auto target_vel = target_velocity_;
      // 加速度限制
      if (target_acceleration_ > 0.001f) {
        const auto dt_s = dt * kMicroToSec;
        const auto delta_v = std::abs(target_acceleration_ * dt_s);
        target_vel = present_velocity_ + std::copysign(delta_v, target_vel);
      }
      const auto vel_error = target_vel - present_velocity_;
      const auto pwm_set = velocity_pid_.Compute(vel_error, dt);
      SetPower(pwm_set);
      break;
    }
    default:
      SetPower(0);
  }
}

bool Servo::IsPositionReached(int16_t pos_error) {
  if (pos_error > 0 && pos_error > cw_insensitive_area_) {
    return false;
  }
  if (pos_error < 0 && -pos_error > ccw_insensitive_area_) {
    return false;
  }
  return true;
}

float Servo::GetAngle(uint32_t dt) {
  const auto direction = static_cast<float>(sensor_direction_);
  const auto raw = angle_sensor_->GetAngle();
  const auto filtered = pos_lpf_.Compute(raw, dt);
  const auto corrected = direction * filtered + position_correction_;
  const auto scaled =
      mapResolution(corrected, angle_sensor_->kResolution, kResolution);
  return scaled;
}

float Servo::GetVelocity(uint32_t dt) {
  const auto direction = static_cast<float>(sensor_direction_);
  const auto raw = angle_sensor_->GetVelocity();
  const auto filtered = velocity_lpf_.Compute(raw, dt);
  const auto corrected = direction * filtered;
  const auto scaled =
      mapResolution(corrected, angle_sensor_->kResolution, kResolution);
  return scaled;
}

float Servo::GetCurrent(uint32_t dt) {
  const auto current = current_lpf_.Compute(current_sense_->GetCurrent(), dt);
  return current;
}

void Servo::SetPower(const float power) {
  present_load_ = power;
  const auto direction = static_cast<float>(motor_direction_);
  driver_->SetPWM(direction * power);
}

void Servo::Break() { driver_->Break(); }
}  // namespace hortor_servo
