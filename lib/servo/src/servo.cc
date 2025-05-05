#include "servo.h"

#include "utils/math/math.h"

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

void Servo::Process(float dt) {
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
        const auto target_vel_abs = std::fabs(target_velocity_);
        if (target_vel_abs > 0.001f) {
          // 目标速度不为0时，取位置PID输出和目标速度中较小的那个
          const auto speed_limit = std::fabs(pos_pid_vel);
          const auto limited_speed = std::fmin(target_vel_abs, speed_limit);
          target_vel = std::copysign(limited_speed, pos_error);
        }

        // 加速度限制
        if (target_acceleration_ > 0.001f) {
          const auto delta_v = std::fabs(target_acceleration_ * dt);
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
        const auto delta_v = std::fabs(target_acceleration_ * dt);
        target_vel = present_velocity_ + std::copysign(delta_v, target_vel);
      }
      const auto vel_error = target_vel - present_velocity_;
      const auto pwm_set = velocity_pid_.Compute(vel_error, dt);
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

bool Servo::IsPositionReached(int16_t pos_error) {
  if (pos_error > 0 && pos_error > cw_insensitive_area_) {
    return false;
  }
  if (pos_error < 0 && -pos_error > ccw_insensitive_area_) {
    return false;
  }
  return true;
}

float Servo::GetAngle(float dt) {
  const auto direction = static_cast<float>(sensor_direction_);
  const auto raw = angle_sensor_->GetAngle();
  const auto raw_mapped =
      mapResolution(raw, angle_sensor_->kResolution.kBits, kResolution.kBits);
  const auto filtered = pos_lpf_.Compute(raw_mapped, dt);
  const auto corrected = direction * filtered + position_correction_;
  return corrected;
}

float Servo::GetVelocity(float dt) {
  const auto direction = static_cast<float>(sensor_direction_);
  const auto raw = angle_sensor_->GetVelocity();
  const auto raw_mapped =
      mapResolution(raw, angle_sensor_->kResolution.kBits, kResolution.kBits);
  const auto filtered = velocity_lpf_.Compute(raw_mapped, dt);
  const auto corrected = direction * filtered;
  return corrected;
}

float Servo::GetCurrent(float dt) {
  const auto raw = current_sense_->GetCurrent();
  const auto filtered = current_lpf_.Compute(raw, dt);
  return filtered;
}

void Servo::SetPower(const float pwm) {
  present_load_ = pwm;
  const auto direction = static_cast<float>(motor_direction_);
  const auto pwm_set = direction * pwm;
  driver_->SetPWM(pwm_set);
}

void Servo::Break() { driver_->Break(); }
}  // namespace hortor_servo
