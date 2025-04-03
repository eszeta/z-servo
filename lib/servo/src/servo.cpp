#include "servo.h"

#include "utils/math/math_types.h"

namespace hortor_servo {

void Servo::LinkDriver(MotorDriver *driver) { this->driver_ = driver; }

void Servo::LinkAngleSensor(Sensor *sensor) { this->angle_sensor_ = sensor; }

void Servo::LinkCurrentSense(Current *current_sense) {
  this->current_sense_ = current_sense;
}

void Servo::Init() {}

void Servo::Action() {
  if (target_position_ < min_position_ || target_position_ > max_position_) {
    action_ = false;
    return;
  }
  action_ = true;
}

void Servo::Process(uint32_t dt) {
  angle_sensor_->Process(dt);
  present_position_ = GetAngle(dt);
  present_velocity_ = GetVelocity(dt);
  present_current_ = GetCurrent(dt);
  if (!enabled_ || !action_) {
    SetPower(0);
    return;
  }
  switch (mode_) {
    case ServoMode::kPosition: {
      const auto pos_error = target_position_ - present_position_;
      if (IsPositionReached(pos_error)) {
        return;
      }
      moving_ = true;
      const auto pwm_set = pos_pid_.Compute(pos_error, dt);
      SetPower(pwm_set);
      break;
    }
    // case MotorMode::kPositionTorque: {
    //   position_set_ = constrain(target_, min_position_, max_position_);
    //   const float posPidOutput = pos_pid_.Compute(position_set_ -
    //   present_position_); current_set = maxCurrent_ * posPidOutput *
    //   torque_; pwm_set_ = current_pid_.Compute(current_set -
    //   present_current_, current_set); voltage_set_ = maxVoltage_ *
    //   pwm_set_; present_load_ = voltage_set_; setPower(pwm_set_); break;
    // }
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
  // 到达目标后是否保持扭矩
  if (torque_enable_) {
    SetPower(0);
  } else {
    Break();
  }
  moving_ = false;
  return true;
}

float Servo::GetAngle(uint32_t dt) {
  const auto direction = static_cast<float>(sensor_direction_);
  const auto angle = pos_lpf_.Compute(angle_sensor_->GetAngle(), dt);
  return direction * angle + position_correction_;
}

float Servo::GetVelocity(uint32_t dt) {
  const auto direction = static_cast<float>(sensor_direction_);
  const auto velocity = velocity_lpf_.Compute(angle_sensor_->GetVelocity(), dt);
  return direction * velocity;
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
