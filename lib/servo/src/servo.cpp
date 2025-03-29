#include "servo.h"

namespace hortor_servo {

void Servo::LinkDriver(MotorDriver *driver) { this->driver_ = driver; }

void Servo::LinkAngleSensor(Sensor *sensor) { this->angle_sensor_ = sensor; }

void Servo::LinkCurrentSense(Current *current_sense) { this->current_sense_ = current_sense; }

void Servo::Init() {}

void Servo::Action() {
  if (target_position_ < min_position_ || target_position_ > max_position_) {
    action_flag_ = false;
    return;
  }
  action_flag_ = true;
}

void Servo::Process(uint32_t dt) {
  angle_sensor_->Process(dt);
  present_position_ = getAngle(dt);
  present_velocity_ = getVelocity(dt);
  present_current_ = getCurrent(dt);
  if (!enabled_ || !action_flag_) {
    setPower(0, true);
  } else {
    switch (mode_) {
      case MotorMode::kPosition: {
        position_set_ = target_position_;
        pwm_set_ = pos_pid_.Compute(position_set_ - present_position_, dt);
        voltage_set_ = max_voltage_ * pwm_set_;
        setPower(pwm_set_, false);
        break;
      }
      // case MotorMode::kPositionTorque: {
      //   position_set_ = constrain(target_, min_position_, max_position_);
      //   const float posPidOutput = pos_pid_.Compute(position_set_ - present_position_);
      //   current_set = maxCurrent_ * posPidOutput * torque_;
      //   pwm_set_ = current_pid_.Compute(current_set - present_current_, current_set);
      //   voltage_set_ = maxVoltage_ * pwm_set_;
      //   present_load_ = voltage_set_;
      //   setPower(pwm_set_);
      //   break;
      // }
      default:
        setPower(0, true);
    }
  }
}

float Servo::getAngle(uint32_t dt) {
  return static_cast<float>(sensor_direction_) * pos_lpf_.Compute(angle_sensor_->GetAngle(), dt) + position_correction_;
}

float Servo::getVelocity(uint32_t dt) {
  return static_cast<float>(sensor_direction_) * velocity_lpf_.Compute(angle_sensor_->GetVelocity(), dt);
}

float Servo::getCurrent(uint32_t dt) {
  return static_cast<float>(sensor_direction_) * current_lpf_.Compute(current_sense_->GetCurrent(), dt);
}

void Servo::setPower(const float power, const bool force) {
  if (this->enabled_ || force) {
    present_load_ = power;
    driver_->SetPWM(static_cast<float>(motor_direction_) * power);
  }
}
}  // namespace hortor_servo
