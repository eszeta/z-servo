#pragma once

#include <Arduino.h>

#include "./core/current.h"
#include "./core/motor_driver.h"
#include "./core/sensor.h"
#include "./servo_types.h"
#include "./utils/math/lowpass_filter.h"
#include "./utils/math/pid.h"

namespace hortor_servo {

class Servo {
 public:
  Servo() = default;

  void Init();
  void LinkDriver(MotorDriver *driver);
  void LinkAngleSensor(Sensor *sensor);
  void LinkCurrentSense(Current *current_sense);
  void Action();
  void Process(uint32_t dt);

  float GetPresentPosition() { return present_position_; }
  float GetPresentVelocity() { return present_velocity_; }
  float GetPresentLoad() { return present_load_; }
  float GetPresentVoltage() { return present_voltage_; }
  float GetPresentTemperature() { return present_temperature_; }
  float GetPresentCurrent() { return present_current_; }
  uint8_t GetErrorStatus() { return error_status_; }
  bool GetMoving() { return moving_; }
  int16_t GetTargetPosition() { return target_position_; }
  ServoMode GetMode() { return mode_; }
  PidController &GetPosPid() { return pos_pid_; }
  PidController &GetVelPid() { return velocity_pid_; }
  LowPassFilter &GetCurrentLpf() { return current_lpf_; }
  LowPassFilter &GetVelocityLpf() { return velocity_lpf_; }
  LowPassFilter &GetPosLpf() { return pos_lpf_; }

  void SetMode(ServoMode mode) { mode_ = mode; }
  void SetMinPosition(uint16_t min_position) { min_position_ = min_position; }
  void SetMaxPosition(uint16_t max_position) { max_position_ = max_position; }
  void SetMaxTemperature(uint8_t max_temperature) {
    max_temperature_ = max_temperature;
  }
  void SetMaxVoltage(float max_voltage) { max_voltage_ = max_voltage; }
  void SetMinVoltage(float min_voltage) { min_voltage_ = min_voltage; }
  void SetMaxTorque(float max_torque) { max_torque_ = max_torque; }
  void SetMinStartupForce(float min_startup_force) {
    min_startup_force_ = min_startup_force;
  }
  void SetCWInsensitiveArea(uint8_t cw_insensitive_area) {
    cw_insensitive_area_ = cw_insensitive_area;
  }
  void SetCCWInsensitiveArea(uint8_t ccw_insensitive_area) {
    ccw_insensitive_area_ = ccw_insensitive_area;
  }
  void SetAngularResolution(uint8_t angular_resolution) {
    angular_resolution_ = angular_resolution;
  }
  void SetPositionCorrection(int16_t position_correction) {
    position_correction_ = position_correction;
  }
  void SetCurrentProtectionThreshold(float current_protection_threshold) {
    current_protection_threshold_ = current_protection_threshold;
  }
  void SetOvercurrentProtectionTime(uint16_t overcurrent_protection_time) {
    overcurrent_protection_time_ = overcurrent_protection_time;
  }
  void SetTorqueProtectionThreshold(float torque_protection_threshold) {
    torque_protection_threshold_ = torque_protection_threshold;
  }
  void SetTorqueProtectionTime(uint16_t torque_protection_time) {
    torque_protection_time_ = torque_protection_time;
  }
  void SetOverloadTorque(float overload_torque) {
    overload_torque_ = overload_torque;
  }

  void SetMotorDirection(Direction motor_direction) {
    motor_direction_ = motor_direction;
  }
  void SetSensorDirection(Direction sensor_direction) {
    sensor_direction_ = sensor_direction;
  }

  void SetTorqueEnable(bool torque_enable) { torque_enable_ = torque_enable; }
  void SetTargetAcceleration(float target_acceleration) {
    target_acceleration_ = target_acceleration;
  }
  void SetTargetPosition(int16_t target_position) {
    target_position_ = target_position;
  }
  void SetTargetTime(int16_t target_time) { target_time_ = target_time; }
  void SetTargetPwm(float target_pwm) { target_pwm_ = target_pwm; }
  void SetTargetVelocity(float target_velocity) {
    target_velocity_ = target_velocity;
  }
  void SetTorqueLimit(float torque_limit) { torque_limit_ = torque_limit; }

 private:
  void SetPower(const float power);
  void Break();
  float GetAngle(uint32_t dt);
  float GetVelocity(uint32_t dt);
  float GetCurrent(uint32_t dt);
  bool CheckTarget(int16_t pos_error);

  bool enabled_;
  bool torque_enable_;

  float target_acceleration_;
  int16_t target_position_;
  int16_t target_time_;
  float target_velocity_;
  float target_pwm_;
  float torque_limit_;

  float present_position_;
  float present_velocity_;
  float present_load_;
  float present_voltage_;
  float present_current_;
  uint8_t present_temperature_;

  uint8_t error_status_;
  bool moving_;

  ServoMode mode_;

  uint16_t min_position_;
  uint16_t max_position_;
  uint8_t max_temperature_;
  float max_voltage_;
  float min_voltage_;
  float max_torque_;
  float min_startup_force_;

  uint8_t cw_insensitive_area_;
  uint8_t ccw_insensitive_area_;
  uint8_t angular_resolution_;
  int16_t position_correction_;

  float current_protection_threshold_;
  uint16_t overcurrent_protection_time_;

  float torque_protection_threshold_;
  uint16_t torque_protection_time_;

  float overload_torque_;

  float velocity_proportional_gain_;
  float velocity_integral_gain_;

  PidController pos_pid_{{.kp = 1.0f,
                          .ki = 0.0f,
                          .kd = 0.0f,
                          .ff = 0.0f,
                          .ramp = 0.0f,
                          .limit = 0.0f,
                          .deadband = 0.0f}};
  PidController velocity_pid_{{.kp = 1.0f,
                               .ki = 0.0f,
                               .kd = 0.0f,
                               .ff = 0.0f,
                               .ramp = 0.0f,
                               .limit = 0.0f,
                               .deadband = 0.0f}};
  PidController current_pid_{{.kp = 1.0f,
                              .ki = 0.0f,
                              .kd = 0.0f,
                              .ff = 0.0f,
                              .ramp = 0.0f,
                              .limit = 0.0f,
                              .deadband = 0.0f}};

  Direction motor_direction_;
  Direction sensor_direction_;

  LowPassFilter current_lpf_;
  LowPassFilter velocity_lpf_;
  LowPassFilter pos_lpf_;

  //
  bool action_;
  float position_set_;
  float current_set;
  float voltage_set_;
  float pwm_set_;

  MotorDriver *driver_;
  Sensor *angle_sensor_;
  Current *current_sense_;
};
}  // namespace hortor_servo
