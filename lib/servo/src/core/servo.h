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

#pragma once

#include <Arduino.h>

#include "core/current.h"
#include "core/motor.h"
#include "core/sensor.h"
#include "core/types.h"
#include "math/lowpass_filter.h"
#include "math/pid.h"

namespace hortor_servo {

/**
 * @brief 舵机控制类
 *
 * 该类实现了舵机的核心控制功能，包括位置控制、速度控制、电流控制等。
 * 支持多种控制模式，并提供完整的PID控制和保护功能。
 */
class Servo {
 public:
  /**
   * @brief 默认构造函数
   */
  Servo() = default;

  void Init();
  void LinkDriver(Motor *driver);
  void LinkAngleSensor(Sensor *sensor);
  void LinkCurrentSense(Current *current_sense);
  void Action();
  void Process(float dt);

  /**
   * @brief 获取角度传感器
   * @return 角度传感器指针
   */
  Sensor *GetSensor() { return angle_sensor_; }

  /**
   * @brief 获取当前位置
   * @return 当前位置值
   */
  float GetPresentPosition() { return present_position_; }
  /**
   * @brief 获取当前速度
   * @return 当前速度值
   */
  float GetPresentVelocity() { return present_velocity_; }
  /**
   * @brief 获取当前负载
   * @return 当前负载值
   */
  float GetPresentLoad() { return present_load_; }
  /**
   * @brief 获取当前电压
   * @return 当前电压值
   */
  float GetPresentVoltage() { return present_voltage_; }
  /**
   * @brief 获取当前温度
   * @return 当前温度值
   */
  float GetPresentTemperature() { return present_temperature_; }
  /**
   * @brief 获取当前电流
   * @return 当前电流值
   */
  float GetPresentCurrent() { return present_current_; }
  /**
   * @brief 获取错误状态
   * @return 错误状态值
   */
  uint8_t GetErrorStatus() { return error_status_; }
  /**
   * @brief 获取运动状态
   * @return 是否在运动
   */
  bool GetMoving() { return moving_; }
  /**
   * @brief 获取目标位置
   * @return 目标位置值
   */
  float GetTargetPosition() { return target_position_; }
  /**
   * @brief 获取舵机模式
   * @return 舵机模式
   */
  ServoMode GetMode() { return mode_; }
  /**
   * @brief 获取位置PID控制器
   * @return 位置PID控制器引用
   */
  PidController &GetPosPid() { return pos_pid_; }
  /**
   * @brief 获取速度PID控制器
   * @return 速度PID控制器引用
   */
  PidController &GetVelPid() { return velocity_pid_; }
  /**
   * @brief 获取电流低通滤波器
   * @return 电流低通滤波器引用
   */
  LowPassFilter &GetCurrentLpf() { return current_lpf_; }
  /**
   * @brief 获取位置低通滤波器
   * @return 位置低通滤波器引用
   */
  LowPassFilter &GetPosLpf() { return pos_lpf_; }
  /**
   * @brief 获取速度低通滤波器
   * @return 速度低通滤波器引用
   */
  LowPassFilter &GetVelocityLpf() { return velocity_lpf_; }

  /**
   * @brief 设置舵机模式
   * @param mode 舵机模式
   */
  void SetMode(ServoMode mode) { mode_ = mode; }
  /**
   * @brief 设置最小位置
   * @param min_position 最小位置值
   */
  void SetMinPosition(uint16_t min_position) { min_position_ = min_position; }
  /**
   * @brief 设置最大位置
   * @param max_position 最大位置值
   */
  void SetMaxPosition(uint16_t max_position) { max_position_ = max_position; }
  /**
   * @brief 设置最大温度
   * @param max_temperature 最大温度值
   */
  void SetMaxTemperature(uint8_t max_temperature) {
    max_temperature_ = max_temperature;
  }
  /**
   * @brief 设置最大电压
   * @param max_voltage 最大电压值
   */
  void SetMaxVoltage(float max_voltage) { max_voltage_ = max_voltage; }
  /**
   * @brief 设置最小电压
   * @param min_voltage 最小电压值
   */
  void SetMinVoltage(float min_voltage) { min_voltage_ = min_voltage; }
  /**
   * @brief 设置最大扭矩
   * @param max_torque 最大扭矩值
   */
  void SetMaxTorque(float max_torque) { max_torque_ = max_torque; }
  /**
   * @brief 设置最小启动力
   * @param min_startup_force 最小启动力值
   */
  void SetMinStartupForce(float min_startup_force) {
    min_startup_force_ = min_startup_force;
  }
  /**
   * @brief 设置顺时针不敏感区域
   * @param cw_insensitive_area 顺时针不敏感区域值
   */
  void SetCWInsensitiveArea(uint8_t cw_insensitive_area) {
    cw_insensitive_area_ = cw_insensitive_area;
  }
  /**
   * @brief 设置逆时针不敏感区域
   * @param ccw_insensitive_area 逆时针不敏感区域值
   */
  void SetCCWInsensitiveArea(uint8_t ccw_insensitive_area) {
    ccw_insensitive_area_ = ccw_insensitive_area;
  }
  /**
   * @brief 设置角度分辨率
   * @param angular_resolution 角度分辨率值
   */
  void SetAngularResolution(uint8_t angular_resolution) {
    angular_resolution_ = angular_resolution;
  }
  /**
   * @brief 设置位置校正
   * @param position_correction 位置校正值
   */
  void SetPositionCorrection(int16_t position_correction) {
    position_correction_ = position_correction;
  }
  /**
   * @brief 设置电流保护阈值
   * @param current_protection_threshold 电流保护阈值
   */
  void SetCurrentProtectionThreshold(float current_protection_threshold) {
    current_protection_threshold_ = current_protection_threshold;
  }
  /**
   * @brief 设置过流保护时间
   * @param overcurrent_protection_time 过流保护时间
   */
  void SetOvercurrentProtectionTime(uint16_t overcurrent_protection_time) {
    overcurrent_protection_time_ = overcurrent_protection_time;
  }
  /**
   * @brief 设置扭矩保护阈值
   * @param torque_protection_threshold 扭矩保护阈值
   */
  void SetTorqueProtectionThreshold(float torque_protection_threshold) {
    torque_protection_threshold_ = torque_protection_threshold;
  }
  /**
   * @brief 设置扭矩保护时间
   * @param torque_protection_time 扭矩保护时间
   */
  void SetTorqueProtectionTime(uint16_t torque_protection_time) {
    torque_protection_time_ = torque_protection_time;
  }
  /**
   * @brief 设置过载扭矩
   * @param overload_torque 过载扭矩值
   */
  void SetOverloadTorque(float overload_torque) {
    overload_torque_ = overload_torque;
  }

  /**
   * @brief 设置传感器方向
   * @param sensor_direction 传感器方向
   */
  void SetSensorDirection(Direction sensor_direction) {
    sensor_direction_ = sensor_direction;
  }
  /**
   * @brief 设置电机方向
   * @param motor_direction 电机方向
   */
  void SetMotorDirection(Direction motor_direction) {
    motor_direction_ = motor_direction;
  }

  /**
   * @brief 设置扭矩使能
   * @param torque_enable 是否使能扭矩
   */
  void SetTorqueEnable(bool torque_enable) { torque_enable_ = torque_enable; }
  /**
   * @brief 设置目标加速度
   * @param target_acceleration 目标加速度值
   */
  void SetTargetAcceleration(float target_acceleration) {
    target_acceleration_ = target_acceleration;
  }
  /**
   * @brief 设置目标位置
   * @param target_position 目标位置值
   */
  void SetTargetPosition(float target_position) {
    target_position_ = target_position;
  }
  /**
   * @brief 设置目标时间
   * @param target_time 目标时间值
   */
  void SetTargetTime(int16_t target_time) { target_time_ = target_time; }
  /**
   * @brief 设置目标PWM
   * @param target_pwm 目标PWM值
   */
  void SetTargetPwm(float target_pwm) { target_pwm_ = target_pwm; }
  /**
   * @brief 设置目标速度
   * @param target_velocity 目标速度值
   */
  void SetTargetVelocity(float target_velocity) {
    target_velocity_ = target_velocity;
  }
  /**
   * @brief 设置扭矩限制
   * @param torque_limit 扭矩限制值
   */
  void SetTorqueLimit(float torque_limit) { torque_limit_ = torque_limit; }

  /** @brief 舵机分辨率（位数），决定了舵机的精度和量程 */
  const Resolution kResolution{11};

 private:
  /**
   * @brief 获取当前位置（映射后的计数值）
   * @param dt 时间间隔(秒)
   * @return 当前位置值
   */
  float GetTotalCounts(float dt);
  /**
   * @brief 获取当前速度
   * @param dt 时间间隔(秒)
   * @return 当前速度值
   */
  float GetVelocity(float dt);
  /**
   * @brief 获取当前电流
   * @param dt 时间间隔(秒)
   * @return 当前电流值
   */
  float GetCurrent(float dt);
  /**
   * @brief 设置电机功率
   * @param pwm PWM值
   */
  void SetPower(const float pwm);
  /**
   * @brief 电机刹车
   */
  void Break();
  /**
   * @brief 检查是否到达目标位置
   * @param pos_error 位置误差
   * @return 是否到达目标位置
   */
  bool IsPositionReached(int16_t pos_error);

  /** @brief 舵机使能状态 */
  bool enabled_ = true;
  /** @brief 扭矩使能状态 */
  bool torque_enable_ = false;

  /** @brief 目标加速度 */
  float target_acceleration_ = 0.0f;
  /** @brief 目标位置 */
  float target_position_ = 0.0f;
  /** @brief 目标时间 */
  int16_t target_time_ = 0;
  /** @brief 目标速度 */
  float target_velocity_ = 0.0f;
  /** @brief 目标PWM */
  float target_pwm_ = 0.0f;
  /** @brief 扭矩限制 */
  float torque_limit_ = 0.0f;

  /** @brief 当前位置 */
  float present_position_ = 0.0f;
  /** @brief 当前速度 */
  float present_velocity_ = 0.0f;
  /** @brief 当前负载 */
  float present_load_ = 0.0f;
  /** @brief 当前电压 */
  float present_voltage_ = 0.0f;
  /** @brief 当前电流 */
  float present_current_ = 0.0f;
  /** @brief 当前温度 */
  uint8_t present_temperature_ = 0;

  /** @brief 错误状态 */
  uint8_t error_status_ = 0;
  /** @brief 运动状态 */
  bool moving_ = false;

  /** @brief 舵机模式 */
  ServoMode mode_ = ServoMode::kPosition;

  /** @brief 最小位置限制 */
  uint16_t min_position_ = 0;
  /** @brief 最大位置限制 */
  uint16_t max_position_ = 0;
  /** @brief 最大温度限制 */
  uint8_t max_temperature_ = 0;
  /** @brief 最大电压限制 */
  float max_voltage_ = 0.0f;
  /** @brief 最小电压限制 */
  float min_voltage_ = 0.0f;
  /** @brief 最大扭矩限制 */
  float max_torque_ = 0.0f;
  /** @brief 最小启动力 */
  float min_startup_force_ = 0.0f;

  /** @brief 顺时针不敏感区域 */
  uint8_t cw_insensitive_area_ = 0;
  /** @brief 逆时针不敏感区域 */
  uint8_t ccw_insensitive_area_ = 0;
  /** @brief 角度分辨率 */
  uint8_t angular_resolution_ = 0;
  /** @brief 位置校正值 */
  int16_t position_correction_ = 0;

  /** @brief 电流保护阈值 */
  float current_protection_threshold_ = 0.0f;
  /** @brief 过流保护时间 */
  uint16_t overcurrent_protection_time_ = 0;

  /** @brief 扭矩保护阈值 */
  float torque_protection_threshold_ = 0.0f;
  /** @brief 扭矩保护时间 */
  uint16_t torque_protection_time_ = 0;

  /** @brief 过载扭矩 */
  float overload_torque_ = 0.0f;

  /** @brief 速度比例增益 */
  float velocity_proportional_gain_ = 0.0f;
  /** @brief 速度积分增益 */
  float velocity_integral_gain_ = 0.0f;

  /** @brief 位置PID控制器 */
  PidController pos_pid_{
      {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .ff = 0.0f, .limit = 0.0f}};
  /** @brief 速度PID控制器（ServoMode::kSpeed模式专用） */
  PidController velocity_pid_{
      {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .ff = 0.0f, .limit = 1.0f}};

  /** @brief 传感器方向 */
  Direction sensor_direction_ = Direction::CW;
  /** @brief 电机方向 */
  Direction motor_direction_ = Direction::CW;

  /** @brief 电流低通滤波器 */
  LowPassFilter current_lpf_;
  /** @brief 位置低通滤波器 */
  LowPassFilter pos_lpf_;
  /** @brief 速度低通滤波器 */
  LowPassFilter velocity_lpf_;

  /** @brief 电机驱动器指针 */
  Motor *driver_ = nullptr;
  /** @brief 角度传感器指针 */
  Sensor *angle_sensor_ = nullptr;
  /** @brief 电流传感器指针 */
  Current *current_sense_ = nullptr;
};
}  // namespace hortor_servo
