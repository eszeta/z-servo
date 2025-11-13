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

#include "current.h"
#include "encoder.h"
#include "hortor.h"
#include "math/encoder_pll.h"
#include "math/lowpass_filter.h"
#include "math/math.h"
#include "math/pid.h"
#include "motor.h"
#include "types.h"
#include "utils/timeout_limiter.h"

namespace hortor::servo {

/**
 * @brief 舵机控制类
 *
 * 该类实现了舵机的核心控制功能，包括位置控制、速度控制、电流控制等。
 * 支持多种控制模式，并提供完整的PID控制和保护功能。
 *
 * @tparam MotorType 电机驱动器类型，必须继承自 Motor<MotorType>
 * @tparam EncoderType 编码器传感器类型，必须继承自 Encoder<EncoderType>
 * @tparam CurrentType 电流传感器类型，必须继承自 Current<CurrentType>
 */
template <typename MotorType,
          typename EncoderType,
          typename CurrentType,
          uint8_t ResolutionBits>
class Servo {
 public:
  /** @brief 舵机分辨率（位数），决定了舵机的精度和量程 */
  static constexpr math::Resolution<ResolutionBits> kResolution{};

  /**
   * @brief 初始化舵机
   */
  Error Init() {
    current_timeout_limiter_.SetTimeoutDuration(0.3f);
    return Error::kOk;
  }

  //==============================================================================
  // 运行模式组
  //==============================================================================
#pragma region "运行模式组"
  /** @brief 驱动模式 */
  DriveModeBits GetDriveMode() const { return drive_mode_; }
  void SetDriveMode(const DriveModeBits drive_mode) {
    drive_mode_ = drive_mode;
  }
  void SetDriveMode(const uint8_t drive_mode) {
    drive_mode_.value = drive_mode;
    motor_->SetReverse(drive_mode_.moto_reverse_mode ? Reverse::kReverse
                                                     : Reverse::kNormal);
    encoder_->SetReverse(drive_mode_.encoder_reverse_mode ? Reverse::kReverse
                                                          : Reverse::kNormal);
  }

  /** @brief 舵机模式 */
  OperatingMode GetOperatingMode() const { return operating_mode_; }
  void SetOperatingMode(const OperatingMode operating_mode) {
    operating_mode_ = operating_mode;
  }
  void SetOperatingMode(const uint8_t operating_mode) {
    operating_mode_ = static_cast<OperatingMode>(operating_mode);
  }

  /** @brief 关断条件 */
  ShutdownBits GetShutdown() const { return shutdown_; }
  void SetShutdown(const ShutdownBits shutdown) { shutdown_ = shutdown; }
  void SetShutdown(const uint8_t shutdown) { shutdown_.value = shutdown; }

#pragma endregion  // "运行模式组"

  //==============================================================================
  // 位置配置组
  //==============================================================================
#pragma region "位置配置组"
  /** @brief 归零偏移 */
  int32_t GetHomingOffset() const {
    const auto kBits = encoder_->kResolutionBits;
    const auto kTargetBits = kResolution.kBits;
    const auto homing_offset = encoder_->GetHomingOffset();
    return math::mapResolution(homing_offset, kBits, kTargetBits);
  }

  void SetHomingOffset(const int32_t homing_offset) {
    const auto kBits = kResolution.kBits;
    const auto kTargetBits = encoder_->kResolutionBits;
    const auto mapped_offset =
        math::mapResolution(homing_offset, kBits, kTargetBits);
    encoder_->SetHomingOffset(mapped_offset);
  }

  /** @brief 运动阈值 */
  float GetMovingThreshold() const { return moving_threshold_; }
  void SetMovingThreshold(const float moving_threshold) {
    moving_threshold_ = moving_threshold;
  }

#pragma endregion  // "位置配置组"

  //==============================================================================
  // 保护限制组
  //==============================================================================
#pragma region "保护限制组"
  /** @brief 温度上限 */
  uint8_t GetTemperatureLimit() const { return temperature_limit_; }
  void SetTemperatureLimit(const uint8_t temperature_limit) {
    temperature_limit_ = temperature_limit;
  }

  /** @brief 最高电压限制 */
  float GetMaxVoltageLimit() const { return max_voltage_limit_; }
  void SetMaxVoltageLimit(const float max_voltage_limit) {
    max_voltage_limit_ = max_voltage_limit;
  }

  /** @brief 最低电压限制 */
  float GetMinVoltageLimit() const { return min_voltage_limit_; }
  void SetMinVoltageLimit(const float min_voltage_limit) {
    min_voltage_limit_ = min_voltage_limit;
  }

  /** @brief PWM上限 */
  float GetPwmLimit() const { return pwm_limit_; }
  void SetPwmLimit(const float pwm_limit) {
    pwm_limit_ = pwm_limit;
    position_pid_.SetOutputLimit(pwm_limit);
  }

  /** @brief 电流上限 */
  float GetCurrentLimit() const { return current_limit_; }
  void SetCurrentLimit(const float current_limit) {
    current_limit_ = current_limit;
    current_timeout_limiter_.SetThreshold(current_limit);
  }

  /** @brief 速度上限 */
  float GetVelocityLimit() const { return velocity_limit_; }
  void SetVelocityLimit(const float velocity_limit) {
    velocity_limit_ = velocity_limit;
  }

  /** @brief 位置下限 */
  uint32_t GetMinPositionLimit() const { return min_position_limit_; }
  void SetMinPositionLimit(const uint32_t min_position_limit) {
    min_position_limit_ = min_position_limit;
  }

  /** @brief 位置上限 */
  uint32_t GetMaxPositionLimit() const { return max_position_limit_; }
  void SetMaxPositionLimit(const uint32_t max_position_limit) {
    max_position_limit_ = max_position_limit;
  }

  /** @brief 保护时间 */
  float GetProtectionTime() const { return protection_time_; }
  void SetProtectionTime(const float protection_time) {
    protection_time_ = protection_time;
    current_timeout_limiter_.SetTimeoutDuration(protection_time);
  }

#pragma endregion  // "保护限制组"

  //==============================================================================
  // PID 参数组
  //==============================================================================
#pragma region "PID 参数组"
  /** @brief 位置环 PID 控制器 */
  math::Pid &GetPositionPid() { return position_pid_; }
  void SetPositionPid(const float kp, const float ki, const float kd) {
    position_pid_.SetProportionalGain(kp);
    position_pid_.SetIntegralGain(ki);
    position_pid_.SetDerivativeGain(kd);
  }

  /** @brief 速度环 PID 控制器 */
  math::Pid &GetVelocityPid() { return velocity_pid_; }
  void SetVelocityPid(const float kp, const float ki, const float kd) {
    velocity_pid_.SetProportionalGain(kp);
    velocity_pid_.SetIntegralGain(ki);
    velocity_pid_.SetDerivativeGain(kd);
  }

  /** @brief 电流低通滤波器 */
  math::LowPassFilter &GetCurrentLpf() { return current_lpf_; }
  void SetCurrentLpf(const float time_constant) {
    current_lpf_.SetTimeConstant(time_constant);
  }

  /** @brief 一阶前馈增益（速度前馈，已转换为浮点数） */
  float GetFeedforward1stGain() const { return feedforward_1st_gain_; }
  void SetFeedforward1stGain(const float feedforward_1st_gain) {
    feedforward_1st_gain_ = feedforward_1st_gain;
  }

  /** @brief 二阶前馈增益（加速度前馈，已转换为浮点数） */
  float GetFeedforward2ndGain() const { return feedforward_2nd_gain_; }
  void SetFeedforward2ndGain(const float feedforward_2nd_gain) {
    feedforward_2nd_gain_ = feedforward_2nd_gain;
  }

#pragma endregion  // "PID 参数组"

  //==============================================================================
  // 轨迹配置组
  //==============================================================================
#pragma region "轨迹配置组"
  /** @brief 轨迹速度（RPM，Velocity-based 模式） */
  float GetProfileVelocity() const { return profile_velocity_; }
  void SetProfileVelocity(const float profile_velocity) {
    profile_velocity_ = profile_velocity;
  }

  /** @brief 轨迹加速度（rev/min²，Velocity-based 模式） */
  float GetProfileAcceleration() const { return profile_acceleration_; }
  void SetProfileAcceleration(const float profile_acceleration) {
    profile_acceleration_ = profile_acceleration;
  }

#pragma endregion  // "轨迹配置组"

  //==============================================================================
  // 控制命令组
  //==============================================================================
#pragma region "控制命令组"
  /** @brief 扭矩使能状态 */
  bool GetTorqueEnable() const { return torque_enable_; }
  void SetTorqueEnable(const bool torque_enable) {
    torque_enable_ = torque_enable;
  }

  /** @brief 硬件错误状态 */
  HardwareErrorStatusBits GetHardwareErrorStatus() const {
    return hardware_error_status_;
  }
  uint8_t GetHardwareErrorStatusValue() const {
    return hardware_error_status_.value;
  }
  void SetHardwareErrorStatus(
      const HardwareErrorStatusBits hardware_error_status) {
    hardware_error_status_ = hardware_error_status;
  }
  void SetHardwareErrorStatus(const uint8_t hardware_error_status) {
    hardware_error_status_.value = hardware_error_status;
  }

#pragma endregion  // "控制命令组"

  //==============================================================================
  // 目标值组
  //==============================================================================
#pragma region "目标值组"
  /** @brief 目标PWM */
  float GetGoalPwm() const { return goal_pwm_; }
  void SetGoalPwm(const float goal_pwm) { goal_pwm_ = goal_pwm; }

  /** @brief 目标电流 */
  float GetGoalCurrent() const { return goal_current_; }
  void SetGoalCurrent(const float goal_current) {
    goal_current_ = goal_current;
  }

  /** @brief 目标速度 */
  float GetGoalVelocity() const { return goal_velocity_; }
  void SetGoalVelocity(const float goal_velocity) {
    goal_velocity_ = goal_velocity;
  }

  /** @brief 目标位置 */
  int32_t GetGoalPosition() const { return goal_position_; }
  void SetGoalPosition(const int32_t goal_position) {
    goal_position_ = goal_position;
  }

#pragma endregion  // "目标值组"

  //==============================================================================
  // 状态反馈组
  //==============================================================================
#pragma region "状态反馈组"
  /** @brief 当前位置 */
  uint32_t GetPresentPosition() const { return present_position_; }
  void SetPresentPosition(const uint32_t present_position) {
    present_position_ = present_position;
  }

  /** @brief 当前速度 */
  uint32_t GetPresentVelocity() const { return present_velocity_; }
  void SetPresentVelocity(const uint32_t present_velocity) {
    present_velocity_ = present_velocity;
  }

  /** @brief 当前电流 */
  float GetPresentCurrent() const { return present_current_; }
  void SetPresentCurrent(const float present_current) {
    present_current_ = present_current;
  }

  /** @brief 当前输入电压 */
  float GetPresentInputVoltage() const { return present_input_voltage_; }
  void SetPresentInputVoltage(const float present_input_voltage) {
    present_input_voltage_ = present_input_voltage;
  }

  /** @brief 当前温度 */
  float GetPresentTemperature() const { return present_temperature_; }
  void SetPresentTemperature(const float present_temperature) {
    present_temperature_ = present_temperature;
  }

  /** @brief 当前PWM */
  float GetPresentPwm() const { return present_pwm_; }
  void SetPresentPwm(const float present_pwm) { present_pwm_ = present_pwm; }

  /** @brief 位置轨迹（Profile 生成的期望位置） */
  int32_t GetPositionTrajectory() const { return position_trajectory_; }
  void SetPositionTrajectory(const int32_t position_trajectory) {
    position_trajectory_ = position_trajectory;
  }

  /** @brief 速度轨迹（Position PID 输出或 Profile 生成的期望速度） */
  float GetVelocityTrajectory() const { return velocity_trajectory_; }
  void SetVelocityTrajectory(const float velocity_trajectory) {
    velocity_trajectory_ = velocity_trajectory;
  }

  /** @brief 运动状态 */
  bool GetMoving() const { return moving_; }
  void SetMoving(const bool moving) { moving_ = moving; }

  /** @brief 运动详细状态 */
  MovingStatusBits GetMovingStatus() const { return moving_status_; }
  uint8_t GetMovingStatusValue() const { return moving_status_.value; }
  void SetMovingStatus(const MovingStatusBits control_mode) {
    moving_status_ = control_mode;
  }
  void SetMovingStatus(const uint8_t moving_status) {
    moving_status_.value = moving_status;
  }

#pragma endregion  // "状态反馈组"

  /** @brief 编码器 */
  EncoderType *GetEncoder() { return encoder_; }
  void LinkEncoder(EncoderType *encoder) { encoder_ = encoder; }

  /** @brief 电流传感器 */
  CurrentType *GetCurrentSensor() { return current_sensor_; }
  void LinkCurrentSensor(CurrentType *current_sensor) {
    current_sensor_ = current_sensor;
  }

  /** @brief 电机驱动器 */
  MotorType *GetMotor() { return motor_; }
  void LinkMotor(MotorType *motor) { motor_ = motor; }

  /**
   * @brief 处理舵机逻辑
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error Process(float dt) {
    CHECK(RefreshPresent(dt));
    CHECK(CheckPresent(dt));
    CHECK(ExecuteOperatingMode(dt));
    return Error::kOk;
  }

  /**
   * @brief 设置为居中位置
   */
  void SetToCenter() { encoder_->SetToCenter(); }

 private:
  //==============================================================================
  // 运行模式组
  //==============================================================================
#pragma region "运行模式组"
  /** @brief 驱动模式 */
  DriveModeBits drive_mode_{};

  /** @brief 舵机模式 */
  OperatingMode operating_mode_ = OperatingMode::kPosition;

  /** @brief 关断条件 */
  ShutdownBits shutdown_{};

#pragma endregion  // "运行模式组"

  //==============================================================================
  // 位置配置组
  //==============================================================================
#pragma region "位置配置组"
  /** @brief 运动阈值 */
  float moving_threshold_ = 0.0f;

#pragma endregion  // "位置配置组"

  //==============================================================================
  // 保护限制组
  //==============================================================================
#pragma region "保护限制组"
  /** @brief 温度上限 */
  uint8_t temperature_limit_ = 0;
  /** @brief 最高电压限制 */
  float max_voltage_limit_ = 0.0f;
  /** @brief 最低电压限制 */
  float min_voltage_limit_ = 0.0f;
  /** @brief PWM上限 */
  float pwm_limit_ = 0.0f;
  /** @brief 电流上限 */
  float current_limit_ = 0;
  /** @brief 速度上限 */
  float velocity_limit_ = 0.0f;
  /** @brief 位置上限 */
  uint32_t max_position_limit_ = kResolution.kMax;
  /** @brief 位置下限 */
  uint32_t min_position_limit_ = 0;
  /** @brief 保护时间 */
  float protection_time_ = 0.0f;
#pragma endregion  // "保护限制组"

  //==============================================================================
  // PID 参数组
  //==============================================================================
#pragma region "PID 参数组"
  /** @brief 速度环 PID 控制器 */
  math::Pid velocity_pid_{math::Pid::Config{.proportional_gain = 0.0f,
                                            .integral_gain = 0.0f,
                                            .derivative_gain = 0.0f,
                                            .antiwindup_gain = 0.0f,
                                            .output_limit = 1.0f}};

  /** @brief 位置环 PID 控制器 */
  math::Pid position_pid_{math::Pid::Config{.proportional_gain = 0.0f,
                                            .integral_gain = 0.0f,
                                            .derivative_gain = 0.0f,
                                            .antiwindup_gain = 0.0f,
                                            .output_limit = 1.0f}};

  /** @brief 电流低通滤波器 */
  math::LowPassFilter current_lpf_{};

  /** @brief 前馈二阶增益 */
  float feedforward_2nd_gain_ = 0.0f;

  /** @brief 前馈一阶增益 */
  float feedforward_1st_gain_ = 0.0f;

#pragma endregion  // "PID 参数组"

  //==============================================================================
  // 轨迹配置组
  //==============================================================================
#pragma region "轨迹配置组"
  /** @brief 轨迹加速度 */
  float profile_acceleration_ = 0.0f;

  /** @brief 轨迹速度 */
  float profile_velocity_ = 0.0f;

#pragma endregion  // "轨迹配置组"

  //==============================================================================
  // 控制命令组
  //==============================================================================
#pragma region "控制命令组"
  /** @brief 扭矩使能状态 */
  bool torque_enable_ = false;

  /** @brief 硬件错误状态 */
  HardwareErrorStatusBits hardware_error_status_{};

#pragma endregion  // "控制命令组"

  //==============================================================================
  // 目标值组
  //==============================================================================
#pragma region "目标值组"
  /** @brief 目标PWM */
  float goal_pwm_ = 0.0f;

  /** @brief 目标电流 */
  float goal_current_ = 0.0f;

  /** @brief 目标速度 */
  float goal_velocity_ = 0.0f;

  /** @brief 目标位置 */
  int32_t goal_position_ = 0;

#pragma endregion  // "目标值组"

  //==============================================================================
  // 状态反馈组
  //==============================================================================
#pragma region "状态反馈组"
  /** @brief 运动状态 */
  bool moving_ = false;

  /** @brief 运动详细状态 */
  MovingStatusBits moving_status_{};

  /** @brief 当前PWM */
  float present_pwm_ = 0.0f;

  /** @brief 当前电流 */
  float present_current_ = 0.0f;

  /** @brief 当前速度 */
  uint32_t present_velocity_ = 0;

  /** @brief 当前位置 */
  uint32_t present_position_ = 0;

  /** @brief 速度轨迹 */
  float velocity_trajectory_ = 0.0f;

  /** @brief 位置轨迹 */
  int32_t position_trajectory_ = 0;

  /** @brief 当前输入电压 */
  float present_input_voltage_ = 0.0f;

  /** @brief 当前温度 */
  float present_temperature_ = 0.0f;

#pragma endregion  // "状态反馈组"

  //==============================================================================
  // 硬件抽象层
  //==============================================================================
#pragma region "硬件抽象层"
  /** @brief 电机驱动器 */
  MotorType *motor_ = nullptr;

  /** @brief 角度传感器 */
  EncoderType *encoder_ = nullptr;

  /** @brief 编码器PLL */
  math::EncoderPll<ResolutionBits> encoder_pll_{};

  /** @brief 电流传感器 */
  CurrentType *current_sensor_ = nullptr;

  /** @brief 电流超时限制器 */
  utils::TimeoutLimiter current_timeout_limiter_{};

#pragma endregion  // "硬件抽象层"

  /**
   * @brief 刷新当前变量
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error RefreshPresent(float dt) {
    // 处理编码器
    CHECK(encoder_->Process(dt));
    // 处理编码器PLL
    const auto pos = encoder_->GetPos();
    CHECK(encoder_pll_.Process(dt, pos, encoder_->kResolution.kBits));

    // 获取当前位置
    const auto reverse = encoder_->GetReverse();
    const auto pos_pll = encoder_pll_.Pos();
    SetPresentPosition(pos_pll * static_cast<int8_t>(reverse));

    // 获取当前速度
    const auto velocity = encoder_pll_.GetRpm();
    SetPresentVelocity(velocity * static_cast<int8_t>(reverse));

    // 获取当前电流
    float current_float;
    CHECK(current_sensor_->GetCurrent(current_float));
    SetPresentCurrent(GetCurrentLpf().Compute(current_float, dt));
    return Error::kOk;
  }

  Error CheckPresent(float dt) {
    const auto present_current = GetPresentCurrent();
    hardware_error_status_.overload_error =
        current_timeout_limiter_.Process(present_current, dt);
    if (hardware_error_status_.overload_error) {
      SetTorqueEnable(false);
    }
    return Error::kOk;
  }

  /**
   * @brief 执行控制模式
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error ExecuteOperatingMode(float dt) {
    if (!GetTorqueEnable()) {
      return Error::kOk;
    }
    switch (GetOperatingMode()) {
      case OperatingMode::kCurrent: {
        /* code */
        break;
      }
      case OperatingMode::kVelocity: {
        /* code */
        break;
      }
      case OperatingMode::kPosition: {
        positionMode(dt);
        break;
      }
      case OperatingMode::kExtendedPosition: {
        /* code */
        break;
      }
      case OperatingMode::kCurrentPosition: {
        /* code */
        break;
      }
      case OperatingMode::kPwm: {
        /* code */
        break;
      }
      default:
        return Error::kModeNotSupport;
        break;
    }
    return Error::kOk;
  }

  /**
   * @brief 位置控制
   * @param dt 时间间隔(秒)
   * @note
   * 1. 指令经总线传输并写入 Goal Position。
   * 2. 由 Profile Velocity 与 Profile Acceleration 将 Goal Position
   * 转换为期望位置轨迹与期望速度轨迹。
   * 3. 期望位置轨迹与期望速度轨迹分别存入 Position Trajectory 与
   * Velocity Trajectory。
   * 4. 前馈与 PID 控制器基于期望轨迹计算电机的 PWM 输出。
   * 5. Goal PWM 对计算得到的 PWM 施加上限并决定最终 PWM 值。
   * 6. 最终 PWM 通过逆变器作用于电机，驱动舵机输出端运动。
   * 7. 结果更新到 Present Position、Present Velocity、Present PWM 与 Present Current。
   */
  void positionMode(float dt) {
    const auto limited_goal_position = GetLimitedGoalPosition();
    const auto velocityTrajectory = 0;
    const auto positionTrajectory = limited_goal_position;
    SetPositionTrajectory(positionTrajectory);
    SetVelocityTrajectory(velocityTrajectory);
    const auto error = positionTrajectory - present_position_;
    const auto velocity_ff = velocityTrajectory * GetFeedforward1stGain();
    const auto acceleration_ff = 0.0f * GetFeedforward2ndGain();
    const auto feedforward = velocity_ff + acceleration_ff;
    const auto pwm = position_pid_.Compute(error, dt, feedforward);
    SetMotorPower(pwm);
  }

  /**
   * @brief 获取限位后的目标位置
   * @return 限位后的目标位置
   */
  int32_t GetLimitedGoalPosition() const {
    return constrain(goal_position_, min_position_limit_, max_position_limit_);
  }

  /**
   * @brief 设置电机功率
   * @param pwm PWM值
   */
  void SetMotorPower(const float pwm) {
    SetPresentPwm(pwm);
    motor_->SetPWM(pwm);
  }

  /**
   * @brief 电机刹车
   */
  void MotorBreak() { motor_->Brake(); }
  /**
   * @brief 电机滑行
   */
  void MotorCoast() { motor_->Coast(); }
};

}  // namespace hortor::servo
