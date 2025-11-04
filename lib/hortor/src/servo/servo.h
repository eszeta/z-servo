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
   * @brief 默认构造函数
   */
  Servo() {}

  /**
   * @brief 初始化舵机
   */
  Error Init() { return Error::kOk; }

  //==============================================================================
  // 运行模式组
  //==============================================================================
#pragma region "运行模式组"
  /** @brief 扭矩使能状态 */
  bool GetTorqueEnable() const { return torque_enable_; }
  void SetTorqueEnable(const bool torque_enable) {
    torque_enable_ = torque_enable;
  }

  /** @brief 舵机模式 */
  OperatingMode GetOperatingMode() const { return operating_mode_; }
  void SetOperatingMode(const OperatingMode operating_mode) {
    operating_mode_ = operating_mode;
  }
  void SetOperatingMode(const uint8_t operating_mode) {
    operating_mode_ = static_cast<OperatingMode>(operating_mode);
  }

  /** @brief 驱动模式 */
  DriveMode GetDriveMode() const { return drive_mode_; }
  void SetDriveMode(const DriveMode drive_mode) { drive_mode_ = drive_mode; }
  void SetDriveMode(const uint8_t drive_mode) {
    drive_mode_.value_ = drive_mode;
    motor_->SetReverse(drive_mode_.moto_reverse_mode_ ? Reverse::kReverse
                                                      : Reverse::kNormal);
    encoder_->SetReverse(drive_mode_.encoder_reverse_mode_ ? Reverse::kReverse
                                                           : Reverse::kNormal);
  }

  int8_t GetReverseMode() const { return drive_mode_.reverse_mode_ ? -1 : 1; }

  /** @brief 控制模式 */
  MovingStatus GetMovingStatus() const { return moving_status_; }
  uint8_t GetMovingStatusValue() const { return moving_status_.value_; }
  void SetMovingStatus(const MovingStatus control_mode) {
    moving_status_ = control_mode;
  }
  void SetMovingStatus(const uint8_t moving_status) {
    moving_status_.value_ = moving_status;
  }

#pragma endregion  // "运行模式组"

#pragma endregion  // "内部设置组"

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

#pragma endregion  // "位置配置组"

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

#pragma endregion  // "PID 参数组"

  //==============================================================================
  // 前馈增益组
  //==============================================================================
#pragma region "前馈增益组"
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

#pragma endregion  // "前馈增益组"

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
  // 目标值组
  //==============================================================================
#pragma region "目标值组"
  /** @brief 目标位置 */
  int32_t GetGoalPosition() const { return goal_position_; }
  void SetGoalPosition(const int32_t goal_position) {
    goal_position_ = goal_position;
  }

  /** @brief 目标PWM */
  float GetGoalPwm() const { return goal_pwm_; }
  void SetGoalPwm(const float goal_pwm) { goal_pwm_ = goal_pwm; }

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

#pragma endregion  // "状态反馈组"

  //==============================================================================
  // 工具方法组
  //==============================================================================
#pragma region "工具方法组"
  /**
   * @brief 获取角度传感器
   * @return 角度传感器指针
   */
  EncoderType *GetEncoder() { return encoder_; }
  void LinkEncoder(EncoderType *encoder) { encoder_ = encoder; }

  CurrentType *GetCurrentSensor() { return current_sensor_; }
  void LinkCurrentSensor(CurrentType *current_sensor) {
    current_sensor_ = current_sensor;
  }

  MotorType *GetMotor() { return motor_; }
  void LinkMotor(MotorType *motor) { motor_ = motor; }

#pragma endregion  // "工具方法组"

  /**
   * @brief 处理舵机逻辑
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error Process(float dt) {
    CHECK(RefreshPresent(dt));
    CHECK(ExecuteOperatingMode(dt));
    return Error::kOk;
  }

  /**
   * @brief 设置为居中位置
   */
  void SetToCenter() { encoder_->SetToCenter(); }

 private:
  // ========== 传感器与电机相关==========
  /** @brief 电机驱动器 */
  MotorType *motor_ = nullptr;
  /** @brief 角度传感器 */
  EncoderType *encoder_ = nullptr;
  /** @brief 编码器PLL */
  math::EncoderPll<ResolutionBits> encoder_pll_{};
  /** @brief 电流传感器 */
  CurrentType *current_sensor_ = nullptr;

  // ========== 模式 ==========
  /** @brief 扭矩使能状态 */
  bool torque_enable_ = false;

  /** @brief 舵机模式 */
  OperatingMode operating_mode_ = OperatingMode::kPosition;

  /** @brief 驱动模式 */
  DriveMode drive_mode_{};

  /** @brief 控制模式 */
  MovingStatus moving_status_{};

  // ========== 当前状态 ==========
  /** @brief 当前位置 */
  uint32_t present_position_ = 0;
  /** @brief 当前速度 */
  uint32_t present_velocity_ = 0;
  /** @brief 当前电流（XL330用电流表示负载） */
  float present_current_ = 0.0f;
  /** @brief 当前输入电压 */
  float present_input_voltage_ = 0.0f;
  /** @brief 当前温度 */
  float present_temperature_ = 0.0f;
  /** @brief 当前PWM */
  float present_pwm_ = 0.0f;

  // ========== 位置控制相关 ==========
  /** @brief 目标位置 */
  int32_t goal_position_ = 0;
  /** @brief 位置下限 */
  uint32_t min_position_limit_ = 0;
  /** @brief 位置上限 */
  uint32_t max_position_limit_ = kResolution.kMax;

  /** @brief 目标PWM */
  float goal_pwm_ = 0.0f;

  // ========== Profile 参数 ==========
  /** @brief 轨迹速度（RPM，Velocity-based 模式） */
  float profile_velocity_ = 0.0f;
  /** @brief 轨迹加速度（rev/min²，Velocity-based 模式） */
  float profile_acceleration_ = 0.0f;

  // ========== PID 控制器 ==========
  /** @brief 位置环 PID 控制器 */
  math::Pid position_pid_{math::Pid::Config{.proportional_gain = 0.0f,
                                            .integral_gain = 0.0f,
                                            .derivative_gain = 0.0f,
                                            .antiwindup_gain = 0.0f,
                                            .output_limit = 1.0f}};
  /** @brief 速度环 PID 控制器 */
  math::Pid velocity_pid_{math::Pid::Config{.proportional_gain = 0.0f,
                                            .integral_gain = 0.0f,
                                            .derivative_gain = 0.0f,
                                            .antiwindup_gain = 0.0f,
                                            .output_limit = 1.0f}};
  /** @brief 电流低通滤波器 */
  math::LowPassFilter current_lpf_{};

  // ========== 前馈增益 ==========
  /** @brief 一阶前馈增益（速度前馈，已转换为浮点数） */
  float feedforward_1st_gain_ = 0.0f;
  /** @brief 二阶前馈增益（加速度前馈，已转换为浮点数） */
  float feedforward_2nd_gain_ = 0.0f;

  // ========== 轨迹输出（用于状态反馈和调试） ==========
  /** @brief 位置轨迹（Profile 生成的期望位置） */
  int32_t position_trajectory_ = 0;
  /** @brief 速度轨迹（Position PID 输出或 Profile 生成的期望速度） */
  float velocity_trajectory_ = 0.0f;

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
   * 7. 结果更新到 Present Position、Present Velocity、Present PWM 与
   * Present Current。
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
    const auto reverse = GetReverseMode();
    const auto pwm_set = pwm * reverse;
    SetPresentPwm(pwm_set);
    motor_->SetPWM(pwm_set);
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
