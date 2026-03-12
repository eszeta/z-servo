// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "error.h"
#include "math/encoder_pll.h"
#include "math/lowpass_filter.h"
#include "math/math.h"
#include "math/pid.h"
#include "math/profile.h"
#include "noncopyable.h"
#include "servo/current.h"
#include "servo/encoder.h"
#include "servo/motor.h"
#include "servo/types.h"
#include "utils/debug_print.h"
#include "utils/timeout_limiter.h"

namespace hortor::servo {

/**
 * @brief 舵机控制类
 *
 * 该类实现了舵机的核心控制功能，包括位置控制、速度控制、电流控制等。
 * 支持多种控制模式，并提供完整的PID控制和保护功能。
 */
template <typename MotorType, typename EncoderType, typename CurrentType, uint8_t Bits>
class Servo : public hortor::Noncopyable {
 public:
  /** @brief 舵机分辨率（位数），决定了舵机的精度和量程 */
  static constexpr math::Resolution<Bits> kResolution{};

  /**
   * @brief 初始化舵机
   */
  Error Init(MotorType* motor, EncoderType* encoder, CurrentType* current_sensor);

  //==============================================================================
  // 运行模式组
  //==============================================================================
#pragma region "运行模式组"
  /** @brief 驱动模式 */
  DriveModeBits drive_mode() const { return drive_mode_; }
  void          set_drive_mode(const DriveModeBits v) { drive_mode_ = v; }
  void          set_drive_mode(const uint8_t v) {
    drive_mode_.value = v;
    motor_->set_reverse(drive_mode_.moto_reverse_mode ? Reverse::kReverse : Reverse::kNormal);
    encoder_pll_.encoder()->set_reverse(drive_mode_.encoder_reverse_mode ? Reverse::kReverse
                                                                                  : Reverse::kNormal);
  }

  /** @brief 舵机模式 */
  OperatingMode operating_mode() const { return operating_mode_; }
  void          set_operating_mode(const OperatingMode v) {
    if (v != operating_mode_) {
      profile_active_goal_ = INT32_MIN;
      position_pid_.Reset();
    }
    operating_mode_ = v;
  }
  void set_operating_mode(const uint8_t v) { set_operating_mode(static_cast<OperatingMode>(v)); }

  /** @brief 关断条件 */
  ShutdownBits shutdown() const { return shutdown_; }
  void         set_shutdown(const ShutdownBits v) { shutdown_ = v; }
  void         set_shutdown(const uint8_t v) { shutdown_.value = v; }

#pragma endregion  // "运行模式组"

  //==============================================================================
  // 位置配置组
  //==============================================================================
#pragma region "位置配置组"
  /** @brief 归零偏移 */
  int32_t homing_offset() const {
    const auto kBits       = encoder_pll_.encoder()->kResolution.kBits;
    const auto kTargetBits = kResolution.kBits;
    const auto homing_off  = encoder_pll_.encoder()->homing_offset();
    return math::mapResolution(homing_off, kBits, kTargetBits);
  }
  void set_homing_offset(const int32_t v) {
    const auto kBits         = kResolution.kBits;
    const auto kTargetBits   = encoder_pll_.encoder()->kResolution.kBits;
    const auto mapped_offset = math::mapResolution(v, kBits, kTargetBits);
    encoder_pll_.encoder()->set_homing_offset(mapped_offset);
  }

  /** @brief 运动阈值 */
  float moving_threshold() const { return moving_threshold_; }
  void  set_moving_threshold(const float v) { moving_threshold_ = v; }

#pragma endregion  // "位置配置组"

  //==============================================================================
  // 保护限制组
  //==============================================================================
#pragma region "保护限制组"
  /** @brief 温度上限 */
  uint8_t temperature_limit() const { return temperature_limit_; }
  void    set_temperature_limit(const uint8_t v) { temperature_limit_ = v; }

  /** @brief 最高电压限制 */
  float max_voltage_limit() const { return max_voltage_limit_; }
  void  set_max_voltage_limit(const float v) { max_voltage_limit_ = v; }

  /** @brief 最低电压限制 */
  float min_voltage_limit() const { return min_voltage_limit_; }
  void  set_min_voltage_limit(const float v) { min_voltage_limit_ = v; }

  /** @brief PWM上限 */
  float pwm_limit() const { return pwm_limit_; }
  void  set_pwm_limit(const float v) {
    pwm_limit_ = v;
    position_pid_.set_limit(v);
  }

  /** @brief 电流上限 */
  float current_limit() const { return current_limit_; }
  void  set_current_limit(const float v) {
    current_limit_ = v;
    current_timeout_limiter_.set_threshold(v);
  }

  /** @brief 速度上限 */
  float velocity_limit() const { return velocity_limit_; }
  void  set_velocity_limit(const float v) { velocity_limit_ = v; }

  /** @brief 位置下限 */
  int32_t min_position_limit() const { return min_position_limit_; }
  void    set_min_position_limit(const int32_t v) { min_position_limit_ = v; }

  /** @brief 位置上限 */
  int32_t max_position_limit() const { return max_position_limit_; }
  void    set_max_position_limit(const int32_t v) { max_position_limit_ = v; }

  /** @brief 保护时间 */
  float protection_time() const { return protection_time_; }
  void  set_protection_time(const float v) {
    protection_time_ = v;
    current_timeout_limiter_.set_timeout_duration(v);
  }

#pragma endregion  // "保护限制组"

  //==============================================================================
  // PID 参数组
  //==============================================================================
#pragma region "PID 参数组"
  /** @brief 位置环 PID 控制器 */
  math::Pid& position_pid() { return position_pid_; }
  void       set_position_pid(const float kp, const float ki, const float kd) {
    position_pid_.set_kp(kp);
    position_pid_.set_ki(ki);
    position_pid_.set_kd(kd);
  }

  /** @brief 速度环 PID 控制器 */
  math::Pid& velocity_pid() { return velocity_pid_; }
  void       set_velocity_pid(const float kp, const float ki, const float kd) {
    velocity_pid_.set_kp(kp);
    velocity_pid_.set_ki(ki);
    velocity_pid_.set_kd(kd);
  }

  /** @brief 电流低通滤波器 */
  math::LowPassFilter& current_lpf() { return current_lpf_; }
  void                 set_current_lpf(const float v) { current_lpf_.set_time_constant(v); }

  /** @brief 一阶前馈增益（速度前馈，已转换为浮点数） */
  float feedforward_1st_gain() const { return feedforward_1st_gain_; }
  void  set_feedforward_1st_gain(const float v) { feedforward_1st_gain_ = v; }

  /** @brief 二阶前馈增益（加速度前馈，已转换为浮点数） */
  float feedforward_2nd_gain() const { return feedforward_2nd_gain_; }
  void  set_feedforward_2nd_gain(const float v) { feedforward_2nd_gain_ = v; }

  /** @brief 轮廓加速度（rev/min/s） */
  float profile_acceleration() const { return profile_acceleration_; }
  void  set_profile_acceleration(const float v) { profile_acceleration_ = v; }

  /** @brief 轮廓速度（rev/min） */
  float profile_velocity() const { return profile_velocity_; }
  void  set_profile_velocity(const float v) { profile_velocity_ = v; }

#pragma endregion  // "PID 参数组"

  //==============================================================================
  // 控制命令组
  //==============================================================================
#pragma region "控制命令组"
  /** @brief 扭矩使能状态 */
  bool torque_enable() const { return torque_enable_; }
  void set_torque_enable(const bool v) {
    if (v && !torque_enable_) {
      profile_active_goal_ = INT32_MIN;
      position_pid_.Reset();
    }
    torque_enable_ = v;
  }

  /** @brief 硬件错误状态 */
  HardwareErrorStatusBits hardware_error_status() const { return hardware_error_status_; }
  uint8_t hardware_error_status_value() const { return hardware_error_status_.value; }
  void    set_hardware_error_status(const HardwareErrorStatusBits v) { hardware_error_status_ = v; }
  void    set_hardware_error_status(const uint8_t v) { hardware_error_status_.value = v; }

#pragma endregion  // "控制命令组"

  //==============================================================================
  // 目标值组
  //==============================================================================
#pragma region "目标值组"
  /** @brief 目标PWM */
  float goal_pwm() const { return goal_pwm_; }
  void  set_goal_pwm(const float v) { goal_pwm_ = v; }

  /** @brief 目标电流 */
  float goal_current() const { return goal_current_; }
  void  set_goal_current(const float v) { goal_current_ = v; }

  /** @brief 目标速度 */
  float goal_velocity() const { return goal_velocity_; }
  void  set_goal_velocity(const float v) { goal_velocity_ = v; }

  /** @brief 目标位置 */
  int32_t goal_position() const { return goal_position_; }
  void    set_goal_position(const int32_t v) { goal_position_ = v; }

#pragma endregion  // "目标值组"

  //==============================================================================
  // 状态反馈组
  //==============================================================================
#pragma region "状态反馈组"
  /** @brief 当前位置 */
  float present_position() const { return present_position_; }
  void  set_present_position(const float v) { present_position_ = v; }

  /** @brief 当前速度 */
  float present_velocity() const { return present_velocity_; }
  void  set_present_velocity(const float v) { present_velocity_ = v; }

  /** @brief 当前电流 */
  float present_current() const { return present_current_; }
  void  set_present_current(const float v) { present_current_ = v; }

  /** @brief 当前输入电压 */
  float present_input_voltage() const { return present_input_voltage_; }
  void  set_present_input_voltage(const float v) { present_input_voltage_ = v; }

  /** @brief 当前温度 */
  float present_temperature() const { return present_temperature_; }
  void  set_present_temperature(const float v) { present_temperature_ = v; }

  /** @brief 当前PWM */
  float present_pwm() const { return present_pwm_; }
  void  set_present_pwm(const float v) { present_pwm_ = v; }

  /** @brief 期望速度轨迹（rev/min） */
  float velocity_trajectory() const { return velocity_trajectory_; }
  void  set_velocity_trajectory(const float v) { velocity_trajectory_ = v; }

  /** @brief 期望位置轨迹（pulse） */
  int32_t position_trajectory() const { return position_trajectory_; }
  void    set_position_trajectory(const int32_t v) { position_trajectory_ = v; }

  /** @brief 运动状态 */
  bool moving() const { return moving_; }
  void set_moving(const bool v) { moving_ = v; }

  /** @brief 运动详细状态 */
  MovingStatusBits moving_status() const { return moving_status_; }
  uint8_t          moving_status_value() const { return moving_status_.value; }
  void             set_moving_status(const MovingStatusBits v) { moving_status_ = v; }
  void             set_moving_status(const uint8_t v) { moving_status_.value = v; }

#pragma endregion  // "状态反馈组"

  /** @brief 编码器 */
  EncoderType* encoder() { return encoder_pll_.encoder(); }

  /** @brief 电流传感器 */
  CurrentType* current_sensor() { return current_sensor_; }

  /** @brief 电机驱动器 */
  MotorType* motor() { return motor_; }

  /**
   * @brief 处理舵机逻辑
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error Process(float dt);

  /**
   * @brief 设置为居中位置
   */
  void AlignToPosition(uint32_t target);

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
  int32_t max_position_limit_ = static_cast<int32_t>(kResolution.kMax);
  /** @brief 位置下限 */
  int32_t min_position_limit_ = 0;
  /** @brief 保护时间 */
  float protection_time_ = 0.0f;
#pragma endregion  // "保护限制组"

  //==============================================================================
  // PID 参数组
  //==============================================================================
#pragma region "PID 参数组"
  /** @brief 速度环 PID 控制器 */
  math::Pid velocity_pid_{
      math::Pid::Config{.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .ka = 0.0f, .limit = 1.0f}};

  /** @brief 位置环 PID 控制器 */
  math::Pid position_pid_{
      math::Pid::Config{.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .ka = 0.0f, .limit = 1.0f}};

  /** @brief 电流低通滤波器 */
  math::LowPassFilter current_lpf_{};

  /** @brief 前馈二阶增益 */
  float feedforward_2nd_gain_ = 0.0f;

  /** @brief 前馈一阶增益 */
  float feedforward_1st_gain_ = 0.0f;

  /** @brief 轮廓加速度 [rev/min²] */
  float profile_acceleration_ = 0.0f;

  /** @brief 轮廓速度 [RPM] */
  float profile_velocity_ = 0.0f;

  /** @brief 速度轮廓生成器 */
  math::Profile profile_{math::Profile::Config{.cpr = kResolution.kEncoderCpr}};

  /**
   * @brief 当前 Profile 对应的目标位置（限位后）
   *
   * 与 profile_.goal() 对比，若不同则触发 Profile 重建。
   * 初始化为极端值确保首次 positionMode 调用时一定重建。
   */
  int32_t profile_active_goal_ = INT32_MIN;

#pragma endregion  // "PID 参数组"

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
  /** @brief 期望速度轨迹 [rev/min] */
  float velocity_trajectory_ = 0.0f;

  /** @brief 期望位置轨迹 [pulse] */
  int32_t position_trajectory_ = 0;

  /** @brief 运动状态 */
  bool moving_ = false;

  /** @brief 运动详细状态 */
  MovingStatusBits moving_status_{};

  /** @brief 当前PWM */
  float present_pwm_ = 0.0f;

  /** @brief 当前电流 */
  float present_current_ = 0.0f;

  /** @brief 当前速度 */
  float present_velocity_ = 0.0f;

  /** @brief 当前位置 */
  float present_position_ = 0.0f;

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
  MotorType* motor_ = nullptr;

  /** @brief 编码器PLL */
  math::EncoderPll<EncoderType, Bits> encoder_pll_{};

  /** @brief 电流传感器 */
  CurrentType* current_sensor_ = nullptr;

  /** @brief 电流超时限制器 */
  utils::TimeoutLimiter current_timeout_limiter_{};

#pragma endregion  // "硬件抽象层"

  /**
   * @brief 刷新当前变量
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error RefreshPresent(float dt);

  Error CheckPresent(float dt);

  /**
   * @brief 执行控制模式
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error ExecuteOperatingMode(float dt);

  /**
   * @brief 位置控制
   * @param dt 时间间隔(秒)
   */
  void positionMode(float dt);

  /**
   * @brief 获取限位后的目标位置
   * @return 限位后的目标位置
   */
  int32_t GetLimitedGoalPosition() const;

  /**
   * @brief 设置电机功率
   * @param pwm PWM值
   */
  void SetMotorPower(const float pwm);

  void MotorCoast();
};

}  // namespace hortor::servo

namespace hortor::servo {

template <typename MotorType, typename EncoderType, typename CurrentType, uint8_t Bits>
Error Servo<MotorType, EncoderType, CurrentType, Bits>::Init(MotorType*   motor,
                                                             EncoderType* encoder,
                                                             CurrentType* current_sensor) {
  current_timeout_limiter_.set_timeout_duration(0.3f);
  CHECK(encoder_pll_.Init(encoder));
  current_sensor_ = current_sensor;
  motor_          = motor;
  return Error::kOk;
}

template <typename MotorType, typename EncoderType, typename CurrentType, uint8_t Bits>
Error Servo<MotorType, EncoderType, CurrentType, Bits>::Process(float dt) {
  CHECK(RefreshPresent(dt));
  CHECK(CheckPresent(dt));
  CHECK(ExecuteOperatingMode(dt));
  return Error::kOk;
}

template <typename MotorType, typename EncoderType, typename CurrentType, uint8_t Bits>
void Servo<MotorType, EncoderType, CurrentType, Bits>::AlignToPosition(uint32_t target) {
  encoder_pll_.encoder()->AlignToPosition(target);
}

template <typename MotorType, typename EncoderType, typename CurrentType, uint8_t Bits>
Error Servo<MotorType, EncoderType, CurrentType, Bits>::RefreshPresent(float dt) {
  CHECK(encoder_pll_.Process(dt));

  const auto pos_pll = encoder_pll_.pos();
  set_present_position(pos_pll);

  const auto velocity = encoder_pll_.rpm();
  set_present_velocity(velocity);

  float current_float;
  CHECK(current_sensor_->ReadCurrent(current_float));
  set_present_current(current_lpf().Compute(current_float, dt));
  return Error::kOk;
}

template <typename MotorType, typename EncoderType, typename CurrentType, uint8_t Bits>
Error Servo<MotorType, EncoderType, CurrentType, Bits>::CheckPresent(float dt) {
  const auto current                    = present_current();
  hardware_error_status_.overload_error = current_timeout_limiter_.Process(current, dt);
  if (hardware_error_status_.overload_error) {
    set_torque_enable(false);
  }
  return Error::kOk;
}

template <typename MotorType, typename EncoderType, typename CurrentType, uint8_t Bits>
Error Servo<MotorType, EncoderType, CurrentType, Bits>::ExecuteOperatingMode(float dt) {
  if (!torque_enable()) {
    MotorCoast();
    return Error::kOk;
  }
  switch (operating_mode()) {
    case OperatingMode::kCurrent:
      break;
    case OperatingMode::kVelocity:
      break;
    case OperatingMode::kPosition:
      positionMode(dt);
      break;
    case OperatingMode::kExtendedPosition:
      break;
    case OperatingMode::kCurrentPosition:
      break;
    case OperatingMode::kPwm:
      break;
    default:
      return Error::kUnsupported;
  }
  return Error::kOk;
}

template <typename MotorType, typename EncoderType, typename CurrentType, uint8_t Bits>
void Servo<MotorType, EncoderType, CurrentType, Bits>::positionMode(float dt) {
  // 1. 目标位置（限位后），目标变更时重建 Profile 并重置 PID
  const int32_t limited_goal = GetLimitedGoalPosition();
  if (limited_goal != profile_active_goal_) {
    profile_.SetGoal(present_position_, limited_goal, profile_velocity_, profile_acceleration_);
    profile_active_goal_ = limited_goal;
    position_pid_.Reset();
  }

  // 2. 推进轮廓
  profile_.Process(dt);

  // 3. 将轨迹写入状态寄存器（AfterProcessImpl 会同步到 Regmap）
  velocity_trajectory_ = profile_.velocity_trajectory_rpm();
  position_trajectory_ = profile_.position_trajectory();

  // 4. PID 误差 = 轨迹位置 - 当前位置（参考块图：Profile 输出作为 PID 参考输入）
  const float error = static_cast<float>(position_trajectory_) - present_position_;

  // 5. 前馈（对应块图中 K_FF1st·s 和 K_FF2nd·s² 两路）
  //    FF1st 乘以轨迹速度 [counts/s]，单位与 encoder_pll_.velocity() 一致
  //    FF2nd 乘以轨迹加速度 [counts/s²]
  const float velocity_ff     = feedforward_1st_gain_ * profile_.velocity_trajectory_cps();
  const float acceleration_ff = feedforward_2nd_gain_ * profile_.acceleration_cps2();
  const float feedforward     = velocity_ff + acceleration_ff;

  // 6. PID 输出 → PWM
  const float pwm = position_pid_.Compute(error, dt, feedforward);
  SetMotorPower(pwm);

  // 7. 更新 moving_ 和 moving_status_
  moving_ = !profile_.is_complete() || (fabsf(present_velocity_) > moving_threshold_);

  MovingStatusBits status{};
  status.profile_ongoing = !profile_.is_complete();
  status.profile_type    = static_cast<uint8_t>(profile_.type());
  // In-Position：Profile 完成且位置误差小于运动阈值对应的脉冲数
  const float threshold_pulse =
      moving_threshold_ * static_cast<float>(kResolution.kEncoderCpr) / 60.0f;
  status.in_position = profile_.is_complete() && (fabsf(error) <= threshold_pulse);
  moving_status_     = status;
}

template <typename MotorType, typename EncoderType, typename CurrentType, uint8_t Bits>
int32_t Servo<MotorType, EncoderType, CurrentType, Bits>::GetLimitedGoalPosition() const {
  return constrain(goal_position_, min_position_limit_, max_position_limit_);
}

template <typename MotorType, typename EncoderType, typename CurrentType, uint8_t Bits>
void Servo<MotorType, EncoderType, CurrentType, Bits>::SetMotorPower(const float pwm) {
  set_present_pwm(pwm);
  motor_->SetPWM(pwm);
}

template <typename MotorType, typename EncoderType, typename CurrentType, uint8_t Bits>
void Servo<MotorType, EncoderType, CurrentType, Bits>::MotorCoast() {
  motor_->Coast();
}

}  // namespace hortor::servo
