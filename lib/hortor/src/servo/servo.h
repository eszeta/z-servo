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
   * @brief 获取角度传感器
   * @return 角度传感器指针
   */
  EncoderType &GetSensor() { return encoder_; }

  CurrentType &GetCurrentSense() { return current_sense_; }

  MotorType &GetMotor() { return motor_; }

  // ========== 位置控制接口 ==========
  /**
   * @brief 设置目标位置
   * @param position 目标位置（pulse）
   */
  void SetGoalPosition(int32_t position) { goal_position_ = position; }

  /**
   * @brief 获取目标位置
   * @return 目标位置（pulse）
   */
  int32_t GetGoalPosition() const { return goal_position_; }

  /**
   * @brief 设置位置下限
   * @param limit 位置下限（pulse）
   */
  void SetMinPositionLimit(uint32_t limit) { min_position_limit_ = limit; }

  /**
   * @brief 设置位置上限
   * @param limit 位置上限（pulse）
   */
  void SetMaxPositionLimit(uint32_t limit) { max_position_limit_ = limit; }

  // ========== PID 参数接口 ==========
  /**
   * @brief 设置位置环 PID 参数
   * @param kp 比例增益（已转换）
   * @param ki 积分增益（已转换）
   * @param kd 微分增益（已转换）
   */
  void SetPositionPID(float kp, float ki, float kd) {
    position_pid_.SetKp(kp);
    position_pid_.SetKi(ki);
    position_pid_.SetKd(kd);
  }

  /**
   * @brief 设置速度环 PI 参数
   * @param kp 比例增益（已转换）
   * @param ki 积分增益（已转换）
   */
  void SetVelocityPI(float kp, float ki) {
    velocity_pid_.SetKp(kp);
    velocity_pid_.SetKi(ki);
  }

  /**
   * @brief 设置前馈增益
   * @param ff1 一阶前馈（速度）
   * @param ff2 二阶前馈（加速度）
   */
  void SetFeedforwardGain(float ff1, float ff2) {
    feedforward_1st_gain_ = ff1;
    feedforward_2nd_gain_ = ff2;
  }

  // ========== 轨迹反馈接口 ==========
  /**
   * @brief 获取位置轨迹
   * @return 位置轨迹（pulse）
   */
  int32_t GetPositionTrajectory() const { return position_trajectory_; }

  /**
   * @brief 获取速度轨迹
   * @return 速度轨迹（counts/s）
   */
  float GetVelocityTrajectory() const { return velocity_trajectory_; }

  /**
   * @brief 获取运动状态
   * @return 运动状态位域
   */
  uint8_t GetMovingStatus() const { return moving_status_.value_; }

  /**
   * @brief 初始化舵机
   */
  Error Init() { return Error::kOk; }

  /**
   * @brief 处理舵机逻辑
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error Process(float dt) {
    CHECK(RefreshPresent(dt));
    if (!enabled_) {
      return Error::kOk;
    }
    switch (operating_mode_) {
      case OperatingMode::kCurrent: {
        /* code */
        break;
      }
      case OperatingMode::kVelocity: {
        /* code */
        break;
      }
      case OperatingMode::kPosition: {
        /* code */
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

 private:
  /** @brief 舵机使能状态 */
  bool enabled_ = true;
  /** @brief 扭矩使能状态 */
  bool torque_enable_ = false;

  /** @brief 驱动模式 */
  union DriveMode {
    uint8_t value_ = 0;
    struct {
      bool reverse_mode_ : 1;           // 位0
      bool reserved_bit1_ : 1;          // 位1 - 保留
      bool profile_configuration_ : 1;  // 位2
      bool reserved_bits3_7_ : 5;       // 位3-7 - 保留（5位）
    };
  };

  union MovingStatus {
    uint8_t value_ = 0;
    struct {
      bool in_position : 1;          // 位0: 已到达目标位置
      bool profile_ongoing : 1;      // 位1: 轨迹进行中
      bool reserved_bit2 : 1;        // 位2: 保留
      bool following_error : 1;      // 位3: 跟随误差
      uint8_t velocity_profile : 2;  // 位4-5: 速度轨迹类型 (
                                     //   00=Step,
                                     //   01=Rect,
                                     //   10=Tri,
                                     //   11=Trap)
      uint8_t reserved_bits6_7 : 2;  // 位6-7: 保留
    };
  };

  /** @brief 舵机模式 */
  OperatingMode operating_mode_ = OperatingMode::kPosition;

  /** @brief 传感器方向 */
  Direction encoder_direction_ = Direction::CCW;
  /** @brief 电机方向 */
  Direction motor_direction_ = Direction::CW;

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

  /** @brief 电流低通滤波器 */
  math::LowPassFilter current_lpf_;

  // ========== 传感器与电机相关==========
  /** @brief 电机驱动器 */
  MotorType motor_;
  /** @brief 角度传感器 */
  EncoderType encoder_;
  /** @brief 编码器PLL */
  math::EncoderPll<ResolutionBits> encoder_pll_{};
  /** @brief 电流传感器 */
  CurrentType current_sense_;

  // ========== 位置控制相关 ==========
  /** @brief 目标位置 */
  int32_t goal_position_ = 0;
  /** @brief 位置下限 */
  uint32_t min_position_limit_ = 0;
  /** @brief 位置上限 */
  uint32_t max_position_limit_ = 4095;

  // ========== Profile 参数 ==========
  /** @brief 轨迹速度（RPM，Velocity-based 模式） */
  float profile_velocity_ = 0.0f;
  /** @brief 轨迹加速度（rev/min²，Velocity-based 模式） */
  float profile_acceleration_ = 0.0f;

  // ========== PID 控制器 ==========
  /** @brief 位置环 PID 控制器 */
  math::PidController position_pid_{{.kp = 0.0f,
                                     .ki = 0.0f,
                                     .kd = 0.0f,
                                     .ff = 0.0f,
                                     .i_limit = 0.0f,
                                     .limit = 0.0f}};
  /** @brief 速度环 PID 控制器 */
  math::PidController velocity_pid_{{.kp = 0.0f,
                                     .ki = 0.0f,
                                     .kd = 0.0f,
                                     .ff = 0.0f,
                                     .i_limit = 0.0f,
                                     .limit = 0.0f}};

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

  // ========== 运动状态 ==========
  /** @brief 运动状态位域 */
  MovingStatus moving_status_{};
  /** @brief 驱动模式配置 */
  DriveMode drive_mode_{};

  /**
   * @brief 刷新当前变量
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error RefreshPresent(float dt) {
    // 处理编码器
    CHECK(encoder_.Process(dt));
    // 处理编码器PLL
    CHECK(encoder_pll_.Process(
        dt, encoder_.GetPosCounts(), encoder_.kResolution.kBits));

    // 获取当前位置
    const auto direction = static_cast<float>(encoder_direction_);
    present_position_ = encoder_pll_.Pulse() * direction;

    // 获取当前速度
    present_velocity_ = encoder_pll_.GetRpm() * direction;

    // 获取当前电流
    float current_float;
    CHECK(current_sense_.GetCurrent(current_float));
    present_current_ = current_lpf_.Compute(current_float, dt);
    return Error::kOk;
  }
  /**
   * @brief 设置电机功率
   * @param pwm PWM值
   */
  void SetMotorPower(const float pwm) {
    present_pwm_ = pwm;
    const auto direction = static_cast<float>(motor_direction_);
    const auto pwm_set = direction * pwm;
    motor_.SetPWM(pwm_set);
  }

  /**
   * @brief 电机刹车
   */
  void MotorBreak() { motor_.Brake(); }
  /**
   * @brief 电机滑行
   */
  void MotorCoast() { motor_.Coast(); }
};

}  // namespace hortor::servo
