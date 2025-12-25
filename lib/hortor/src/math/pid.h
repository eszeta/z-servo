// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "math/math.h"

namespace hortor::math {

/**
 * @brief PID控制器类
 */
class Pid {
 public:
  struct Config {
    float kp;     // 比例增益 Kp
    float ki;     // 积分增益 Ki
    float kd;     // 微分增益 Kd
    float ka;     // 抗饱和增益 Ka，用于抑制积分饱和
    float limit;  // 输出限幅值
  };

  explicit Pid(const Config& config) : config_(config) {}
  Pid(const Pid&) = delete;
  Pid& operator=(const Pid&) = delete;

  /**
   * @brief 计算PID控制器输出（无前馈）
   * @param error - 控制误差
   * @param dt - 时间间隔(秒)
   * @return 控制输出
   */
  float Compute(const float error,
                const float dt,
                const float feedforward = 0.0f) {
    // 比例项：直接响应当前误差
    const float proportional = config_.kp * error;

    // 积分项：使用梯形积分法，并应用抗饱和反馈
    const float integral = integral_accumulator_ +
                           config_.ki * dt * 0.5f * (error + previous_error_) -
                           config_.ka * antiwindup_feedback_;

    // 微分项：使用误差变化率来抑制超调
    const float derivative = config_.kd * (error - previous_error_) / dt;

    // 综合所有控制项和前馈
    const float output = proportional + integral + derivative + feedforward;

    // 限幅处理
    const float output_clamped =
        constrain(output, -config_.limit, config_.limit);

    // 计算饱和误差，用于下次抗饱和反馈
    const float saturation_error = output - output_clamped;

    // 状态更新
    integral_accumulator_ = integral;
    previous_error_ = error;
    antiwindup_feedback_ = saturation_error;
    return output_clamped;
  }

  /**
   * @brief 重置PID控制器状态
   * 清除积分项和上一次的状态
   */
  void Reset() {
    integral_accumulator_ = 0.0f;
    antiwindup_feedback_ = 0.0f;
    previous_error_ = 0.0f;
  }

  // 参数设置和获取接口
  void set_kp(float kp) { config_.kp = kp; }
  float kp() const { return config_.kp; }

  void set_ki(float ki) { config_.ki = ki; }
  float ki() const { return config_.ki; }

  void set_kd(float kd) { config_.kd = kd; }
  float kd() const { return config_.kd; }

  void set_ka(float ka) { config_.ka = ka; }
  float ka() const { return config_.ka; }

  void set_limit(float limit) { config_.limit = limit; }
  float limit() const { return config_.limit; }

 private:
  // 控制器参数
  Config config_;

  // 控制器状态
  float previous_error_ = 0.0f;        // 上一次的控制误差，用于计算微分项
  float integral_accumulator_ = 0.0f;  // 积分累加值，用于消除静态误差
  float antiwindup_feedback_ = 0.0f;   // 抗饱和反馈值，为输出饱和时被削减的部分
};

}  // namespace hortor::math
