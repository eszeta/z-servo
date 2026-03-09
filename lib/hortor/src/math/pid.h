// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "hortor.h"
#include "math/math.h"

namespace hortor::math {

/**
 * @brief PID控制器类
 */
class Pid : public hortor::Noncopyable {
 public:
  struct Config {
    float kp;     // 比例增益 Kp
    float ki;     // 积分增益 Ki
    float kd;     // 微分增益 Kd
    float ka;     // 抗饱和增益 Ka，用于抑制积分饱和
    float limit;  // 输出限幅值
  };

  explicit Pid(const Config& config);

  /**
   * @brief 计算PID控制器输出（无前馈）
   * @param error - 控制误差
   * @param dt - 时间间隔(秒)
   * @return 控制输出
   */
  float Compute(const float error, const float dt, const float feedforward = 0.0f);

  /**
   * @brief 重置PID控制器状态
   */
  void Reset();

  // 参数设置和获取接口
  void  set_kp(float kp);
  float kp() const;

  void  set_ki(float ki);
  float ki() const;

  void  set_kd(float kd);
  float kd() const;

  void  set_ka(float ka);
  float ka() const;

  void  set_limit(float limit);
  float limit() const;

 private:
  Config config_;
  float  previous_error_       = 0.0f;
  float  integral_accumulator_ = 0.0f;
  float  antiwindup_feedback_  = 0.0f;
};

}  // namespace hortor::math

namespace hortor::math {

inline Pid::Pid(const Config& config) : config_(config) {}

inline float Pid::Compute(const float error, const float dt, const float feedforward) {
  const float proportional = config_.kp * error;

  const float integral = integral_accumulator_ +
                         config_.ki * dt * 0.5f * (error + previous_error_) -
                         config_.ka * antiwindup_feedback_;

  const float derivative = config_.kd * (error - previous_error_) / dt;

  const float output = proportional + integral + derivative + feedforward;

  const float output_clamped = constrain(output, -config_.limit, config_.limit);

  const float saturation_error = output - output_clamped;

  integral_accumulator_ = integral;
  previous_error_       = error;
  antiwindup_feedback_  = saturation_error;
  return output_clamped;
}

inline void Pid::Reset() {
  integral_accumulator_ = 0.0f;
  antiwindup_feedback_  = 0.0f;
  previous_error_       = 0.0f;
}

inline void Pid::set_kp(float kp) {
  config_.kp = kp;
}
inline float Pid::kp() const {
  return config_.kp;
}

inline void Pid::set_ki(float ki) {
  config_.ki = ki;
}
inline float Pid::ki() const {
  return config_.ki;
}

inline void Pid::set_kd(float kd) {
  config_.kd = kd;
}
inline float Pid::kd() const {
  return config_.kd;
}

inline void Pid::set_ka(float ka) {
  config_.ka = ka;
}
inline float Pid::ka() const {
  return config_.ka;
}

inline void Pid::set_limit(float limit) {
  config_.limit = limit;
}
inline float Pid::limit() const {
  return config_.limit;
}

}  // namespace hortor::math
