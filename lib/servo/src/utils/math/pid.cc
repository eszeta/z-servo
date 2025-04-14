#include "pid.h"

#include <Arduino.h>

namespace hortor_servo {

PidController::PidController(const PIDParam param) : param_(param) {}

float PidController::Compute(const float error, const uint32_t dt) {
  return Compute(error, 0, dt);
}

/**
 * @brief 计算PID控制器输出
 */
inline float PidController::Compute(const float error,
                                    const float feed,
                                    const uint32_t dt) {
  // 1. 时间计算
  const float dt_sec = dt * kMicroToSec;

  // 2. PID各项计算
  // 2.1 比例项：p = Kp * e
  // 直接响应当前误差，提供基本的控制作用
  const float p = param_.kp * error;

  // 2.2 积分项：i = integral_sum + Ki * dt * 0.5 * (e + error_prev)
  // 使用梯形积分方法，0.5 * (e + error_prev) 为梯形面积，用于消除静态误差
  float i = integral_sum_ + param_.ki * dt_sec * 0.5f * (error + error_prev_);

  // 对积分项进行限幅，防止积分饱和
  if (param_.limit > 0) {
    i = constrain(i, -param_.limit, param_.limit);
  }

  // 2.3 微分项：d = Kd * de
  // 使用误差变化率，用于抑制超调
  const float de = (error - error_prev_) / dt_sec;
  const float d = param_.kd * de;

  // 2.4 前馈项：f = Kff * feed
  // 直接响应输入变化，提高系统响应速度
  const float f = param_.ff * feed;

  // 3. 输出计算
  // 综合所有控制项：u = p + i + d + f
  float u = p + i + d + f;

  // 4. 输出限幅
  // 限制输出在 [-limit, limit] 范围内
  if (param_.limit > 0) {
    u = constrain(u, -param_.limit, param_.limit);
  }

  // 5. 状态更新
  integral_sum_ = i;
  output_prev_ = u;
  error_prev_ = error;

  return u;
}

void PidController::Reset() {
  integral_sum_ = 0.0f;
  output_prev_ = 0.0f;
  error_prev_ = 0.0f;
}

}  // namespace hortor_servo
