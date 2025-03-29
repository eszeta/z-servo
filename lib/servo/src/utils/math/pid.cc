#include "pid.h"

#include <Arduino.h>

namespace hortor_servo {

PidController::PidController(const PIDParam param) : param_(param) {}

float PidController::Compute(const float error, const uint32_t dt) { return Compute(error, 0, dt); }

/**
 * @brief 计算PID控制器输出
 */
inline float PidController::Compute(const float error, const float feed, const uint32_t dt) {
  // 1. 时间计算
  float dt_sec = static_cast<float>(dt) * kMicroToSec;
  if (dt_sec <= 0 || dt_sec > kMaxDt) {
    dt_sec = kDefaultDt;
  }

  // 2. 死区处理
  float e = error;
  if (std::abs(error) < param_.deadband) {
    e = 0.0f;
  }

  // 3. 误差变化率计算
  // 计算误差的导数 de = (e - error_prev) / dt，用于计算微分项
  float de = (e - error_prev_) / dt_sec;

  // 4. PID各项计算
  // 4.1 比例项：p = Kp * e
  // 直接响应当前误差，提供基本的控制作用
  const float p = param_.kp * e;

  // 4.2 积分项：i = integral_sum + Ki * dt * 0.5 * (e + error_prev)
  // 使用梯形积分方法，0.5 * (e + error_prev) 为梯形面积，用于消除静态误差
  float i = integral_sum_ + param_.ki * dt_sec * kHalf * (e + error_prev_);

  // 对积分项进行限幅，防止积分饱和
  if (param_.limit > 0) {
    i = constrain(i, -param_.limit, param_.limit);
  }

  // 4.3 微分项：d = Kd * de
  // 使用误差变化率，用于抑制超调
  const float d = param_.kd * de;

  // 4.4 前馈项：f = Kff * feed
  // 直接响应输入变化，提高系统响应速度
  const float f = param_.ff * feed;

  // 5. 输出计算
  // 综合所有控制项：u = p + i + d + f
  float u = p + i + d + f;

  // 6. 输出限幅
  // 限制输出在 [-limit, limit] 范围内
  if (param_.limit > 0) {
    u = constrain(u, -param_.limit, param_.limit);
  }

  // 7. 输出变化率限制
  // 限制输出变化速度在 [-ramp, ramp] 范围内，防止输出突变
  if (param_.ramp > 0) {
    const float max_change = param_.ramp * dt_sec;
    const float change = u - output_prev_;
    if (change > max_change) {
      u = output_prev_ + max_change;
    } else if (change < -max_change) {
      u = output_prev_ - max_change;
    }
  }

  // 8. 状态更新
  integral_sum_ = i;
  output_prev_ = u;
  error_prev_ = e;

  return u;
}

void PidController::Reset() {
  integral_sum_ = 0.0f;
  output_prev_ = 0.0f;
  error_prev_ = 0.0f;
}

}  // namespace hortor_servo
