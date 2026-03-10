// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

// Native 平台仿真示例：PID 控制器驱动一阶惯性系统
// 用于在 PC 上验证控制参数，无需烧录硬件
#include <Arduino.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

// ---------------------------------------------------------------------------
// 工具
// ---------------------------------------------------------------------------

static float clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ---------------------------------------------------------------------------
// PID 控制器（带梯形积分 + 抗积分饱和）
// ---------------------------------------------------------------------------

struct PidConfig {
  float kp    = 1.0f;
  float ki    = 0.5f;
  float kd    = 0.05f;
  float ka    = 0.1f;   // 抗饱和回馈系数
  float limit = 12.0f;  // 输出限幅 (V)
};

class Pid {
 public:
  explicit Pid(const PidConfig& cfg) : cfg_(cfg) {}

  float Compute(float error, float dt) {
    const float p = cfg_.kp * error;
    const float i = integral_ + cfg_.ki * dt * 0.5f * (error + prev_error_) - cfg_.ka * antiwindup_;
    const float d = cfg_.kd * (error - prev_error_) / dt;
    const float output  = p + i + d;
    const float clamped = clamp(output, -cfg_.limit, cfg_.limit);

    integral_   = i;
    prev_error_ = error;
    antiwindup_ = output - clamped;
    return clamped;
  }

  void Reset() { integral_ = prev_error_ = antiwindup_ = 0.0f; }

 private:
  PidConfig cfg_;
  float     integral_   = 0.0f;
  float     prev_error_ = 0.0f;
  float     antiwindup_ = 0.0f;
};

// ---------------------------------------------------------------------------
// 一阶惯性系统（电机简化模型）
//   tau * dy/dt + y = K * u
// ---------------------------------------------------------------------------

class FirstOrderPlant {
 public:
  struct Config {
    float tau = 0.2f;  // 时间常数 (s)
    float K   = 1.0f;  // 增益
  };

  explicit FirstOrderPlant(const Config& cfg) : cfg_(cfg) {}

  float Step(float u, float dt) {
    // 欧拉法离散化
    const float dy = (cfg_.K * u - y_) / cfg_.tau * dt;
    y_ += dy;
    return y_;
  }

  float output() const { return y_; }

 private:
  Config cfg_;
  float  y_ = 0.0f;
};

// ---------------------------------------------------------------------------
// 仿真主循环
// ---------------------------------------------------------------------------

int main() {
  constexpr float kDt        = 0.001f;  // 仿真步长 1 ms
  constexpr float kTotalTime = 2.0f;    // 仿真时长 2 s
  constexpr float kSetpoint  = 1.0f;    // 目标值

  // 纯 PI（无微分项），避免阶跃响应时的导数冲击
  PidConfig pid_cfg{
      .kp    = 8.0f,
      .ki    = 12.0f,
      .kd    = 0.0f,
      .ka    = 0.3f,
      .limit = 12.0f,
  };

  FirstOrderPlant::Config plant_cfg{
      .tau = 0.15f,
      .K   = 1.0f,
  };

  Pid             pid(pid_cfg);
  FirstOrderPlant plant(plant_cfg);

  std::printf("%-10s  %-10s  %-10s  %-10s\n", "time(s)", "setpoint", "output", "error");
  std::printf("%-10s  %-10s  %-10s  %-10s\n", "-------", "--------", "------", "-----");

  // 每隔 50 ms 打印一行
  constexpr int kPrintEvery = static_cast<int>(0.05f / kDt);
  int           steps       = static_cast<int>(kTotalTime / kDt);

  for (int i = 0; i <= steps; ++i) {
    const float t     = i * kDt;
    const float y     = plant.output();
    const float error = kSetpoint - y;
    const float u     = pid.Compute(error, kDt);

    if (i % kPrintEvery == 0) {
      std::printf("%-10.3f  %-10.4f  %-10.4f  %-10.4f\n", t, kSetpoint, y, error);
    }

    plant.Step(u, kDt);
  }

  // 稳态误差
  const float final_error = kSetpoint - plant.output();
  std::printf("\n稳态误差: %.6f\n", final_error);
  std::printf("仿真完成.\n");

  return 0;
}
