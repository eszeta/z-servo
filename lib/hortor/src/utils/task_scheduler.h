// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include "base/types.h"

namespace hortor::utils {

/**
 * @brief 简单的基于时间片的协作式任务调度器
 *
 * 特点：
 * - 非阻塞，每个任务各自维护时间基准
 * - 固定容量（避免动态分配）
 * - 支持集中 Tick 与可选的按最快任务周期睡眠
 */
class TaskScheduler {
 public:
  using Callback = Error (*)(float dt);

  struct TaskDesc {
    Callback callback = nullptr;  // 任务回调
    uint32_t period_us = 0;       // 任务周期（微秒）
    uint32_t last_time_us = 0;    // 上次执行时间（微秒）
  };

  static constexpr float kMicroToSec = 1.0f / 1000000.0f;

  explicit TaskScheduler(uint32_t max_tasks = kDefaultMaxTasks)
      : capacity_(max_tasks), size_(0), min_period_us_(0) {}

  /**
   * @brief 获取当前已注册任务中最小周期（微秒）
   */
  uint32_t min_period_us() const { return min_period_us_; }

  /**
   * @brief 返回已注册任务数量
   */
  uint32_t size() const { return size_; }

  /**
   * @brief 最大容量
   */
  uint32_t capacity() const { return capacity_; }

  /**
   * @brief 注册任务
   * @param callback 任务回调（形如 void (*)(float dt)）
   * @param rate_hz 任务频率（Hz）
   * @return Error::kOk 成功；kInvalidArg 参数/容量/周期无效
   */
  Error Register(Callback callback, uint32_t rate_hz) {
    if (callback == nullptr || rate_hz == 0) {
      return Error::kInvalidArg;
    }
    if (size_ >= capacity_) {
      return Error::kInvalidArg;
    }
    const auto period_us = 1000000u / rate_hz;
    if (period_us == 0u) {
      return Error::kInvalidArg;
    }
    const uint32_t idx = size_;
    tasks_[idx].callback = callback;
    tasks_[idx].period_us = period_us;
    tasks_[idx].last_time_us = micros() - period_us;
    size_ += 1;
    if (period_us < min_period_us_) {
      min_period_us_ = period_us;
    }
    return Error::kOk;
  }

  /**
   * @brief 非阻塞 Tick：到时的任务将被执行，未到时立即返回
   * @return 执行的任务数量
   */
  Error TickNonBlocking() {
    const uint32_t now = micros();
    for (uint32_t i = 0; i < size_; ++i) {
      TaskDesc& t = tasks_[i];
      const uint32_t elapsed = now - t.last_time_us;
      if (elapsed >= t.period_us) {
        const float dt = static_cast<float>(elapsed) * kMicroToSec;
        t.last_time_us = now;
        CHECK(t.callback(dt));
      }
    }
    return Error::kOk;
  }

  /**
   * @brief 阻塞 Tick：执行到时任务后，按最快任务周期剩余时间进行延迟
   */
  Error Tick() {
    const uint32_t start = micros();
    CHECK(TickNonBlocking());

    const uint32_t min_period = min_period_us();
    const uint32_t elapsed = micros() - start;
    if (elapsed < min_period) {
      delayMicroseconds(min_period - elapsed);
    }
    return Error::kOk;
  }

 private:
  static constexpr uint32_t kDefaultMaxTasks = 16u;

  uint32_t capacity_;
  uint32_t size_;
  TaskDesc tasks_[kDefaultMaxTasks];
  uint32_t min_period_us_;
};

}  // namespace hortor::utils