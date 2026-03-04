// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include <array>

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
template <uint32_t kMaxTasks = 16>
class TaskScheduler {
 public:
  using Callback = Error (*)(float dt);

  struct TaskDesc {
    Callback callback = nullptr;  // 任务回调
    uint32_t period_us = 0;       // 任务周期（微秒）
    uint32_t last_time_us = 0;    // 上次执行时间（微秒）
  };

  static constexpr float kMicroToSec = 1.0f / 1000000.0f;

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
  uint32_t capacity() const { return tasks_.size(); }

  /**
   * @brief 注册任务
   * @param callback 任务回调（形如 void (*)(float dt)）
   * @param rate_hz 任务频率（Hz）
   * @return Error::kOk 成功；kInvalidArg 参数/容量/周期无效
   */
  Error AddTask(Callback callback, uint32_t rate_hz) {
    VERIFY(callback != nullptr && rate_hz != 0, Error::kInvalidArg);
    VERIFY(size_ < tasks_.size(), Error::kInvalidArg);
    const auto period_us = 1000000u / rate_hz;
    VERIFY(period_us != 0u, Error::kInvalidArg);
    const uint32_t idx = size_;
    tasks_[idx].callback = callback;
    tasks_[idx].period_us = period_us;
    tasks_[idx].last_time_us = micros() - period_us;
    size_ += 1;
    UpdateMinPeriod();
    return Error::kOk;
  }

  /**
   * @brief 注销任务
   * @param callback 要移除的任务回调
   * @return Error::kOk 成功或未找到；
   */
  Error RemoveTask(Callback callback) {
    VERIFY(callback != nullptr, Error::kInvalidArg);
    uint32_t found = size_;
    for (uint32_t i = 0; i < size_; ++i) {
      if (tasks_[i].callback == callback) {
        found = i;
        break;
      }
    }
    if (found == size_) {
      return Error::kOk;
    }
    for (uint32_t i = found; i < size_ - 1; ++i) {
      tasks_[i] = tasks_[i + 1];
    }
    size_ -= 1;
    UpdateMinPeriod();
    return Error::kOk;
  }

  Error ClearTasks() {
    size_ = 0;
    min_period_us_ = 0;
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
  void UpdateMinPeriod() {
    min_period_us_ = 0;
    for (uint32_t i = 0; i < size_; ++i) {
      if (min_period_us_ == 0 || tasks_[i].period_us < min_period_us_) {
        min_period_us_ = tasks_[i].period_us;
      }
    }
  }

  uint32_t size_{0};
  std::array<TaskDesc, kMaxTasks> tasks_{};
  uint32_t min_period_us_{0};
};

}  // namespace hortor::utils