// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file test_scheduler.h
 * @brief TaskScheduler 单元测试：初始状态、AddTask/RemoveTask/ClearTasks、
 *       Tick() 触发回调与 dt、以及 Tick() 返回值。
 */

#include <unity.h>

#include "error.h"
#include "utils/task_scheduler.h"

namespace SchedulerTest {

using Scheduler = hortor::utils::TaskScheduler<>;
using Error     = hortor::Error;

/// 占位回调，仅返回 kOk，用于 AddTask/RemoveTask 等不关心执行逻辑的用例。
static hortor::Error DummyCallback(float /*dt*/) {
  return hortor::Error::kOk;
}

/// Tick 测试用：记录回调被调用次数。
static uint32_t g_tick_call_count = 0;
/// Tick 测试用：记录最后一次回调收到的 dt（秒）。
static float g_tick_last_dt = 0.0f;

/// 在 Tick 测试中用于统计调用次数并记录 dt。
static hortor::Error CountingCallback(float dt) {
  g_tick_call_count += 1;
  g_tick_last_dt = dt;
  return hortor::Error::kOk;
}

// 测试用例：初始状态
// 新建未添加任何任务的调度器，应满足 size==0、capacity==16、min_period_us==0。
void test_initial_state(void) {
  Scheduler s;
  TEST_ASSERT_EQUAL_UINT32(0, s.size());
  TEST_ASSERT_EQUAL_UINT32(16, s.capacity());
  TEST_ASSERT_EQUAL_UINT32(0, s.min_period_us());
}

// 测试用例：添加任务成功
// 以合法参数 AddTask(callback, 1000) 添加一个 1000Hz 任务，应返回 kOk，
// size==1，min_period_us 为 1000000/1000 == 1000。
void test_add_task_success(void) {
  Scheduler  s;
  const auto err = s.AddTask(DummyCallback, 1000);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(1, s.size());
  TEST_ASSERT_EQUAL_UINT32(1000, s.min_period_us());  // 1000000/1000
}

// 测试用例：添加任务非法参数
// 1) callback 为 nullptr 时 AddTask 应返回 kInvalidArg，size 保持 0。
// 2) rate_hz 为 0 时 AddTask 应返回 kInvalidArg，size 保持 0。
void test_add_task_invalid_args(void) {
  Scheduler s;
  auto      err = s.AddTask(nullptr, 1000);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kInvalidArg), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(0, s.size());

  err = s.AddTask(DummyCallback, 0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kInvalidArg), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(0, s.size());
}

// 测试用例：任务列表已满
// 使用容量为 2 的调度器，先添加 2 个任务成功，再添加第 3 个任务应返回 kInvalidArg，
// size 仍为 2。
void test_add_task_full(void) {
  hortor::utils::TaskScheduler<2> s;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(s.AddTask(DummyCallback, 100)));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(s.AddTask(CountingCallback, 200)));
  const auto err = s.AddTask(DummyCallback, 300);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kInvalidArg), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(2, s.size());
}

// 测试用例：按回调移除任务
// 添加 2 个任务后，按 callback 移除其中一个，应返回 kOk 且 size 变为 1。
// 再次移除同一 callback（已不存在）应仍返回 kOk，size 不变。
// 最后移除另一个任务，size 变为 0。
void test_remove_task(void) {
  Scheduler s;
  s.AddTask(DummyCallback, 1000);
  s.AddTask(CountingCallback, 500);
  TEST_ASSERT_EQUAL_UINT32(2, s.size());

  const auto err = s.RemoveTask(CountingCallback);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(1, s.size());

  const auto err2 = s.RemoveTask(CountingCallback);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err2));
  TEST_ASSERT_EQUAL_UINT32(1, s.size());

  s.RemoveTask(DummyCallback);
  TEST_ASSERT_EQUAL_UINT32(0, s.size());
}

// 测试用例：清空所有任务
// 添加 2 个任务后调用 ClearTasks()，应返回 kOk，且 size==0、min_period_us==0。
void test_clear_tasks(void) {
  Scheduler s;
  s.AddTask(DummyCallback, 1000);
  s.AddTask(CountingCallback, 500);
  TEST_ASSERT_EQUAL_UINT32(2, s.size());

  const auto err = s.ClearTasks();
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(0, s.size());
  TEST_ASSERT_EQUAL_UINT32(0, s.min_period_us());
}

// 测试用例：Tick 触发回调
// 添加一个 1000Hz 任务后调用一次 Tick()，该任务回调应被调用恰好 1 次，
// 且传入的 dt 约为 0.001s（1/1000）。
void test_tick_invokes_callback(void) {
  Scheduler s;
  g_tick_call_count = 0;
  g_tick_last_dt    = 0.0f;
  s.AddTask(CountingCallback, 1000);

  const auto err = s.Tick();
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(1, g_tick_call_count);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.001f, g_tick_last_dt);  // 1/1000 Hz
}

// 测试用例：Tick 正常返回
// 添加任务后调用 Tick()（内部会执行任务并在不足周期时 delay），应返回 kOk 且不崩溃。
void test_tick_returns_ok(void) {
  Scheduler s;
  s.AddTask(DummyCallback, 1000);
  const auto err = s.Tick();
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
}

void run_tests(void) {
  RUN_TEST(SchedulerTest::test_initial_state);
  RUN_TEST(SchedulerTest::test_add_task_success);
  RUN_TEST(SchedulerTest::test_add_task_invalid_args);
  RUN_TEST(SchedulerTest::test_add_task_full);
  RUN_TEST(SchedulerTest::test_remove_task);
  RUN_TEST(SchedulerTest::test_clear_tasks);
  RUN_TEST(SchedulerTest::test_tick_invokes_callback);
  RUN_TEST(SchedulerTest::test_tick_returns_ok);
}

}  // namespace SchedulerTest
