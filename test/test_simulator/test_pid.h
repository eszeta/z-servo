// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file test_pid.h
 * @brief Pid 单元测试：P 项、积分累加与 Reset、限幅与抗饱和、D 项、getter/setter。
 */

#include <unity.h>

#include "math/pid.h"

namespace PidTest {

using Pid    = hortor::math::Pid;
using Config = hortor::math::Pid::Config;

// 测试用例：仅 P 项时输出 = kp * error
void test_p_only(void) {
  Config      cfg{1.0f, 0.0f, 0.0f, 0.0f, 100.0f};
  Pid         pid(cfg);
  const float out = pid.Compute(5.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -3.0f, pid.Compute(-3.0f, 0.01f));
}

// 测试用例：前馈项叠加到输出，output = PID(error, dt) + feedforward
void test_feedforward(void) {
  Config      cfg{1.0f, 0.0f, 0.0f, 0.0f, 100.0f};
  Pid         pid(cfg);
  const float out = pid.Compute(5.0f, 0.01f, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f + 1.0f, out);
}

// 测试用例：积分累加与 Reset 后积分归零
// 首步梯形积分：integral = ki*dt*0.5*(e + e_prev)，Reset 后 e_prev=0 故首步 out = ki*dt*0.5*1
void test_integral_and_reset(void) {
  Config cfg{0.0f, 10.0f, 0.0f, 0.0f, 100.0f};
  Pid    pid(cfg);
  pid.Compute(1.0f, 0.01f);
  float out = pid.Compute(1.0f, 0.01f);
  TEST_ASSERT_TRUE(out > 0.0f);
  pid.Reset();
  out = pid.Compute(1.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.0f * 0.01f * 0.5f * (1.0f + 0.0f), out);
}

// 测试用例：输出限幅与抗饱和
void test_limit_and_antiwindup(void) {
  Config cfg{10.0f, 100.0f, 0.0f, 1.0f, 1.0f};
  Pid    pid(cfg);
  float  out = pid.Compute(1.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, out);
  for (int i = 0; i < 20; ++i) {
    out = pid.Compute(1.0f, 0.01f);
  }
  TEST_ASSERT_TRUE(out <= 1.0f);
  TEST_ASSERT_TRUE(out >= -1.0f);
}

// 测试用例：D 项 D = kd*(error - previous_error)/dt，此处 kd=2、dt=0.01、delta_error=1 => D=200
// limit 需大于 200，否则输出被限幅无法验证 D 项数值。
void test_derivative(void) {
  Config cfg{0.0f, 0.0f, 2.0f, 0.0f, 500.0f};
  Pid    pid(cfg);
  pid.Compute(0.0f, 0.01f);
  float out = pid.Compute(1.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 200.0f, out);
}

// 测试用例：kp/ki/kd/ka/limit getter/setter
void test_getters_setters(void) {
  Config cfg{1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
  Pid    pid(cfg);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.kp());
  TEST_ASSERT_EQUAL_FLOAT(2.0f, pid.ki());
  TEST_ASSERT_EQUAL_FLOAT(3.0f, pid.kd());
  TEST_ASSERT_EQUAL_FLOAT(4.0f, pid.ka());
  TEST_ASSERT_EQUAL_FLOAT(5.0f, pid.limit());
  pid.set_kp(10.0f);
  pid.set_limit(20.0f);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, pid.kp());
  TEST_ASSERT_EQUAL_FLOAT(20.0f, pid.limit());
}

void run_tests(void) {
  RUN_TEST(PidTest::test_p_only);
  RUN_TEST(PidTest::test_feedforward);
  RUN_TEST(PidTest::test_integral_and_reset);
  RUN_TEST(PidTest::test_limit_and_antiwindup);
  RUN_TEST(PidTest::test_derivative);
  RUN_TEST(PidTest::test_getters_setters);
}

}  // namespace PidTest
