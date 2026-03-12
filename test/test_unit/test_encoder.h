// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* SimulatorEncoder 单元测试，按功能模块分组：
 * 1. 初始化与配置
 * 2. 位置处理与多圈
 * 3. 对齐功能
 * 4. 方向控制
 * 5. 零点偏移
 * 6. 原始位置管理
 */

#include <unity.h>

#include "error.h"
#include "servo/types.h"
#include "simulator/encoder.h"

namespace EncoderTest {

using Encoder = hortor::simulator::SimulatorEncoder;
using Config  = Encoder::Config;
using Error   = hortor::Error;
using Reverse = hortor::servo::Reverse;

constexpr uint32_t kCpr = (1U << hortor::simulator::kSimEncoderBits);

// ============================================================================
// 1. 初始化与配置（Init & Configuration）
// ============================================================================

// 基本 Init 流程验证
void test_init_basic(void) {
  Encoder enc;
  Config  config{};
  config.homing_offset = 0;
  config.reverse       = Reverse::kNormal;
  const auto err       = enc.Init(config);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
}

// 带 homing_offset 初始化
void test_init_with_homing_offset(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{50, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(150, enc.pos());
  TEST_ASSERT_EQUAL_INT32(50, enc.homing_offset());
}

// 带 reverse 初始化
void test_init_with_reverse(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kReverse})));
  const int32_t expected_pos = static_cast<int32_t>(kCpr) - 100;
  TEST_ASSERT_EQUAL_INT32(expected_pos, enc.pos());
  TEST_ASSERT_EQUAL(static_cast<int>(Reverse::kReverse), static_cast<int>(enc.reverse()));
}

// 边界值 raw=0 初始化
void test_init_boundary_zero(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(0, enc.pos());
  TEST_ASSERT_EQUAL_INT32(0, enc.revolutions());
}

// 边界值 raw=CPR-1 初始化
void test_init_boundary_cpr_minus_one(void) {
  Encoder enc;
  enc.SetRawPosition(static_cast<int32_t>(kCpr - 1));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(-1, enc.pos());
}

// offset+reverse 组合初始化
void test_init_combined_offset_and_reverse(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(
      static_cast<int>(Error::kOk),
      static_cast<int>(enc.Init(Config{static_cast<int32_t>(kCpr - 1), Reverse::kNormal})));
  TEST_ASSERT_TRUE(enc.pos() < 0);
}

// ============================================================================
// 2. 位置处理与多圈（Position Processing & Multi-turn）
// ============================================================================

// 顺时针跨 CPR 边界（CPR-1 -> 0）
void test_process_cross_cpr_cw(void) {
  Encoder enc;
  enc.SetRawPosition(kCpr - 1);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  const int32_t pos_before = enc.pos();

  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_EQUAL_INT32(pos_before + 1, enc.pos());
}

// 逆时针跨 CPR 边界（0 -> CPR-1）
void test_process_cross_cpr_ccw(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  const int32_t pos_before = enc.pos();

  enc.SetRawPosition(static_cast<int32_t>(kCpr - 1));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_EQUAL_INT32(pos_before - 1, enc.pos());
}

// 顺时针一圈验证
void test_process_one_revolution_cw(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));

  for (uint32_t raw = 1; raw < kCpr; ++raw) {
    enc.SetRawPosition(static_cast<int32_t>(raw));
    TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  }
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));

  TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(kCpr), enc.pos());
  TEST_ASSERT_EQUAL_INT32(1, enc.revolutions());
}

// 逆时针一圈验证
void test_process_one_revolution_ccw(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));

  for (uint32_t i = 0; i < kCpr - 1; ++i) {
    enc.SetRawPosition(static_cast<int32_t>(kCpr - 1 - i));
    TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  }
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));

  TEST_ASSERT_EQUAL_INT32(-static_cast<int32_t>(kCpr), enc.pos());
  TEST_ASSERT_EQUAL_INT32(-1, enc.revolutions());
}

// 两圈验证
void test_process_two_revolutions(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));

  for (uint32_t i = 0; i < 2; ++i) {
    for (uint32_t raw = 1; raw < kCpr; ++raw) {
      enc.SetRawPosition(static_cast<int32_t>(raw));
      TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
    }
    enc.SetRawPosition(0);
    TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  }

  TEST_ASSERT_EQUAL_INT32(2 * static_cast<int32_t>(kCpr), enc.pos());
  TEST_ASSERT_EQUAL_INT32(2, enc.revolutions());
}

// revolutions() 计算验证
void test_process_revolutions_calculation(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{100, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(enc.pos() / static_cast<int32_t>(kCpr), enc.revolutions());

  enc.SetRawPosition(500);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_EQUAL_INT32(enc.pos() / static_cast<int32_t>(kCpr), enc.revolutions());
}

// Reverse 下位置处理
void test_process_reverse_direction(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kReverse})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));

  for (uint32_t raw = 1; raw < kCpr; ++raw) {
    enc.SetRawPosition(static_cast<int32_t>(raw));
    TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  }
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));

  TEST_ASSERT_EQUAL_INT32(-static_cast<int32_t>(kCpr), enc.pos());
  TEST_ASSERT_EQUAL_INT32(-1, enc.revolutions());
}

// 最短路径计算验证
void test_process_shortest_wrap_distance(void) {
  const uint32_t raw_a             = 10;
  const uint32_t raw_b             = kCpr - 10;
  const int32_t  kExpectedShortest = 20;

  Encoder enc_a;
  enc_a.SetRawPosition(static_cast<int32_t>(raw_a));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc_a.Init(Config{0, Reverse::kNormal})));
  Encoder enc_b;
  enc_b.SetRawPosition(static_cast<int32_t>(raw_b));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc_b.Init(Config{0, Reverse::kNormal})));

  const int32_t delta_pos = enc_b.pos() - enc_a.pos();
  const int32_t delta_mag = (delta_pos >= 0) ? delta_pos : -delta_pos;
  TEST_ASSERT_EQUAL_INT32(kExpectedShortest, delta_mag);
}

// ============================================================================
// 3. 对齐功能（Alignment）
// ============================================================================

// 对齐到 0
void test_align_to_zero(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.AlignToPosition(0)));
  TEST_ASSERT_EQUAL_INT32(0, enc.pos());
}

// 对齐到指定位置
void test_align_to_position(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.AlignToPosition(500)));
  TEST_ASSERT_EQUAL_INT32(500, enc.pos());
}

// 对齐后移动验证
void test_align_then_move(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.AlignToPosition(500)));
  TEST_ASSERT_EQUAL_INT32(500, enc.pos());

  enc.SetRawPosition(200);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_EQUAL_INT32(600, enc.pos());
}

// 跨边界对齐验证
void test_align_across_boundary(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.AlignToPosition(0)));
  TEST_ASSERT_EQUAL_INT32(0, enc.pos());
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_EQUAL_INT32(0, enc.pos());

  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.AlignToPosition(500)));
  TEST_ASSERT_EQUAL_INT32(500, enc.pos());
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_EQUAL_INT32(500, enc.pos());
}

// ============================================================================
// 4. 方向控制（Direction Control）
// ============================================================================

// 基本方向切换验证
void test_set_reverse_basic(void) {
  Encoder enc;
  enc.SetRawPosition(200);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Reverse::kNormal), static_cast<int>(enc.reverse()));
  TEST_ASSERT_EQUAL_INT32(200, enc.pos());

  enc.set_reverse(Reverse::kReverse);
  TEST_ASSERT_EQUAL(static_cast<int>(Reverse::kReverse), static_cast<int>(enc.reverse()));
  const int32_t expected = static_cast<int32_t>(kCpr) - 200;
  TEST_ASSERT_EQUAL_INT32(expected, enc.pos());
}

// 切换后位置重算验证
void test_set_reverse_recalculates_pos(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(100, enc.pos());

  enc.set_reverse(Reverse::kReverse);
  const int32_t expected = static_cast<int32_t>(kCpr) - 100;
  TEST_ASSERT_EQUAL_INT32(expected, enc.pos());
}

// 边界位置切换验证
void test_set_reverse_at_boundary(void) {
  Encoder enc;
  enc.SetRawPosition(1);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(1, enc.pos());

  enc.set_reverse(Reverse::kReverse);
  TEST_ASSERT_EQUAL_INT32(-1, enc.pos());
}

// 动态多次切换验证
void test_set_reverse_dynamic(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));

  // Normal -> Reverse
  enc.set_reverse(Reverse::kReverse);
  const int32_t pos_reverse = enc.pos();
  TEST_ASSERT_EQUAL(static_cast<int>(Reverse::kReverse), static_cast<int>(enc.reverse()));

  // Reverse -> Normal
  enc.set_reverse(Reverse::kNormal);
  TEST_ASSERT_EQUAL(static_cast<int>(Reverse::kNormal), static_cast<int>(enc.reverse()));
  TEST_ASSERT_EQUAL_INT32(100, enc.pos());

  // 验证 Reverse 位置计算正确
  const int32_t expected_reverse = static_cast<int32_t>(kCpr) - 100;
  enc.set_reverse(Reverse::kReverse);
  TEST_ASSERT_EQUAL_INT32(expected_reverse, enc.pos());
}

// ============================================================================
// 5. 零点偏移（Homing Offset）
// ============================================================================

// 基本偏移设置验证
void test_set_homing_offset_basic(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{20, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(20, enc.homing_offset());
  TEST_ASSERT_EQUAL_INT32(120, enc.pos());
}

// 偏移调整 pos 验证
void test_set_homing_offset_adjusts_pos(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{20, Reverse::kNormal})));

  const int32_t old_pos = enc.pos();
  enc.set_homing_offset(50);
  TEST_ASSERT_EQUAL_INT32(50, enc.homing_offset());
  TEST_ASSERT_EQUAL_INT32(old_pos + (50 - 20), enc.pos());
}

// 负偏移验证
void test_set_homing_offset_negative(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{-50, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(-50, enc.homing_offset());
  TEST_ASSERT_EQUAL_INT32(50, enc.pos());
}

// 偏移与方向组合验证
void test_homing_offset_with_reverse(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{50, Reverse::kReverse})));
  TEST_ASSERT_EQUAL_INT32(50, enc.homing_offset());

  // Reverse 下位置计算：(CPR - 100) + 50
  const int32_t expected_pos = static_cast<int32_t>(kCpr) - 100 + 50;
  TEST_ASSERT_EQUAL_INT32(expected_pos, enc.pos());

  // 调整偏移
  enc.set_homing_offset(100);
  TEST_ASSERT_EQUAL_INT32(100, enc.homing_offset());
  TEST_ASSERT_EQUAL_INT32(expected_pos + 50, enc.pos());
}

// ============================================================================
// 6. 原始位置管理（Raw Position）
// ============================================================================

// raw_pos 范围验证
void test_raw_pos_in_range(void) {
  Encoder enc;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_TRUE(enc.raw_pos() < kCpr);
  TEST_ASSERT_EQUAL(static_cast<int>(100), static_cast<int>(enc.raw_pos()));
}

// 超出范围自动归一化验证
void test_raw_pos_out_of_range_wraps(void) {
  Encoder enc;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));

  // 设置超出 CPR 的值
  enc.SetRawPosition(static_cast<int32_t>(kCpr + 100));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_TRUE(enc.raw_pos() < kCpr);
  TEST_ASSERT_EQUAL(static_cast<int>(100), static_cast<int>(enc.raw_pos()));
}

// Process 后 raw_pos 保持验证
void test_raw_pos_after_process(void) {
  const uint32_t raw_set = 123;
  Encoder        enc;
  enc.SetRawPosition(static_cast<int32_t>(raw_set));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(raw_set), static_cast<int>(enc.raw_pos()));

  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_EQUAL(static_cast<int>(raw_set), static_cast<int>(enc.raw_pos()));
}

void run_tests(void) {
  // 1. 初始化与配置
  RUN_TEST(EncoderTest::test_init_basic);
  RUN_TEST(EncoderTest::test_init_with_homing_offset);
  RUN_TEST(EncoderTest::test_init_with_reverse);
  RUN_TEST(EncoderTest::test_init_boundary_zero);
  RUN_TEST(EncoderTest::test_init_boundary_cpr_minus_one);
  RUN_TEST(EncoderTest::test_init_combined_offset_and_reverse);

  // 2. 位置处理与多圈
  RUN_TEST(EncoderTest::test_process_cross_cpr_cw);
  RUN_TEST(EncoderTest::test_process_cross_cpr_ccw);
  RUN_TEST(EncoderTest::test_process_one_revolution_cw);
  RUN_TEST(EncoderTest::test_process_one_revolution_ccw);
  RUN_TEST(EncoderTest::test_process_two_revolutions);
  RUN_TEST(EncoderTest::test_process_revolutions_calculation);
  RUN_TEST(EncoderTest::test_process_reverse_direction);
  RUN_TEST(EncoderTest::test_process_shortest_wrap_distance);

  // 3. 对齐功能
  RUN_TEST(EncoderTest::test_align_to_zero);
  RUN_TEST(EncoderTest::test_align_to_position);
  RUN_TEST(EncoderTest::test_align_then_move);
  RUN_TEST(EncoderTest::test_align_across_boundary);

  // 4. 方向控制
  RUN_TEST(EncoderTest::test_set_reverse_basic);
  RUN_TEST(EncoderTest::test_set_reverse_recalculates_pos);
  RUN_TEST(EncoderTest::test_set_reverse_at_boundary);
  RUN_TEST(EncoderTest::test_set_reverse_dynamic);

  // 5. 零点偏移
  RUN_TEST(EncoderTest::test_set_homing_offset_basic);
  RUN_TEST(EncoderTest::test_set_homing_offset_adjusts_pos);
  RUN_TEST(EncoderTest::test_set_homing_offset_negative);
  RUN_TEST(EncoderTest::test_homing_offset_with_reverse);

  // 6. 原始位置管理
  RUN_TEST(EncoderTest::test_raw_pos_in_range);
  RUN_TEST(EncoderTest::test_raw_pos_out_of_range_wraps);
  RUN_TEST(EncoderTest::test_raw_pos_after_process);
}

}  // namespace EncoderTest
