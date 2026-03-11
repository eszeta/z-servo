// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* SimulatorEncoder 单元测试，按功能域分组：Init、Init 末端、Process/多圈、
 * pos-revolutions、Align、reverse、homing_offset。 */

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

// Init 与配置

void test_init_returns_ok(void) {
  Encoder enc;
  Config  config{};
  config.homing_offset = 0;
  config.reverse       = Reverse::kNormal;
  const auto err       = enc.Init(config);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
}

// Init 后 pos/revolutions 与 raw、homing_offset、reverse 一致。
void test_pos_and_revolutions_after_init(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(0, enc.pos());
  TEST_ASSERT_EQUAL_INT32(0, enc.revolutions());

  Encoder enc2;
  enc2.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc2.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(100, enc2.pos());
  TEST_ASSERT_EQUAL_INT32(0, enc2.revolutions());

  Encoder enc3;
  enc3.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc3.Init(Config{50, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(150, enc3.pos());

  Encoder enc4;
  enc4.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc4.Init(Config{0, Reverse::kReverse})));
  const int32_t expected_pos_reverse = static_cast<int32_t>(kCpr) - 100;
  TEST_ASSERT_EQUAL_INT32(expected_pos_reverse, enc4.pos());
}

// Init/Process 后 raw_pos() 与设定一致。
void test_raw_pos_after_init_and_process(void) {
  const uint32_t raw_set = 123;
  Encoder        enc;
  enc.SetRawPosition(static_cast<int32_t>(raw_set));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(raw_set), static_cast<int>(enc.raw_pos()));

  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_EQUAL(static_cast<int>(raw_set), static_cast<int>(enc.raw_pos()));
}

// SetRawPosition 超出 [0, CPR-1] 时 Process 后 raw 模归一到该区间。
void test_raw_pos_in_bounds_after_process(void) {
  Encoder enc;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_TRUE(enc.raw_pos() < kCpr);

  enc.SetRawPosition(static_cast<int32_t>(kCpr + 100));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_TRUE(enc.raw_pos() < kCpr);
}

// Init 末端分支（pos 为负的边界）

// Init 时 edge 两侧 pos 分别为非负与负。
void test_init_edge_boundary_pos_positive_and_negative(void) {
  const int32_t  edge = Encoder::kRecalibrateEdgeThreshold;
  const uint32_t pos_boundary_else = kCpr - static_cast<uint32_t>(edge);
  const uint32_t pos_boundary_if   = pos_boundary_else + 1u;

  Encoder enc;
  enc.SetRawPosition(static_cast<int32_t>(pos_boundary_else));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(pos_boundary_else), enc.pos());

  Encoder enc_if;
  enc_if.SetRawPosition(static_cast<int32_t>(pos_boundary_if));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc_if.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(pos_boundary_if) - static_cast<int32_t>(kCpr),
                          enc_if.pos());
}

// Init 末端 pos 为负，随后 Process raw 0→0 时跨 CPR 连续到 0。
void test_init_near_cpr_end_pos_negative_then_continuous_to_zero(void) {
  Encoder enc;
  enc.SetRawPosition(static_cast<int32_t>(kCpr - 1));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  const int32_t pos_near_end = enc.pos();
  TEST_ASSERT_TRUE(pos_near_end < 0);

  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_EQUAL_INT32(pos_near_end + 1, enc.pos());
  TEST_ASSERT_EQUAL_INT32(0, enc.pos());
}

// homing_offset 正/负使零点在末端时 Init 后 pos < 0。
void test_init_homing_end_pos_negative(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(
      static_cast<int>(Error::kOk),
      static_cast<int>(enc.Init(Config{static_cast<int32_t>(kCpr - 1), Reverse::kNormal})));
  TEST_ASSERT_TRUE(enc.pos() < 0);

  Encoder enc2;
  enc2.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc2.Init(Config{-1, Reverse::kNormal})));
  TEST_ASSERT_TRUE(enc2.pos() < 0);
}

// Reverse 下 raw=1 时 Init 后 pos == -1。
void test_init_reverse_near_zero_pos_negative(void) {
  Encoder enc;
  enc.SetRawPosition(1);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kReverse})));
  TEST_ASSERT_EQUAL_INT32(-1, enc.pos());
}

// Process：跨 CPR 连续与多圈

// raw CPR-1→0：Normal 时 pos +1，Reverse 时 pos -1。
void test_pos_continuous_across_cpr_boundary(void) {
  Encoder enc;
  enc.SetRawPosition(kCpr - 1);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  const int32_t pos_before = enc.pos();

  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_EQUAL_INT32(pos_before + 1, enc.pos());

  Encoder enc_rev;
  enc_rev.SetRawPosition(kCpr - 1);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc_rev.Init(Config{0, Reverse::kReverse})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc_rev.Process(0.001f)));
  const int32_t pos_rev_before = enc_rev.pos();
  enc_rev.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc_rev.Process(0.001f)));
  TEST_ASSERT_EQUAL_INT32(pos_rev_before - 1, enc_rev.pos());
}

// raw 0→CPR-1（逆时针一步）Normal 下 pos -1。
void test_pos_continuous_across_cpr_boundary_ccw(void) {
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

// 顺时针一圈 pos += CPR，revolutions += 1。
void test_one_revolution_pos_increases_by_cpr(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  const int32_t pos_start = enc.pos();

  for (uint32_t raw = 1; raw < kCpr; ++raw) {
    enc.SetRawPosition(static_cast<int32_t>(raw));
    TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  }
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));

  TEST_ASSERT_EQUAL_INT32(pos_start + static_cast<int32_t>(kCpr), enc.pos());
  TEST_ASSERT_EQUAL_INT32(1, enc.revolutions());
  TEST_ASSERT_EQUAL_INT32(enc.pos() / static_cast<int32_t>(kCpr), enc.revolutions());
}

// 顺时针两圈 pos/revolutions。
void test_two_revolutions_pos_and_revolutions(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  const int32_t pos_start = enc.pos();

  for (uint32_t i = 0; i < 2; ++i) {
    for (uint32_t raw = 1; raw < kCpr; ++raw) {
      enc.SetRawPosition(static_cast<int32_t>(raw));
      TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
    }
    enc.SetRawPosition(0);
    TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  }

  TEST_ASSERT_EQUAL_INT32(pos_start + 2 * static_cast<int32_t>(kCpr), enc.pos());
  TEST_ASSERT_EQUAL_INT32(2, enc.revolutions());
}

// Reverse 下顺时针一圈 pos -= CPR，revolutions == -1。
void test_one_revolution_reverse_pos_decreases_by_cpr(void) {
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

// Reverse 下 raw 增加则 pos 减少。
void test_process_reverse_decreases_pos(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kReverse})));
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_EQUAL_INT32(-100, enc.pos());
}

// pos / revolutions 关系

// revolutions() == pos() / CPR 在 Init/Process 后成立。
void test_revolutions_equals_pos_div_cpr(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{100, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(enc.pos() / static_cast<int32_t>(kCpr), enc.revolutions());

  enc.SetRawPosition(500);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(enc.Process(0.001f)));
  TEST_ASSERT_EQUAL_INT32(enc.pos() / static_cast<int32_t>(kCpr), enc.revolutions());
}

// CPR 两侧两点 Init 后 pos 差绝对值等于最短弧长。
void test_pos_difference_equals_shortest_wrap(void) {
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

// AlignToPosition

// 对齐到 0 与 500 后 Process 且 raw 不变则 pos 不变。
void test_align_then_unchanged(void) {
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

// Align 后 raw 增加则 pos 正确累加。
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

// reverse 与 set_reverse

// set_reverse 后 pos 按新方向重算，reverse() 正确。
void test_reverse_flips_direction(void) {
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

// set_reverse 后当前 raw 在反向末端时 pos 变负。
void test_set_reverse_at_edge_pos_negative(void) {
  Encoder enc;
  enc.SetRawPosition(1);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(1, enc.pos());

  enc.set_reverse(Reverse::kReverse);
  TEST_ASSERT_EQUAL_INT32(-1, enc.pos());
}

// homing_offset 与 set_homing_offset

// set_homing_offset 后 pos 变化量等于 offset 变化量。
void test_homing_offset_shifts_pos(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(enc.Init(Config{20, Reverse::kNormal})));
  TEST_ASSERT_EQUAL_INT32(20, enc.homing_offset());
  TEST_ASSERT_EQUAL_INT32(120, enc.pos());

  const int32_t old_pos = enc.pos();
  enc.set_homing_offset(50);
  TEST_ASSERT_EQUAL_INT32(50, enc.homing_offset());
  TEST_ASSERT_EQUAL_INT32(old_pos + (50 - 20), enc.pos());
}

void run_tests(void) {
  RUN_TEST(EncoderTest::test_init_returns_ok);
  RUN_TEST(EncoderTest::test_pos_and_revolutions_after_init);
  RUN_TEST(EncoderTest::test_raw_pos_after_init_and_process);
  RUN_TEST(EncoderTest::test_raw_pos_in_bounds_after_process);
  RUN_TEST(EncoderTest::test_init_edge_boundary_pos_positive_and_negative);
  RUN_TEST(EncoderTest::test_init_near_cpr_end_pos_negative_then_continuous_to_zero);
  RUN_TEST(EncoderTest::test_init_homing_end_pos_negative);
  RUN_TEST(EncoderTest::test_init_reverse_near_zero_pos_negative);
  RUN_TEST(EncoderTest::test_pos_continuous_across_cpr_boundary);
  RUN_TEST(EncoderTest::test_pos_continuous_across_cpr_boundary_ccw);
  RUN_TEST(EncoderTest::test_one_revolution_pos_increases_by_cpr);
  RUN_TEST(EncoderTest::test_two_revolutions_pos_and_revolutions);
  RUN_TEST(EncoderTest::test_one_revolution_reverse_pos_decreases_by_cpr);
  RUN_TEST(EncoderTest::test_process_reverse_decreases_pos);
  RUN_TEST(EncoderTest::test_revolutions_equals_pos_div_cpr);
  RUN_TEST(EncoderTest::test_pos_difference_equals_shortest_wrap);
  RUN_TEST(EncoderTest::test_align_then_unchanged);
  RUN_TEST(EncoderTest::test_align_then_move);
  RUN_TEST(EncoderTest::test_reverse_flips_direction);
  RUN_TEST(EncoderTest::test_set_reverse_at_edge_pos_negative);
  RUN_TEST(EncoderTest::test_homing_offset_shifts_pos);
}

}  // namespace EncoderTest
