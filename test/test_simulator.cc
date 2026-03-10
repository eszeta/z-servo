// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0
//
// Simulator 逻辑验证：仅通过 pio test -e Simulator 运行。
// 如需 I/O 验证，在 setup/loop 或 MainLoopCallback 里加 printf，跑 test 即可看到输出。

#include <ArduinoFake.h>
#include <unity.h>

using namespace fakeit;

void setUp(void) { ArduinoFakeReset(); }

static void stub_arduino_for_simulator() {
  When(Method(ArduinoFake(), micros)).AlwaysReturn(0u);
  When(Method(ArduinoFake(), delayMicroseconds)).AlwaysReturn();
  When(Method(ArduinoFake(), delay)).AlwaysReturn();
  When(Method(ArduinoFake(), millis)).AlwaysReturn(0u);
}

void test_simulator_runs_setup_and_loop(void) {
  stub_arduino_for_simulator();

  setup();

  for (int i = 0; i < 10; ++i) {
    loop();
  }

  // 不崩溃、未抛 UnexpectedMethodCallException 即通过
  TEST_PASS();
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_simulator_runs_setup_and_loop);
  return UNITY_END();
}
