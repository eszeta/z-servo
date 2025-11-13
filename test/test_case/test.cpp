/*
 Copyright (c) 2014-present PlatformIO <contact@platformio.org>

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
**/

#include <stdio.h>
#include <unity.h>

void test_case(void) {
  const auto a = static_cast<uint16_t>(int8_t(-1));
  const auto b = static_cast<uint16_t>(static_cast<uint8_t>(int8_t(-1)));
  printf("a: %d, b: %d\n", a, b);
  TEST_ASSERT_EQUAL(a, a);
}

void RUN_UNITY_TESTS() {
  UNITY_BEGIN();
  RUN_TEST(test_case);
  UNITY_END();
}

#ifdef ARDUINO

#include <Arduino.h>
void setup() {
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  delay(2000);

  RUN_UNITY_TESTS();
}

void loop() {
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(500);
}

#else

int main(int argc, char **argv) {
  RUN_UNITY_TESTS();
  return 0;
}

#endif
