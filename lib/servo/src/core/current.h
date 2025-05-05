// Copyright 2025 ES_ZETA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

namespace hortor_servo {

/**
 * @brief 电流传感器抽象基类
 * @details
 * 所有具体的电流传感器实现都必须继承自此类。该类提供了电流检测的基本接口定义。
 */
class Current {
 public:
  /**
   * @brief 获取当前电流读数
   * @return float 以安培(A)为单位的电流值
   */
  virtual float GetCurrent() = 0;

  /**
   * @brief 读取并平均多次电流采样值
   * @param n 采样次数，默认为100次
   * @return float N次采样的平均电流值（单位：安培）
   * @note 采样频率取决于具体实现，通常在1kHz以上
   */
  float ReadAverageCurrents(int n = 100);
};

}  // namespace hortor_servo
