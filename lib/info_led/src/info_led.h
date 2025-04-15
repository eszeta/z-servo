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

#include <vector>

#include "Arduino.h"
#include "led.h"

namespace hortor_servo {
namespace InfoLED {

/**
 * @brief LED闪烁模式的基本单元
 */
struct BlinkUnit {
  float duration;  // 持续时间(秒)
  bool state;      // true表示亮，false表示灭
};

/**
 * @brief 信息类型
 */
enum class InfoType {
  kOk,          // 正常状态
  kWarning,     // 警告
  kError,       // 错误
  kFatalError,  // 致命错误
  kMax
};

/**
 * @brief 信息LED
 */
class InfoLED {
 public:
  /**
   * @brief 构造函数
   */
  InfoLED() = default;
  /**
   * @brief 初始化
   */
  void Init(const uint32_t pin, const Mode mode = Mode::kPushPull);
  /**
   * @brief 初始化
   * @param pinName 引脚名
   * @param mode 模式
   */
  void Init(const PinName pinName, const Mode mode = Mode::kPushPull);

  /**
   * @brief 设置预定义的信息类型
   * @param type 信息类型
   */
  void SetInfo(InfoType type);

  /**
   * @brief 停止显示信息
   */
  void Stop();

  /**
   * @brief 需要在主循环中调用以更新LED状态
   * @param dt 时间间隔(秒)
   */
  void Process(float dt);

 private:
  /**
   * @brief LED
   */
  LED led_;
  /**
   * @brief 当前模式
   */
  std::vector<BlinkUnit>* current_pattern_;
  /**
   * @brief 当前步骤
   */
  size_t current_step_;
  /**
   * @brief 当前步骤已运行时间(秒)
   */
  float elapsed_time_;
  /**
   * @brief 预定义的信息类型
   */
  std::vector<std::vector<BlinkUnit>> patterns_ = {
      // OK - 慢闪
      {{0.5, true}, {0.5, false}},
      // WARNING - 快闪
      {{0.2, true}, {0.2, false}},
      // ERROR - 一长两闪
      {{1.0, true},
       {0.5, false},
       {0.2, true},
       {0.2, false},
       {0.2, true},
       {0.2, false}},
      // FATAL_ERROR - 三闪一长
      {{0.2, true},
       {0.2, false},
       {0.2, true},
       {0.2, false},
       {0.2, true},
       {0.2, false},
       {1, true},
       {1, false}},
  };
};

}  // namespace InfoLED
}  // namespace hortor_servo