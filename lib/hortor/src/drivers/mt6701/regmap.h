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

#include <Arduino.h>

#include <functional>

#include "regmap_i2c_bus.h"
#include "servo/types.h"
#include "types.h"

namespace hortor::drivers::MT6701 {

/**
 * @brief MT6701控制器类（CRTP模式）
 * @note 该类负责与MT6701传感器进行通信，并提供读取角度值和配置传感器的功能
 *
 * Controller类封装了MT6701磁性角度传感器的所有操作，包括：
 * - 角度读取（原始值、角度值、弧度值）
 * - 传感器配置（工作模式、分辨率、零位等）
 * - 输出设置（模拟输出、PWM输出）
 *
 * 通过 CRTP 继承自 RegMapI2CBus，实现编译期静态多态。
 */
// todo：除了ReadRaw,其它函数都没测试过
class MT6701RegMap : public RegMapI2CBus {
 public:
  /**
   * @brief 读取原始位置和状态值
   * @param angle_raw 原始角度指针 [0...16383]
   * @param field_status 磁场状态指针
   * @param button_pushed 按钮状态指针，如果按钮被按下则为true，否则为false
   * @param track_loss 跟踪丢失状态指针，如果检测到丢失则为true，否则为false
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 从MT6701传感器读取完整的状态信息，包括原始角度值和各种状态标志。
   * 参数可以为nullptr，表示不需要该信息。
   * 注意：I2C模式下只支持角度读取，其他状态信息不可用。
   */
  Error ReadRaw(uint16_t& angle_raw,
                Status& field_status,
                bool& button_pushed,
                bool& track_loss);

  /**
   * @brief 读取磁场状态
   * @return 磁场状态枚举值
   *
   * 读取当前磁场状态，用于诊断磁铁位置问题。
   * 返回值指示磁场是否正常、过强或过弱。
   */
  Status ReadFieldStatus();

  /**
   * @brief 设置UVW模式
   * @param pairs 极对数 [1...16]
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 配置MT6701传感器工作在UVW模式，适用于无刷电机控制。
   * 极对数参数应与电机的极对数匹配。
   */
  Error SetUvmMode(const uint8_t pairs);

  /**
   * @brief 设置ABZ模式
   * @param pulses_per_round 每圈脉冲数 [1...1024]
   * @param z_pulse_width Z脉冲宽度，定义Z信号的脉冲宽度
   * @param hysteresis 迟滞设置，用于抑制抖动
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 配置MT6701传感器工作在ABZ模式，提供增量式编码器输出。
   * 可以设置分辨率、Z信号宽度和迟滞参数。
   */
  Error SetAbzMode(const uint16_t pulses_per_round,
                   const PulseWidth z_pulse_width,
                   const Hyst hysteresis);

  /**
   * @brief 启用-a-b-z UVW模式（仅适用于QFN封装）
   * @param enable 是否启用负逻辑输出
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 配置MT6701传感器的UVW输出使用负逻辑（-a-b-z而不是abz）。
   * 注意：此功能仅在QFN封装的MT6701芯片上可用。
   */
  Error SetNaNbNzEnable(bool enable);  // 仅适用于QFN版本

  /**
   * @brief 设置模拟输出模式
   * @param start 起始角度 [0...360.0)度
   * @param stop 停止角度 [0...360.0)度
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 配置MT6701传感器工作在模拟输出模式，并设置输出范围。
   * 输出电压将在起始角度和停止角度之间线性变化。
   */
  Error SetAnalogMode(const float start = 0.0f, const float stop = 360.0f);

  /**
   * @brief 设置PWM输出模式
   * @param frequency 频率选项
   * @param polarity 极性选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 配置MT6701传感器工作在PWM输出模式，并设置频率和极性。
   * PWM占空比将随角度线性变化。
   */
  Error SetPwmMode(const PwmFreq frequency = PwmFreq::kPWMFreq497_2,
                   const PwmPol polarity = PwmPol::kHigh);

  /**
   * @brief 设置旋转方向
   * @param direction 方向选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器的角度计数方向。
   * 可以选择顺时针或逆时针方向作为角度增加的方向。
   */
  Error SetDirection(const Direction direction);
  /**
   * @brief 获取旋转方向
   * @param direction 方向选项的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 获取MT6701传感器的当前旋转方向。
   */
  Error GetDirection(Direction& direction);
  /**
   * @brief 将当前设置保存到EEPROM
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 将当前的所有配置参数保存到MT6701传感器的内部EEPROM中。
   * 保存后的设置在传感器断电后仍然有效。
   * 注意：EEPROM写入操作有寿命限制，不应频繁调用此方法。
   */
  Error ProgramEEPROM();

  /**
   * @brief 设置ABZ模式的分辨率
   * @param pulses 每圈脉冲数 [1...1024]
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器在ABZ模式下的分辨率（每圈脉冲数）。
   * 此参数决定了A和B信号的精度。
   */
  Error SetABZPulsePerRound(uint16_t pulses);

  /**
   * @brief 设置UVW模式的极对数
   * @param pairs 极对数 [1...16]
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器在UVW模式下的极对数。
   * 此参数应与电机的极对数匹配，以确保正确的换相。
   */
  Error SetUVWPolePair(uint8_t pairs);

  /**
   * @brief 设置工作模式
   * @param mode 模式选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器的基本工作模式。
   * 可以选择UVW模式（适用于无刷电机）或ABZ模式（增量式编码器）。
   */
  Error SetMode(Mode mode);

  /**
   * @brief 设置12位位置值的零偏移
   * @param zero 原始角度值 [0...4096]
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 使用原始值设置MT6701传感器的零位偏移。
   * 此方法使用12位分辨率的原始值（0-4096范围）。
   */
  Error SetZeroRaw(uint16_t zero);

  /**
   * @brief 设置浮点值的零偏移
   * @param zero 角度 [0...360.0)度
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 使用角度值设置MT6701传感器的零位偏移。
   * 此方法使用浮点角度值（0-360度范围）。
   */
  Error SetZero(float zero);

  /**
   * @brief 设置迟滞参数
   * @param hysteresis 迟滞选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器的迟滞参数，用于抑制角度输出的抖动。
   * 较大的迟滞值可以减少噪声，但会降低灵敏度。
   */
  Error SetHyst(Hyst hysteresis);

  /**
   * @brief 设置模拟输出的起始和停止角度（原始值）
   * @param start 起始角度原始值 [0...4096]
   * @param stop 停止角度原始值 [0...4096]
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 使用原始值设置MT6701传感器模拟输出模式的角度范围。
   * 此方法使用12位分辨率的原始值（0-4096范围）。
   * 注意：停止角度必须大于起始角度。
   */
  Error SetStartStopRaw(uint16_t start, uint16_t stop);

  /**
   * @brief 设置模拟输出的起始和停止角度（浮点值）
   * @param start 起始角度 [0...360.0)度
   * @param stop 停止角度 [0...360.0)度
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 使用角度值设置MT6701传感器模拟输出模式的角度范围。
   * 此方法使用浮点角度值（0-360度范围）。
   * 注意：停止角度必须大于起始角度。
   */
  Error SetStartStop(float start, float stop);

  /**
   * @brief 设置Z信号的脉冲宽度
   * @param width 脉冲宽度选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器在ABZ模式下Z信号的脉冲宽度。
   */
  Error SetPulseWidth(PulseWidth width);
  /**
   * @brief 获取Z信号的脉冲宽度
   * @param width 脉冲宽度选项的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 获取MT6701传感器的当前Z信号脉冲宽度。
   */
  Error GetPulseWidth(PulseWidth& width);
  /**
   * @brief 设置PWM输出频率
   * @param freq 频率选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器PWM输出模式的频率。
   */
  Error SetPwmFreq(PwmFreq freq);
  /**
   * @brief 获取PWM输出频率
   * @param freq 频率选项的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 获取MT6701传感器的当前PWM输出频率。
   */
  Error GetPwmFreq(PwmFreq& freq);
  /**
   * @brief 设置PWM输出极性
   * @param polarity 极性选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器PWM输出的极性。
   * 高电平有效时，占空比随角度增加而增加；
   * 低电平有效时，占空比随角度增加而减少。
   */
  Error SetPwmPolarity(PwmPol polarity);
  /**
   * @brief 获取PWM输出极性
   * @param polarity 极性选项的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 获取MT6701传感器的当前PWM输出极性。
   */
  Error GetPwmPolarity(PwmPol& polarity);
  /**
   * @brief 设置输出模式
   * @param mode 输出模式选项
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 设置MT6701传感器的输出信号类型。
   * 可以选择模拟输出（电压随角度变化）或PWM输出（占空比随角度变化）。
   */
  Error SetOutMode(const OutMode mode);
  /**
   * @brief 获取输出模式
   * @param mode 输出模式选项的存储指针
   * @return 错误码，成功返回OK，否则参见Error错误码
   *
   * 获取MT6701传感器的当前输出模式。
   */
  Error GetOutMode(OutMode& mode);
};

}  // namespace hortor::drivers::MT6701
