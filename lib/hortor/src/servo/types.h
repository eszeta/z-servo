// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file types.h
 * @brief 舵机相关类型（工作模式、方向、位域状态）
 */

#pragma once

#include <Arduino.h>

namespace hortor::servo {

/**
 * @brief 伺服电机工作模式枚举
 *
 * 定义了伺服电机支持的各种控制模式，每种模式对应不同的控制策略和参数。
 * 更改工作模式前必须先禁用扭矩输出（Torque Enable = 0）。
 */
enum class OperatingMode : uint8_t {
  /**
   * @brief 电流控制模式
   *
   * 直接控制电机输出电流，实现力矩控制。
   * - 控制参数：Goal Current（目标电流）
   * - 输出特性：力矩与电流成正比
   * - 应用场景：力矩控制、柔顺控制、力反馈
   */
  kCurrent = 0,

  /**
   * @brief 速度控制模式
   *
   * 控制电机转速，实现速度伺服。
   * - 控制参数：Goal Velocity（目标速度）
   * - 输出特性：恒定转速输出
   * - 应用场景：轮式机器人、传送带、风扇控制
   */
  kVelocity = 1,

  /**
   * @brief 位置控制模式（单圈）
   *
   * 控制电机位置，实现位置伺服，限制在单圈范围内。
   * - 控制参数：Goal Position（目标位置）
   * - 位置范围：0-4095（对应0-360°）
   * - 限制条件：受 Min/Max Position Limit 限制
   * - 应用场景：关节控制、单圈定位、舵机应用
   */
  kPosition = 3,

  /**
   * @brief 扩展位置控制模式（多圈）
   *
   * 支持多圈位置控制，允许电机连续旋转。
   * - 控制参数：Goal Position（目标位置）
   * - 位置范围：-1,048,575 ~ 1,048,575（对应-256 ~ 256圈）
   * - 限制条件：不受 Min/Max Position Limit 限制
   * - 应用场景：多圈定位、旋转计数、连续旋转应用
   */
  kExtendedPosition = 4,

  /**
   * @brief 电流限制位置控制模式
   *
   * 位置控制与电流限制相结合，提供安全的位置控制。
   * - 控制参数：Goal Position（目标位置）、Goal Current（最大电流限制）
   * - 输出特性：位置控制 + 电流保护
   * - 安全特性：遇到阻力时电流不超过设定值
   * - 应用场景：柔性抓取、防撞控制、安全定位
   */
  kCurrentPosition = 5,

  /**
   * @brief PWM 开环控制模式
   *
   * 直接控制 PWM 占空比，不进行闭环控制。
   * - 控制参数：Goal PWM（PWM 占空比）
   * - 输出特性：开环控制，无反馈
   * - 应用场景：开环控制、调试测试、简单速度控制
   */
  kPwm = 16
};

/**
 * @brief 方向
 */
enum class Reverse : int8_t {
  kNormal  = 1,   ///< 顺时针
  kReverse = -1,  ///< 逆时针
};

/**
 * @brief 驱动模式
 */
union DriveModeBits {
  uint8_t value = 0;
  struct {
    bool moto_reverse_mode    : 1;  // 位0 0: 正转, 1: 反转
    bool encoder_reverse_mode : 1;  // 位1 0: 正转, 1: 反转
    bool reserved_bit2        : 1;  // 位2 保留
    bool reserved_bit3        : 1;  // 位3 保留
    bool reserved_bit4        : 1;  // 位4 保留
    bool reserved_bit5        : 1;  // 位5 保留
    bool reserved_bit6        : 1;  // 位6 保留
    bool reserved_bit7        : 1;  // 位7 保留
  };
};

/**
 * @brief 运动状态
 *
 * 位布局与 Dynamixel XL330 Moving Status(123) 对齐：
 *   Bit0: In-Position
 *   Bit1: Profile Ongoing（Profile 正在执行中）
 *   Bit2: 保留
 *   Bit3: Following Error（跟随误差过大）
 *   Bit4-5: Profile 类型（0=Step, 1=Rect, 2=Triangular, 3=Trapezoidal）
 *   Bit6-7: 保留
 */
union MovingStatusBits {
  uint8_t value = 0;
  struct {
    bool    in_position      : 1;  // 位0: 已到达目标位置
    bool    profile_ongoing  : 1;  // 位1: Profile 执行中
    bool    reserved_bit2    : 1;  // 位2: 保留
    bool    following_error  : 1;  // 位3: 跟随误差
    uint8_t profile_type     : 2;  // 位4-5: Profile类型(0=Step,1=Rect,2=Tri,3=Trap)
    uint8_t reserved_bits6_7 : 2;  // 位6-7: 保留
  };
};

/**
 * @brief 关断条件
 */
union ShutdownBits {
  uint8_t value = 0;
  struct {
    bool input_voltage_error    : 1;  // 位0: 输入电压超出范围
    bool overheating_error      : 1;  // 位1: 温度超过上限
    bool motor_encoder_error    : 1;  // 位2: 编码器故障
    bool electrical_shock_error : 1;  // 位3: 电气冲击
    bool overload_error         : 1;  // 位4: 过载
  };
};

/**
 * @brief 硬件错误状态
 */
union HardwareErrorStatusBits {
  uint8_t value = 0;
  struct {
    bool input_voltage_error    : 1;  // 位0: 输入电压超出范围
    bool overheating_error      : 1;  // 位1: 温度超过上限
    bool motor_encoder_error    : 1;  // 位2: 编码器故障
    bool electrical_shock_error : 1;  // 位3: 电气冲击
    bool overload_error         : 1;  // 位4: 过载
    bool angle_limit_error      : 1;  // 位5: 角度超出范围
    bool range_error            : 1;  // 位6: 范围错误
  };
};
}  // namespace hortor::servo