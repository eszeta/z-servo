#pragma once

#include <Arduino.h>

#include "./utils/math/pid.h"

namespace hortor_servo {

/**
 * @def CHECK
 * @brief 错误处理辅助宏
 * @param x 要检查的表达式
 *
 * 执行表达式并检查返回的错误码。如果发生错误，立即返回该错误码。
 * 用于简化连续操作的错误处理。
 */
#define CHECK(x)                       \
  do {                                 \
    Error err = (x);                   \
    if (err != Error::kOk) return err; \
  } while (0)

/**
 * @brief 错误码枚举
 *
 * 定义MT6701传感器操作可能返回的错误码，用于诊断问题。
 */
enum class Error : uint8_t {
  /**
   * @brief 成功，操作正常完成
   */
  kOk = 0,
  /**
   * @brief 一般错误，未指定具体原因
   */
  kGeneralErr = 1,
  /**
   * @brief IO错误，通信过程中发生错误
   */
  kIOErr = 3,
  /**
   * @brief 超出范围，参数值超出有效范围
   */
  kOutOfRange = 4,
  /**
   * @brief 无效参数，参数组合无效
   */
  kInvalidParameter = 5,
  /**
   * @brief 未初始化，设备未完成初始化
   */
  kUninitialized = 6,
  /**
   * @brief 无效包，包格式错误
   */
  kInvalidPacket = 7,
  /**
   * @brief 无效指令，指令码无效
   */
  kInvalidInstruction = 8,
  /**
   * @brief 数组越界
   */
  kArrayOutOfRange = 9
};

/**
 * @brief 电机控制类型
 */
enum class ServoMode : uint8_t {
  /**
   * @brief 位置伺服模式
   */
  kPosition = 0,
  /**
   * @brief 恒速模式
   */
  kVelocity = 1,
  /**
   * @brief PWM开环调速度模式
   */
  kPwm = 2,
  /**
   * @brief 步进伺服模式
   */
  kStep = 3
};

/**
 * @brief 电机状态位
 */
namespace ServoStatusMask {
static constexpr uint8_t kVoltage = 1 << 0;
static constexpr uint8_t kSensor = 1 << 1;
static constexpr uint8_t kTemperature = 1 << 2;
static constexpr uint8_t kCurrent = 1 << 3;
static constexpr uint8_t kLoad = 1 << 5;
};  // namespace ServoStatusMask

/**
 * @brief 方向
 */
enum class Direction : int8_t {
  CW = 1,      // clockwise
  CCW = -1,    // counterclockwise
  unknown = 0  // not yet known or invalid state
};

}  // namespace hortor_servo