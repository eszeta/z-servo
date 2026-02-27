// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "regmap/reg_field.h"

namespace hortor::drivers::LSM6DSOW {
/**
 * @brief LSM6DSOW传感器I2C地址
 * @note 默认地址为0x6A
 */
constexpr uint8_t kI2CAddress = 0x6A;

/**
 * @enum ODR_XL
 * @brief 加速度计ODR寄存器设置
 */
enum class ODR_XL {
  kPowerDown = 0b0000,
  k12_5Hz = 0b0001,
  k26Hz = 0b0010,
  k52Hz = 0b0011,
  k104Hz = 0b0100,
  k208Hz = 0b0101,
  k416Hz = 0b0110,
  k833Hz = 0b0111,
  k1660Hz = 0b1000,
  k3330Hz = 0b1001,
  k6660Hz = 0b1010,
  k1_6Hz_12_5Hz = 0b1011,
};

/**
 * @enum FS_XL
 * @brief 加速度计满量程选择
 * @note (默认) 2g
 */
enum class FS_XL {
  k2g = 0b00,
  k16g_2g = 0b01,
  k4g = 0b10,
  k8g = 0b11,
};

/**
 * @enum LPF2_XL_EN
 * @brief 加速度计高分辨率选择
 */
enum class LPF2_XL_EN {
  /**
   * @brief 选择第一级数字滤波器输出（默认）
   */
  kDisabled = 0b0,
  /**
   * @brief 选择LPF2第二级滤波器输出
   */
  kEnabled = 0b1,
};

/**
 * @enum ODR_G
 * @brief 陀螺仪ODR配置设置
 */
enum class ODR_G {
  kPowerDown = 0b0000,
  k12_5Hz = 0b0001,
  k26Hz = 0b0010,
  k52Hz = 0b0011,
  k104Hz = 0b0100,
  k208Hz = 0b0101,
  k416Hz = 0b0110,
  k833Hz = 0b0111,
  k1660Hz = 0b1000,
  k3330Hz = 0b1001,
  k6660Hz = 0b1010,
};

/**
 * @enum FS_G
 * @brief 陀螺仪链满量程选择
 */
enum class FS_G {
  k250dps = 0b00,
  k500dps = 0b01,
  k1000dps = 0b10,
  k2000dps = 0b11,
};

/**
 * @enum FS_125
 * @brief 选择陀螺仪链满量程125 dps
 */
enum class FS_125 {
  /**
   * @brief 通过FS_G位选择满量程
   */
  kDisabled = 0b00,
  /**
   * @brief 满量程设置为125 dps
   */
  kEnabled = 0b01,
};

/**
 * @brief 禁用陀螺仪的高性能工作模式。默认值：0
 */
enum class G_HM_MODE {
  /**
   * @brief 启用高性能工作模式
   */
  kEnabled = 0b0,
  /**
   * @brief 禁用高性能工作模式
   */
  kDisabled = 0b1,
};

/**
 * @brief 启用陀螺仪数字高通滤波器。仅当陀螺仪处于HP模式时，滤波器才会启用。
 * @note 默认值：0
 */
enum class HP_EN_G {
  /**
   * @brief 高通滤波器禁用
   */
  kDisabled = 0b0,
  /**
   * @brief 高通滤波器启用
   */
  kEnabled = 0b1,
};

/**
 * @brief 陀螺仪数字高通滤波器截止频率选择
 * @note 默认值：00
 */
enum class HPM_G {
  k16mHz = 0b00,
  k65mHz = 0b01,
  k260mHz = 0b10,
  k1_04Hz = 0b11,
};

/**
 * @brief 启用加速度计用户偏移校正模块
 * @note 默认值：0
 */
enum class USR_OFF_ON_OUT {
  /**
   * @brief 绕过加速度计用户偏移校正模块
   */
  kDisabled = 0b0,
  /**
   * @brief 启用加速度计用户偏移校正模块
   */
  kEnabled = 0b1,
};

/**
 * @brief 加速度计LPF2和高通滤波器配置及截止频率设置
 */
enum class HPCF_XL {
  k0 = 0b000,
  k1 = 0b001,
  k2 = 0b010,
  k3 = 0b011,
  k4 = 0b100,
  k5 = 0b101,
  k6 = 0b110,
  k7 = 0b111,
};

/**
 * @brief 启用加速度计高通滤波器参考模式
 * @note 仅在高通路径有效 - HP_SLOPE_XL_EN位必须为'1'
 * @note 默认值：0
 */
enum class HP_REF_MODE_XL {
  kDisabled = 0b0,
  kEnabled = 0b1,
};

/**
 * @brief 启用加速度计LPF2和HPF快速稳定模式
 * @note 写入此位后，滤波器在第二个样本后设置
 * @note 仅在设备退出掉电模式时有效
 * @note 默认值：0
 */
enum class FASTSETTL_MODE_XL {
  kDisabled = 0b0,
  kEnabled = 0b1,
};

/**
 * @brief 加速度计斜率滤波器/高通滤波器选择
 */
enum class HP_SLOPE_XL_EN {
  kDisabled = 0b0,
  kEnabled = 0b1,
};

/**
 * @brief 6D功能的LPF2选择
 * @note 默认值：0
 */
enum class LOW_PASS_ON_6D {
  /**
   * @brief ODR/2低通滤波数据发送到6D中断功能
   */
  kDisabled = 0b0,
  /**
   * @brief LPF2输出数据发送到6D中断功能
   */
  kEnabled = 0b1,
};

/**
 * @brief 陀螺仪控制寄存器，用于控制陀螺仪的工作模式
 */
namespace LSM6DSOWRegs {
using RegFieldU08 = regmap::RegFieldU08;
/**
   * @brief WHO_AM_I寄存器
   * @note 这是一个只读寄存器。其值固定为6Ch。
   */
constexpr RegFieldU08 kWHO_AM_I{0X0F, 0, 8};

/**
   * @brief CTRL1_XL (10h)
   */
constexpr RegFieldU08 kCTRL1_XL{0X10, 0, 8};
/**
   * @brief 加速度计ODR选择
   */
constexpr RegFieldU08 kODR_XL{0X10, 4, 4};
/**
   * @brief 加速度计满量程选择
   */
constexpr RegFieldU08 kFS_XL{0X10, 2, 2};
/**
   * @brief 加速度计高分辨率选择
   */
constexpr RegFieldU08 kLPF2_XL_EN{0X10, 1, 1};

/**
   * @brief CTRL2_G (11h)
   */
constexpr RegFieldU08 kCTRL2_G{0X11, 0, 8};
/**
   * @brief 陀螺仪ODR选择
   */
constexpr RegFieldU08 kODR_G{0X11, 4, 4};
/**
   * @brief 陀螺仪满量程选择
   */
constexpr RegFieldU08 kFS_G{0X11, 2, 2};
/**
   * @brief 陀螺仪125 dps满量程选择
   */
constexpr RegFieldU08 kFS_125{0X11, 1, 1};

/**
   * @brief CTRL7_G (16h)
   */
constexpr RegFieldU08 kCTRL7_G{0X16, 0, 8};
/**
   * @brief 陀螺仪高性能工作模式选择
   */
constexpr RegFieldU08 kG_HM_MODE{0X16, 7, 1};
/**
   * @brief 陀螺仪高通滤波器使能
   */
constexpr RegFieldU08 kHP_EN_G{0X16, 6, 1};
/**
   * @brief 陀螺仪高通滤波器截止频率选择
   */
constexpr RegFieldU08 kHPM_G{0X16, 4, 2};
/**
   * @brief 加速度计用户偏移校正模块使能
   */
constexpr RegFieldU08 kUSR_OFF_ON_OUT{0X16, 1, 1};

/**
   * @brief CTRL8_XL (17h)
   */
constexpr RegFieldU08 kCTRL8_XL{0X17, 0, 8};
/**
   * @brief 加速度计LPF2和高通滤波器配置及截止频率设置
   */
constexpr RegFieldU08 kHPCF_XL{0X17, 5, 3};
/**
   * @brief 加速度计高通滤波器参考模式使能
   */
constexpr RegFieldU08 kHP_REF_MODE_XL{0X17, 4, 1};
/**
   * @brief 加速度计LPF2和HPF快速稳定模式使能
   */
constexpr RegFieldU08 kFASTSETTL_MODE_XL{0X17, 3, 1};
/**
   * @brief 加速度计斜率滤波器/高通滤波器选择
   */
constexpr RegFieldU08 kHP_SLOPE_XL_EN{0X17, 2, 1};
/**
   * @brief 6D功能的LPF2选择
   */
constexpr RegFieldU08 kLOW_PASS_ON_6D{0X17, 0, 1};

/**
   * @brief STATUS_REG (1Eh)
   */
constexpr RegFieldU08 kSTATUS_REG{0X1E, 0, 8};
/**
   * @brief 温度数据可用标志
   */
constexpr RegFieldU08 kTDA{0x1E, 2, 1};
/**
   * @brief 陀螺仪数据可用标志
   */
constexpr RegFieldU08 kGDA{0x1E, 1, 1};
/**
   * @brief 加速度计数据可用标志
   */
constexpr RegFieldU08 kXLDA{0x1E, 0, 1};

/**
   * @brief 温度数据输出寄存器
   */
constexpr RegFieldU08 kOUT_TEMP{0x20, 0, 8};
/**
   * @brief 温度数据输出寄存器低字节
   */
constexpr RegFieldU08 kOUT_TEMP_L{0x20, 0, 8};
/**
   * @brief 温度数据输出寄存器高字节
   */
constexpr RegFieldU08 kOUT_TEMP_H{0x21, 0, 8};

/**
   * @brief 陀螺仪X轴角速度数据输出寄存器
   */
constexpr RegFieldU08 kOUTX_G{0x22, 0, 8};
/**
   * @brief 陀螺仪X轴角速度数据输出寄存器低字节
   */
constexpr RegFieldU08 kOUTX_L_G{0x22, 0, 8};
/**
   * @brief 陀螺仪X轴角速度数据输出寄存器高字节
   */
constexpr RegFieldU08 kOUTX_H_G{0x23, 0, 8};

/**
   * @brief 陀螺仪Y轴角速度数据输出寄存器
   */
constexpr RegFieldU08 kOUTY_G{0x24, 0, 8};
/**
   * @brief 陀螺仪Y轴角速度数据输出寄存器低字节
   */
constexpr RegFieldU08 kOUTY_L_G{0x24, 0, 8};
/**
   * @brief 陀螺仪Y轴角速度数据输出寄存器高字节
   */
constexpr RegFieldU08 kOUTY_H_G{0x25, 0, 8};

/**
   * @brief 陀螺仪Z轴角速度数据输出寄存器
   */
constexpr RegFieldU08 kOUTZ_G{0x26, 0, 8};
/**
   * @brief 陀螺仪Z轴角速度数据输出寄存器低字节
   */
constexpr RegFieldU08 kOUTZ_L_G{0x26, 0, 8};
/**
   * @brief 陀螺仪Z轴角速度数据输出寄存器高字节
   */
constexpr RegFieldU08 kOUTZ_H_G{0x27, 0, 8};

/**
   * @brief 加速度计X轴加速度数据输出寄存器
   */
constexpr RegFieldU08 kOUTX_A{0x28, 0, 8};
/**
   * @brief 加速度计X轴加速度数据输出寄存器低字节
   */
constexpr RegFieldU08 kOUTX_L_A{0x28, 0, 8};
/**
   * @brief 加速度计X轴加速度数据输出寄存器高字节
   */
constexpr RegFieldU08 kOUTX_H_A{0x29, 0, 8};

/**
   * @brief 加速度计Y轴加速度数据输出寄存器
   */
constexpr RegFieldU08 kOUTY_A{0x2A, 0, 8};
/**
   * @brief 加速度计Y轴加速度数据输出寄存器低字节
   */
constexpr RegFieldU08 kOUTY_L_A{0x2A, 0, 8};
/**
   * @brief 加速度计Y轴加速度数据输出寄存器高字节
   */
constexpr RegFieldU08 kOUTY_H_A{0x2B, 0, 8};

/**
   * @brief 加速度计Z轴加速度数据输出寄存器
   */
constexpr RegFieldU08 kOUTZ_A{0x2C, 0, 8};
/**
   * @brief 加速度计Z轴加速度数据输出寄存器低字节
   */
constexpr RegFieldU08 kOUTZ_L_A{0x2C, 0, 8};
/**
   * @brief 加速度计Z轴加速度数据输出寄存器高字节
   */
constexpr RegFieldU08 kOUTZ_H_A{0x2D, 0, 8};
};  // namespace LSM6DSOWRegs

}  // namespace hortor::drivers::LSM6DSOW