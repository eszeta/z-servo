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

#include "register/register_field.h"
#include "servo/types.h"

namespace hortor::protocol {
/**
 * @brief 指令包索引
 */
namespace PacketIndex {
static constexpr uint8_t kHeader1 = 0;
static constexpr uint8_t kHeader2 = 1;
static constexpr uint8_t kId = 2;
static constexpr uint8_t kLength = 3;
static constexpr uint8_t kInstruction = 4;
static constexpr uint8_t kError = 4;
static constexpr uint8_t kParameter = 5;
};  // namespace PacketIndex

constexpr uint8_t kBroadcastId = 0xfe;

template <typename T = uint8_t>
struct __packed ControlTableItem {
  const reg::RegisterField<T> reg;
  const T default_value;

  constexpr ControlTableItem(const uint8_t address,
                             const uint8_t shift,
                             const uint8_t bits,
                             const T default_value)
      : reg(address, shift, bits), default_value(default_value) {}
};

#define CTI_08(name, address, default_value)           \
  static constexpr ControlTableItem<uint8_t> name {    \
    address, 0, 8, static_cast<uint8_t>(default_value) \
  }
#define CTI_16(name, address, default_value)             \
  static constexpr ControlTableItem<uint16_t> name {     \
    address, 0, 16, static_cast<uint16_t>(default_value) \
  }
#define CTI_32(name, address, default_value)             \
  static constexpr ControlTableItem<uint32_t> name {     \
    address, 0, 32, static_cast<uint32_t>(default_value) \
  }

struct ControlTableBlock {
  const uint8_t begin;
  const uint8_t end;

  constexpr uint8_t size() const { return end - begin; }
  constexpr bool InBlock(const uint8_t address, const uint8_t size) const {
    return address < end && address + size > begin;
  }
};

// /**
//  * @brief 伺服寄存器
//  *
//  * 如果功能地址采用两个字节数的数据，低位字节在前面地址，高位字节在后面地址
//  */
// struct ServoRegs {
//   //-----------EEPROM（只读）-------------------

//   /**
//    * @brief 固件主版本号
//    */
//   static constexpr Register kFirmwareMajor{0x00, 0, 8};
//   /**
//    * @brief 固件次版本号
//    */
//   static constexpr Register kFirmwareMinor{0x01, 0, 8};
//   /**
//    * @brief END
//    */
//   static constexpr Register kEnd{0x02, 0, 8};
//   /**
//    * @brief 伺服主版本号
//    */
//   static constexpr Register kServoMajor{0x03, 0, 8};
//   /**
//    * @brief 伺服次版本号
//    */
//   static constexpr Register kServoMinor{0x04, 0, 8};

//   //-----------EEPROM（读写）-------------------

//   /**
//    * @brief 设备ID [0-253]
//    * 单位:号
//    * 总线上唯一的身份识别码，同一总线不可出现重复ID号
//    * 254号(OxFE)为广播ID，广播不返回应答包
//    */
//   static constexpr const Register kId{0x05, 0, 8};
//   /**
//    * @brief 波特率 [0-7]
//    * 0-7分别代表波特率如下:
//    * 1000000，500000，250000，128000，115200，76800，57600，38400
//    */
//   static constexpr const Register kBaudrate{0x06, 0, 8};
//   /**
//    * @brief 返回延时 [0-254]
//    * 单位:2us
//    * 最大可设置返回延时 254*2=508us(STS此地址无功能)
//    */
//   static constexpr const Register kResponseDelay{0x07, 0, 8};
//   /**
//    * @brief 应答状态级别
//    *
//    * 0:除读指令与PING指令外其它指令不返回应答包 1:对所有指令返回应答包
//    */
//   static constexpr const Register kResponseLevel{0x08, 0, 8};
//   /**
//    * @brief 最小角度 [0-4094]
//    * 单位:步
//    * 设置运动行程最小值限制，取值小于最大角度限制，多圈绝对位置控制时此值为0
//    */
//   static constexpr const Register kMinPositionL{0x09, 0, 8};
//   static constexpr const Register kMinPositionH{0x0A, 0, 8};
//   /**
//    * @brief 最大角度 [1-4095]
//    * 单位:步
//    * 设置运动行程最大值限制，取值大于最小角度限制，多圈绝对位置控制时此值为0
//    */
//   static constexpr const Register kMaxPositionL{0x0B, 0, 8};
//   static constexpr const Register kMaxPositionH{0x0C, 0, 8};
//   /**
//    * @brief 最高温度上限 [0-100]
//    * 单位:°C
//    * 最高工作温度限制，如设置为70 则最高温度为70摄氏度，设置精度为1摄氏度
//    */
//   static constexpr const Register kMaxTemperature{0x0D, 0, 8};
//   /**
//    * @brief 最高输入电压 [0-254]
//    * 单位:0.1V
//    * 最高输入电压如设置为254，则最高工作电压限制为25.4V，设置精度为0.1V
//    */
//   static constexpr const Register kMaxVoltage{0x0E, 0, 8};
//   /**
//    * @brief 最低输入电压 [0-254]
//    * 单位:0.1V
//    * 最低输入电压如设置为40，则最低工作电压限制为4.0V，设置精度为0.1V
//    */
//   static constexpr const Register kMinVoltage{0x0F, 0, 8};
//   /**
//    * @brief 最大扭矩 [0-1000]
//    * 单位:0.10%
//    * 设置舵机的最大输出扭矩限制，设1000 = 100% *
//    * 堵转扭力，上电赋值给48号地址转矩限制
//    */
//   static constexpr const Register kMaxTorqueL{0x10, 0, 8};
//   static constexpr const Register kMaxTorqueH{0x11, 0, 8};
//   /**
//    * @brief 选项 特殊功能字节，无特别需求不可修改,详见特殊字节位解析
//    */
//   static constexpr const Register kOption{0x12, 0, 8};
//   /**
//    * @brief 卸载条件
//    *
//    * 对应位设置1为开启相应保护,对应位设置0为关闭相应保护,详见特殊字节位解析
//    */
//   static constexpr const Register kUnloadCondition{0x13, 0, 8};
//   /**
//    * @brief LED报警条件 [0-254]
//    *
//    * 对应位设置1为开启闪灯报警,对应位设置0为关闭闪灯报警,详见特殊字节位解析
//    */
//   static constexpr const Register kLedAlarmCondition{0x14, 0, 8};
//   /**
//    * @brief 位置环P比例系数 [0-254]
//    * 控制电机的比例系数
//    */
//   static constexpr const Register kPosPidKp{0x15, 0, 8};
//   /**
//    * @brief 位置环D微分系数 [0-254]
//    * 控制电机的微分系数
//    */
//   static constexpr const Register kPosPidKd{0x16, 0, 8};
//   /**
//    * @brief 位置环I积分系数 [0-254]
//    * 单位:0.1
//    * 控制电机的积分系数
//    */
//   static constexpr const Register kPosPidKi{0x17, 0, 8};
//   /**
//    * @brief 最小启动力 [0-254]
//    * 单位:0.1%
//    * 设置舵机的最小输出启动扭矩，设10 = 1% * 堵转扭力
//    */
//   static constexpr const Register kMinStartupForce{0x18, 0, 8};
//   /**
//    * @brief 积分限制值 [0-254]
//    * 最大积分值=积分限制值*4,0表示关闭积分限制功能，位置模式0与步进模式3生效
//    */
//   static constexpr const Register kPosPidILimit{0x19, 0, 8};
//   /**
//    * @brief 顺时针不灵敏区 [0-32]
//    * 单位:步
//    * 最小单位为一个最小分辨角度
//    */
//   static constexpr const Register kCWInsensitiveArea{0x1A, 0, 8};
//   /**
//    * @brief 逆时针不灵敏区 [0-32]
//    * 单位:步
//    * 最小单位为一个最小分辨角度
//    */
//   static constexpr const Register kCCWInsensitiveArea{0x1B, 0, 8};
//   /**
//    * @brief 保护电流 [0-511]
//    * 单位:6.5mA
//    * 最大可设置电流为 500 * 6.5mA= 3250mA
//    */
//   static constexpr const Register kCurrentProtectionThL{0x1C, 0, 8};
//   static constexpr const Register kCurrentProtectionThH{0x1D, 0, 8};
//   /**
//    * @brief 角度分辨率 [1-3]
//    * 对传感器最小分辨角度（度/步）的放大系数，修改此值可以扩展控制圈数,多圈控制时需修改地址0x12参数BIT4置1，当前位置反馈值会被修改为大角度反馈
//    */
//   static constexpr const Register kAngularResolution{0x1E, 0, 8};
//   /**
//    * @brief 位置校正 [-2047-2047]
//    * 单位:步
//    * BIT11为方向位，表示正负方向，其他位可表示范围为0-2047步
//    */
//   static constexpr const Register kPositionCorrectionL{0x1F, 0, 8};
//   static constexpr const Register kPositionCorrectionH{0x20, 0, 8};
//   /**
//    * @brief 运行模式 [0-3]
//    * 0：位置伺服模式
//    * 1：电机恒速模式,用参数0x2E运行速度参数控制，最高位BIT15为方向位
//    * 2：PWM开环调速度模式，用参数0x2C 运行时间参数控制，BIT10为方向位
//    * 3：步进伺服模式，用参数0x2A
//    * 目标位置表示步进步数，最高位BIT15为方向位，工作在模式3时，必须把0x9与0xB
//    * 最小最大角度限制设为0，否则不能无限步进
//    */
//   static constexpr const Register kMode{0x21, 0, 8};
//   /**
//    * @brief 保护扭矩 [0-100]
//    * 单位:1.0%
//    * 进入过载保护后输出扭矩，如设20表示20%的最大扭矩
//    */
//   static constexpr const Register kTorqueProtectionTh{0x22, 0, 8};
//   /**
//    * @brief 保护时间 [0-254]
//    * 单位:10ms
//    * 当前负载输出超过过载扭力并保持的计时时长，如设200表示2秒，最大可设2.5秒
//    */
//   static constexpr const Register kTorqueProtectionTime{0x23, 0, 8};
//   /**
//    * @brief 过载扭矩 [0-100]
//    * 单位:1.0%
//    * 启动过载保护时间计时的最大扭力阀值，如设80表示80%的最大扭矩
//    */
//   static constexpr const Register kOverloadTorque{0x24, 0, 8};
//   /**
//    * @brief 速度闭环P比例系数 [0-254]
//    * 单位:0.1
//    * 电机恒速模式（模式1）下,速度环比例系数
//    */
//   static constexpr const Register kVelPidKp{0x25, 0, 8};
//   /**
//    * @brief 过流保护时间 [0-254]
//    * 单位:10ms
//    * 最大可设置254 * 10ms = 2540ms
//    */
//   static constexpr const Register kOvercurrentProtectionTime{0x26, 0, 8};
//   /**
//    * @brief 速度闭环I积分系数 [0-254]
//    * 单位:0.1
//    * 电机恒速模式（模式1）下,速度环积分系数（变动备注：速度闭环I积分系数缩小10倍较3.6版本）
//    */
//   static constexpr const Register kVelPidKi{0x27, 0, 8};

//   //-----------RAM（读写）-------------------

//   /**
//    * @brief 扭矩开关 [0-128]
//    * 写0：关闭扭力输出； 写1：打开扭力输出；写128：任意当前位置较正为2048
//    */
//   static constexpr const Register kTorqueEnable{0x28, 0, 8};
//   /**
//    * @brief 加速度 [0-254]
//    * 单位:100步/s^2
//    * 如设置为10 则按1000步每秒平方的加减速度变速
//    */
//   static constexpr const Register kGoalAcceleration{0x29, 0, 8};
//   /**
//    * @brief 目标位置 [-32766-32766]
//    * 单位:步
//    * 每步为一个最小分辨角度，绝对位置控制方式，最大对应最大有效角度
//    */
//   static constexpr const Register kGoalPositionL{0x2A, 0, 8};
//   static constexpr const Register kGoalPositionH{0x2B, 0, 8};
//   /**
//    * @brief 运行时间 [0-1000]
//    * 单位:0.10%
//    * PWM开环调速度模式下，取值范围50-1000，BIT10为方向位
//    */
//   static constexpr const Register kGoalTimeL{0x2C, 0, 8};
//   static constexpr const Register kGoalTimeH{0x2D, 0, 8};
//   /**
//    * @brief 运行速度 [-32766-32766]
//    * 单位:步/s
//    * 单位时间（每秒）内运动的步数，50步/秒= 0.732 RPM(圈每分钟）
//    */
//   static constexpr const Register kGoalVelocityL{0x2E, 0, 8};
//   static constexpr const Register kGoalVelocityH{0x2F, 0, 8};
//   /**
//    * @brief 转矩限制 [0-1000]
//    * 单位:1.0%
//    * 上电初始值会由最大扭矩（0x10）赋值，用户可程序修改此值来控制最大扭矩的输出
//    */
//   static constexpr const Register kTorqueLimitL{0x30, 0, 8};
//   static constexpr const Register kTorqueLimitH{0x31, 0, 8};
//   /**
//    * @brief 锁标志 [0-1]
//    * 写0关闭写入锁，写入EEPROM地址的值掉电保存
//    * 写1打开写入锁，写入EEPROM地址的值掉电不保存
//    */
//   static constexpr const Register kWriteLock{0x37, 0, 8};

//   //-----------RAM（只读）-------------------

//   /**
//    * @brief 当前位置
//    * 单位:步
//    * 反馈当前所处位置的步数，每步为一个最小分辨角度；绝对位置控制方式，最大值对应最大有效角度
//    */
//   static constexpr const Register kPresentPositionL{0x38, 0, 8};
//   static constexpr const Register kPresentPositionH{0x39, 0, 8};
//   /**
//    * @brief 当前速度
//    * 单位:步/s
//    * 反馈当前电机转动的速度，单位时间（每秒）内运动的步数
//    */
//   static constexpr const Register kPresentVelocityL{0x3A, 0, 8};
//   static constexpr const Register kPresentVelocityH{0x3B, 0, 8};
//   /**
//    * @brief 当前负载
//    * 单位:0.1%
//    * 当前控制输出驱动电机的电压占空比
//    */
//   static constexpr const Register kPresentLoadL{0x3C, 0, 8};
//   static constexpr const Register kPresentLoadH{0x3D, 0, 8};
//   /**
//    * @brief 当前电压
//    * 单位:0.1V
//    * 当前舵机工作电压
//    */
//   static constexpr const Register kPresentVoltage{0x3E, 0, 8};
//   /**
//    * @brief 当前温度
//    * 单位:°C
//    * 当前舵机内部工作温度
//    */
//   static constexpr const Register kPresentTemperature{0x3F, 0, 8};
//   /**
//    * @brief 异步写标志 [0-1]
//    * 采用异步写指令时，标志位被置1表示相应错误出现,对应位0为无相应该错误,详见特殊字节位解析
//    */
//   static constexpr const Register kAsyncWriteSt{0x40, 0, 8};
//   /**
//    * @brief 舵机状态 [0-1]
//    * 对应位被置1表示相应错误出现,对应位0为无相应该错误,详见特殊字节位解析
//    */
//   static constexpr const Register kStatus{0x41, 0, 8};
//   /**
//    * @brief 移动标志 [0-1]
//    * 舵机在运动时标志为1，舵机停止时为0
//    */
//   static constexpr const Register kMoving{0x42, 0, 8};
//   /**
//    * @brief 当前电流
//    * 单位:6.5mA
//    * 最大可测量电流为 500 * 6.5mA= 3250mA
//    */
//   static constexpr const Register kPresentCurrentL{0x45, 0, 8};
//   static constexpr const Register kPresentCurrentH{0x46, 0, 8};

//   //-----------内部EEPROM（读写）-------------------
//   /**
//    * @brief 传感器旋转方向
//    * 0:正转 1:反转
//    */
//   static constexpr const Register kSensorDirection{0x60, 0, 8};
//   /**
//    * @brief 电机旋转方向
//    * 0:正转 1:反转
//    */
//   static constexpr const Register kMotorDirection{0x61, 0, 8};
//   /**
//    * @brief ADC采样电阻
//    * 单位:100Ω
//    */
//   static constexpr const Register kAdcShuntResistor{0x62, 0, 8};
//   /**
//    * @brief ADC电流镜系数
//    * 单位:100
//    */
//   static constexpr const Register kAdcCurrentFactor{0x63, 0, 8};
//   /**
//    * @brief 位置PID控制器前馈增益
//    * 单位:0.1
//    */
//   static constexpr const Register kPosPidKf{0x64, 0, 8};
//   /**
//    * @brief 位置滤波器
//    * 单位:0.001
//    */
//   static constexpr const Register kPosFilter{0x65, 0, 8};
//   /**
//    * @brief 电流滤波器
//    * 单位:0.001
//    */
//   static constexpr const Register kCurrentFilter{0x66, 0, 8};
//   /**
//    * @brief 速度滤波器
//    * 单位:0.001
//    */
//   static constexpr const Register kVelocityFilter{0x67, 0, 8};
// };

// struct RegsDefaultValues {
//   // EEPROM（只读）
//   static constexpr uint8_t kDefFirmwareMajor = 1;  // 0x00
//   static constexpr uint8_t kDefFirmwareMinor = 0;  // 0x01
//   static constexpr uint8_t kDefEnd = 0;            // 0x02
//   static constexpr uint8_t kDefServoMajor = 0x77;  // 0x03
//   static constexpr uint8_t kDefServoMinor = 0x07;  // 0x04

//   // EEPROM（读写）
//   static constexpr uint8_t kDefId = 1;                             // 0x05
//   static constexpr uint8_t kDefBaudrate = 0;                       // 0x06
//   static constexpr uint16_t kDefResponseDelay = 500;               // 0x07
//   static constexpr bool kDefResponseLevel = true;                  // 0x08
//   static constexpr uint16_t kDefMinPosition = 0;                   //
//   0x09-0x0A static constexpr uint16_t kDefMaxPosition = 4095; // 0x0B-0x0C
//   static constexpr uint8_t kDefMaxTemperature = 70;                // 0x0D
//   static constexpr float kDefMaxVoltage = 7.4f;                    // 0x0E
//   static constexpr float kDefMinVoltage = 4.0f;                    // 0x0F
//   static constexpr uint16_t kDefMaxTorque = 1;                     //
//   0x10-0x11 static constexpr uint8_t kDefOption = 12; // 0x12 static
//   constexpr uint8_t kDefUnloadCondition = 44;               // 0x13 static
//   constexpr uint8_t kDefLedAlarmCondition = 47;             // 0x14 static
//   constexpr float kDefPosPidKp = 3.2f;                      // 0x15 static
//   constexpr float kDefPosPidKd = 3.2f;                      // 0x16 static
//   constexpr float kDefPosPidKi = 0.0f;                      // 0x17 static
//   constexpr float kDefMinStartupForce = 0.0f;               // 0x18 static
//   constexpr float kDefPosPidILimit = 0.0f;                  // 0x19 static
//   constexpr uint8_t kDefCWInsensitiveArea = 1;              // 0x1A static
//   constexpr uint8_t kDefCCWInsensitiveArea = 1;             // 0x1B static
//   constexpr float kDefCurrentProtectionTh = 0.0f;           // 0x1C-0x1D
//   static constexpr uint8_t kDefAngularResolution = 1;              // 0x1E
//   static constexpr uint16_t kDefPositionCorrection = 0;            //
//   0x1F-0x20 static constexpr ServoMode kDefMode = ServoMode::kPosition; //
//   0x21 static constexpr float kDefTorqueProtectionTh = 0.2f;            //
//   0x22 static constexpr uint16_t kDefTorqueProtectionTime = 2000;       //
//   0x23 static constexpr float kDefOverloadTorque = 0.0f;                //
//   0x24 static constexpr float kDefVelPidKp = 0.0f;                      //
//   0x25 static constexpr uint16_t kDefOvercurrentProtectionTime = 2000;  //
//   0x26 static constexpr float kDefVelPidKi = 0.0f;                      //
//   0x27

//   // 内部EEPROM（读写）
//   static constexpr Direction kDefSensorDirection = Direction::CW;  // 0x60
//   static constexpr Direction kDefMotorDirection = Direction::CCW;  // 0x61
//   static constexpr uint16_t kDefAdcShuntResistor = 1000;           // 0x62
//   static constexpr uint16_t kDefAdcCurrentFactor = 1500;           // 0x63
//   static constexpr float kDefPosPidFf = 0;                         // 0x64
//   static constexpr float kDefPosFilter = 0.01f;                    // 0x65
//   static constexpr float kDefCurrentFilter = 0;                    // 0x66
//   static constexpr float kDefVelocityFilter = 0.01f;               // 0x67
// };

struct Instruction {
  static constexpr uint8_t kPing = 0x01;
  static constexpr uint8_t kReadData = 0x02;
  static constexpr uint8_t kWriteData = 0x03;
  static constexpr uint8_t kRegWrite = 0x04;
  static constexpr uint8_t kAction = 0x05;
  static constexpr uint8_t kReset = 0x06;
  static constexpr uint8_t kReboot = 0x08;
  static constexpr uint8_t kSyncWrite = 0x82;
  static constexpr uint8_t kBulkRead = 0x92;
};

}  // namespace hortor::protocol