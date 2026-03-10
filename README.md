# Hortor-Servo

Hortor-Servo 是一个基于 Arduino 的舵机控制系统，支持 STM32。该系统提供了精确的位置控制、速度控制和电流检测功能，适用于各类舵机应用场景。

## 核心特性

- 多模式控制
  - 位置伺服模式：精确的位置控制
  - 恒速模式：稳定的速度控制
  - PWM开环调速度模式：简单的速度控制
  - 步进伺服模式：步进电机控制

- 丰富的保护功能
  - 过流保护
  - 过温保护
  - 过压保护
  - 堵转保护
  - 位置限位保护

- 精确的控制算法
  - PID位置控制
  - 速度闭环控制
  - 前馈控制
  - 死区控制
  - 输出限幅

- 多种传感器支持
  - MT6701（I2C接口）
  - MA330（SPI接口）
  - 支持14位分辨率

- 多种驱动芯片支持
  - DRV8231A
  - MP6515

## 开发环境

本项目使用 PlatformIO 构建系统，支持以下MCU：

- STM32G431CB
- STM32G030C8T6

## 许可证

本项目采用Apache License 2.0许可证 - 详情请参阅[LICENSE](LICENSE)文件。

## 致谢

感谢以下开源项目对本项目的启发：

- [Arduino-FOC](https://github.com/simplefoc/Arduino-FOC)
- [Arduino_LSM6DSOX](https://github.com/arduino-libraries/Arduino_LSM6DSOX)
- [SMSMod](https://github.com/pat92fr/SMSMod)
- [STS_servos](https://github.com/matthieuvigne/STS_servos)

## 贡献指南

欢迎提交 Issue 和 Pull Request 来帮助改进这个项目。
