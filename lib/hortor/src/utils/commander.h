// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

namespace hortor::utils {

// 命令回调函数指针定义
typedef void (*CommandCallback)(int argc, char** argv);

/**
 * Commander - 简化的通用命令解析器
 *
 * 提供基本的命令注册和回调执行功能，去除所有特定功能
 * 只保留核心的命令ID匹配和回调机制
 */
class Commander {
 public:
  /**
   * 构造函数
   */
  Commander() = default;

  /**
   * 注册命令
   * @param id - 命令字符标识符
   * @param onCommand - 回调函数指针
   * @param label - 命令描述标签（可选）
   */
  void add(char id, CommandCallback onCommand);

  /**
   * 解析并执行命令
   * @param user_input - 用户输入的命令字符串
   */
  void run(char* user_input);

 private:
  char input_buffer[64]{};          // 存储输入字符串的副本（用于strtok处理）
  CommandCallback call_list[20]{};  // 回调函数数组
  char call_ids[20]{};              // 命令ID数组
  int call_count = 0;               // 已注册命令数量
};

}  // namespace hortor::utils
