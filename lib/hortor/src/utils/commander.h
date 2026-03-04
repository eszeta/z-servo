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
   * 注册命令
   * @param id - 命令字符标识符
   * @param onCommand - 回调函数指针
   * @param label - 命令描述标签（可选）
   */
  void add(char id, CommandCallback onCommand) {
    if (call_count < 20) {
      call_list[call_count] = onCommand;
      call_ids[call_count] = id;
      call_count++;
    }
  }

  /**
   * 解析并执行命令
   * @param user_input - 用户输入的命令字符串
   */
  void run(char* user_input) {
    char id = user_input[0];

    // 复制到input_buffer
    strncpy(input_buffer, user_input, 63);
    input_buffer[63] = '\0';

    // 清理尾部的换行符
    int len = strlen(input_buffer);
    while (len > 0 &&
           (input_buffer[len - 1] == '\n' || input_buffer[len - 1] == '\r')) {
      input_buffer[len - 1] = '\0';
      len--;
    }

    // 查找匹配的命令
    for (int i = 0; i < call_count; i++) {
      if (id == call_ids[i]) {
        // 局部变量存储参数
        char* argv[10];
        int argc = 0;

        // 分拆参数
        char* token = strtok(&input_buffer[1], " ");
        while (token != nullptr && argc < 10) {
          argv[argc++] = token;
          token = strtok(nullptr, " ");
        }

        // 调用回调
        call_list[i](argc, argv);
        break;
      }
    }
  }

 private:
  char input_buffer[64]{};          // 存储输入字符串的副本（用于strtok处理）
  CommandCallback call_list[20]{};  // 回调函数数组
  char call_ids[20]{};              // 命令ID数组
  int call_count = 0;               // 已注册命令数量
};

}  // namespace hortor::utils
