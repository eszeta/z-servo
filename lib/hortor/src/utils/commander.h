// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file commander.h
 * @brief 简单串口命令解析（单字符 ID + 空格分隔参数）
 */

#pragma once

#include <Arduino.h>

#include <cstring>

#include "noncopyable.h"

namespace hortor::utils {

typedef void (*CommandCallback)(int argc, char** argv);

/// @brief 串口命令解析器，首字符为命令 ID，后接参数
class Commander : public hortor::Noncopyable {
 public:
  /**
   * @brief 注册命令：id 为首字符，onCommand 为回调
   * @param id 命令首字符
   * @param onCommand 回调 (argc, argv)
   */
  void add(char id, CommandCallback onCommand);
  /**
   * @brief 解析并执行一行输入（首字符匹配则调用对应回调）
   * @param user_input 输入字符串（首字符为命令 ID，后接空格分隔参数）
   */
  void run(char* user_input);

 private:
  char            input_buffer[64]{};
  CommandCallback call_list[20]{};
  char            call_ids[20]{};
  int             call_count = 0;
};

}  // namespace hortor::utils

namespace hortor::utils {

inline void Commander::add(char id, CommandCallback onCommand) {
  if (call_count < 20) {
    call_list[call_count] = onCommand;
    call_ids[call_count]  = id;
    call_count++;
  }
}

inline void Commander::run(char* user_input) {
  char id = user_input[0];

  strncpy(input_buffer, user_input, 63);
  input_buffer[63] = '\0';

  int len = strlen(input_buffer);
  while (len > 0 && (input_buffer[len - 1] == '\n' || input_buffer[len - 1] == '\r')) {
    input_buffer[len - 1] = '\0';
    len--;
  }

  for (int i = 0; i < call_count; i++) {
    if (id == call_ids[i]) {
      char* argv[10];
      int   argc = 0;

      char* token = strtok(&input_buffer[1], " ");
      while (token != nullptr && argc < 10) {
        argv[argc++] = token;
        token        = strtok(nullptr, " ");
      }

      call_list[i](argc, argv);
      break;
    }
  }
}

}  // namespace hortor::utils
