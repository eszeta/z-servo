// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#include "commander.h"
namespace hortor::utils {

void Commander::add(char id, CommandCallback onCommand) {
  if (call_count < 20) {
    call_list[call_count] = onCommand;
    call_ids[call_count] = id;
    call_count++;
  }
}

void Commander::run(char* user_input) {
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
}  // namespace hortor::utils