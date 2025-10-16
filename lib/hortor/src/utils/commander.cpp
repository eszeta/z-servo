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