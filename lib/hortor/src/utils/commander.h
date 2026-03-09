// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>

#include <cstring>

#include "hortor.h"

namespace hortor::utils {

typedef void (*CommandCallback)(int argc, char** argv);

class Commander : public hortor::Noncopyable {
 public:
  void add(char id, CommandCallback onCommand);
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
