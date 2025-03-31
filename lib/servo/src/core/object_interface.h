#pragma once

namespace hortor_servo {

class ObjectInterface {
 public:
  ObjectInterface() = default;

  /** @brief 复制构造函数 */
  ObjectInterface(const ObjectInterface& other) = delete;

  /** @brief 赋值操作符 */
  ObjectInterface& operator=(const ObjectInterface& other) = delete;
};

}  // namespace hortor_servo