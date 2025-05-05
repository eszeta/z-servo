#include <Arduino.h>
namespace hortor_servo {

struct Resolution {
 public:
  Resolution(uint8_t resolution_bits) : kBits(resolution_bits) {}
  /** @brief 目标分辨率（位数），决定了传感器的精度和量程 */
  const uint8_t kBits;
  /** @brief Counts Per Revolution */
  const uint16_t kEncoderCpr = (1 << kBits);
  /** @brief 角度到原始值的转换系数，用于将角度转换为原始值 */
  const float kAngleToRaw = kEncoderCpr / 360.0f;
  /** @brief 弧度到原始值的转换系数，用于将弧度转换为原始值 */
  const float kRadianToRaw = kEncoderCpr / TWO_PI;
  /** @brief 原始值到角度的转换系数，用于将原始值转换为角度 */
  const float kRawToAngle = 360.0f / kEncoderCpr;
  /** @brief 原始值到弧度的转换系数，用于将原始值转换为弧度 */
  const float kRawToRadian = TWO_PI / kEncoderCpr;
};

}  // namespace hortor_servo