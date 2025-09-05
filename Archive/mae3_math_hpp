#pragma once
#include <cstdint>

namespace mae3 {
struct MathConstants final {
  static constexpr std::uint32_t kResolution = 4096U;
  static constexpr std::uint16_t kMaxIndex = 4095U;
};

constexpr std::uint16_t dutyToPosition(std::uint32_t ton_us,
                                       std::uint32_t toff_us) noexcept {
  const std::uint32_t period = ton_us + toff_us;
  if (period == 0U) {
    return 0U;
  }
  const std::uint32_t raw =
      (ton_us * (MathConstants::kResolution + 1U)) / period;
  const int pos = static_cast<int>(raw) - 1;
  return static_cast<std::uint16_t>(
      pos < 0 ? 0
              : (pos > static_cast<int>(MathConstants::kMaxIndex)
                     ? MathConstants::kMaxIndex
                     : pos));
}
}  // namespace mae3
