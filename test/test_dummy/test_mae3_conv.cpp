#include <gtest/gtest.h>

#include "mae3_math.hpp"

using mae3::dutyToPosition;

static bool near(std::uint16_t v, std::uint16_t t, std::uint16_t tol) {
  return v >= t - tol && v <= t + tol;
}

TEST(MAE3_Conv, LowNearZero) { EXPECT_LE(dutyToPosition(1U, 4099U), 2U); }

TEST(MAE3_Conv, MidNearHalf) {
  EXPECT_TRUE(near(dutyToPosition(2000U, 2000U), 2048U, 8U));
  EXPECT_TRUE(near(dutyToPosition(2100U, 2100U), 2048U, 16U));
}

TEST(MAE3_Conv, HighNearFull) { EXPECT_GE(dutyToPosition(4090U, 10U), 4090U); }

TEST(MAE3_Conv, WrapEdges) {
  EXPECT_LE(dutyToPosition(2U, 4200U), 3U);
  EXPECT_GE(dutyToPosition(4200U, 2U), 4092U);
}
