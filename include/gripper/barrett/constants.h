#pragma once

#include <cmath>

namespace gripper {
namespace barrett {

constexpr double ENCODER_FINGER_MIN = 0;
constexpr double ENCODER_FINGER_MAX = 17500;

constexpr double ENCODER_SPREAD_MIN = 0;
constexpr double ENCODER_SPREAD_MAX = 3150;

constexpr double RADIAN_FINGER_MIN = 0.0;
constexpr double RADIAN_FINGER_MAX = 140.0 * M_PI / 180.0;

constexpr double RADIAN_SPREAD_MIN = 0.0;
constexpr double RADIAN_SPREAD_MAX = M_PI;

constexpr double ENCODER_COUNTS_PER_RADIAN_FINGER =
    (ENCODER_FINGER_MAX - ENCODER_SPREAD_MIN) / (RADIAN_FINGER_MAX - RADIAN_FINGER_MIN);
constexpr double ENCODER_COUNTS_PER_RADIAN_SPREAD =
    (ENCODER_SPREAD_MAX - ENCODER_SPREAD_MIN) / (RADIAN_SPREAD_MAX - RADIAN_SPREAD_MIN);

constexpr double EXTERNAL_CLOCK = 1.25e6;
constexpr double SAMPLE_TIME = 16 * (32) * (1 / EXTERNAL_CLOCK); // From HCTL-1100 docs

constexpr double TICKS_PER_SECOND = 100.0; // Each tick on embedded CPU is 10ms

} // namespace barrett
} // namespace gripper
