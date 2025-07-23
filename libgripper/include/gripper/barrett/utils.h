#pragma once

#include "gripper/barrett/constants.h"
#include "gripper/barrett/barrett_hand_driver.h"
#include <cmath>

namespace gripper {
namespace barrett {

inline double countsToRadians(int32_t counts, MotorID motor)  {
    if (motor == MotorID::F1 || motor == MotorID::F2 || motor == MotorID::F3) {
        return static_cast<double>(counts) / ENCODER_COUNTS_PER_RADIAN_FINGER;
    } else {
        return static_cast<double>(counts) / ENCODER_COUNTS_PER_RADIAN_SPREAD;
    }
}

inline int32_t radiansToCounts(double radians, MotorID motor)  {
    if (motor == MotorID::F1 || motor == MotorID::F2 || motor == MotorID::F3) {
        return static_cast<int32_t>(radians * ENCODER_COUNTS_PER_RADIAN_FINGER);
    } else {
        return static_cast<int32_t>(radians * ENCODER_COUNTS_PER_RADIAN_SPREAD);
    }
}

inline int8_t prepareVelocity(double velocity_count, uint8_t LCVC)  {
    double vel_shifted = 16 * velocity_count / static_cast<double>(LCVC);
    if (vel_shifted > 127) {
        vel_shifted = 127;
    }
    if (vel_shifted < -128) {
        vel_shifted = -128;
    }
    return static_cast<int8_t>(vel_shifted);
}

inline double velocityRadToCounts(double velocity_rad_per_sec, MotorID motor)  {
    if (motor == MotorID::F1 || motor == MotorID::F2 || motor == MotorID::F3) {
        return velocity_rad_per_sec * ENCODER_COUNTS_PER_RADIAN_FINGER * SAMPLE_TIME;
    } else {

        return velocity_rad_per_sec * ENCODER_COUNTS_PER_RADIAN_SPREAD * SAMPLE_TIME;
    }
}

inline double velocityCountsToRad(int8_t velocity_counts, MotorID motor)  {
    if (motor == MotorID::F1 || motor == MotorID::F2 || motor == MotorID::F3) {
        return (static_cast<double>(velocity_counts) * TICKS_PER_SECOND) / ENCODER_COUNTS_PER_RADIAN_FINGER;
    } else {
        return (static_cast<double>(velocity_counts) * TICKS_PER_SECOND) / ENCODER_COUNTS_PER_RADIAN_SPREAD;
    }
}
} // namespace barrett
} // namespace gripper
