#pragma once

#include "gripper/barrett/barrett_hand_driver.h"
#include <memory>
#include <atomic>
#include <mutex>

namespace gripper {
namespace barrett {

constexpr double ENCODER_COUNTS_PER_RADIAN_FINGER = 8010.17763;
constexpr double ENCODER_COUNTS_PER_RADIAN_SPREAD = 1001.40056;

/**
 * @class BarrettHand
 * @brief A high-level teleoperation controller for the BarrettHand. This is the
 * recommended class for most users.
 */

struct HandState {
    std::array<double, 4> joint_positions;
    std::array<double, 4> joint_velocities;
    boost::optional<double> temperature_celsius;
};

class BarrettHand {
public:
    BarrettHand();
    ~BarrettHand();

    BarrettHand(const BarrettHand&) = delete;
    BarrettHand& operator=(const BarrettHand&) = delete;

    bool initialize(const std::string& port);
    void shutdown();
    void setGraspVelocity(double velocity);
    void setSpread(double spread_position);
    HandState getLatestState() const;

private:
    boost::optional<RealtimeControlSetpoint> velocityPassthroughCallback(const RealtimeFeedback& feedback);
    bool startRealtimeControl();

    double countsToRadians(int32_t counts, MotorID motor) const;
    int32_t radiansToCounts(double radians, MotorID motor) const;
    int8_t velocityRadToCounts(double velocity_rad_per_sec, MotorID motor) const;
    double velocityCountsToRad(int8_t velocity_counts, MotorID motor) const;

    std::unique_ptr<BarrettHandDriver> driver_;
    std::atomic<double> target_grasp_velocity_{0.0};
    mutable std::mutex state_mutex_;
    HandState latest_state_;
    double last_position;
    std::chrono::steady_clock::time_point last_time;
    bool is_first;
};

} // namespace barrett
} // namespace gripper
