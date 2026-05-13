#pragma once

#include "gripper/barrett/barrett_hand_driver.h"
#include "gripper/barrett/low_pass_filter.h"
#include "gripper/barrett/position_controller.h"
#include "gripper/barrett/velocity_controller.h"
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

namespace gripper {
namespace barrett {

/**
 * @class BarrettHand
 * @brief A high-level controller for the BarrettHand. This is the
 * recommended class for most users.
 */

struct HandState {
    std::array<double, 4> joint_positions;
    std::array<double, 4> joint_velocities;
    boost::optional<double> temperature_celsius;
};

enum class ControlMode { None, Position, Velocity };

class BarrettHand {
  public:
    BarrettHand();
    ~BarrettHand();

    BarrettHand(const BarrettHand&) = delete;
    BarrettHand& operator=(const BarrettHand&) = delete;

    bool initialize(const std::string& port, bool force = false);
    void shutdown();
    void setPosition(const std::array<double, 4>& positions);
    void setVelocity(const std::array<double, 4>& positions, bool sync_position = true);
    void setVelocity(const double& finger, const double& spread);
    void moveTo(const MotorGroup& group, const double& position);
    void open(const MotorGroup& group);
    void close(const MotorGroup& group);
    HandState getLatestState() const;

  private:
    boost::optional<RealtimeControlSetpoint> controlLoopCallback(const RealtimeFeedback& feedback);
    bool startRealtimeControl();
    bool isDeviceInitialized();
    void applyFingerPositionSync(
        std::array<double, 4>& commanded_velocity,
        const HandState& state,
        const std::vector<MotorID>& finger_motors,
        double dt
    );

    std::unique_ptr<BarrettHandDriver> driver_;
    std::array<double, 4> target_position_{};
    std::array<double, 4> target_velocity_{};
    mutable std::mutex state_mutex_;
    mutable std::mutex target_mutex_;
    HandState latest_state_;
    HandState prev_state_;
    std::chrono::steady_clock::time_point last_time;
    bool is_first{true};
    RealtimeSettings realtime_settings_;
    ControlMode control_mode_{ControlMode::None};
    PositionController position_contoller_{{18.0, 18.0, 18.0, 0.1}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
    PositionController sync_position_contoller_{{2.0, 2.0, 2.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
    bool sync_position_{false};
    std::array<double, 4> sync_velocity_trim_{};
    VelocityController velocity_contoller_{{8, 8, 8, 0.05}, {0.0, 0.0, 0.0, 0.0}, {0.01, 0.01, 0.01, 0.0}};
    static constexpr double FINGER_ALPHA = 0.5;
    static constexpr double SPREAD_ALPHA = 0.3;
    LowPassFilter velocity_filter_{{FINGER_ALPHA, FINGER_ALPHA, FINGER_ALPHA, SPREAD_ALPHA}};
};

} // namespace barrett
} // namespace gripper
