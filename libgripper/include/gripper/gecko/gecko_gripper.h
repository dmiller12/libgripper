#pragma once

#include "gecko_types.h"
#include <memory>
#include <mutex>
#include <string>

namespace gripper {
namespace gecko {

class GeckoGripperDriver;

/**
 * @class GeckoGripper
 * @brief A high-level controller for the Gecko Gripper. This is the
 * recommended class for most users.
 */
class GeckoGripper {
  public:
    GeckoGripper();
    ~GeckoGripper();

    GeckoGripper(const GeckoGripper&) = delete;
    GeckoGripper& operator=(const GeckoGripper&) = delete;

    bool initialize();
    void shutdown();
    
    void setPosition(double position);
    void setVelocity(double velocity);

    double getGripperClosePos();
    double getGripperOpenPos();
    
    void controlLoopCallback();
    
    GripperState getLatestState() const;
    void updateLocalState();

  private:
    std::unique_ptr<GeckoGripperDriver> driver_;
    
    mutable std::mutex target_mutex_;
    double target_position_{0.0};
    double target_velocity_{0.0};
    double gripper_close_pos_{0.0};
    double gripper_open_pos_{-0.2};
    ControlMode control_mode_{ControlMode::None};

    mutable std::mutex state_mutex_;
    GripperState latest_state_;
};

} // namespace gecko
} // namespace gripper
