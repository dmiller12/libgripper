#pragma once

#include "magnum_types.h"
#include <memory>
#include <mutex>
#include <string>

namespace gripper {
namespace magnum_opus {

class MagnumGripperDriver;

/**
 * @class MagnumGripper
 * @brief A high-level controller for the Magnum Gripper. This is the
 * recommended class for most users.
 */
class MagnumGripper {
  public:
    MagnumGripper();
    ~MagnumGripper();

    MagnumGripper(const MagnumGripper&) = delete;
    MagnumGripper& operator=(const MagnumGripper&) = delete;

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
    std::unique_ptr<MagnumGripperDriver> driver_;
    
    mutable std::mutex target_mutex_;
    double target_position_{0.0};
    double target_velocity_{0.0};
    double gripper_close_pos_{0.0};
    double gripper_open_pos_{-0.2};
    ControlMode control_mode_{ControlMode::None};

    mutable std::mutex state_mutex_;
    GripperState latest_state_;
};

} // namespace magnum_opus
} // namespace gripper