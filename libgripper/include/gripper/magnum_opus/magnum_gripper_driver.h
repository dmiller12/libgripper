#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "moteus.h"

namespace gripper {
namespace magnum_opus {

namespace moteus = mjbots::moteus;

struct GripperState {
    double position{0.0};
    double velocity{0.0};
    double torque{0.0};
    double temperature_c{0.0};
};

/**
 * @class MagnumGripperDriver
 * @brief Low-level driver for the Magnum Gripper communicating over Moteus CAN FD.
 */
class MagnumGripperDriver {
  public:
    MagnumGripperDriver();
    ~MagnumGripperDriver();

    MagnumGripperDriver(const MagnumGripperDriver&) = delete;
    MagnumGripperDriver& operator=(const MagnumGripperDriver&) = delete;

    bool connect(const std::string& can_interface = "can0");
    void disconnect();
    bool isConnected() const;

    bool executeControlCycle(const moteus::PositionMode::Command& cmd);
    
    GripperState getLatestState() const;

  private:
    std::atomic<bool> is_connected_{false};

    std::shared_ptr<moteus::Transport> transport_;
    std::shared_ptr<moteus::Controller> controller_;

    std::vector<moteus::CanFdFrame> send_frames_;
    std::vector<moteus::CanFdFrame> receive_frames_;

    mutable std::mutex state_mutex_;
    GripperState latest_state_;
};

} // namespace magnum_opus
} // namespace gripper