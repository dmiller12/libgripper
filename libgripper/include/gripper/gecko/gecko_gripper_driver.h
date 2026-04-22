#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "moteus.h"
#include "gecko_gripper_config.h"
#include "gecko_types.h"

namespace gripper {
namespace gecko {

namespace moteus = mjbots::moteus;

/**
 * @class GeckoGripperDriver
 * @brief Low-level driver for the Gecko Gripper communicating over Moteus CAN FD.
 */
class GeckoGripperDriver {
  public:
    GeckoGripperDriver();
    ~GeckoGripperDriver();

    GeckoGripperDriver(const GeckoGripperDriver&) = delete;
    GeckoGripperDriver& operator=(const GeckoGripperDriver&) = delete;

    bool connect(const GeckoGripperConfig& config);
    void disconnect();
    bool isConnected() const;

    bool queryState();
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

    bool sendCmd();

    int loop_counter = 0;
};

} // namespace gecko
} // namespace gripper
