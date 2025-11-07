#pragma once

#include "gripper/robotiq/modbus_client.h"

#include <array>
#include <boost/optional.hpp>
#include <mutex>
#include <string>

namespace gripper {
namespace robotiq {

struct GripperStatus {
    bool valid{false};
    bool activated{false};
    bool ready{false};
    bool moving{false};
    bool object_detected{false};
    uint8_t object_status{0};
    uint8_t fault_code{0};
    uint8_t requested_position{0};
    uint8_t actual_position{0};
    uint8_t motor_current{0};
    double width_mm{0.0};
};

class RobotiqGripper {
  public:
    RobotiqGripper();
    ~RobotiqGripper();

    RobotiqGripper(const RobotiqGripper&) = delete;
    RobotiqGripper& operator=(const RobotiqGripper&) = delete;
    RobotiqGripper(RobotiqGripper&&) = delete;
    RobotiqGripper& operator=(RobotiqGripper&&) = delete;

    bool connect(const std::string& port, unsigned int baud_rate = 115200, uint8_t slave_id = 0x09);
    void disconnect();
    bool isConnected() const;

    bool activate(bool auto_release = false, double default_speed = 0.3, double default_force = 0.5);
    bool reset();
    bool stop();

    bool setWidth(double width_mm, double speed = 0.3, double force = 0.5);
    bool open(double speed = 0.3);
    bool close(double speed = 0.3, double force = 1.0);

    GripperStatus getStatus() const;

    static uint8_t rawFromWidth(double width_mm);
    static double widthFromRaw(uint8_t raw);

  private:
    bool sendCommand(uint8_t act, uint8_t gto, uint8_t atr, uint8_t position, uint8_t speed, uint8_t force);
    boost::optional<std::vector<uint16_t>> readStatusRegisters() const;

    std::unique_ptr<RobotiqModbusClient> client_;
    mutable std::mutex command_mutex_;
    std::array<uint8_t, 6> last_command_{};
    bool last_command_valid_{false};
    uint8_t slave_id_{0x09};
};

} // namespace robotiq
} // namespace gripper
