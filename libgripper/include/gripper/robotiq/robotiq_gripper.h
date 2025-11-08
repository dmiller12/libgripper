#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "gripper/communication/serial_communicator.h"

namespace gripper {
namespace robotiq {

struct RobotiqState {
    bool is_activated = false;
    bool is_ready = false;
    bool is_moving = false;
    bool object_contact = false;
    uint8_t object_status = 0;
    uint8_t gripper_status = 0;
    uint8_t fault_status = 0;
    uint8_t current_raw = 0;
    double requested_width_m = 0.0;
    double actual_width_m = 0.0;
};

class RobotiqGripper {
  public:
    explicit RobotiqGripper(uint8_t slave_id = 0x09);
    ~RobotiqGripper();

    bool connect(const std::string& port, unsigned int baud_rate = 115200);
    void disconnect();
    bool isConnected() const;

    bool activate(bool wait_for_ready = true);
    bool deactivate();

    bool setWidth(double width_m);
    bool setSpeedRatio(double ratio);
    bool setForceRatio(double ratio);
    bool move(double width_m);
    bool open();
    bool close();

    bool readState(RobotiqState& state);
    double maxStrokeMeters() const;

  private:
    struct CommandRegisters {
        uint8_t rACT = 0;
        uint8_t rGTO = 0;
        uint8_t rATR = 0;
        uint8_t rPR = 0;
        uint8_t rSP = 128;
        uint8_t rFR = 128;
    };

    static constexpr double kMaxStrokeMeters = 0.140;
    static constexpr uint16_t kCommandRegisterAddress = 0x03E8;
    static constexpr uint16_t kStatusRegisterAddress = 0x07D0;
    static constexpr int kRegisterCount = 3;
    static constexpr int kSerialTimeoutMs = 200;

    static double clamp(double value, double min, double max);
    static uint8_t widthToRegister(double width_m);
    static double registerToWidth(uint8_t reg_value);
    static uint16_t computeCRC(const std::vector<uint8_t>& buffer);
    static bool verifyCRC(const std::vector<uint8_t>& buffer);

    bool writeCommandLocked();
    bool writeHoldingRegisters(uint16_t address, const std::vector<uint16_t>& values);
    bool readHoldingRegisters(uint16_t address, uint16_t count, std::vector<uint16_t>& values_out);
    std::vector<uint8_t> buildReadRequest(uint16_t address, uint16_t count) const;
    std::vector<uint8_t> buildWriteRequest(uint16_t address, const std::vector<uint16_t>& values) const;

    bool waitForCondition(std::function<bool(const RobotiqState&)> predicate, std::chrono::milliseconds timeout);

    uint8_t slave_id_;
    SerialCommunicator serial_;
    CommandRegisters command_;
    mutable std::mutex mutex_;
    bool activated_ = false;
};

} // namespace robotiq
} // namespace gripper
