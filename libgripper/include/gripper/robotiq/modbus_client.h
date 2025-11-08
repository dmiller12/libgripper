#pragma once

#include "gripper/robotiq/register_map.h"
#include <boost/optional.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace gripper {
namespace robotiq {

class SerialCommunicator;

class RobotiqModbusClient {
  public:
    explicit RobotiqModbusClient(uint8_t slave_id = 0x09);
    ~RobotiqModbusClient();

    RobotiqModbusClient(const RobotiqModbusClient&) = delete;
    RobotiqModbusClient& operator=(const RobotiqModbusClient&) = delete;
    RobotiqModbusClient(RobotiqModbusClient&&) = delete;
    RobotiqModbusClient& operator=(RobotiqModbusClient&&) = delete;

    bool connect(const std::string& port, unsigned int baud_rate);
    void disconnect();
    bool isConnected() const;

    bool writeRegisters(uint16_t start_address, const std::vector<uint16_t>& registers);
    boost::optional<std::vector<uint16_t>>
    readHoldingRegisters(uint16_t start_address, uint16_t register_count, int timeout_ms = 200);

  private:
    bool transact(
        const std::vector<uint8_t>& request,
        size_t expected_response_size,
        std::vector<uint8_t>& response,
        int timeout_ms
    );
    bool ensureConnected() const;

    static uint16_t computeCRC(const std::vector<uint8_t>& payload);
    static void appendCRC(std::vector<uint8_t>& payload);

    const uint8_t slave_id_;
    std::unique_ptr<SerialCommunicator> communicator_;
    mutable std::mutex io_mutex_;
};

} // namespace robotiq
} // namespace gripper
