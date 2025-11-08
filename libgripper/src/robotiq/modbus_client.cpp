#include "gripper/robotiq/modbus_client.h"
#include "../communication/serial_communicator.h"

#include <iostream>

namespace gripper {
namespace robotiq {

namespace {
constexpr uint8_t kFunctionReadHoldingRegisters = 0x03;
constexpr uint8_t kFunctionWriteMultipleRegisters = 0x10;

uint8_t hi(uint16_t value) {
    return static_cast<uint8_t>((value >> 8) & 0xFF);
}

uint8_t lo(uint16_t value) {
    return static_cast<uint8_t>(value & 0xFF);
}
} // namespace

RobotiqModbusClient::RobotiqModbusClient(uint8_t slave_id)
    : slave_id_(slave_id) {
}

bool RobotiqModbusClient::connect(const std::string& port, unsigned int baud_rate) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (ensureConnected())
        return true;

    communicator_ = std::make_unique<SerialCommunicator>();
    if (!communicator_->connect(port, baud_rate)) {
        communicator_.reset();
        return false;
    }
    return true;
}

void RobotiqModbusClient::disconnect() {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (communicator_) {
        communicator_->disconnect();
        communicator_.reset();
    }
}

bool RobotiqModbusClient::isConnected() const {
    std::lock_guard<std::mutex> lock(io_mutex_);
    return ensureConnected();
}

bool RobotiqModbusClient::writeRegisters(uint16_t start_address, const std::vector<uint16_t>& registers) {
    if (registers.empty())
        return true;

    std::vector<uint8_t> request;
    request.reserve(8 + registers.size() * 2);
    request.push_back(slave_id_);
    request.push_back(kFunctionWriteMultipleRegisters);
    request.push_back(hi(start_address));
    request.push_back(lo(start_address));
    request.push_back(hi(registers.size()));
    request.push_back(lo(registers.size()));
    request.push_back(static_cast<uint8_t>(registers.size() * 2));
    for (uint16_t value : registers) {
        request.push_back(hi(value));
        request.push_back(lo(value));
    }
    appendCRC(request);

    std::vector<uint8_t> response;
    if (!transact(request, 8, response, 200))
        return false;

    if (response.size() != 8 || response[0] != slave_id_ || response[1] != kFunctionWriteMultipleRegisters) {
        std::cerr << "[Robotiq] Unexpected response to write registers.\n";
        return false;
    }
    return true;
}

boost::optional<std::vector<uint16_t>> RobotiqModbusClient::readHoldingRegisters(
    uint16_t start_address,
    uint16_t register_count,
    int timeout_ms
) {
    if (register_count == 0)
        return std::vector<uint16_t>{};

    std::vector<uint8_t> request;
    request.reserve(8);
    request.push_back(slave_id_);
    request.push_back(kFunctionReadHoldingRegisters);
    request.push_back(hi(start_address));
    request.push_back(lo(start_address));
    request.push_back(hi(register_count));
    request.push_back(lo(register_count));
    appendCRC(request);

    const size_t expected_length = 5 + static_cast<size_t>(register_count) * 2;
    std::vector<uint8_t> response;
    if (!transact(request, expected_length, response, timeout_ms))
        return boost::optional<std::vector<uint16_t>>{};

    if (response.size() < 5 || response[0] != slave_id_) {
        std::cerr << "[Robotiq] Invalid Modbus response header.\n";
        return boost::optional<std::vector<uint16_t>>{};
    }

    if (response[1] == (kFunctionReadHoldingRegisters | 0x80)) {
        std::cerr << "[Robotiq] Modbus exception code: " << static_cast<int>(response[2]) << "\n";
        return boost::optional<std::vector<uint16_t>>{};
    }

    if (response[1] != kFunctionReadHoldingRegisters) {
        std::cerr << "[Robotiq] Unexpected function code in response: " << static_cast<int>(response[1]) << "\n";
        return boost::optional<std::vector<uint16_t>>{};
    }

    const uint8_t byte_count = response[2];
    if (byte_count != register_count * 2 || response.size() != 5 + byte_count) {
        std::cerr << "[Robotiq] Response byte count mismatch.\n";
        return boost::optional<std::vector<uint16_t>>{};
    }

    std::vector<uint16_t> values;
    values.reserve(register_count);
    size_t data_offset = 3;
    for (uint16_t i = 0; i < register_count; ++i) {
        uint16_t value = static_cast<uint16_t>(response[data_offset + 2 * i] << 8);
        value |= static_cast<uint16_t>(response[data_offset + 2 * i + 1]);
        values.push_back(value);
    }
    return values;
}

bool RobotiqModbusClient::transact(
    const std::vector<uint8_t>& request,
    size_t expected_response_size,
    std::vector<uint8_t>& response,
    int timeout_ms
) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!ensureConnected())
        return false;

    auto write_future = communicator_->write(request);
    if (!write_future.get()) {
        std::cerr << "[Robotiq] Failed to write Modbus request.\n";
        return false;
    }

    auto response_future = communicator_->read(expected_response_size, timeout_ms);
    response = response_future.get();
    if (response.size() != expected_response_size) {
        std::cerr << "[Robotiq] Timed out waiting for Modbus response.\n";
        return false;
    }

    const uint16_t received_crc = static_cast<uint16_t>(response[response.size() - 2])
        | static_cast<uint16_t>(response[response.size() - 1] << 8);

    std::vector<uint8_t> payload(response.begin(), response.end() - 2);
    if (computeCRC(payload) != received_crc) {
        std::cerr << "[Robotiq] CRC check failed.\n";
        return false;
    }

    return true;
}

bool RobotiqModbusClient::ensureConnected() const {
    return communicator_ && communicator_->isOpen();
}

uint16_t RobotiqModbusClient::computeCRC(const std::vector<uint8_t>& payload) {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : payload) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void RobotiqModbusClient::appendCRC(std::vector<uint8_t>& payload) {
    const uint16_t crc = computeCRC(payload);
    payload.push_back(static_cast<uint8_t>(crc & 0xFF));
    payload.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
}

} // namespace robotiq
} // namespace gripper
