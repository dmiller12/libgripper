#include "gripper/robotiq/robotiq_gripper.h"

#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <thread>
#include <vector>

namespace gripper {
namespace robotiq {
namespace {
constexpr uint8_t kReadHoldingRegistersFn = 0x03;
constexpr uint8_t kWriteMultipleRegistersFn = 0x10;
constexpr std::chrono::milliseconds kActivationTimeout(3000);
constexpr std::chrono::milliseconds kPollInterval(50);
}

RobotiqGripper::RobotiqGripper(uint8_t slave_id)
    : slave_id_(slave_id) {
}

RobotiqGripper::~RobotiqGripper() {
    disconnect();
}

bool RobotiqGripper::connect(const std::string& port, unsigned int baud_rate) {
    activated_ = false;
    return serial_.connect(port, baud_rate);
}

void RobotiqGripper::disconnect() {
    serial_.disconnect();
    activated_ = false;
}

bool RobotiqGripper::isConnected() const {
    return serial_.isOpen();
}

bool RobotiqGripper::activate(bool wait_for_ready) {
    if (!isConnected()) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = CommandRegisters{};
        if (!writeCommandLocked()) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        command_.rACT = 1;
        command_.rGTO = 1;
        if (!writeCommandLocked()) {
            return false;
        }
        activated_ = true;
    }

    if (!wait_for_ready) {
        return true;
    }

    return waitForCondition([](const RobotiqState& state) { return state.is_ready; }, kActivationTimeout);
}

bool RobotiqGripper::deactivate() {
    if (!isConnected()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    command_.rACT = 0;
    command_.rGTO = 0;
    activated_ = false;
    return writeCommandLocked();
}

bool RobotiqGripper::setWidth(double width_m) {
    std::lock_guard<std::mutex> lock(mutex_);
    command_.rPR = widthToRegister(width_m);
    command_.rGTO = 1;
    return writeCommandLocked();
}

bool RobotiqGripper::setSpeedRatio(double ratio) {
    std::lock_guard<std::mutex> lock(mutex_);
    double clamped = clamp(ratio, 0.0, 1.0);
    command_.rSP = static_cast<uint8_t>(std::round(clamped * 255.0));
    return writeCommandLocked();
}

bool RobotiqGripper::setForceRatio(double ratio) {
    std::lock_guard<std::mutex> lock(mutex_);
    double clamped = clamp(ratio, 0.0, 1.0);
    command_.rFR = static_cast<uint8_t>(std::round(clamped * 255.0));
    return writeCommandLocked();
}

bool RobotiqGripper::move(double width_m) {
    return setWidth(width_m);
}

bool RobotiqGripper::open() {
    return setWidth(maxStrokeMeters());
}

bool RobotiqGripper::close() {
    return setWidth(0.0);
}

double RobotiqGripper::maxStrokeMeters() const {
    return kMaxStrokeMeters;
}

bool RobotiqGripper::readState(RobotiqState& state) {
    std::vector<uint16_t> raw_registers;
    if (!readHoldingRegisters(kStatusRegisterAddress, kRegisterCount, raw_registers)) {
        return false;
    }

    if (raw_registers.size() < static_cast<size_t>(kRegisterCount)) {
        return false;
    }

    std::array<uint8_t, kRegisterCount * 2> bytes{};
    for (size_t i = 0; i < raw_registers.size(); ++i) {
        bytes[i * 2] = static_cast<uint8_t>((raw_registers[i] >> 8) & 0xFF);
        bytes[i * 2 + 1] = static_cast<uint8_t>(raw_registers[i] & 0xFF);
    }

    const uint8_t status_byte = bytes[0];

    state.is_activated = (status_byte & 0x01) != 0;
    state.gripper_status = (status_byte >> 4) & 0x03;
    state.is_ready = (state.gripper_status == 3);
    state.object_status = (status_byte >> 6) & 0x03;
    state.object_contact = (state.object_status == 1 || state.object_status == 2);
    state.is_moving = (state.object_status == 0);
    state.fault_status = bytes[2];
    state.current_raw = bytes[5];
    state.requested_width_m = registerToWidth(bytes[3]);
    state.actual_width_m = registerToWidth(bytes[4]);

    return true;
}

double RobotiqGripper::clamp(double value, double min, double max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

uint8_t RobotiqGripper::widthToRegister(double width_m) {
    double normalized = clamp(width_m / kMaxStrokeMeters, 0.0, 1.0);
    return static_cast<uint8_t>(std::round(normalized * 255.0));
}

double RobotiqGripper::registerToWidth(uint8_t reg_value) {
    double normalized = static_cast<double>(reg_value) / 255.0;
    return normalized * kMaxStrokeMeters;
}

uint16_t RobotiqGripper::computeCRC(const std::vector<uint8_t>& buffer) {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : buffer) {
        crc ^= static_cast<uint16_t>(byte);
        for (int i = 0; i < 8; ++i) {
            if ((crc & 0x0001) != 0) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

bool RobotiqGripper::verifyCRC(const std::vector<uint8_t>& buffer) {
    if (buffer.size() < 3) {
        return false;
    }

    std::vector<uint8_t> data_without_crc(buffer.begin(), buffer.end() - 2);
    uint16_t expected_crc = computeCRC(data_without_crc);
    uint16_t received_crc = static_cast<uint16_t>(buffer[buffer.size() - 2]) |
        (static_cast<uint16_t>(buffer[buffer.size() - 1]) << 8);
    return expected_crc == received_crc;
}

bool RobotiqGripper::writeCommandLocked() {
    std::array<uint8_t, 6> message{};
    message[0] = static_cast<uint8_t>(command_.rACT + (command_.rGTO << 3) + (command_.rATR << 4));
    message[1] = 0;
    message[2] = 0;
    message[3] = command_.rPR;
    message[4] = command_.rSP;
    message[5] = command_.rFR;

    std::vector<uint16_t> registers;
    registers.reserve(message.size() / 2);
    for (size_t i = 0; i < message.size(); i += 2) {
        uint16_t reg = static_cast<uint16_t>(message[i] << 8) | static_cast<uint16_t>(message[i + 1]);
        registers.push_back(reg);
    }

    return writeHoldingRegisters(kCommandRegisterAddress, registers);
}

bool RobotiqGripper::writeHoldingRegisters(uint16_t address, const std::vector<uint16_t>& values) {
    if (!serial_.isOpen()) {
        return false;
    }

    std::vector<uint8_t> frame = buildWriteRequest(address, values);
    auto write_future = serial_.write(frame);
    if (!write_future.get()) {
        return false;
    }

    auto response = serial_.read(8, kSerialTimeoutMs).get();
    if (response.size() != 8 || !verifyCRC(response)) {
        return false;
    }

    if (response[0] != slave_id_) {
        return false;
    }

    if (response[1] & 0x80) {
        return false;
    }

    return response[1] == kWriteMultipleRegistersFn;
}

bool RobotiqGripper::readHoldingRegisters(uint16_t address, uint16_t count, std::vector<uint16_t>& values_out) {
    if (!serial_.isOpen()) {
        return false;
    }

    std::vector<uint8_t> frame = buildReadRequest(address, count);
    auto write_future = serial_.write(frame);
    if (!write_future.get()) {
        return false;
    }

    size_t expected_bytes = 5 + (count * 2);
    auto response = serial_.read(expected_bytes, kSerialTimeoutMs).get();
    if (response.size() != expected_bytes || !verifyCRC(response)) {
        return false;
    }

    if (response[0] != slave_id_ || response[1] != kReadHoldingRegistersFn) {
        return false;
    }

    uint8_t byte_count = response[2];
    if (byte_count != count * 2) {
        return false;
    }

    values_out.resize(count);
    for (size_t i = 0; i < count; ++i) {
        uint16_t value = static_cast<uint16_t>(response[3 + i * 2] << 8) |
            static_cast<uint16_t>(response[4 + i * 2]);
        values_out[i] = value;
    }

    return true;
}

std::vector<uint8_t> RobotiqGripper::buildReadRequest(uint16_t address, uint16_t count) const {
    std::vector<uint8_t> frame;
    frame.reserve(8);
    frame.push_back(slave_id_);
    frame.push_back(kReadHoldingRegistersFn);
    frame.push_back(static_cast<uint8_t>((address >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(address & 0xFF));
    frame.push_back(static_cast<uint8_t>((count >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(count & 0xFF));
    uint16_t crc = computeCRC(frame);
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    return frame;
}

std::vector<uint8_t> RobotiqGripper::buildWriteRequest(uint16_t address, const std::vector<uint16_t>& values) const {
    std::vector<uint8_t> frame;
    const uint16_t count = static_cast<uint16_t>(values.size());
    frame.reserve(9 + values.size() * 2);
    frame.push_back(slave_id_);
    frame.push_back(kWriteMultipleRegistersFn);
    frame.push_back(static_cast<uint8_t>((address >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(address & 0xFF));
    frame.push_back(static_cast<uint8_t>((count >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(count & 0xFF));
    frame.push_back(static_cast<uint8_t>(count * 2));

    for (uint16_t value : values) {
        frame.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>(value & 0xFF));
    }

    uint16_t crc = computeCRC(frame);
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    return frame;
}

bool RobotiqGripper::waitForCondition(std::function<bool(const RobotiqState&)> predicate, std::chrono::milliseconds timeout) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    RobotiqState state;
    while (std::chrono::steady_clock::now() < deadline) {
        if (readState(state) && predicate(state)) {
            return true;
        }
        std::this_thread::sleep_for(kPollInterval);
    }
    return false;
}

} // namespace robotiq
} // namespace gripper
