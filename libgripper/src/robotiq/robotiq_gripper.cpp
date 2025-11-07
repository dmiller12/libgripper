#include "gripper/robotiq/robotiq_gripper.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace gripper {
namespace robotiq {

namespace {
constexpr std::chrono::milliseconds kActivationPollInterval(50);
constexpr std::chrono::milliseconds kActivationTimeout(4000);

uint8_t normalizedToByte(double value) {
    const double clamped = clamp(value, 0.0, 1.0);
    return static_cast<uint8_t>(std::round(clamped * 255.0));
}
} // namespace

RobotiqGripper::RobotiqGripper() = default;

RobotiqGripper::~RobotiqGripper() {
    disconnect();
}

bool RobotiqGripper::connect(const std::string& port, unsigned int baud_rate, uint8_t slave_id) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    slave_id_ = slave_id;
    client_ = std::make_unique<RobotiqModbusClient>(slave_id_);
    last_command_valid_ = false;
    if (!client_->connect(port, baud_rate)) {
        client_.reset();
        return false;
    }
    return true;
}

void RobotiqGripper::disconnect() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (client_) {
        client_->disconnect();
        client_.reset();
    }
    last_command_valid_ = false;
}

bool RobotiqGripper::isConnected() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return client_ && client_->isConnected();
}

bool RobotiqGripper::activate(bool auto_release, double default_speed, double default_force) {
    if (!isConnected())
        return false;

    if (!reset()) {
        std::cerr << "[Robotiq] Failed to reset gripper before activation.\n";
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const uint8_t atr = auto_release ? 1 : 0;
    const uint8_t speed = normalizedToByte(default_speed);
    const uint8_t force = normalizedToByte(default_force);
    const uint8_t open_position = rawFromWidth(kRobotiqMaxWidthMm);

    if (!sendCommand(1, 0, atr, open_position, speed, force)) {
        std::cerr << "[Robotiq] Activation command failed.\n";
        return false;
    }

    bool ready = false;
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < kActivationTimeout) {
        auto status = getStatus();
        if (status.valid && status.ready) {
            ready = true;
            break;
        }
        std::this_thread::sleep_for(kActivationPollInterval);
    }

    if (!ready) {
        std::cerr << "[Robotiq] Timed out waiting for ready status during activation.\n";
        return false;
    }

    // Transition to go-to mode so future setWidth calls take effect.
    if (!sendCommand(1, 1, atr, open_position, speed, force)) {
        std::cerr << "[Robotiq] Failed to arm go-to mode after activation.\n";
        return false;
    }

    return true;
}

bool RobotiqGripper::reset() {
    if (!isConnected())
        return false;

    if (!sendCommand(0, 0, 0, 0, 0, 0))
        return false;

    std::lock_guard<std::mutex> lock(command_mutex_);
    last_command_valid_ = false;
    return true;
}

bool RobotiqGripper::stop() {
    if (!isConnected())
        return false;

    std::array<uint8_t, 6> snapshot{{1, 0, 0, 0, 0, 0}};
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (last_command_valid_)
            snapshot = last_command_;
        snapshot[0] = 1; // keep activated
        snapshot[1] = 0; // go-to off -> stop
    }
    return sendCommand(snapshot[0], snapshot[1], snapshot[2], snapshot[3], snapshot[4], snapshot[5]);
}

bool RobotiqGripper::setWidth(double width_mm, double speed, double force) {
    if (!isConnected())
        return false;

    const uint8_t target_position = rawFromWidth(width_mm);
    const uint8_t target_speed = normalizedToByte(speed);
    const uint8_t target_force = normalizedToByte(force);

    uint8_t atr = 0;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (last_command_valid_) {
            atr = last_command_[2];
        }
    }

    return sendCommand(1, 1, atr, target_position, target_speed, target_force);
}

bool RobotiqGripper::open(double speed) {
    double force = 0.5;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (last_command_valid_) {
            force = static_cast<double>(last_command_[5]) / 255.0;
        }
    }
    return setWidth(kRobotiqMaxWidthMm, speed, force);
}

bool RobotiqGripper::close(double speed, double force) {
    return setWidth(0.0, speed, force);
}

GripperStatus RobotiqGripper::getStatus() const {
    GripperStatus status;

    auto registers = readStatusRegisters();
    if (!registers || registers->size() < 8)
        return status;

    status.valid = true;
    status.activated = ((*registers)[0] & 0x00FF) == 0x01;
    const uint8_t gGTO = static_cast<uint8_t>((*registers)[1] & 0x00FF);
    const uint8_t gSTA = static_cast<uint8_t>((*registers)[2] & 0x00FF);
    status.ready = (gSTA == 0x03);
    status.moving = (gGTO == 0x01) && (gSTA == 0x02);
    status.object_status = static_cast<uint8_t>((*registers)[3] & 0x00FF);
    status.object_detected = (status.object_status == 0x01 || status.object_status == 0x02);
    status.fault_code = static_cast<uint8_t>((*registers)[4] & 0x00FF);
    status.requested_position = static_cast<uint8_t>((*registers)[5] & 0x00FF);
    status.actual_position = static_cast<uint8_t>((*registers)[6] & 0x00FF);
    status.motor_current = static_cast<uint8_t>((*registers)[7] & 0x00FF);
    status.width_mm = widthFromRaw(status.actual_position);

    return status;
}

uint8_t RobotiqGripper::rawFromWidth(double width_mm) {
    const double clamped_width = clamp(width_mm, 0.0, kRobotiqMaxWidthMm);
    const double normalized = 1.0 - (clamped_width / kRobotiqMaxWidthMm);
    return static_cast<uint8_t>(std::round(normalized * 255.0));
}

double RobotiqGripper::widthFromRaw(uint8_t raw) {
    const double normalized = static_cast<double>(raw) / 255.0;
    return (1.0 - normalized) * kRobotiqMaxWidthMm;
}

bool RobotiqGripper::sendCommand(
    uint8_t act,
    uint8_t gto,
    uint8_t atr,
    uint8_t position,
    uint8_t speed,
    uint8_t force
) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!client_ || !client_->isConnected())
        return false;

    std::array<uint8_t, 6> command{{act, gto, atr, position, speed, force}};
    if (last_command_valid_ && command == last_command_)
        return true;

    std::vector<uint16_t> registers;
    registers.reserve(command.size());
    for (uint8_t value : command) {
        registers.push_back(static_cast<uint16_t>(value));
    }

    if (!client_->writeRegisters(kCommandRegisterBase, registers))
        return false;

    last_command_ = command;
    last_command_valid_ = true;
    return true;
}

boost::optional<std::vector<uint16_t>> RobotiqGripper::readStatusRegisters() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!client_ || !client_->isConnected())
        return boost::optional<std::vector<uint16_t>>{};
    return client_->readHoldingRegisters(kStatusRegisterBase, 8);
}

} // namespace robotiq
} // namespace gripper
