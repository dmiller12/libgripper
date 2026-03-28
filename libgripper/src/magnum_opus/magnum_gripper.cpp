#include "gripper/magnum_opus/magnum_gripper.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace gripper {
namespace magnum_opus {

MagnumGripper::MagnumGripper()
    : driver_(std::make_unique<MagnumGripperDriver>()) {
}

MagnumGripper::~MagnumGripper() {
    shutdown();
}

bool MagnumGripper::initialize(const std::string& interface) {
    if (!driver_->connect(interface)) {
        std::cerr << "Failed to initialize Magnum Gripper." << std::endl;
        return false;
    }

    target_position_ = 0.0;
    control_mode_ = ControlMode::Position;
    
    return true;
}

void MagnumGripper::shutdown() {
    if (driver_ && driver_->isConnected()) {
        driver_->disconnect();
    }
}

void MagnumGripper::setPosition(double position) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_position_ = position;
    control_mode_ = ControlMode::Position;
}

void MagnumGripper::setVelocity(double velocity) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_velocity_ = velocity;
    control_mode_ = ControlMode::Velocity;
}

void MagnumGripper::controlLoopCallback() {
    if (!driver_->isConnected()) return;

    moteus::PositionMode::Command cmd;

    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (control_mode_ == ControlMode::Position) {
            cmd.position = target_position_; 
        } else if (control_mode_ == ControlMode::Velocity) {
            cmd.position = std::numeric_limits<double>::quiet_NaN();
            cmd.velocity = target_velocity_;
        }
    }

    if (!driver_->executeControlCycle(cmd)) {
        std::cerr << "Warning: Missed reply from Moteus controller." << std::endl;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = driver_->getLatestState();
}

GripperState MagnumGripper::getLatestState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}

} // namespace magnum_opus
} // namespace gripper
