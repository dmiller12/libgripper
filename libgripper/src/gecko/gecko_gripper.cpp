#include "gripper/gecko/gecko_gripper.h"
#include "gripper/gecko/gecko_gripper_driver.h"
#include "gripper/gecko/gecko_gripper_config.h"
#include "gripper/utils.h"
#include <algorithm>
#include <cmath>
#include <iostream>


namespace gripper {
namespace gecko {

GeckoGripper::GeckoGripper()
    : driver_(std::make_unique<GeckoGripperDriver>()) {
}

GeckoGripper::~GeckoGripper() {
    shutdown();
}

bool GeckoGripper::initialize() {
    const std::string config_dir = get_config_directory();
    if (config_dir.empty()) {
        throw std::runtime_error("No valid configuration directory found.");
    }

    const GeckoGripperConfig config = load_config(config_dir);

    if (!driver_->connect(config)) {
        std::cerr << "Failed to initialize Gecko Gripper." << std::endl;
        return false;
    }

    gripper_close_pos_ = config.max_pos;
    gripper_open_pos_ = config.min_pos;

    // update local state without sending pos command
    driver_->queryState();

    GripperState gripper_state = driver_->getLatestState();
    target_position_ = gripper_state.position;
    target_velocity_ = gripper_state.velocity;
    control_mode_ = ControlMode::Position;
    
    return true;
}

void GeckoGripper::shutdown() {
    if (driver_ && driver_->isConnected()) {
        driver_->disconnect();
    }
}

double GeckoGripper::getGripperClosePos() {
    std::lock_guard<std::mutex> lock(target_mutex_);
    return gripper_close_pos_;
}

double GeckoGripper::getGripperOpenPos() {
    std::lock_guard<std::mutex> lock(target_mutex_);
    return gripper_open_pos_;
}

void GeckoGripper::setPosition(double position) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_position_ = position;
    control_mode_ = ControlMode::Position;
}

void GeckoGripper::setVelocity(double velocity) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_velocity_ = velocity;
    control_mode_ = ControlMode::Velocity;
}

// sending a command with this function will update local state so you wont need to poll.
void GeckoGripper::controlLoopCallback() {
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

// call updateLocalState before this function if you did not call controlLoopCallback recently
GripperState GeckoGripper::getLatestState() const {
    return driver_->getLatestState();
}

void GeckoGripper::updateLocalState() {
    driver_->queryState();
}

} // namespace gecko
} // namespace gripper
