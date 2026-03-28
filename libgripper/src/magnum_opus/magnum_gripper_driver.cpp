#include "gripper/magnum_opus/magnum_gripper_driver.h"

#include <algorithm>
#include <boost/outcome/try.hpp>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <vector>

namespace gripper {
namespace magnum_opus {

MagnumGripperDriver::MagnumGripperDriver() 
    : is_connected_(false) {
}

MagnumGripperDriver::~MagnumGripperDriver() {
    disconnect();
}

bool MagnumGripperDriver::connect(const std::string& can_interface) {
    if (is_connected_) {
        return true;
    }

    try {
        std::vector<std::string> args = {"--can-iface", can_interface};
        moteus::Controller::ProcessTransportArgs(args);
        transport_ = moteus::Controller::MakeSingletonTransport({});

        // Configure the single Moteus controller (assuming ID 1)
        moteus::Controller::Options opts;
        opts.id = 1;
        
        auto& pf = opts.position_format;
        pf.position = moteus::kFloat;
        pf.velocity = moteus::kFloat;
        pf.feedforward_torque = moteus::kFloat;

        controller_ = std::make_shared<moteus::Controller>(opts);

        controller_->DiagnosticWrite("tel stop\n");
        controller_->DiagnosticFlush();
        controller_->SetStop();

        is_connected_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Moteus Connection Failed: " << e.what() << std::endl;
        return false;
    }
}

void MagnumGripperDriver::disconnect() {
    if (!is_connected_) return;

    if (controller_) {
        std::cout << "Engaging Moteus brake before disconnect." << std::endl;
        controller_->SetBrake();
    }

    transport_.reset();
    controller_.reset();
    is_connected_ = false;
}

bool MagnumGripperDriver::isConnected() const {
    return is_connected_;
}

bool MagnumGripperDriver::executeControlCycle(const moteus::PositionMode::Command& cmd) {
    if (!is_connected_) return false;

    send_frames_.clear();
    send_frames_.push_back(controller_->MakePosition(cmd));

    receive_frames_.clear();
    transport_->BlockingCycle(&send_frames_[0], send_frames_.size(), &receive_frames_);

    for (auto it = receive_frames_.rbegin(); it != receive_frames_.rend(); ++it) {
        if (it->source == controller_->options().id) {
            auto result = moteus::Query::Parse(it->data, it->size);
            
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_.position = result.position;
            latest_state_.velocity = result.velocity;
            latest_state_.torque = result.torque;
            latest_state_.temperature_c = result.temperature;
            return true;
        }
    }
    return false;
}

GripperState MagnumGripperDriver::getLatestState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}

} // namespace barrett
} // namespace gripper
