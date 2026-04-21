#include "gripper/gecko/gecko_gripper_driver.h"

#include <algorithm>
#include <boost/outcome/try.hpp>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <vector>

namespace gripper {
namespace gecko {

GeckoGripperDriver::GeckoGripperDriver() 
    : is_connected_(false) {
}

GeckoGripperDriver::~GeckoGripperDriver() {
    disconnect();
}

bool GeckoGripperDriver::connect(const GeckoGripperConfig& config) {
    if (is_connected_) {
        return true;
    }

   try {
        // change the transport depending on the config.moteus.transport_args
        transport_ = std::make_shared<moteus::Fdcanusb>(config.can_interface);
        // moteus::Socketcan::Options can_opts;
        // can_opts.ifname = config.can_interface;
        // transport_ = std::make_shared<moteus::Socketcan>(can_opts);

        moteus::Controller::Options opts;
        opts.id = config.motor_id;
        opts.transport = transport_;
        
        auto& pf = opts.position_format;
        pf.position = moteus::kFloat;
        pf.velocity = moteus::kFloat;
        pf.feedforward_torque = moteus::kFloat;

        controller_ = std::make_shared<moteus::Controller>(opts);

        controller_->DiagnosticWrite("tel stop\n");
        controller_->DiagnosticFlush();

        controller_->DiagnosticCommand("conf set servo.pid_position.kp " + std::to_string(config.pid_position.kp));
        controller_->DiagnosticCommand("conf set servo.pid_position.ki " + std::to_string(config.pid_position.ki));
        controller_->DiagnosticCommand("conf set servo.pid_position.kd " + std::to_string(config.pid_position.kd));
        controller_->DiagnosticCommand("conf set servopos.position_min " + std::to_string(config.min_pos));
        controller_->DiagnosticCommand("conf set servopos.position_max " + std::to_string(config.max_pos));
        controller_->DiagnosticCommand("conf set servopos.maximum_velocity " + std::to_string(config.max_vel));
        controller_->DiagnosticCommand("conf set servopos.maximum_acceleration " + std::to_string(config.max_acc));
        
        controller_->SetStop();

        is_connected_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Moteus Connection Failed: " << e.what() << std::endl;
        return false;
    }
}

void GeckoGripperDriver::disconnect() {
    if (!is_connected_) return;

    if (controller_) {
        std::cout << "Engaging Moteus brake before disconnect." << std::endl;
        controller_->SetBrake();
    }

    transport_.reset();
    controller_.reset();
    is_connected_ = false;
}

bool GeckoGripperDriver::isConnected() const {
    return is_connected_;
}

// send a command to the controller and update our local state of the motor
bool GeckoGripperDriver::sendCmd() {
    if (!is_connected_) return false;

    receive_frames_.clear();
    
    // Transmit whatever is currently in send_frames_
    transport_->BlockingCycle(&send_frames_[0], send_frames_.size(), &receive_frames_);

    // Parse the response
    for (auto it = receive_frames_.rbegin(); it != receive_frames_.rend(); ++it) {
        if (it->source == controller_->options().id) {
            auto result = moteus::Query::Parse(it->data, it->size);
            
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_.position = result.position;
            latest_state_.velocity = result.velocity;
            latest_state_.torque = result.torque;
            return true;
        }
    }
    return false;
}

bool GeckoGripperDriver::queryState() {
    send_frames_.clear();
    send_frames_.push_back(controller_->MakeQuery());

    return sendCmd();
}

bool GeckoGripperDriver::executeControlCycle(const moteus::PositionMode::Command& cmd) {
    send_frames_.clear();
    send_frames_.push_back(controller_->MakePosition(cmd));

    return sendCmd();
}

GripperState GeckoGripperDriver::getLatestState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}

} // namespace gecko
} // namespace gripper
