#include "gripper/barrett/barrett_hand.h"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace gripper {
namespace barrett {

BarrettHand::BarrettHand() : driver_(std::make_unique<BarrettHandDriver>()), is_first(true) {}
BarrettHand::~BarrettHand() { shutdown(); }


bool BarrettHand::isInitialized() {
    auto status_result = driver_->sendSupervisoryCommand("1234FGET S").get();

    if (!status_result) {
        return false;
    }

    std::stringstream raw_status(status_result.value());
    std::array<int, 4> status;

    // 0 indicates motor is initialized
    if (raw_status >> status[0] >> status[1] >> status[2] >> status[3]) {
        for (const int& s: status) {
            std::cout << s << std::endl;
            if (s != 0){
                return false;
            }
        }
        return true;
    }

    return false;
}

bool BarrettHand::initialize(const std::string& port, bool force) {
    if (!driver_->connect(port)) {
        return false;
    }

    if (force || !isInitialized()) {

        auto initilize_result = driver_->sendSupervisoryCommand("HI").get();
        
        if (!initilize_result) {
            std::cerr << "Failed to initialize hand: " << initilize_result.error().message() << std::endl;
            return false;
        }
    }

    RealtimeSettings realtime_settings;
    PerMotorRealtimeSettings finger_settings;

    // --- Define and apply the desired real-time configuration ---
    finger_settings.LCV = true;  // We will send velocity commands
    finger_settings.LCPG = true;  // We will send velocity commands
    finger_settings.LFV = true;  // We want velocity feedback
    finger_settings.LFAP = true; // We want absolute position feedback

    realtime_settings.motor_settings[MotorID::F1] = finger_settings;
    realtime_settings.motor_settings[MotorID::F2] = finger_settings;
    realtime_settings.motor_settings[MotorID::F3] = finger_settings;

    PerMotorRealtimeSettings spread_settings;
    spread_settings.LFV = true;
    spread_settings.LFAP = true;

    realtime_settings.motor_settings[MotorID::Spread] = spread_settings;
    realtime_settings.LFT = true;

    auto rt_config_result = driver_->configureRealtime(realtime_settings);

    if (!rt_config_result) {
        std::cerr << "Failed to configure RealTime: " << rt_config_result.error().message() << std::endl;
        return false;
    }
    
    return this->startRealtimeControl();
}

void BarrettHand::shutdown() {
    if (driver_ && driver_->isConnected()) {
        driver_->stopRealtimeControl();
        driver_->disconnect();
    }
}

bool BarrettHand::startRealtimeControl() {

    auto rt_start_result = driver_->startRealtimeControl(
        [this](const RealtimeFeedback& fb) { return this->velocityPassthroughCallback(fb); },
        MotorGroup::All
    );
    if (!rt_start_result) {
        std::cerr << "Failed to start RealTime: " << rt_start_result.error().message()  << std::endl;
        driver_->disconnect();
        return false;
    }
    return true;
}

void BarrettHand::setGraspVelocity(double velocity) {
    target_grasp_velocity_ = velocity;
}

void BarrettHand::setSpread(double spread_position) {
    if (!driver_ || !driver_->isConnected()) return;

    float clamped_spread = std::max(0.0, std::min(spread_position, M_PI));
    int target_position = radiansToCounts(clamped_spread, MotorID::Spread);
    
    driver_->stopRealtimeControl();
    driver_->sendSupervisoryCommand("SM " + std::to_string(target_position));
    this->startRealtimeControl();
}

HandState BarrettHand::getLatestState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}

void printState(const HandState& handState) {
    // Print joint positions
    std::cout << "Joint Positions: [";
    for (size_t i = 0; i < handState.joint_positions.size(); ++i) {
        std::cout << handState.joint_positions[i] << (i < handState.joint_positions.size() - 1 ? ", " : "");
    }
    std::cout << "]" << std::endl;

    // Print joint velocities
    std::cout << "Joint Velocities: [";
    for (size_t i = 0; i < handState.joint_velocities.size(); ++i) {
        std::cout << handState.joint_velocities[i] << (i < handState.joint_velocities.size() - 1 ? ", " : "");
    }
    std::cout << "]" << std::endl;
    
    // Safely print the optional temperature
    std::cout << "Temperature: ";
    if (handState.temperature_celsius) {
        // The optional contains a value, so we can access it with *
        std::cout << *handState.temperature_celsius << " C" << std::endl;
    } else {
        // The optional is empty
        std::cout << "N/A" << std::endl;
    }
}

boost::optional<RealtimeControlSetpoint> BarrettHand::velocityPassthroughCallback(const RealtimeFeedback& feedback) {
    HandState local_state;

    for (const auto& pair : feedback.positions) {
        local_state.joint_positions[static_cast<int>(pair.first)] = countsToRadians(pair.second, pair.first);
    }
    for (const auto& pair : feedback.velocities) {
        local_state.joint_velocities[static_cast<int>(pair.first)] = velocityCountsToRad(pair.second, pair.first);
    }

    if (feedback.temperature_c) {
        local_state.temperature_celsius = static_cast<double>(feedback.temperature_c.value());
    } else {
        local_state.temperature_celsius = boost::none;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_ = local_state;
    }

    auto now = std::chrono::steady_clock::now();
    if (!is_first) {
        auto elapsed_time = now - last_time;
        double dt = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count();
        double empirical_vel = (local_state.joint_positions[0] - last_position) / (dt / 1000.0);
        std::cout << "Empirical vel: " <<  empirical_vel << std::endl;
    }

    is_first = false;
    last_time = now;
    last_position = local_state.joint_positions[0];

    // Convert desired physical velocity to raw command counts
    double commanded_vel_rad_per_sec = target_grasp_velocity_.load();
    int8_t velocity_cmd_counts = velocityRadToCounts(commanded_vel_rad_per_sec, MotorID::F1);

    std::cout << "commanded vel: " << static_cast<int>(velocity_cmd_counts) << std::endl;

    // Create the raw control setpoint for the driver
    RealtimeControlSetpoint setpoint;
    setpoint.velocity_commands[MotorID::F1] = velocity_cmd_counts;
    setpoint.velocity_commands[MotorID::F2] = velocity_cmd_counts;
    setpoint.velocity_commands[MotorID::F3] = velocity_cmd_counts;

    setpoint.proportional_gains[MotorID::F1] = 32;
    setpoint.proportional_gains[MotorID::F2] = 32;
    setpoint.proportional_gains[MotorID::F3] = 32;

    printState(local_state);


    return setpoint;
}


double BarrettHand::countsToRadians(int32_t counts, MotorID motor) const {
    if (motor == MotorID::F1 ||  motor == MotorID::F2 || motor == MotorID::F3) {
        return static_cast<double>(counts) / ENCODER_COUNTS_PER_RADIAN_FINGER;
    } else {
        return static_cast<double>(counts) / ENCODER_COUNTS_PER_RADIAN_SPREAD;
    }
}

int32_t BarrettHand::radiansToCounts(double radians, MotorID motor) const {
    if (motor == MotorID::F1 ||  motor == MotorID::F2 || motor == MotorID::F3) {
        return static_cast<int32_t>(radians * ENCODER_COUNTS_PER_RADIAN_FINGER);
    } else {
        return static_cast<int32_t>(radians * ENCODER_COUNTS_PER_RADIAN_SPREAD);
    }
}

int8_t BarrettHand::velocityRadToCounts(double velocity_rad_per_sec, MotorID motor) const {
    double sample_time = 2.56e-4; 
    if (motor == MotorID::F1 || motor == MotorID::F2 || motor == MotorID::F3) {
        double high_prec = std::round(16*velocity_rad_per_sec * ENCODER_COUNTS_PER_RADIAN_FINGER * sample_time);
        if (high_prec > 127) {
            high_prec = 127;
        }
        if (high_prec < -128) {
            high_prec = -128;
        }
        return static_cast<int8_t>(high_prec);
    } else {

        double high_prec = std::round(16*velocity_rad_per_sec * ENCODER_COUNTS_PER_RADIAN_SPREAD * sample_time);
        if (high_prec > 127) {
            high_prec = 127;
        }
        if (high_prec < -128) {
            high_prec = -128;
        }
        return static_cast<int8_t>(high_prec);
    }
}

double BarrettHand::velocityCountsToRad(int8_t velocity_counts, MotorID motor) const {
    double ticks_per_second = 100.0;
    if (motor == MotorID::F1 ||  motor == MotorID::F2 || motor == MotorID::F3) {
        return (static_cast<double>(velocity_counts) * ticks_per_second) / ENCODER_COUNTS_PER_RADIAN_FINGER; 
    } else {
        return (static_cast<double>(velocity_counts) * ticks_per_second )/ ENCODER_COUNTS_PER_RADIAN_SPREAD; 
    }
}

} // namespace barrett
} // namespace gripper
