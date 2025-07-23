#include "gripper/barrett/barrett_hand.h"
#include "gripper/barrett/utils.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace gripper {
namespace barrett {

BarrettHand::BarrettHand()
    : driver_(std::make_unique<BarrettHandDriver>()) {
}
BarrettHand::~BarrettHand() {
    shutdown();
}

bool BarrettHand::isInitialized() {
    auto status_result = driver_->sendSupervisoryCommand("1234FGET S").get();

    if (!status_result) {
        return false;
    }

    std::stringstream raw_status(status_result.value());
    std::array<int, 4> status;

    // 0 indicates motor is initialized
    if (raw_status >> status[0] >> status[1] >> status[2] >> status[3]) {
        for (const int& s : status) {
            std::cout << s << std::endl;
            if (s != 0) {
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

    // Ctrl-C
    // driver_->sendSupervisoryCommand({0x03});

    if (force || !isInitialized()) {

        auto initilize_result = driver_->sendSupervisoryCommand("HI").get();

        if (!initilize_result) {
            std::cerr << "Failed to initialize hand: " << initilize_result.error().message() << std::endl;
            return false;
        }
    }

    auto initilize_result_spread = driver_->sendSupervisoryCommand("SFSET FPG 5 FIP 1 FDZ 100").get();
    auto initilize_result_grasp = driver_->sendSupervisoryCommand("GFSET FPG 10 FIP 0 FDZ 0").get();

    PerMotorRealtimeSettings finger_settings;

    // --- Define and apply the desired real-time configuration ---
    finger_settings.LCT = false;
    finger_settings.LCPG = false;
    finger_settings.LCV = true;
    finger_settings.LCVC = 2;
    finger_settings.LFV = false; // We want velocity feedback
    finger_settings.LFAP = true; // We want absolute position feedback

    realtime_settings_.motor_settings[MotorID::F1] = finger_settings;
    realtime_settings_.motor_settings[MotorID::F2] = finger_settings;
    realtime_settings_.motor_settings[MotorID::F3] = finger_settings;

    PerMotorRealtimeSettings spread_settings;
    spread_settings.LCT = true;
    spread_settings.LCVC = 0;
    spread_settings.LFV = false;
    spread_settings.LFAP = true;

    realtime_settings_.motor_settings[MotorID::Spread] = spread_settings;
    realtime_settings_.LFT = true;

    auto rt_config_result = driver_->configureRealtime(realtime_settings_);

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
        [this](const RealtimeFeedback& fb) { return this->controlLoopCallback(fb); }, MotorGroup::All
    );
    if (!rt_start_result) {
        std::cerr << "Failed to start RealTime: " << rt_start_result.error().message() << std::endl;
        driver_->disconnect();
        return false;
    }
    return true;
}

void BarrettHand::setPosition(const std::array<double, 4>& positions) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_position_ = positions;
    control_mode_ = ControlMode::Position;
}

void BarrettHand::setVelocity(const std::array<double, 4>& velocities, bool sync_position) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_velocity_ = velocities;
    control_mode_ = ControlMode::Velocity;
    sync_position_ = sync_position;
}

void BarrettHand::setVelocity(const double& finger, const double& spread) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_velocity_ = {finger, finger, finger, spread};
    control_mode_ = ControlMode::Velocity;
    sync_position_ = true;
}

void BarrettHand::open(const MotorGroup& group) {
    std::string prefix = motorGroupToPrefix(group);

    std::stringstream cmd;
    cmd << prefix << "O";

    driver_->stopRealtimeControl();
    driver_->sendSupervisoryCommand(cmd.str());
    this->startRealtimeControl();
    
}

void BarrettHand::close(const MotorGroup& group) {
    std::string prefix = motorGroupToPrefix(group);

    std::stringstream cmd;
    cmd << prefix << "C";

    driver_->stopRealtimeControl();
    (void)driver_->sendSupervisoryCommand(cmd.str()).get();
    this->startRealtimeControl();
    

}

bool hasIntersection(const std::vector<MotorID>& motors, const std::vector<MotorID>& targets) {
    for (MotorID targetMotor : targets) {
        // Check if the target motor exists in the main vector
        if (std::find(motors.begin(), motors.end(), targetMotor) != motors.end()) {
            return true;
        }
    }
    return false;
}

void BarrettHand::moveTo(const MotorGroup& group, const double& position) {

    std::vector<MotorID> motors = getMotorsInGroup(group);

    auto it = std::find(motors.begin(), motors.end(), MotorID::Spread);

    if (it != motors.end()) {

        motors.erase(it);
        
        std::string prefix = motorGroupToPrefix(MotorGroup::Spread);
        double clamped_pos = std::max(RADIAN_SPREAD_MIN, std::min(position, RADIAN_SPREAD_MAX));
        int32_t pos_count = radiansToCounts(clamped_pos, MotorID::Spread);
        std::stringstream cmd;
        cmd << prefix << " " << static_cast<int>(pos_count);
        (void)driver_->sendSupervisoryCommand(cmd.str()).get(); //ignore result

    }

    if (hasIntersection(motors, getMotorsInGroup(MotorGroup::AllFingers))) {

        double clamped_pos = std::max(RADIAN_FINGER_MIN, std::min(position, RADIAN_FINGER_MAX));
        int32_t pos_count = radiansToCounts(clamped_pos, MotorID::Spread);
        std::stringstream cmd;
        for (const auto& m  : motors) {
            cmd << motorIDToPrefix(m);
        }
        cmd <<  " " << static_cast<int>(pos_count);
        (void)driver_->sendSupervisoryCommand(cmd.str()).get(); //ignore result

    }

    driver_->stopRealtimeControl();
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

boost::optional<RealtimeControlSetpoint> BarrettHand::controlLoopCallback(const RealtimeFeedback& feedback) {
    HandState local_state;

    for (const auto& pair : feedback.positions) {
        local_state.joint_positions[static_cast<int>(pair.first)] = countsToRadians(pair.second, pair.first);
    }

    if (feedback.temperature_c) {
        local_state.temperature_celsius = static_cast<double>(feedback.temperature_c.value());
    } else {
        local_state.temperature_celsius = boost::none;
    }

    auto now = std::chrono::steady_clock::now();
    if (is_first) {
        is_first = false;
        last_time = now;
        return boost::none;
    }

    std::vector<MotorID> allMotors = getMotorsInGroup(MotorGroup::All);
    std::vector<MotorID> fingerMotors = getMotorsInGroup(MotorGroup::AllFingers);

    auto elapsed_time = now - last_time;
    double dt = std::chrono::duration<double>(elapsed_time).count();
    double max_dt = 0.1;
    double min_dt = 0.001;
    dt = std::min(std::max(min_dt, dt), max_dt);

    std::array<double, 4> raw_vel;

    for (const MotorID& m : allMotors) {
        size_t mIdx = static_cast<size_t>(m);
        double empirical_vel = (local_state.joint_positions[mIdx] - prev_state_.joint_positions[mIdx]) / dt;
        raw_vel[mIdx] = empirical_vel;
    }
    local_state.joint_velocities = velocity_filter_.update(raw_vel);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_ = local_state;
    }

    last_time = now;
    prev_state_ = local_state;

    // Convert desired physical velocity to raw command counts
    std::array<double, 4> commanded_position_rad;
    std::array<double, 4> commanded_velocity_rad;
    bool local_sync_position;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        commanded_position_rad = target_position_;
        commanded_velocity_rad = target_velocity_;
        local_sync_position = sync_position_;
    }

    // Create the raw control setpoint for the driver
    RealtimeControlSetpoint setpoint;
    std::array<double, 4> control_rad;
    switch (control_mode_) {
        case ControlMode::None:
            return boost::none;

        case ControlMode::Position:

            control_rad = position_contoller_.computeControl(commanded_position_rad, local_state.joint_positions, dt);
            break;
        case ControlMode::Velocity:

            if (local_sync_position) {
                double position_total = 0;
                for (const MotorID& m : fingerMotors) {
                    size_t mIdx = static_cast<size_t>(m);
                    position_total += local_state.joint_positions[mIdx];
                }
                double position_avg = position_total / fingerMotors.size();

                std::array<double, 4> correction = sync_position_contoller_.computeControl(
                    {position_avg, position_avg, position_avg, 0}, local_state.joint_positions, dt
                );

                for (const MotorID& m : fingerMotors) {
                    size_t mIdx = static_cast<size_t>(m);
                    commanded_velocity_rad[mIdx] += correction[mIdx];
                }
            }

            auto accel = velocity_contoller_.computeControl(
                commanded_velocity_rad, local_state.joint_velocities, local_state.joint_positions, dt
            );

            for (const MotorID& m : fingerMotors) {
                size_t mIdx = static_cast<size_t>(m);
                control_rad[mIdx] = local_state.joint_velocities[mIdx] + accel[mIdx] * dt;
            }
            size_t spreadIdx = static_cast<size_t>(MotorID::Spread);
            control_rad[spreadIdx] = accel[spreadIdx];
            break;
    }

    for (const auto& m : allMotors) {
        const PerMotorRealtimeSettings& motor_setting = realtime_settings_.motor_settings[m];
        size_t mIdx = static_cast<size_t>(m);
        if (m == MotorID::Spread) {
            int32_t control_count = static_cast<int32_t>(radiansToCounts(control_rad[mIdx], m));
            setpoint.torque_commands[m] = control_count;
        } else {

            int8_t vel_count = prepareVelocity(velocityRadToCounts(control_rad[mIdx], m), motor_setting.LCVC);

            setpoint.velocity_commands[m] = vel_count;
            setpoint.proportional_gains[m] = 20;
        }
    }

    printState(local_state);
    return setpoint;
}

} // namespace barrett
} // namespace gripper
