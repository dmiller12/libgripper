#include "gripper/barrett/barrett_hand.h"
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

void BarrettHand::setSpread(double spread_position) {
    if (!driver_ || !driver_->isConnected())
        return;

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

double BarrettHand::countsToRadians(int32_t counts, MotorID motor) const {
    if (motor == MotorID::F1 || motor == MotorID::F2 || motor == MotorID::F3) {
        return static_cast<double>(counts) / ENCODER_COUNTS_PER_RADIAN_FINGER;
    } else {
        return static_cast<double>(counts) / ENCODER_COUNTS_PER_RADIAN_SPREAD;
    }
}

int32_t BarrettHand::radiansToCounts(double radians, MotorID motor) const {
    if (motor == MotorID::F1 || motor == MotorID::F2 || motor == MotorID::F3) {
        return static_cast<int32_t>(radians * ENCODER_COUNTS_PER_RADIAN_FINGER);
    } else {
        return static_cast<int32_t>(radians * ENCODER_COUNTS_PER_RADIAN_SPREAD);
    }
}

int8_t BarrettHand::prepareVelocity(double velocity_count, uint8_t LCVC) const {
    double vel_shifted = 16 * velocity_count / static_cast<double>(LCVC);
    if (vel_shifted > 127) {
        vel_shifted = 127;
    }
    if (vel_shifted < -128) {
        vel_shifted = -128;
    }
    return static_cast<int8_t>(vel_shifted);
}

double BarrettHand::velocityRadToCounts(double velocity_rad_per_sec, MotorID motor) const {
    if (motor == MotorID::F1 || motor == MotorID::F2 || motor == MotorID::F3) {
        return velocity_rad_per_sec * ENCODER_COUNTS_PER_RADIAN_FINGER * SAMPLE_TIME;
    } else {

        return velocity_rad_per_sec * ENCODER_COUNTS_PER_RADIAN_SPREAD * SAMPLE_TIME;
    }
}

double BarrettHand::velocityCountsToRad(int8_t velocity_counts, MotorID motor) const {
    double ticks_per_second = 100.0;
    if (motor == MotorID::F1 || motor == MotorID::F2 || motor == MotorID::F3) {
        return (static_cast<double>(velocity_counts) * ticks_per_second) / ENCODER_COUNTS_PER_RADIAN_FINGER;
    } else {
        return (static_cast<double>(velocity_counts) * ticks_per_second) / ENCODER_COUNTS_PER_RADIAN_SPREAD;
    }
}

} // namespace barrett
} // namespace gripper
