#include "gripper/barrett/barrett_hand_driver.h"
#include "gripper/barrett/utils.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include <csignal>
#include <vector>
#include <numeric>
#include <map>

using namespace gripper::barrett;

// --- Global variables for graceful shutdown on Ctrl-C ---
std::atomic<bool> shutdown_flag{false};
BarrettHandDriver* driver_ptr = nullptr;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received." << std::endl;
    shutdown_flag = true;
    if (driver_ptr) {
        if(driver_ptr->isInRealtimeControl()) {
            driver_ptr->stopRealtimeControl();
        }
        driver_ptr->disconnect();
    }
    exit(signum);
}

// A simple helper to handle the outcome::result, printing errors and exiting.
void check_outcome(const outcome::result<void, std::error_code>& result, const std::string& message) {
    if (!result) {
        std::cerr << "ERROR: " << message << " - " << result.error().message() << std::endl;
        exit(EXIT_FAILURE);
    }
}

/**
 * @brief Runs a constant velocity test to gather data for one data point.
 * @param driver A reference to the connected BarrettHandDriver.
 * @param motor_to_test The ID of the motor to test.
 * @param target_velocity_command The constant velocity command to send.
 * @return The average measured velocity. Returns 0 on failure.
 */
double run_viscous_test_for_motor(BarrettHandDriver& driver, MotorID motor_to_test, int8_t target_velocity_command) {
    // --- Configuration ---
    RealtimeSettings settings;
    PerMotorRealtimeSettings motor_settings;
    motor_settings.LCV = true;
    motor_settings.LFV = true;
    motor_settings.LCVC = 2;
    motor_settings.LCPG = true;
    settings.motor_settings[motor_to_test] = motor_settings;

    driver.sendSupervisoryCommand("GO").get();

    const MotorGroup motor_to_test_group = static_cast<MotorGroup>(motor_to_test);
    check_outcome(driver.configureRealtime(settings), "Failed to configure real-time mode");

    // --- State variables for this test run ---
    std::atomic<bool> test_running{true};
    std::vector<double> velocity_history;
    const auto test_duration = std::chrono::seconds(1);
    const auto data_collection_start_time = 0.5 * std::chrono::seconds(1);

    // --- The Real-time Callback ---
    auto velocity_control_callback =
        [&](const RealtimeFeedback& feedback) -> boost::optional<RealtimeControlSetpoint> {

        if (!test_running || shutdown_flag) {
            return boost::none;
        }

        // Record the actual measured velocity
        if (feedback.velocities.count(motor_to_test)) {
            velocity_history.push_back(velocityCountsToRad(feedback.velocities.at(motor_to_test), motor_to_test));
        }
        double counts_per_tick = velocityRadToCounts(target_velocity_command, motor_to_test);
        int8_t velocity_command = prepareVelocity(counts_per_tick, motor_settings.LCVC);

        // Send the constant target velocity command directly to the hand's internal controller
        RealtimeControlSetpoint setpoint;
        setpoint.velocity_commands[motor_to_test] = velocity_command;
        setpoint.proportional_gains[motor_to_test] = 30; // A nominal gain
        return setpoint;
    };

    check_outcome(
        driver.startRealtimeControl(velocity_control_callback, motor_to_test_group),
        "Failed to start real-time control"
    );

    // --- Main Test Loop ---
    auto start_time = std::chrono::steady_clock::now();
    size_t collection_start_index = 0;
    bool collecting_data = false;

    while(test_running && !shutdown_flag && driver.isInRealtimeControl()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = now - start_time;

        if (!collecting_data && elapsed >= data_collection_start_time) {
            collection_start_index = velocity_history.size();
            collecting_data = true;
        }

        if (elapsed >= test_duration) {
            test_running = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    driver.stopRealtimeControl();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // --- Analyze Results ---
    if (collection_start_index >= velocity_history.size()) {
        return 0.0; // Not enough data collected
    }

    double sum = std::accumulate(
        velocity_history.begin() + collection_start_index,
        velocity_history.end(),
        0.0
    );
    return sum / (velocity_history.size() - collection_start_index);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>" << std::endl;
        return EXIT_FAILURE;
    }
    std::string port = argv[1];

    signal(SIGINT, signalHandler);

    BarrettHandDriver driver;
    driver_ptr = &driver;

    std::cout << "Connecting to BarrettHand on port " << port << "..." << std::endl;
    if (!driver.connect(port)) {
        std::cerr << "Failed to connect to the BarrettHand." << std::endl;
        return EXIT_FAILURE;
    }

    driver.sendSupervisoryCommand("HI").get();

    const std::vector<MotorID> FINGERS_TO_TEST = {MotorID::F1, MotorID::F2, MotorID::F3};
    const std::vector<double> VELOCITY_COMMANDS = {0.9, 1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1};
    std::map<MotorID, std::map<double, double>> results;

    for (const auto& finger_id : FINGERS_TO_TEST) {
        std::cout << "\n--- Testing Motor " << static_cast<int>(finger_id) + 1 << " ---" << std::endl;
        for (const auto& command_v : VELOCITY_COMMANDS) {
            std::cout << "  Running test for command velocity: " << command_v << "..." << std::flush;
            double avg_velocity = run_viscous_test_for_motor(driver, finger_id, command_v);
            results[finger_id][command_v] = avg_velocity;
            std::cout << " Done. Avg Measured Velocity: " << avg_velocity << std::endl;
            if (shutdown_flag) break;
        }
        if (shutdown_flag) break;
    }

    // --- Report Final Results ---
    std::cout << "\n\n----------------------------------------------------" << std::endl;
    std::cout << "      Viscous Friction Estimation Data" << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;
    std::cout << "Plot 'Avg Measured Velocity' vs 'Commanded Velocity' to find the slope.\n" << std::endl;

    for (const auto& finger_id : FINGERS_TO_TEST) {
        std::cout << "--- Motor " << static_cast<int>(finger_id) + 1 << " Results ---" << std::endl;
        std::cout << "Commanded Velocity | Avg Measured Velocity" << std::endl;
        std::cout << "-------------------|-----------------------" << std::endl;
        for (const auto& pair : results[finger_id]) {
            printf("        %f          |        %f\n", pair.first, pair.second);
        }
        std::cout << std::endl;
    }
    std::cout << "----------------------------------------------------" << std::endl;

    driver.disconnect();
    return EXIT_SUCCESS;
}
