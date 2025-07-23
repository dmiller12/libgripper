#include "gripper/barrett/barrett_hand_driver.h"
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
 * @brief Runs a single breakaway velocity test for a specified motor.
 * @param driver A reference to the connected BarrettHandDriver.
 * @param motor_to_test The ID of the motor to test.
 * @return The velocity command at which movement was first detected. Returns 0 on failure.
 */
int8_t run_test_for_motor(BarrettHandDriver& driver, MotorID motor_to_test) {
    // --- Configuration for Friction Estimation ---
    RealtimeSettings settings;
    PerMotorRealtimeSettings motor_settings;
    motor_settings.LCV = true;  // Enable velocity control
    motor_settings.LFV = true;  // Enable velocity feedback
    motor_settings.LCPG = true; // Enable proportional gain control (even if fixed)
    settings.motor_settings[motor_to_test] = motor_settings;

    const MotorGroup motor_to_test_group = static_cast<MotorGroup>(motor_to_test);

    check_outcome(driver.configureRealtime(settings), "Failed to configure real-time mode");

    // --- State variables for this specific test run ---
    std::atomic<int8_t> current_velocity_command{0};
    std::atomic<bool> test_complete{false};
    std::atomic<int8_t> breakaway_velocity_command{0};

    // --- Constants for the test ramp ---
    const int8_t VELOCITY_RAMP_STEP = 1;
    const auto RAMP_INTERVAL = std::chrono::milliseconds(100);
    const int8_t MAX_VELOCITY_COMMAND = 127;

    // --- The Real-time Callback ---
    auto friction_estimation_callback =
        [&](const RealtimeFeedback& feedback) -> boost::optional<RealtimeControlSetpoint> {

        if (test_complete || shutdown_flag) {
            return boost::none;
        }

        if (feedback.velocities.count(motor_to_test) && std::abs(feedback.velocities.at(motor_to_test)) > 0) {
            breakaway_velocity_command = current_velocity_command.load();
            test_complete = true;
            std::cout << "  Movement detected!" << std::endl;
            return boost::none;
        }

        RealtimeControlSetpoint setpoint;
        setpoint.velocity_commands[motor_to_test] = current_velocity_command.load();
        setpoint.proportional_gains[motor_to_test] = 20; // A nominal gain
        return setpoint;
    };

    check_outcome(
        driver.startRealtimeControl(friction_estimation_callback, motor_to_test_group),
        "Failed to start real-time control"
    );

    // --- Main Test Loop ---
    while (!test_complete && !shutdown_flag && driver.isInRealtimeControl()) {
        std::this_thread::sleep_for(RAMP_INTERVAL);
        current_velocity_command += VELOCITY_RAMP_STEP;

        if (current_velocity_command > MAX_VELOCITY_COMMAND) {
            std::cerr << "  Test stopped: Velocity command limit reached without movement." << std::endl;
            test_complete = true;
        }
    }

    driver.stopRealtimeControl();
    // Give the hand a moment to settle before the next test
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
    
    return breakaway_velocity_command.load();
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

    const int NUM_RUNS = 3;
    const std::vector<MotorID> FINGERS_TO_TEST = {MotorID::F1, MotorID::F2, MotorID::F3};
    std::map<MotorID, std::vector<int8_t>> results;

    for (int i = 1; i <= NUM_RUNS; ++i) {
        std::cout << "\n--- Starting Run " << i << " of " << NUM_RUNS << " ---" << std::endl;
        for (const auto& finger_id : FINGERS_TO_TEST) {
            std::cout << "Testing Motor " << static_cast<int>(finger_id) + 1 << "..." << std::endl;
            int8_t result = run_test_for_motor(driver, finger_id);
            if (result > 0) {
                results[finger_id].push_back(result);
            }
            if (shutdown_flag) break;
        }
        if (shutdown_flag) break;
    }

    // --- Report Final Results ---
    std::cout << "\n\n----------------------------------------------------" << std::endl;
    std::cout << "  Minimum Velocity Command Estimation Complete" << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;

    for (const auto& finger_id : FINGERS_TO_TEST) {
        std::cout << "Motor " << static_cast<int>(finger_id) + 1 << ":" << std::endl;
        const auto& motor_results = results[finger_id];
        if (motor_results.empty()) {
            std::cout << "  No valid results recorded." << std::endl;
            continue;
        }

        double sum = std::accumulate(motor_results.begin(), motor_results.end(), 0.0);
        double average = sum / motor_results.size();

        std::cout << "  - Individual Runs: ";
        for(size_t i = 0; i < motor_results.size(); ++i) {
            std::cout << static_cast<int>(motor_results[i]) << (i == motor_results.size() - 1 ? "" : ", ");
        }
        std::cout << std::endl;
        std::cout << "  - Average Breakaway Command: " << average << std::endl;
    }
    std::cout << "----------------------------------------------------" << std::endl;

    driver.disconnect();
    return EXIT_SUCCESS;
}
