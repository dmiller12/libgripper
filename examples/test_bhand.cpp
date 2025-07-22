
#include "gripper/barrett/barrett_hand.h"
#include <cmath>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    // Check for command line argument for the serial port
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>" << std::endl;
        std::cerr << "Example: " << argv[0] << " /dev/ttyUSB0" << std::endl;
        return 1;
    }

    std::string port = argv[1];
    std::cout << "Attempting to connect to BarrettHand on port " << port << "..." << std::endl;

    // Create an instance of the driver
    gripper::barrett::BarrettHand bhand;

    bhand.initialize(port, true);

    std::atomic<bool> stop_thread(false);

    // --- Configuration ---
    const double max_position = 3.14/3.0;
    const double min_position = 0.0;
    const double cycles_per_second = 1.0 / 2.0;
    const double frequency = cycles_per_second * 2 * 3.14;

    std::thread position_thread([&bhand, &stop_thread, max_position, min_position, frequency] {
        const double vertical_offset = (max_position + min_position) / 2.0;
        const double amplitude = (max_position - min_position) / 2.0;
        const double phase_shift = -3.14 / 2.0;

        auto start_time = std::chrono::steady_clock::now();
        while (!stop_thread) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            double seconds = std::chrono::duration<double>(elapsed).count();
            double wave = std::sin(seconds * frequency + phase_shift);
            double position = vertical_offset + amplitude * wave;

            bhand.setPosition({position, position, position, 0});
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });


    std::this_thread::sleep_for(std::chrono::seconds(8));
    stop_thread = true;
    position_thread.join();
    
    // bhand.setPosition({0, 0, 0, 3.14 / 4.0});

    std::this_thread::sleep_for(std::chrono::seconds(1));
    // bhand.setGraspVelocity(-3.0);
    // std::this_thread::sleep_for(std::chrono::seconds(5));

    // Connect to the hand
    // Disconnect cleanly
    bhand.shutdown();

    return 0;
}


