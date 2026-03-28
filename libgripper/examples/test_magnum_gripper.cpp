#include "gripper/magnum_opus/magnum_gripper.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace gripper::magnum_opus;

int main(int argc, char** argv) {
    std::string can_iface = "can0";
    if (argc > 1) {
        can_iface = argv[1];
    }

    std::cout << "--- Magnum Gripper Basic Test ---" << std::endl;
    std::cout << "Attempting to connect on interface: " << can_iface << std::endl;

    MagnumGripper gripper;

    if (!gripper.initialize(can_iface)) {
        std::cerr << "ERROR: Failed to initialize Magnum Gripper." << std::endl;
        return -1;
    }
    std::cout << "Successfully connected to Moteus controller." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "\nStarting state reading loop (10 iterations)..." << std::endl;
    for (int i = 0; i < 10; ++i) {
        gripper.controlLoopCallback();

        GripperState state = gripper.getLatestState();
        
        std::cout << "Iter " << i << " | "
                  << "Pos: " << state.position << " | "
                  << "Vel: " << state.velocity << " | "
                  << "Trq: " << state.torque << " | "
                  << "Temp: " << state.temperature_c << " °C" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\nShutting down..." << std::endl;
    gripper.shutdown();
    std::cout << "Disconnected successfully. Test complete." << std::endl;

    return 0;
}