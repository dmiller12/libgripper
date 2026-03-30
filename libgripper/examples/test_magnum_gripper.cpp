#include "gripper/magnum_opus/magnum_gripper.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace gripper::magnum_opus;

int main(int argc, char** argv) {
    MagnumGripper gripper;

    if (!gripper.initialize()) {
        std::cerr << "ERROR: Failed to initialize Magnum Gripper." << std::endl;
        return -1;
    }
    std::cout << "Successfully connected to Moteus controller." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    gripper.setPosition(-0.01);
    for (int i = 0; i < 200; ++i) {
        // gripper.updateLocalState();
        gripper.controlLoopCallback();

        GripperState state = gripper.getLatestState();
        
        std::cout << "Iter " << i << " | "
                  << "Pos: " << state.position << " | "
                  << "Vel: " << state.velocity << " | "
                  << "Trq: " << state.torque << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gripper.shutdown();
    std::cout << "Disconnected successfully." << std::endl;

    return 0;
}