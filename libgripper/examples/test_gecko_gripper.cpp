#include "gripper/gecko/gecko_gripper.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace gripper::gecko;

int main(int argc, char** argv) {
    GeckoGripper gripper;

    if (!gripper.initialize()) {
        std::cerr << "ERROR: Failed to initialize Gecko Gripper." << std::endl;
        return -1;
    }
    std::cout << "Successfully connected to Moteus controller." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    gripper.setPosition(-0.2);
    for (int i = 0; i < 1000; ++i) {
        // gripper.updateLocalState();
        gripper.controlLoopCallback();

        GripperState state = gripper.getLatestState();
        
        std::cout << "Iter " << i << " | "
                  << "Pos: " << state.position << " | "
                  << "Vel: " << state.velocity << " | "
                  << "Trq: " << state.torque << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    gripper.shutdown();
    std::cout << "Disconnected successfully." << std::endl;

    return 0;
}
