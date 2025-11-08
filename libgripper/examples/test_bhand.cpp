
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

    bhand.close(gripper::barrett::MotorGroup::Spread);

    bhand.shutdown();

    return 0;
}


