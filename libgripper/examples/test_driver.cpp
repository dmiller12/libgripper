
#include "gripper/barrett/barrett_hand_driver.h"
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
    gripper::barrett::BarrettHandDriver driver;

    // Connect to the hand
    if (!driver.connect(port)) {
        std::cerr << "Failed to connect to the BarrettHand." << std::endl;
        return 1;
    }

    std::cout << "Successfully connected." << std::endl;

    // Send the HI supervisory command to initialize all motors
    auto result_future = driver.sendSupervisoryCommand("HI");

    auto result_future2 = driver.sendSupervisoryCommand("1234FGET S");
    auto result = result_future.get();

    if (result) {
        std::cout << "Result: " << result.value() << std::endl;
    } else {
        std::cerr << "Error: " << result.error().message() << std::endl;
    }

    auto result2 = result_future2.get();

    if (result2) {
        std::cout << "Result: " << result2.value() << std::endl;
    } else {
        std::cerr << "Error: " << result2.error().message() << std::endl;
    }

    // Disconnect cleanly
    driver.disconnect();

    return 0;
}


