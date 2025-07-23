
#include "gripper/barrett/barrett_hand.h"
#include <cmath>
#include <iostream>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

enum KeyCode {
    KEY_UNKNOWN = -1,
    KEY_UP = 1000,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_Q = 'q',
};

class TerminalManager {
  public:
    TerminalManager() {
        // Get current terminal settings
        tcgetattr(STDIN_FILENO, &oldt_);
        newt_ = oldt_;
        // Set terminal to non-canonical mode (read characters immediately)
        // and disable echo (don't print typed characters).
        newt_.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);
        // Set stdin to non-blocking.
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
    }

    ~TerminalManager() {
        // Restore original terminal settings upon destruction.
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
    }

  private:
    struct termios oldt_, newt_;
};

KeyCode get_key_press() {
    char c;
    // Read a single character from stdin.
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n <= 0) {
        // No character read, so no key was pressed.
        return KEY_UNKNOWN;
    }

    // Check for an escape sequence, which is how arrow keys are sent.
    // The sequence is typically '\x1b' followed by '[' and a letter.
    if (c == '\x1b') {
        char seq[2];
        // Try to read the next two characters of the sequence.
        if (read(STDIN_FILENO, &seq[0], 1) != 1)
            return KEY_UNKNOWN;
        if (read(STDIN_FILENO, &seq[1], 1) != 1)
            return KEY_UNKNOWN;

        if (seq[0] == '[') {
            switch (seq[1]) {
                case 'A':
                    return KEY_UP;
                case 'B':
                    return KEY_DOWN;
                case 'C':
                    return KEY_RIGHT;
                case 'D':
                    return KEY_LEFT;
            }
        }
        return KEY_UNKNOWN;
    } else {
        // It's a regular key, not an escape sequence.
        return (KeyCode) c;
    }
}

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
    bool stop_program = false;

    TerminalManager term_manager;

    std::cout << "Keyboard control enabled." << std::endl;
    std::cout << "Press UP arrow for velocity 1.0, DOWN arrow for -1.0." << std::endl;
    std::cout << "Press 'q' to quit." << std::endl;

    auto last_key_time = std::chrono::steady_clock::now();
    // Timeout in milliseconds. If no key is pressed for this duration, velocity goes to 0.
    const auto key_timeout = std::chrono::milliseconds(250);

    // Main control loop, running until 'q' is pressed.
    while (!stop_program) {
        KeyCode key = get_key_press();

        if (key != KEY_UNKNOWN) {
            // If a key was pressed, update the timestamp.
            last_key_time = std::chrono::steady_clock::now();

            switch (key) {
                case KEY_UP:
                    bhand.setVelocity({2.0, 2.0, 2.0, 0.0});
                    break;
                case KEY_DOWN:
                    bhand.setVelocity({-2.0, -2.0, -2.0, 0.0});
                    break;
                case KEY_RIGHT:
                    // Example: Control the 4th motor (spread)
                    bhand.setVelocity({0.0, 0.0, 0.0, 1.0});
                    break;
                case KEY_LEFT:
                    // Example: Control the 4th motor (spread)
                    bhand.setVelocity({0.0, 0.0, 0.0, -1.0});
                    break;
                case KEY_Q:
                    bhand.setVelocity({0.0, 0.0, 0.0, 0.0}); // Stop before quitting
                    stop_program = true;
                    break;
                default:
                    break;
            }
        } else {
            // No key was pressed in this cycle. Check if the timeout has passed.
            auto now = std::chrono::steady_clock::now();
            if (now - last_key_time > key_timeout) {
                // It's been a while since the last key press, so set velocity to 0.
                bhand.setVelocity({0.0, 0.0, 0.0, 0.0});
            }
        }

        // Sleep for a short duration to prevent the loop from consuming 100% CPU.
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    bhand.shutdown();

    return 0;
}
