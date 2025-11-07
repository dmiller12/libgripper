#pragma once

#include <cstdint>

namespace gripper {
namespace robotiq {

// Holding register base addresses from the Robotiq 2F gripper Modbus specification.
static constexpr uint16_t kCommandRegisterBase = 0x03E8; // rACT register
static constexpr uint16_t kStatusRegisterBase = 0x07D0;  // gACT register

// Command register offsets (write-only)
static constexpr uint16_t kRegisterRACT = kCommandRegisterBase + 0; // Activation
static constexpr uint16_t kRegisterRGTO = kCommandRegisterBase + 1; // Go to
static constexpr uint16_t kRegisterRATR = kCommandRegisterBase + 2; // Auto-release
static constexpr uint16_t kRegisterRPR  = kCommandRegisterBase + 3; // Position request
static constexpr uint16_t kRegisterRSP  = kCommandRegisterBase + 4; // Speed request
static constexpr uint16_t kRegisterRFR  = kCommandRegisterBase + 5; // Force request

// Status register offsets (read-only)
static constexpr uint16_t kRegisterGACT = kStatusRegisterBase + 0; // Activation status
static constexpr uint16_t kRegisterGGTO = kStatusRegisterBase + 1; // Action status
static constexpr uint16_t kRegisterGSTA = kStatusRegisterBase + 2; // Gripper status
static constexpr uint16_t kRegisterGOBJ = kStatusRegisterBase + 3; // Object detection
static constexpr uint16_t kRegisterGFLT = kStatusRegisterBase + 4; // Fault code
static constexpr uint16_t kRegisterGPR  = kStatusRegisterBase + 5; // Requested position echo
static constexpr uint16_t kRegisterGPO  = kStatusRegisterBase + 6; // Actual position
static constexpr uint16_t kRegisterGCU  = kStatusRegisterBase + 7; // Motor current

static constexpr double kRobotiqMaxWidthMm = 140.0;

inline double clamp(double value, double min_value, double max_value) {
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

} // namespace robotiq
} // namespace gripper

