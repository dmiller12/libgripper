#pragma once

#include <string>
#include <system_error>
namespace gripper {
namespace barrett {

enum class BarrettHandError {
    Success = 0,
    // Device Status Codes (Bitmask)
    NO_MOTOR_BOARD_FOUND = 1,
    NO_MOTOR_FOUND = 2,
    MOTOR_NOT_INITIALIZED = 4,
    COULDNT_REACH_POSITION = 16,
    UNKNOWN_COMMAND = 32,
    UNKNOWN_PARAMETER_NAME = 64,
    INVALID_VALUE = 128,
    WRITE_TO_READ_ONLY = 256,
    TOO_MANY_ARGUMENTS = 1024,
    INVALID_REALTIME_HEADER = 2048,
    INVALID_MOTOR_PREFIX = 4096,
    OVERTEMPERATURE_FAULT = 8192,
    CTRL_C_ABORT = 16384,

    // Driver-Level Errors (Unique Codes)
    NO_COMMUNICATOR = 0x10000001,
    COMMAND_TIMEOUT = 0x10000002,
    REALTIME_CONTROL_ACTIVE = 0x10000003,
    NO_LOOP_ACK = 0x10000004,
    PARSING_ERROR = 0x10000005,
};

class BarrettHandCategory : public std::error_category {
  public:
    const char* name() const noexcept override {
        return "BarrettHand";
    }

    std::string message(int condition) const override {
        // First, check for our special driver-level codes
        switch (static_cast<BarrettHandError>(condition)) {
            case BarrettHandError::NO_COMMUNICATOR:
                return "Communicator is not available";
            case BarrettHandError::COMMAND_TIMEOUT:
                return "The command timed out before a response was received";
            case BarrettHandError::REALTIME_CONTROL_ACTIVE:
                return "Cannot send supervisory command while in real-time control mode";
            case BarrettHandError::NO_LOOP_ACK:
                return "Did not receive loop acknowledgement";
            case BarrettHandError::PARSING_ERROR:
                return "Error parsing feedback block";
            default:
                break; // fall through to bitmask decoding
        }

        // Decode bitmask
        std::string msg;
        if (condition & static_cast<int>(BarrettHandError::NO_MOTOR_BOARD_FOUND))
            msg += "No motor board found; ";
        if (condition & static_cast<int>(BarrettHandError::NO_MOTOR_FOUND))
            msg += "No motor found; ";
        if (condition & static_cast<int>(BarrettHandError::MOTOR_NOT_INITIALIZED))
            msg += "Motor not initialized; ";
        if (condition & static_cast<int>(BarrettHandError::COULDNT_REACH_POSITION))
            msg += "Couldn't reach position; ";
        if (condition & static_cast<int>(BarrettHandError::UNKNOWN_COMMAND))
            msg += "Unknown command; ";
        if (condition & static_cast<int>(BarrettHandError::UNKNOWN_PARAMETER_NAME))
            msg += "Unknown parameter name; ";
        if (condition & static_cast<int>(BarrettHandError::INVALID_VALUE))
            msg += "Invalid value; ";
        if (condition & static_cast<int>(BarrettHandError::WRITE_TO_READ_ONLY))
            msg += "Tried to write a read-only property; ";
        if (condition & static_cast<int>(BarrettHandError::TOO_MANY_ARGUMENTS))
            msg += "Too many arguments; ";
        if (condition & static_cast<int>(BarrettHandError::INVALID_REALTIME_HEADER))
            msg += "Invalid RealTime header; ";
        if (condition & static_cast<int>(BarrettHandError::INVALID_MOTOR_PREFIX))
            msg += "Invalid motor prefix; ";
        if (condition & static_cast<int>(BarrettHandError::OVERTEMPERATURE_FAULT))
            msg += "Overtemperature fault; ";
        if (condition & static_cast<int>(BarrettHandError::CTRL_C_ABORT))
            msg += "Ctrl-C abort received; ";

        return msg.empty() ? "Unknown device error" : msg;
    }
};

const BarrettHandCategory& barrett_hand_category() {
    static BarrettHandCategory instance;
    return instance;
}

inline std::error_code make_error_code(BarrettHandError e) {
    return {static_cast<int>(e), barrett_hand_category()};
}

} // namespace barrett
} // namespace gripper

namespace std {
template <>
struct is_error_code_enum<gripper::barrett::BarrettHandError> : true_type {};
} // namespace std
