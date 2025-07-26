#include "gripper/barrett/barrett_hand_driver.h"
#include "../communication/serial_communicator.h"
#include "gripper/barrett/barrett_error.h"
#include "task_worker.h"

#include <algorithm>
#include <boost/outcome/try.hpp>
#include <iostream>
#include <sstream>
#include <vector>

namespace gripper {
namespace barrett {

std::string motorGroupToPrefix(MotorGroup group) {
    switch (group) {
        case MotorGroup::F1:
            return "1";
        case MotorGroup::F2:
            return "2";
        case MotorGroup::F3:
            return "3";
        case MotorGroup::Spread:
            return "S";
        case MotorGroup::AllFingers:
            return "G";
        case MotorGroup::All:
            return "1234";
    }
    return "";
}

std::string motorIDToPrefix(MotorID id) {
    switch (id) {
        case MotorID::F1:
            return "1";
        case MotorID::F2:
            return "2";
        case MotorID::F3:
            return "3";
        case MotorID::Spread:
            return "4";
    }
    return "";
}

// Ensure the order matches what is expected from RealTime loop
std::vector<MotorID> getMotorsInGroup(MotorGroup group) {
    switch (group) {
        case MotorGroup::F1:
            return {MotorID::F1};
        case MotorGroup::F2:
            return {MotorID::F2};
        case MotorGroup::F3:
            return {MotorID::F3};
        case MotorGroup::Spread:
            return {MotorID::Spread};
        case MotorGroup::AllFingers:
            return {MotorID::F1, MotorID::F2, MotorID::F3};
        case MotorGroup::All:
            return {MotorID::F1, MotorID::F2, MotorID::F3, MotorID::Spread};
    }
    return {};
}

void printByte(uint8_t byte) {
    std::cout << "  Received byte: '" << static_cast<char>(byte) << "' (ASCII: " << static_cast<int>(byte)
              << ", Hex: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << std::dec
              << ")" << std::endl; // Switch back to decimal
    return;
}

struct BarrettHandDriver::Impl {
    std::unique_ptr<TaskWorker> worker;
};

BarrettHandDriver::BarrettHandDriver()
    : pimpl_(std::make_unique<Impl>()) {
}

BarrettHandDriver::~BarrettHandDriver() {
    disconnect();
}

bool BarrettHandDriver::connect(const std::string& port, unsigned int baud_rate) {
    if (isConnected()) {
        return true;
    }

    communicator_ = std::make_unique<SerialCommunicator>();
    if (!communicator_->connect(port, baud_rate)) {
        communicator_.reset();
        mode_ = OperatingMode::Disconnected;
        return false;
    }

    pimpl_->worker = std::make_unique<TaskWorker>();

    stop_threads_ = false;
    mode_ = OperatingMode::Supervisory;

    // clear previous errors and buffer
    communicator_->write({0x03}).get();
    communicator_->read_until("=> ", 1500).get();

    return true;
}

void BarrettHandDriver::disconnect() {
    if (!isConnected())
        return;

    stop_threads_ = true;

    if (realtime_thread_.joinable()) {
        realtime_thread_.join();
    }

    if (pimpl_->worker) {
        pimpl_->worker.reset();
    }


    if (communicator_) {
        // Cancel last request
        communicator_->write({0x03}).get();
        communicator_.reset();
    }
    mode_ = OperatingMode::Disconnected;
}

// TODO: Should this be internal mode or status of communicator?
bool BarrettHandDriver::isConnected() const {
    return communicator_ && communicator_->isOpen();
}

bool BarrettHandDriver::isInRealtimeControl() const {
    return mode_ == OperatingMode::RealTime;
}

std::future<outcome::result<std::string, std::error_code>>
BarrettHandDriver::sendSupervisoryCommand(const std::string& command) {
    if (isInRealtimeControl()) {
        auto promise = std::promise<outcome::result<std::string, std::error_code>>();
        auto future = promise.get_future();
        promise.set_value(make_error_code(BarrettHandError::REALTIME_CONTROL_ACTIVE));
        return future;
    }
    return sendAsynchronousCommand(command, 15000);
}

outcome::result<void, std::error_code> BarrettHandDriver::configureRealtime(const RealtimeSettings& settings) {
    current_rt_settings_ = settings; // Store for later use in parsing
    // TODO: Could do some setting validation here

    std::vector<MotorID> all_motors_ordered = {MotorID::F1, MotorID::F2, MotorID::F3, MotorID::Spread};
    for (MotorID motor_id : all_motors_ordered) {
        if (settings.motor_settings.count(motor_id)) {
            const PerMotorRealtimeSettings& motor_setting = settings.motor_settings.at(motor_id);
            std::string prefix = motorGroupToPrefix(static_cast<MotorGroup>(motor_id)); // Use motor ID as group prefix

            std::stringstream cmd;
            cmd << prefix << "FSET";
            cmd << " LCV " << (motor_setting.LCV ? 1 : 0);
            cmd << " LCVC " << static_cast<int>(motor_setting.LCVC);
            cmd << " LCPG " << (motor_setting.LCPG ? 1 : 0);
            cmd << " LCT " << (motor_setting.LCT ? 1 : 0);
            cmd << " LFV " << (motor_setting.LFV ? 1 : 0);
            cmd << " LFVC " << static_cast<int>(motor_setting.LFVC);
            cmd << " LFAP " << (motor_setting.LFAP ? 1 : 0);
            cmd << " LFS 0"; // No strain gauge on our bhand
            cmd << " LFDP 0";
            cmd << " LFBP 0";
            cmd << " LFAIN 0";

            BOOST_OUTCOME_TRY(sendSupervisoryCommand(cmd.str()).get());
        }
    }

    BOOST_OUTCOME_TRY(sendSupervisoryCommand("PSET LFT " + std::to_string(settings.LFT ? 1 : 0)).get());

    return outcome::success();
}

outcome::result<void, std::error_code>
BarrettHandDriver::startRealtimeControl(RealtimeCallback callback, MotorGroup group) {
    if (!isConnected()) {
        return make_error_code(BarrettHandError::NO_COMMUNICATOR);
    }

    if (mode_ == OperatingMode::RealTime) {
        return make_error_code(BarrettHandError::REALTIME_CONTROL_ACTIVE);
    }

    realtime_callback_ = std::move(callback);

    std::string loop_cmd = motorGroupToPrefix(group) + "LOOP";
    BOOST_OUTCOME_TRY(response, sendAsynchronousCommand(loop_cmd, 500, true).get());

    if (response.find('*') == std::string::npos) {
        return make_error_code(BarrettHandError::NO_LOOP_ACK);
    }

    mode_ = OperatingMode::RealTime;
    realtime_thread_ = std::thread(&BarrettHandDriver::realtimeControlLoop, this, group);
    return outcome::success();
}

void BarrettHandDriver::stopRealtimeControl() {
    OperatingMode expected = OperatingMode::RealTime;
    if (!mode_.compare_exchange_strong(expected, OperatingMode::Supervisory))
        return;

    if (realtime_thread_.joinable()) {
        realtime_thread_.join();
    }

    // Cancel realtime and wait for prompt
    communicator_->write({0x03}).get();
    communicator_->read_until("=> ", 1500).get();
}

void BarrettHandDriver::realtimeControlLoop(MotorGroup group) {
    size_t feedback_size = calculateFeedbackBlockSize(group);

    std::vector<uint8_t> control_block;
    control_block.push_back('A');

    if (!communicator_->write(control_block).get()) {
        std::cerr << "BarrettHandDriver RT ERROR: Write failed." << std::endl;
        return;
    }

    std::vector<uint8_t> feedback_block = communicator_->read(feedback_size, 100).get();

    if (feedback_block.empty()) {
        std::cerr << "Empty response" << std::endl;
        return;
    }
    auto parsed_feedback_result = parseFeedbackBlock(feedback_block, group);

    if (!parsed_feedback_result) {
        std::cerr << "Parsing Error: " << parsed_feedback_result.error().message() << std::endl;
        return;
    }

    RealtimeFeedback feedback = parsed_feedback_result.value();

    while (mode_ == OperatingMode::RealTime && !stop_threads_) {
        auto maybe_setpoint = realtime_callback_(feedback);

        std::vector<uint8_t> control_block;
        if (maybe_setpoint) {
            RealtimeControlSetpoint setpoint = maybe_setpoint.value();
            control_block.push_back('C');

            std::vector<MotorID> all_motors_ordered = getMotorsInGroup(group);
            for (MotorID motor_id : all_motors_ordered) {
                if (current_rt_settings_.motor_settings.count(motor_id)) {

                    const PerMotorRealtimeSettings& motor_settings = current_rt_settings_.motor_settings.at(motor_id);
                    if (motor_settings.LCV) {
                        control_block.push_back(
                            setpoint.velocity_commands.count(motor_id) ? setpoint.velocity_commands.at(motor_id) : 0
                        );
                    }
                    if (motor_settings.LCPG) {
                        control_block.push_back(
                            setpoint.proportional_gains.count(motor_id) ? setpoint.proportional_gains.at(motor_id) : 0
                        );
                    }
                    if (motor_settings.LCT) {
                        const int16_t torque_command =
                            setpoint.torque_commands.count(motor_id) ? setpoint.torque_commands.at(motor_id) : 0;
                        control_block.push_back(static_cast<uint8_t>(torque_command >> 8));
                        control_block.push_back(static_cast<uint8_t>(torque_command & 0xFF));
                    }
                }
            }
        } else {
            control_block.push_back('A');
        }

        if (!communicator_->write(control_block).get()) {
            std::cerr << "BarrettHandDriver RT ERROR: Write failed." << std::endl;
            return;
        }

        // TODO: More consistent timing
        std::vector<uint8_t> feedback_block = communicator_->read(feedback_size, 100).get();
        if (feedback_block.empty()) {
            std::cerr << "Empty response" << std::endl;
            return;
        }

        auto parsed_feedback_result = parseFeedbackBlock(feedback_block, group);

        if (!parsed_feedback_result) {
            std::cerr << "Parsing Error: " << parsed_feedback_result.error().message() << std::endl;
            return;
        } 

        feedback = parsed_feedback_result.value();
    }
}

std::future<outcome::result<std::string, std::error_code>>
BarrettHandDriver::sendAsynchronousCommand(const std::string& command_str, int timeout_ms, bool is_loop_cmd) {

    if (!pimpl_->worker) {
        std::promise<outcome::result<std::string, std::error_code>> promise;
        //TODO: Need different error code here
        promise.set_value(make_error_code(BarrettHandError::NO_COMMUNICATOR));
        return promise.get_future();
    }

    return pimpl_->worker->enqueue(
        [this, command_str, timeout_ms, is_loop_cmd]() -> outcome::result<std::string, std::error_code> {
            if (!communicator_) {
                return make_error_code(BarrettHandError::NO_COMMUNICATOR);
            }
            if (!communicator_->isOpen()) {
                return make_error_code(BarrettHandError::NO_COMMUNICATOR);
            }

            std::vector<uint8_t> command_bytes_to_send(command_str.begin(), command_str.end());
            command_bytes_to_send.push_back('\r');

            if (!communicator_->write(command_bytes_to_send).get()) {
                return make_error_code(BarrettHandError::NO_COMMUNICATOR);
            }

            // Read and discard the echo
            std::vector<uint8_t> echo_bytes = communicator_->read(command_bytes_to_send.size() - 1, timeout_ms).get();

            //TODO: add debug mode?
            // for (const uint8_t byte : echo_bytes) {
            //     printByte(byte);
            // }

            bool echo_correct = !echo_bytes.empty() && echo_bytes.size() <= command_bytes_to_send.size()
                                && std::equal(echo_bytes.begin(), echo_bytes.end(), command_bytes_to_send.begin());

            if (!echo_correct) {
                std::cerr << "Echo incorrect" << std::endl;
                return make_error_code(BarrettHandError::COMMAND_TIMEOUT);
            }

            std::vector<uint8_t> response_bytes;
            if (is_loop_cmd) {
                response_bytes = communicator_->read(1, timeout_ms).get();

                if (response_bytes.empty()) {
                    return make_error_code(BarrettHandError::COMMAND_TIMEOUT);
                }

                if (static_cast<char>(response_bytes[0]) == '*') {
                    return "*";
                }
            }

            // Get actual response
            std::string end_of_response_marker = "\n\r=> ";
            std::vector<uint8_t> utill_marker_response_bytes =
                communicator_->read_until(end_of_response_marker, timeout_ms).get();

            response_bytes.insert(
                response_bytes.end(), utill_marker_response_bytes.begin(), utill_marker_response_bytes.end()
            );

            //TODO: add debug mode?
            // for (const uint8_t byte : response_bytes) {
            //     printByte(byte);
            // }
            std::string actual_response_buffer(response_bytes.begin(), response_bytes.end());
            bool response_correct =
                !actual_response_buffer.empty() && actual_response_buffer.length() >= end_of_response_marker.length()
                && actual_response_buffer.substr(actual_response_buffer.length() - end_of_response_marker.length())
                       == end_of_response_marker;

            if (!response_correct) {
                return make_error_code(BarrettHandError::COMMAND_TIMEOUT);
            }

            // Check for Barrett Hand error response (e.g., "ERR 123 =>")
            std::string error_prefix = "\n\rERR ";
            size_t error_pos = actual_response_buffer.find(error_prefix);
            if (error_pos != std::string::npos) {
                try {
                    std::string error_code_str = actual_response_buffer.substr(
                        error_pos + error_prefix.length(),
                        actual_response_buffer.length() - error_pos - end_of_response_marker.length()
                    );
                    int error_val = std::stoi(error_code_str);
                    return make_error_code(static_cast<BarrettHandError>(error_val));
                } catch (const std::exception& e) {
                    std::cerr << "BarrettHandDriver WARNING: Failed to parse Barrett error code: " << e.what()
                              << ". Response: '" << actual_response_buffer << "'" << std::endl;
                    return make_error_code(BarrettHandError::UNKNOWN_COMMAND);
                }
            }

            // This is a successful response, remove end of response marker before returning
            actual_response_buffer.resize(actual_response_buffer.length() - end_of_response_marker.length());
            return actual_response_buffer;
        }
    );
}

size_t BarrettHandDriver::calculateFeedbackBlockSize(MotorGroup group) const {

    size_t size = 1; // Account for header size
    std::vector<MotorID> all_motors_in_feedback_order = getMotorsInGroup(group);

    for (MotorID motor_id : all_motors_in_feedback_order) {
        if (current_rt_settings_.motor_settings.count(motor_id)) {
            const PerMotorRealtimeSettings& motor_setting = current_rt_settings_.motor_settings.at(motor_id);
            if (motor_setting.LFV) {
                size += 1; // Velocity (1 byte)
            }
            if (motor_setting.LFAP) {
                size += 2; // Absolute Position (2 bytes)
            }
        }
    }
    if (current_rt_settings_.LFT)
        size += 1; // Temperature (1 bytes, global)
    return size;
}

outcome::result<RealtimeFeedback, std::error_code>
BarrettHandDriver::parseFeedbackBlock(const std::vector<uint8_t>& block, MotorGroup group) const {
    RealtimeFeedback feedback;
    size_t expected_size = calculateFeedbackBlockSize(group);

    if (block.size() != expected_size) {
        std::cerr << "BarrettHandDriver ERROR: Feedback block size mismatch. Expected " << expected_size << ", got "
                  << block.size() << std::endl;
        return make_error_code(BarrettHandError::PARSING_ERROR);
    }

    const uint8_t* ptr = block.data();
    if (static_cast<char>(ptr[0]) != '*') {
        return make_error_code(BarrettHandError::NO_LOOP_ACK);
    }
    ptr += 1;

    std::vector<MotorID> all_motors_in_feedback_order = getMotorsInGroup(group);

    for (MotorID motor_id : all_motors_in_feedback_order) {
        // Check if this motor's settings are present and if its feedback was enabled
        if (current_rt_settings_.motor_settings.count(motor_id)) {
            const PerMotorRealtimeSettings& motor_setting = current_rt_settings_.motor_settings.at(motor_id);

            if (motor_setting.LFV) {
                if (ptr + 1 > block.data() + block.size())
                    return make_error_code(BarrettHandError::PARSING_ERROR);
                feedback.velocities[motor_id] = *reinterpret_cast<const int8_t*>(ptr);
                ptr++;
            }
            if (motor_setting.LFAP) {
                if (ptr + 2 > block.data() + block.size())
                    return make_error_code(BarrettHandError::PARSING_ERROR);
                auto pos = static_cast<uint16_t>((ptr[0] << 8) | ptr[1]);
                feedback.positions[motor_id] = pos;
                ptr += 2;
            }
        }
    }
    if (current_rt_settings_.LFT) {
        if (ptr + 1 > block.data() + block.size())
            return make_error_code(BarrettHandError::PARSING_ERROR);
        auto temp = static_cast<int8_t>(*ptr);
        feedback.temperature_c = temp;
        ptr += 1;
    }
    return feedback;
}
} // namespace barrett
} // namespace gripper
