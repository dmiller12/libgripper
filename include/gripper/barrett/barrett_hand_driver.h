#pragma once

#include <future>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <boost/optional.hpp>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <boost/outcome/result.hpp> 

namespace outcome = BOOST_OUTCOME_V2_NAMESPACE;

class SerialCommunicator;

namespace gripper {
namespace barrett {

enum class MotorID { F1 = 0, F2 = 1, F3 = 2, Spread = 3 };
enum class MotorGroup { F1, F2, F3, Spread, AllFingers, All };

std::vector<MotorID> getMotorsInGroup(MotorGroup group);


struct RealtimeFeedback {
    std::map<MotorID, int32_t> positions;
    std::map<MotorID, int8_t> velocities;
    boost::optional<int8_t> temperature_c;
};

struct RealtimeControlSetpoint {
    std::map<MotorID, int8_t> velocity_commands;
    std::map<MotorID, uint8_t> proportional_gains;
    std::map<MotorID, int16_t> torque_commands;
};

struct PerMotorRealtimeSettings {
    bool LCV = false; // Loop Control Velocity
    uint8_t LCVC = 1; // Loop Control Velocity Coefficient
    bool LCT= false; // Loop Control Torque
    uint8_t LFVC = 1; // Loop Feedback Velocity Coefficient
    bool LCPG = false; // Loop Control Proportional Gain
    bool LFV = false; // Loop Feedback Velocity
    bool LFAP = false; // Loop Feedback Absolute Position
};

struct RealtimeSettings {
    std::map<MotorID, PerMotorRealtimeSettings> motor_settings;
    bool LFT = false; // Loop Feedback Temperature
};

/**
 * @class BarrettHandDriver
 * @brief Low-level driver for the BH8-Series BarrettHand.
 */
class BarrettHandDriver {
public:
    using RealtimeCallback = std::function<boost::optional<RealtimeControlSetpoint>(const RealtimeFeedback&)>;

    BarrettHandDriver();
    ~BarrettHandDriver();
    
    BarrettHandDriver(const BarrettHandDriver&) = delete;
    BarrettHandDriver& operator=(const BarrettHandDriver&) = delete;

    bool connect(const std::string& port, unsigned int baud_rate = 9600);
    void disconnect();
    bool isConnected() const;
    bool isInRealtimeControl() const;
    
    std::future<outcome::result<std::string, std::error_code>> sendSupervisoryCommand(const std::string& command);
    outcome::result<void, std::error_code> configureRealtime(const RealtimeSettings& settings);
    
    outcome::result<void, std::error_code> startRealtimeControl(RealtimeCallback callback, MotorGroup group);
    void stopRealtimeControl();

private:
    void realtimeControlLoop(MotorGroup group);
    std::future<outcome::result<std::string, std::error_code>> sendSynchronousCommand(const std::string& command_str, int timeout_ms, bool is_loop_cmd=false);
    outcome::result<RealtimeFeedback, std::error_code> parseFeedbackBlock(const std::vector<uint8_t>& block, MotorGroup group) const;
    size_t calculateFeedbackBlockSize(MotorGroup group) const;

    std::unique_ptr<SerialCommunicator> communicator_;
    std::atomic<bool> is_connected_{false};
    std::atomic<bool> in_realtime_mode_{false};
    std::atomic<bool> stop_threads_{false};
    
    std::mutex supervisory_mutex_;
    
    std::thread realtime_thread_;
    RealtimeCallback realtime_callback_;
    RealtimeSettings current_rt_settings_;

    struct Impl;
    // TODO: move more private state to impl
    std::unique_ptr<Impl> pimpl_;
};

} // namespace barrett
} // namespace gripper
