#pragma once

namespace gripper {
namespace gecko {

struct GripperState {
    double position{0.0};
    double velocity{0.0};
    double torque{0.0};
};

enum class ControlMode { None, Position, Velocity };

} // namespace gecko
} // namespace gripper
