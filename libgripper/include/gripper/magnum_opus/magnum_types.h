#pragma once

namespace gripper {
namespace magnum_opus {

struct GripperState {
    double position{0.0};
    double velocity{0.0};
    double torque{0.0};
};

enum class ControlMode { None, Position, Velocity };

} // namespace magnum_opus
} // namespace gripper