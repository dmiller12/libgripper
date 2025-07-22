#include "gripper/barrett/velocity_controller.h"
#include "gripper/barrett/constants.h"

template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

VelocityController::VelocityController(std::array<double, 4> kp, std::array<double, 4> ki, std::array<double, 4> kd)
    : kp_{kp}
    , ki_{ki}
    , kd_{kd} {
}

void VelocityController::setGains(std::array<double, 4> kp, std::array<double, 4> ki, std::array<double, 4> kd) {
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;
}

std::array<double, 4> VelocityController::computeControl(
    std::array<double, 4> ref, std::array<double, 4> actual, std::array<double, 4> actual_position, double dt
) {

    std::array<double, 4> control;

    for (int i = 0; i < 4; i++) {
        double min_position;
        double max_position;
        if (i < 3) {
            min_position = gripper::barrett::RADIAN_FINGER_MIN;
            max_position = gripper::barrett::RADIAN_FINGER_MAX;
        } else {

            min_position = gripper::barrett::RADIAN_SPREAD_MIN;
            max_position = gripper::barrett::RADIAN_SPREAD_MAX;
        }

        if ((actual_position[i] >= max_position && ref[i] > 0) || (actual_position[i] <= min_position && ref[i] < 0)) {
            ref[i] = 0.0;
        }

        double error = ref[i] - actual[i];
        double error_dt = (error - prev_error_[i]) / dt;

        double feedforward = 0;
        // approx friction compensation
        if ((i < 3) && (std::abs(ref[i]) > 0.0)) {
            feedforward = 10.0 * sgn(ref[i]);
        }

        double accel = feedforward + kp_[i] * error + ki_[i] * integral_error_[i] + kd_[i] * error_dt;

        bool is_at_pos_limit =
            (actual_position[i] >= max_position && accel > 0) || (actual_position[i] <= min_position && accel < 0);

        if (is_at_pos_limit) {
            integral_error_[i] = 0.0;
        } else {
            integral_error_[i] += error * dt;
        }

        // TODO: Clean up hardcoded values
        integral_error_[i] = std::min(std::max(integral_error_[i], - 2.0), 2.0);
        control[i] = accel;
        prev_error_[i] = error;
    }

    return control;
}
