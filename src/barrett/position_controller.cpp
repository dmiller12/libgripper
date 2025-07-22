#include "gripper/barrett/position_controller.h"
#include "gripper/barrett/constants.h"

PositionController::PositionController(std::array<double, 4> kp, std::array<double, 4> ki, std::array<double, 4> kd)
    : kp_{kp}
    , ki_{ki}
    , kd_{kd} {
}

void PositionController::setGains(std::array<double, 4> kp, std::array<double, 4> ki, std::array<double, 4> kd) {
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;
}

std::array<double, 4>
PositionController::computeControl(std::array<double, 4> ref, std::array<double, 4> actual, double dt) {

    std::array<double, 4> control;
    for (int i = 0; i < 4; i++) {
        double error = ref[i] - actual[i];
        double error_dt = (error - prev_error_[i]) / dt;

        control[i] = kp_[i] * error + ki_[i] * integral_error_[i] + kd_[i] * error_dt;

        double min_position;
        double max_position;
        if (i < 3) {
            min_position = gripper::barrett::RADIAN_FINGER_MIN;
            max_position = gripper::barrett::RADIAN_FINGER_MAX;
        } else {

            min_position = gripper::barrett::RADIAN_SPREAD_MIN;
            max_position = gripper::barrett::RADIAN_SPREAD_MAX;
        }

        bool is_at_pos_limit =
            (ref[i] >= max_position && control[i] > 0) || (ref[i] <= min_position && control[i] < 0);
        

        if (is_at_pos_limit) {
            integral_error_[i] = 0.0;
        } else {
            integral_error_[i] += error * dt;
        }
        prev_error_[i] = error;
    }
    return control;
}
