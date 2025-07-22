#include <array>

class PositionController {
  public:
    PositionController(std::array<double, 4> kp, std::array<double, 4> ki, std::array<double, 4> kd);

    void setGains(std::array<double, 4> kp, std::array<double, 4> ki, std::array<double, 4> kd);
    std::array<double, 4> computeControl(std::array<double, 4> ref, std::array<double, 4> actual, double dt);

  private:
    std::array<double, 4> prev_error_{};
    std::array<double, 4> integral_error_{};
    std::array<double, 4> kp_;
    std::array<double, 4> ki_;
    std::array<double, 4> kd_;
};
