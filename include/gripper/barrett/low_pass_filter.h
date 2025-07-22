
#include <array>
class LowPassFilter {
public:
    LowPassFilter(std::array<double, 4> alpha) : alpha_(alpha), filtered_values_{}, is_initialized_(false) {}
    std::array<double, 4> update(std::array<double, 4> raw_value) {
        if (!is_initialized_) {
            filtered_values_ = raw_value;
            is_initialized_ = true;
        } else {
            for (int i=0; i < filtered_values_.size(); i++) {
                filtered_values_[i] = (alpha_[i] * raw_value[i]) + (1.0 - alpha_[i]) * filtered_values_[i];
            }
        }
        return filtered_values_;
    }
private:
    std::array<double, 4> alpha_;
    std::array<double, 4> filtered_values_;
    bool is_initialized_;
};
