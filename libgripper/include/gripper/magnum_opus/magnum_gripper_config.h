#pragma once

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>


namespace gripper {
namespace magnum_opus {

struct PidConfig {
    double kp{50.0};
    double ki{1.0};
    double kd{0.2};
};

struct MagnumGripperConfig {
    std::string can_interface{"can0"};
    int motor_id{1};
    PidConfig pid_position;
    double min_pos;
    double max_pos;
    double max_vel;
    double max_acc;
};

MagnumGripperConfig load_config(const std::string& config_dir);

} // namespace magnum_opus
} // namespace gripper

namespace YAML {

template <>
struct convert<gripper::magnum_opus::PidConfig> {
    static bool decode(const Node& node, gripper::magnum_opus::PidConfig& c) {
        if (node["kp"]) c.kp = node["kp"].as<double>();
        if (node["ki"]) c.ki = node["ki"].as<double>();
        if (node["kd"]) c.kd = node["kd"].as<double>();
        return true;
    }
};

template <>
struct convert<gripper::magnum_opus::MagnumGripperConfig> {
    static bool decode(const Node& node, gripper::magnum_opus::MagnumGripperConfig& c) {
        if (node["can_interface"]) c.can_interface = node["can_interface"].as<std::string>();
        if (node["motor_id"]) c.motor_id = node["motor_id"].as<int>();
        if (node["pid_position"]) c.pid_position = node["pid_position"].as<gripper::magnum_opus::PidConfig>();
        c.min_pos = node["min_pos"].as<double>();
        c.max_pos = node["max_pos"].as<double>();
        c.max_vel = node["max_vel"].as<double>();
        c.max_acc = node["max_acc"].as<double>();
        return true;
    }
};

} // namespace YAML

namespace gripper {
namespace magnum_opus {

inline MagnumGripperConfig load_config(const std::string& config_dir) {
    try {
        boost::filesystem::path main_config_path = boost::filesystem::path(config_dir) / "magnum_gripper.yaml";
        YAML::Node main_yaml = YAML::LoadFile(main_config_path.string());

        return main_yaml["magnum_gripper"].as<MagnumGripperConfig>();
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading from config dir (" << config_dir << "): " << e.what() << std::endl;
        throw;
    }
}

} // namespace magnum_opus
} // namespace gripper