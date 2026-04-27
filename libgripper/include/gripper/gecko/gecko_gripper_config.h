#pragma once

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>


namespace gripper {
namespace gecko {

struct PidConfig {
    double kp{50.0};
    double ki{1.0};
    double kd{0.2};
};

struct GeckoGripperConfig {
    std::string transport_type;
    std::string transport_usb;
    std::string transport_pcie;
    int motor_id{1};
    PidConfig pid_position;
    double min_pos;
    double max_pos;
    double max_vel;
    double max_acc;
};

GeckoGripperConfig load_config(const std::string& config_dir);

} // namespace gecko
} // namespace gripper

namespace YAML {

template <>
struct convert<gripper::gecko::PidConfig> {
    static bool decode(const Node& node, gripper::gecko::PidConfig& c) {
        if (node["kp"]) c.kp = node["kp"].as<double>();
        if (node["ki"]) c.ki = node["ki"].as<double>();
        if (node["kd"]) c.kd = node["kd"].as<double>();
        return true;
    }
};

template <>
struct convert<gripper::gecko::GeckoGripperConfig> {
    static bool decode(const Node& node, gripper::gecko::GeckoGripperConfig& c) {
        // fallback to usbfd
        if (node["transport_type"]) {
            c.transport_type = node["transport_type"].as<std::string>();
        } else {
            c.transport_type = "usb"; 
        }

        // Decode specific transport arguments
        if (node["transport_usb"]) {
            c.transport_usb = node["transport_usb"].as<std::string>();
        }
        if (node["transport_pcie"]) {
            c.transport_pcie = node["transport_pcie"].as<std::string>();
        }

        if (node["motor_id"]) c.motor_id = node["motor_id"].as<int>();
        if (node["pid_position"]) c.pid_position = node["pid_position"].as<gripper::gecko::PidConfig>();
        c.min_pos = node["min_pos"].as<double>();
        c.max_pos = node["max_pos"].as<double>();
        c.max_vel = node["max_vel"].as<double>();
        c.max_acc = node["max_acc"].as<double>();
        return true;
    }
};

} // namespace YAML

namespace gripper {
namespace gecko {

inline GeckoGripperConfig load_config(const std::string& config_dir) {
    try {
        boost::filesystem::path main_config_path = boost::filesystem::path(config_dir) / "gecko_gripper.yaml";
        YAML::Node main_yaml = YAML::LoadFile(main_config_path.string());

        return main_yaml["gecko_gripper"].as<GeckoGripperConfig>();
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading from config dir (" << config_dir << "): " << e.what() << std::endl;
        throw;
    }
}

} // namespace gecko
} // namespace gripper
