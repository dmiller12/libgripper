#include <algorithm>
#include <cctype>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <gripper/robotiq/robotiq_gripper.h>
#include <gripper_ros_common/PositionSetpoint.h>
#include <gripper_ros_common/VelocitySetpoint.h>

using gripper::robotiq::GripperStatus;
using gripper::robotiq::RobotiqGripper;
using gripper::robotiq::kRobotiqMaxWidthMm;

namespace {
double clamp(double value, double min_value, double max_value) {
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}
} // namespace

class RobotiqROSNode {
  public:
    RobotiqROSNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh)
        , private_nh_(private_nh) {
        private_nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
        private_nh_.param<int>("baud_rate", baud_rate_, 115200);
        private_nh_.param<int>("slave_id", slave_id_, 9);
        private_nh_.param<bool>("auto_activate", auto_activate_, true);
        private_nh_.param<double>("default_speed", default_speed_, 0.3);
        private_nh_.param<double>("default_force", default_force_, 0.5);
        private_nh_.param<double>("activation_speed", activation_speed_, default_speed_);
        private_nh_.param<double>("activation_force", activation_force_, default_force_);
        private_nh_.param<std::string>("position_units", position_units_, std::string("normalized"));
        private_nh_.param<double>("publish_rate", publish_rate_, 25.0);

        normalizeUnitString();

        last_speed_ = clamp(default_speed_, 0.0, 1.0);
        last_force_ = clamp(default_force_, 0.0, 1.0);

        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
        position_setpoint_sub_ = nh_.subscribe("position_setpoint", 1, &RobotiqROSNode::positionSetpointCb, this);
        velocity_setpoint_sub_ = nh_.subscribe("velocity_setpoint", 1, &RobotiqROSNode::velocitySetpointCb, this);

        initialize_srv_ = nh_.advertiseService("initialize", &RobotiqROSNode::initializeSrv, this);
        open_srv_ = nh_.advertiseService("open_grasp", &RobotiqROSNode::openSrv, this);
        close_srv_ = nh_.advertiseService("close_grasp", &RobotiqROSNode::closeSrv, this);
        stop_srv_ = nh_.advertiseService("stop", &RobotiqROSNode::stopSrv, this);

        if (!gripper_.connect(port_, static_cast<unsigned int>(baud_rate_), static_cast<uint8_t>(slave_id_))) {
            ROS_FATAL_STREAM("Failed to connect to Robotiq gripper on port " << port_);
            return;
        }

        if (auto_activate_) {
            if (!gripper_.activate(false, activation_speed_, activation_force_)) {
                ROS_ERROR("Robotiq gripper activation failed.");
            } else {
                ROS_INFO("Robotiq gripper activated.");
            }
        }
    }

    void publishState() {
        GripperStatus status = gripper_.getStatus();
        if (!status.valid) {
            ROS_WARN_THROTTLE(5.0, "Unable to read Robotiq gripper status.");
            return;
        }

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"finger_joint"};
        msg.position = {status.width_mm / 1000.0};
        msg.velocity = {0.0};
        msg.effort = {static_cast<double>(status.motor_current) / 255.0};
        joint_state_pub_.publish(msg);
    }

    double publishRate() const {
        return publish_rate_;
    }

  private:
    void normalizeUnitString() {
        std::transform(position_units_.begin(), position_units_.end(), position_units_.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        if (position_units_ != "normalized" && position_units_ != "mm" && position_units_ != "m") {
            ROS_WARN_STREAM("Unknown value for ~position_units: " << position_units_ << ". Falling back to 'normalized'.");
            position_units_ = "normalized";
        }
    }

    double convertCommandToMm(double value) const {
        if (position_units_ == "mm") {
            return clamp(value, 0.0, kRobotiqMaxWidthMm);
        }
        if (position_units_ == "m") {
            return clamp(value * 1000.0, 0.0, kRobotiqMaxWidthMm);
        }
        return clamp(value, 0.0, 1.0) * kRobotiqMaxWidthMm;
    }

    void positionSetpointCb(const gripper_ros_common::PositionSetpointConstPtr& msg) {
        double width_mm = convertCommandToMm(msg->grasp);
        if (!gripper_.setWidth(width_mm, last_speed_, last_force_)) {
            ROS_WARN_THROTTLE(2.0, "Failed to send Robotiq position command.");
        }
    }

    void velocitySetpointCb(const gripper_ros_common::VelocitySetpointConstPtr& msg) {
        last_speed_ = clamp(msg->grasp, 0.0, 1.0);
        last_force_ = clamp(msg->spread, 0.0, 1.0);
    }

    bool initializeSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
        if (!gripper_.isConnected()) {
            if (!gripper_.connect(port_, static_cast<unsigned int>(baud_rate_), static_cast<uint8_t>(slave_id_))) {
                ROS_ERROR("Robotiq gripper reconnection failed.");
                return false;
            }
        }
        if (!gripper_.activate(false, activation_speed_, activation_force_)) {
            ROS_ERROR("Robotiq gripper activation failed.");
            return false;
        }
        ROS_INFO("Robotiq gripper initialized.");
        return true;
    }

    bool openSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
        if (!gripper_.open(last_speed_)) {
            ROS_ERROR("Robotiq gripper failed to open.");
            return false;
        }
        return true;
    }

    bool closeSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
        if (!gripper_.close(last_speed_, last_force_)) {
            ROS_ERROR("Robotiq gripper failed to close.");
            return false;
        }
        return true;
    }

    bool stopSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
        if (!gripper_.stop()) {
            ROS_ERROR("Robotiq gripper stop command failed.");
            return false;
        }
        return true;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::string port_;
    int baud_rate_{115200};
    int slave_id_{9};
    bool auto_activate_{true};
    double default_speed_{0.3};
    double default_force_{0.5};
    double activation_speed_{0.3};
    double activation_force_{0.5};
    std::string position_units_{"normalized"};

    double last_speed_{0.3};
    double last_force_{0.5};
    double publish_rate_{25.0};

    RobotiqGripper gripper_;

    ros::Publisher joint_state_pub_;
    ros::Subscriber position_setpoint_sub_;
    ros::Subscriber velocity_setpoint_sub_;

    ros::ServiceServer initialize_srv_;
    ros::ServiceServer open_srv_;
    ros::ServiceServer close_srv_;
    ros::ServiceServer stop_srv_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robotiq_gripper_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    RobotiqROSNode node(nh, private_nh);

    ros::Rate rate(node.publishRate());
    while (ros::ok()) {
        node.publishState();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
