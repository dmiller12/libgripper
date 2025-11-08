#include <algorithm>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <gripper/robotiq/robotiq_gripper.h>
#include <gripper_ros_common/PositionSetpoint.h>
#include <gripper_ros_common/VelocitySetpoint.h>

using gripper::robotiq::RobotiqGripper;
using gripper::robotiq::RobotiqState;

class RobotiqNode {
  public:
    explicit RobotiqNode(ros::NodeHandle& nh)
        : nh_(nh) {
        nh_.param<std::string>("port", port_, std::string("/dev/ttyUSB0"));
        nh_.param<int>("baud_rate", baud_rate_, 115200);
        nh_.param<double>("default_speed_ratio", default_speed_ratio_, 0.4);
        nh_.param<double>("default_force_ratio", default_force_ratio_, 0.5);
        nh_.param<bool>("auto_initialize", auto_initialize_, false);
        nh_.param<bool>("invert_service_direction", invert_service_direction_, false);

        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
        position_setpoint_sub_ = nh_.subscribe("position_setpoint", 1, &RobotiqNode::positionSetpointCb, this);
        velocity_setpoint_sub_ = nh_.subscribe("velocity_setpoint", 1, &RobotiqNode::velocitySetpointCb, this);

        initialize_srv_ = nh_.advertiseService("initialize", &RobotiqNode::handleInitialize, this);
        open_grasp_srv_ = nh_.advertiseService("open_grasp", &RobotiqNode::handleOpenGrasp, this);
        close_grasp_srv_ = nh_.advertiseService("close_grasp", &RobotiqNode::handleCloseGrasp, this);

        if (auto_initialize_) {
            if (handleInitialize(empty_req_, empty_res_)) {
                ROS_INFO("Robotiq gripper automatically initialized");
            } else {
                ROS_WARN("Auto initialization failed. Call the initialize service manually.");
            }
        } else {
            ROS_INFO("Robotiq node ready. Call %s/initialize to connect to the gripper.", ros::this_node::getName().c_str());
        }
    }

    void publishState() {
        if (!gripper_.isConnected()) {
            return;
        }

        RobotiqState state;
        if (!gripper_.readState(state)) {
            ROS_WARN_THROTTLE(5.0, "Failed to read Robotiq state");
            return;
        }

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"robotiq_left_finger_joint", "robotiq_right_finger_joint"};

        double half_width = state.actual_width_m * 0.5;
        msg.position = {half_width, -half_width};
        msg.velocity = {0.0, 0.0};
        double current_amps = static_cast<double>(state.current_raw) * 0.01;
        msg.effort = {current_amps, current_amps};

        joint_state_pub_.publish(msg);
    }

  private:
    bool ensureReady() {
        if (initialized_) {
            return true;
        }
        ROS_WARN_THROTTLE(5.0, "Robotiq gripper not initialized yet. Call the initialize service.");
        return false;
    }

    bool handleInitialize(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
        if (!gripper_.isConnected()) {
            if (!gripper_.connect(port_, static_cast<unsigned int>(baud_rate_))) {
                ROS_ERROR("Unable to open serial port %s", port_.c_str());
                initialized_ = false;
                return false;
            }
        }

        if (!gripper_.setSpeedRatio(default_speed_ratio_)) {
            ROS_WARN("Failed to set Robotiq speed ratio");
        }
        if (!gripper_.setForceRatio(default_force_ratio_)) {
            ROS_WARN("Failed to set Robotiq force ratio");
        }

        if (!gripper_.activate(true)) {
            ROS_ERROR("Robotiq activation timed out");
            initialized_ = false;
            return false;
        }

        initialized_ = true;
        ROS_INFO("Robotiq gripper initialized on %s", port_.c_str());
        return true;
    }

    bool handleOpenGrasp(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        (void)req;
        (void)res;
        if (!ensureReady()) {
            return false;
        }
        if (invert_service_direction_) {
            return gripper_.close();
        }
        return gripper_.open();
    }

    bool handleCloseGrasp(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        (void)req;
        (void)res;
        if (!ensureReady()) {
            return false;
        }
        if (invert_service_direction_) {
            return gripper_.open();
        }
        return gripper_.close();
    }

    void positionSetpointCb(const gripper_ros_common::PositionSetpointConstPtr& msg) {
        if (!ensureReady()) {
            return;
        }
        double width = msg->grasp;
        width = std::max(0.0, std::min(width, gripper_.maxStrokeMeters()));
        if (!gripper_.setWidth(width)) {
            ROS_WARN_THROTTLE(2.0, "Failed to send Robotiq width command");
        }
    }

    void velocitySetpointCb(const gripper_ros_common::VelocitySetpointConstPtr& msg) {
        if (!ensureReady()) {
            return;
        }
        double ratio = std::fabs(msg->grasp);
        ratio = std::max(0.0, std::min(ratio, 1.0));
        if (!gripper_.setSpeedRatio(ratio)) {
            ROS_WARN_THROTTLE(2.0, "Failed to update Robotiq speed ratio");
        }
    }

    ros::NodeHandle nh_;
    std::string port_;
    int baud_rate_ = 115200;
    double default_speed_ratio_ = 0.4;
    double default_force_ratio_ = 0.5;
    bool auto_initialize_ = false;
    bool invert_service_direction_ = false;
    bool initialized_ = false;

    RobotiqGripper gripper_;

    ros::Publisher joint_state_pub_;
    ros::Subscriber position_setpoint_sub_;
    ros::Subscriber velocity_setpoint_sub_;
    ros::ServiceServer initialize_srv_;
    ros::ServiceServer open_grasp_srv_;
    ros::ServiceServer close_grasp_srv_;

    std_srvs::Empty::Request empty_req_;
    std_srvs::Empty::Response empty_res_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robotiq_gripper_node");
    ros::NodeHandle nh("~");

    RobotiqNode node(nh);

    ros::Rate loop_rate(25);
    while (ros::ok()) {
        node.publishState();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
